#!/usr/bin/env python3
"""2번 하위 제어 — 작업공간 임피던스 (토크) 노드. 김민기.

근거 문서: 「작업공간 임피던스 제어 PD gain 설정」(HANDOFF v2 9절 #1~#13), MR 8.6 · 11.4.1 · 11.7.1, Khatib(1987)
식 ↔ 코드: 각 블록 위 `# [문서 #n]` 주석. 문서에 없는 ROS·Gazebo 배관은 `# [문서 밖]`.

  [문서 #2]  τ = Jᵀ F_cmd + g(q) + N_T τ0,   F_cmd = K e + D ė
  [문서 #1]  Λ = (J M⁻¹ Jᵀ)⁻¹
  [문서 #3]  J̄ᵀ = Λ J M⁻¹,   N_T = I − Jᵀ J̄ᵀ,   τ0 = K_q (q_nom − q) − D_q q̇
  [문서 #4]  e = [p_d − p ; log3(R_d Rᵀ)],   ė = V_d − J q̇
  [문서 #5]  D = 2ζ √(Λ_ii K_ii)  (Λ 대각 근사, 매 주기 재계산)
  [문서 #9]  τ += J_vᵀ [0, 0, m g / 2]ᵀ   (박스 무게 피드포워드, 램프)

입력 [문서 밖]: /<robot>/robot_description (모델), /sim/<robot>/joint_states (q, q̇)
출력 [문서 밖]: /sim/<robot>/effort (Actuators.normalized = 관절 토크, NaN = 그 관절은 위치 서보 유지)
명령 [문서 밖]: impedance/<s>/goal (PoseStamped, base_link) · impedance/<s>/stiffness (Vector3 K_pos) ·
               impedance/<s>/weight (Float64, 피드포워드 질량) · impedance/enable (Bool)
"""
import csv
import math
import os
import time

import numpy as np
import pinocchio as pin
import yaml
import rclpy
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String

# [문서 밖] 현대 sim_command_adapter·Gazebo 플러그인의 명령 순서(joint_map.yaml commanded_joints)
COMMANDED = [f'arm_r_joint{i}' for i in range(1, 8)] + ['gripper_r_joint1'] + \
            [f'arm_l_joint{i}' for i in range(1, 8)] + ['gripper_l_joint1']
G = 9.81


def quintic(tau):
    """5차 시간 스케일링 s(τ), ṡ(τ)  — MR 9.2"""
    tau = min(max(tau, 0.0), 1.0)
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5, 30 * tau**2 - 60 * tau**3 + 30 * tau**4


class Arm:
    def __init__(self, node, side, model):
        self.side = side
        self.names = [f'arm_{side}_joint{i}' for i in range(1, 8)]
        self.qi = [model.joints[model.getJointId(n)].idx_q for n in self.names]
        self.vi = [model.joints[model.getJointId(n)].idx_v for n in self.names]
        self.fid = model.getFrameId(f'end_effector_{side}_link')
        self.cmd_idx = [COMMANDED.index(n) for n in self.names]
        P = node.get_parameter
        role = P('role').value                      # [문서 #7] 리더 전 축 단단 / 팔로워 수평(xy)만 부드럽게, z는 둘 다 단단
        K_pos = P(f'{role}_K_pos').value if role in ('leader', 'follower') else P('K_pos').value
        self.K = np.diag(list(K_pos) + list(P('K_rot').value))
        node.get_logger().info(f'[{side}] role={role} K_pos={list(K_pos)}')
        self.q_nom = None          # 영공간 자세 목표 (켤 때 자세)
        self.xd = None             # (p_d, R_d)
        self.traj = None           # (t0, T, p0, R0, pf, Rf)
        self.m_ff = 0.0            # 피드포워드 질량 목표 [kg]
        self.m_ff_now = 0.0        # 램프 중인 현재 값
        node.create_subscription(PoseStamped, f'impedance/{side}/goal', self.on_goal, 10)
        node.create_subscription(Vector3, f'impedance/{side}/stiffness', self.on_stiffness, 10)
        node.create_subscription(Vector3, f'impedance/{side}/offset', self.on_offset, 10)
        node.create_subscription(Float64, f'impedance/{side}/weight', lambda m: setattr(self, 'm_ff', m.data), 10)
        self.node = node

    def on_goal(self, msg):
        if self.xd is None:
            return
        o = msg.pose.orientation
        pf = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        Rf = pin.Quaternion(o.w, o.x, o.y, o.z).normalized().toRotationMatrix() if (o.x, o.y, o.z, o.w) != (0, 0, 0, 0) \
            else self.xd[1].copy()       # 자세를 안 주면 현재 목표 자세 유지
        T = float(self.node.get_parameter('move_time').value)
        self.traj = (time.monotonic(), T, self.xd[0].copy(), self.xd[1].copy(), pf, Rf)

    def on_offset(self, msg):
        """현재 목표에서 (dx, dy, dz)만큼 이동 — 자세 유지, move_time 동안 5차 궤적 (T5 계단: move_time 작게)"""
        if self.xd is None:
            return
        pf = self.xd[0] + np.array([msg.x, msg.y, msg.z])
        T = float(self.node.get_parameter('move_time').value)
        self.traj = (time.monotonic(), T, self.xd[0].copy(), self.xd[1].copy(), pf, self.xd[1].copy())

    def on_stiffness(self, msg):
        self.K[0, 0], self.K[1, 1], self.K[2, 2] = msg.x, msg.y, msg.z

    def desired(self):
        """현재 목표 (p_d, R_d, V_d). 목표 가속도는 0으로 가정 — [문서 #4]"""
        if self.traj is None:
            return self.xd[0], self.xd[1], np.zeros(6)
        t0, T, p0, R0, pf, Rf = self.traj
        s, ds = quintic((time.monotonic() - t0) / T)
        w = pin.log3(R0.T @ Rf)
        pd = p0 + s * (pf - p0)
        Rd = R0 @ pin.exp3(w * s)
        Vd = np.concatenate([(pf - p0) * ds / T, R0 @ w * ds / T])
        self.xd = (pd, Rd) if s < 1.0 else (pf, Rf)
        if s >= 1.0:
            self.traj = None
        return pd, Rd, Vd


class ImpedanceNode(Node):
    def __init__(self):
        super().__init__('impedance_controller')
        d = self.declare_parameter
        d('robot', 'robot1'); d('sides', ['r'])
        d('role', 'none')                             # none | leader | follower  (리더–팔로워 강성 차등)
        d('leader_K_pos', [800.0, 800.0, 1000.0])     # gain_design 범위: K_xy ≤ 1500, K_z ≥ 490 (박스 자세)
        d('follower_K_pos', [300.0, 300.0, 1000.0])
        d('output', 'sim')                            # sim: Gazebo 토크 / dxl: 실물 DYNAMIXEL 전류(Goal Current raw)
        d('motor_params', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'motor_params.yaml'))
        d('mode', 'task')                             # position: 팔은 현대 QP(위치) 그대로, 그리퍼만 / gravity: τ=g(q)만(T12) / task: [문서 #2]
        d('K_pos', [500.0, 500.0, 1000.0])            # [N/m]  x, y, z (base_link)
        d('K_rot', [30.0, 30.0, 30.0])                # [N·m/rad]  문서 #12: 위치·회전 K 단위 분리
        d('zeta', 1.0)                                # 문서 #5: ζ=1로 시작
        # [문서 #3] 영공간(팔꿈치) 자세 유지 τ0 게인. 작업공간 K로는 이 방향을 못 버팀 → 너무 작으면 팔꿈치 처짐
        # D_null 상한 [문서 #8]: D·dt/M_min < 1 (손목 M≈0.01, dt=2 ms → D < 5). 1.5 → 0.3
        d('K_null', 20.0); d('D_null', 1.5)
        # 교수님 지시: 손목 5·6·7 관성(모터 armature) 반영해 Λ 구성 — 실물(output dxl)에만. Gazebo 모델엔 armature가 없어
        # 시뮬에서 넣으면 제어기가 손목을 실제보다 무겁게 봐 D가 커지고 발산(11/3 그리퍼 연 뒤 57 Hz 떨림)
        d('armature', [0.0] * 7)
        d('armature_real', [0.0, 0.0, 0.0, 0.0, 0.05, 0.05, 0.05])
        d('D_safety', 0.5)                            # [문서 #8] D_ii ≤ D_safety · 2Λ_ii / T  (T = 실측 루프 주기)
        d('tau_max', [40.0, 40.0, 30.0, 30.0, 10.0, 10.0, 10.0])
        d('ramp_time', 0.0)                           # 전환 순간 자세 = 목표라 초기 오차 0 → 램프 불필요 (0 = 즉시 K 전부)
        d('weight_ramp', 1.0)                         # [문서 #9] 피드포워드 램프 시간 [s]
        d('move_time', 3.0)
        d('state_timeout', 0.05)
        d('log_dir', os.path.expanduser('~/impedance_logs'))
        self.robot = self.get_parameter('robot').value
        self.model = None
        self.enabled = True
        self.last_state = 0.0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, f'/{self.robot}/robot_description', self.on_urdf, qos)
        state_topic = f'/sim/{self.robot}/joint_states' if self.get_parameter('output').value == 'sim' \
            else f'/{self.robot}/joint_states'
        self.create_subscription(JointState, state_topic, self.on_state, qos_profile_sensor_data)
        self.create_subscription(Bool, 'impedance/enable', self.on_enable, 10)
        self.create_subscription(String, 'impedance/mode', self.on_mode, 10)
        self.gripper = {}                                 # side → 위치 목표 [rad] (0 열림, 1.1 닫힘)
        for s in ('r', 'l'):
            self.create_subscription(Float64, f'impedance/{s}/gripper',
                                     lambda m, s=s: self.gripper.__setitem__(s, m.data), 10)
        self.output = self.get_parameter('output').value
        if self.output == 'dxl':
            mp = yaml.safe_load(open(self.get_parameter('motor_params').value))
            self.motor = {k: mp[k] for k in ('Kt_eff', 'raw_per_A', 'current_limit_raw')}
            missing = [k for k, v in self.motor.items() if v is None or any(x is None for x in v)]
            if missing:
                raise RuntimeError(f'motor_params.yaml 값 없음 {missing} — 데이터시트·calib_kt.py로 채운 뒤 실행 (안전)')
            self.motor = {k: np.array(v, dtype=float) for k, v in self.motor.items()}
            self.pub_dxl = {s: self.create_publisher(Float64MultiArray, f'/{self.robot}/arm_{s}_effort_controller/commands', 1)
                            for s in ('r', 'l')}
        self.pub = self.create_publisher(Actuators, f'/sim/{self.robot}/effort', 1)
        os.makedirs(self.get_parameter('log_dir').value, exist_ok=True)
        fn = os.path.join(self.get_parameter('log_dir').value, time.strftime('%m%d_%H%M%S') + f'_task_{self.robot}.csv')
        self.logf = open(fn, 'w', newline='')
        self.log = csv.writer(self.logf)
        cols = ['t', 'side'] + [f'{a}{c}' for a in ('p', 'pd', 'e', 'F', 'Fhat') for c in 'xyz'] + \
               ['Kx', 'Ky', 'Kz', 'Dx', 'Dy', 'Dz', 'Lx', 'Ly', 'Lz', 'm_ff'] + [f'tau{i}' for i in range(1, 8)] + \
               [f'q{i}' for i in range(1, 8)]
        self.log.writerow(cols)
        self.t0 = time.monotonic()
        self.get_logger().info(f'로그: {fn}')

    def on_enable(self, msg):
        if msg.data and not self.enabled:
            for a in getattr(self, 'arms', []):
                a.xd = None              # 다시 켜면 그때 자세를 목표로
        self.enabled = msg.data
        self.get_logger().info(f'enable = {msg.data}')

    def torque_to_current(self, tau):
        """관절 토크 → 모터 전류 명령(Goal Current raw).  I[A] = τ / Kt_eff,  raw = I · raw_per_A,  전류 한계로 포화.
        Kt_eff[N·m/A]는 감속비·효율 포함 관절 출력 기준 (MR 11.8: τ = Kt·G·η·I). 값은 motor_params.yaml(실측 보정)"""
        raw = tau / self.motor['Kt_eff'] * self.motor['raw_per_A']
        return np.clip(np.round(raw), -self.motor['current_limit_raw'], self.motor['current_limit_raw'])

    def on_mode(self, msg):
        if msg.data in ('position', 'gravity', 'task'):
            self.set_parameters([rclpy.parameter.Parameter('mode', value=msg.data)])
            self.get_logger().info(f'mode = {msg.data}')

    def on_urdf(self, msg):
        if self.model is not None:
            return
        self.model = pin.buildModelFromXML(msg.data)
        self.data = self.model.createData()
        self.qmap = {self.model.names[j]: self.model.joints[j].idx_q
                     for j in range(1, self.model.njoints) if self.model.joints[j].nq == 1}
        self.arms = [Arm(self, s, self.model) for s in self.get_parameter('sides').value]
        self.get_logger().info(f'모델: nq={self.model.nq}, 팔 {[a.side for a in self.arms]}')

    def on_state(self, msg):
        """[문서 밖] 관절 상태가 올 때마다 1주기 계산 → 상태 주기(state_rate) = 토크 루프 주기"""
        if self.model is None or not self.enabled:
            return
        now = time.monotonic()
        dt = now - self.last_state if self.last_state else 0.0
        self.last_state = now
        m, d = self.model, self.data
        q = pin.neutral(m)
        v = np.zeros(m.nv)
        for n, name in enumerate(msg.name):
            name = name.split('::')[-1]
            if name in self.qmap:
                q[self.qmap[name]] = msg.position[n]
                if len(msg.velocity) > n:
                    v[m.joints[m.getJointId(name)].idx_v] = msg.velocity[n]
        P = lambda k: np.array(self.get_parameter(k).value, dtype=float)  # noqa: E731
        # [문서 #12-③] g(q): Pinocchio = 중력을 지탱하는 토크 → τ에 +g (MR 8.3 RNEA(q,0,0))
        g_full = pin.computeGeneralizedGravity(m, d, q).copy()
        pin.computeJointJacobians(m, d, q)
        pin.updateFramePlacements(m, d)
        # [문서 #1] crba는 위 삼각만 채움 → 대칭화
        M_full = pin.crba(m, d, q)
        M_full = np.triu(M_full) + np.triu(M_full, 1).T
        out = [math.nan] * len(COMMANDED)
        pos = [math.nan] * len(COMMANDED)
        for side, target in self.gripper.items():          # (v는 관절 속도 배열 — 덮어쓰지 않게 이름 분리)
            pos[COMMANDED.index(f'gripper_{side}_joint1')] = target
        mode = self.get_parameter('mode').value
        for a in self.arms:
            if mode == 'position':                              # 토크 전환 전: 팔 NaN → 현대 QP 위치 서보
                a.xd = None
                continue
            vi = a.vi
            qa, dqa = q[a.qi], v[vi]
            oMf = d.oMf[a.fid]
            p, R = oMf.translation.copy(), oMf.rotation.copy()
            if a.xd is None:                                   # 켤 때 현재 자세를 목표로
                a.xd, a.q_nom, a.t_on = (p.copy(), R.copy()), qa.copy(), now
            ramp = min(1.0, (now - a.t_on) / max(P('ramp_time').item(), 1e-3))
            # [문서 #1] J: 팔 7축 열만 (리프트 열 제외), 6×7, LOCAL_WORLD_ALIGNED(base_link 축)
            J = pin.getFrameJacobian(m, d, a.fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, vi]
            M = M_full[np.ix_(vi, vi)] + np.diag(P('armature_real' if self.output == 'dxl' else 'armature'))
            M_inv = np.linalg.inv(M)
            Lambda = np.linalg.inv(J @ M_inv @ J.T + 1e-6 * np.eye(6))
            # [문서 #4] 6D 오차
            pd, Rd, Vd = a.desired()
            e = np.concatenate([pd - p, pin.log3(Rd @ R.T)])
            edot = Vd - J @ dqa
            # [문서 #5] D = 2ζ√(Λ_ii K_ii), 매 주기 재계산
            K = a.K * ramp
            D_ii = 2 * self.get_parameter('zeta').value * np.sqrt(np.clip(np.diag(Lambda) * np.diag(K), 0, None))
            # [문서 #8] 이산화 감쇠 상한 D < 2Λ/T — 실측 루프 주기 T(지연·저속 루프 반영)로 넘지 않게
            self.T_loop = 0.98 * getattr(self, 'T_loop', 0.002) + 0.02 * dt if 0 < dt < 0.05 else getattr(self, 'T_loop', 0.002)
            D = np.diag(np.minimum(D_ii, P('D_safety').item() * 2 * np.diag(Lambda) / self.T_loop))
            # [문서 #2] F_cmd = K e + D ė,  τ_task = Jᵀ F_cmd
            F_cmd = K @ e + D @ edot
            tau_task = J.T @ F_cmd
            # [문서 #3] 영공간: J̄ᵀ = Λ J M⁻¹,  N_T = I − Jᵀ J̄ᵀ,  τ0 = K_q(q_nom − q) − D_q q̇
            J_bar_T = Lambda @ J @ M_inv
            N_T = np.eye(7) - J.T @ J_bar_T
            tau0 = ramp * (self.get_parameter('K_null').value * (a.q_nom - qa) - self.get_parameter('D_null').value * dqa)
            tau_null = N_T @ tau0
            # [문서 #9] 박스 무게 피드포워드 τ += J_vᵀ[0,0,m g/2]ᵀ — 손 하나가 절반, 램프로 켜고 끔
            step = dt / max(self.get_parameter('weight_ramp').value, 1e-3)
            a.m_ff_now += float(np.clip(a.m_ff - a.m_ff_now, -step * max(a.m_ff, a.m_ff_now, 1e-9), step * max(a.m_ff, a.m_ff_now, 1e-9)))
            tau_ff = J[:3].T @ np.array([0.0, 0.0, a.m_ff_now * G / 2])
            # [문서 #2] τ_cmd = τ_task + g + τ_null (+ 피드포워드), 포화
            if mode == 'gravity':
                tau_cmd = np.clip(g_full[vi], -P('tau_max'), P('tau_max'))     # [T12] g(q)만 → 팔이 떠 있어야 함
            else:
                tau_cmd = np.clip(tau_task + g_full[vi] + tau_null + tau_ff, -P('tau_max'), P('tau_max'))
            for k, idx in enumerate(a.cmd_idx):
                out[idx] = float(tau_cmd[k])
            near = [k + 1 for k, i in enumerate(a.qi) if min(qa[k] - m.lowerPositionLimit[i], m.upperPositionLimit[i] - qa[k]) < 0.05]
            if near:
                self.get_logger().warn(f'[{a.side}] 관절 {near} 가동 한계 3° 이내 — 이 방향으로는 손이 못 움직임', throttle_duration_sec=2.0)
            if self.output == 'dxl':
                self.pub_dxl[a.side].publish(Float64MultiArray(data=self.torque_to_current(tau_cmd).tolist()))
            # [문서 #11] 모니터링: F̂_ext = Λë + Dė + Ke (ë는 ė 차분 — 기록용)
            ddot = (edot - a.edot_prev) / dt if (dt > 0 and hasattr(a, 'edot_prev')) else np.zeros(6)
            a.edot_prev = edot
            F_hat = Lambda @ ddot + D @ edot + K @ e
            self.log.writerow([f'{time.time():.4f}', a.side, *np.round(p, 5), *np.round(pd, 5), *np.round(e[:3], 5),
                               *np.round(F_cmd[:3], 3), *np.round(F_hat[:3], 3), *np.round(np.diag(K)[:3], 1),
                               *np.round(np.diag(D)[:3], 2), *np.round(np.diag(Lambda)[:3], 3), round(a.m_ff_now, 3),
                               *np.round(tau_cmd, 3), *np.round(qa, 4)])
        if now - getattr(self, 'last_flush', 0.0) > 0.1:      # 로그를 0.1 s마다 파일에 씀 (버퍼에만 있으면 분석 불가)
            self.logf.flush()
            self.last_flush = now
        cmd = Actuators()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.normalized = out
        cmd.position = pos
        if self.output == 'sim':
            self.pub.publish(cmd)


def main():
    rclpy.init()
    node = ImpedanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.logf.close()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
