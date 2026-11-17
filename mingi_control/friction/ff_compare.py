#!/usr/bin/env python3
"""마찰 피드포워드 보상 비교 (시뮬, 임피던스 없이 관절 궤적 추종만) — MR 식·개념만. 김민기 (11/17).

제어기 (한 관절)
  τ = M̃(θ_d)θ̈_d + h̃(θ_d, θ̇_d) + K_p θ_e + K_d θ̇_e          MR 식 (11.32) 피드포워드 + (11.24) PD
  h̃ = g(θ_d) [+ b_static·sgn(θ̇_d)] [+ b_viscous·θ̇_d]          (11.21) h에 마찰 포함, 그림 8.13 — 케이스 none/coulomb/viscous/both
게인 K_p (값을 고르지 않고 식으로 정함) — --stiff
  design: 기존 설계 강성 (gains.yaml K_pos, K_rot, 11/5 게인 문서)을 관절로: K_p = J_jᵀ diag(K_rot, K_pos) J_j
          (MR 5.2 τ = JᵀF, F = K·Δx, Δx = J_j Δθ_j — 한 관절만 움직일 때)
  kmin:   강성 하한 K_p = b_static / δθ_j  — 정지 목표에서 남는 마찰 = b_static (그림 8.13(b)) → |θ_e| ≤ b_static/K_p (MR (11.27) 꼴)
          δθ_j = δx / ‖J_v,j(θ0)‖, 손끝 허용 오차 δx = 5 mm (게인 문서), 한 관절만 돌 때 손끝 변위 = J_v,j Δθ_j (MR 5장)
  K_d = 2√(K_p M) − b_viscous  MR (11.26) ζ = (b + K_d)/(2√(K_p M)) = 1 (MR 본문: 임계 감쇠)
궤적  3차 시간 스케일링 θ0 → θ0+Δ → θ0 (MR 식 (9.11)~(9.13))
  Δ = 실측 때 왕복 폭, θ̇_max = 3Δ/(2T) (MR 330쪽) = 실측 최고 속도 → T = 3Δ/(2 v_max)   (마찰 모델을 잰 범위 안)
정지 구간  시작 전·끝 뒤 2 % 정착 시간 t_s = 4/(ζω_n) (MR 412쪽), ω_n = √(K_p/M) → 끝 시점 오차 = 정상상태 오차
           (DART 마찰은 정지 마찰보다 작은 힘에서도 아주 천천히 미끄러져(creep) 속도가 정확히 0이 안 됨 → 멈춤 판정 대신 t_s 사용)
지표 (plot_ff_compare.py): 이동 중 RMS 오차, 끝 정지 구간 마지막 오차 vs δθ_j

사용 (깨끗한 셸 + scripts/environment.bash, impedance_node 꺼진 상태):
  python3 ff_compare.py --robot robot1 --side r --joint 4 --out ../../real_logs/1117_fric/sim
  마찰 0 시뮬(robots.yaml joint_friction: {}): --cases none --tag nofric --frictionless (K_p 같음, K_d는 플랜트 b = 0으로)
"""
import argparse
import csv
import glob
import math
import os

import numpy as np
import pinocchio as pin
import yaml
import rclpy
from actuator_msgs.msg import Actuators
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

COMMANDED = [f'arm_r_joint{i}' for i in range(1, 8)] + ['gripper_r_joint1'] + \
            [f'arm_l_joint{i}' for i in range(1, 8)] + ['gripper_l_joint1']
HERE = os.path.dirname(os.path.abspath(__file__))
REAL = os.path.join(HERE, '../../real_logs/1117_fric')

ap = argparse.ArgumentParser()
ap.add_argument('--robot', default='robot1')
ap.add_argument('--side', default='r')
ap.add_argument('--joint', type=int, required=True)
ap.add_argument('--cases', nargs='+', default=['none', 'coulomb', 'viscous', 'both'])
ap.add_argument('--dx', type=float, default=0.005, help='손끝 허용 오차 [m] (게인 문서)')
ap.add_argument('--tag', default='fric')
ap.add_argument('--stiff', choices=['design', 'kmin'], default='design')
ap.add_argument('--frictionless', action='store_true', help='마찰 0 시뮬: 플랜트 b = 0 → K_d = 2√(K_p M) (11.26)')
ap.add_argument('--out', default='logs')
a = ap.parse_args()
JN = f'arm_{a.side}_joint{a.joint}'
FR = {int(r['joint']): r for r in csv.DictReader(open(os.path.join(REAL, f'friction_{a.side}.csv')))}[a.joint]
B_S, B_V = float(FR['b_static']), float(FR['b_viscous'])
# 실측 왕복 폭 Δ, 최고 속도 (이 관절을 잰 파일들에서)
DELTA, VMAX = 0.0, 0.0
for p in glob.glob(os.path.join(REAL, f'{a.side}_visc*_j{a.joint}.csv')):
    R = [r for r in csv.DictReader(open(p)) if r['phase'] in ('ramp', 'cruise')]
    q = np.array([float(r[f'q{a.joint}']) for r in R])
    DELTA = max(DELTA, q.max() - q.min()); VMAX = max(VMAX, max(float(r['target']) for r in R))
T = 3 * DELTA / (2 * VMAX)


class Run(Node):
    def __init__(self):
        super().__init__('ff_compare')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.model = None
        self.create_subscription(String, f'/{a.robot}/robot_description', self.on_urdf, qos)
        self.create_subscription(JointState, f'/sim/{a.robot}/joint_states', self.on_state, qos_profile_sensor_data)
        self.pub = self.create_publisher(Actuators, f'/sim/{a.robot}/effort', 1)
        self.rows, self.t0 = [], None

    def on_urdf(self, msg):
        if self.model is None:
            self.model = pin.buildModelFromXML(msg.data)
            self.data = self.model.createData()

    def setup(self, q, qj):
        m, d = self.model, self.data
        jt = m.joints[m.getJointId(JN)]
        pin.computeJointJacobians(m, d, q); pin.framesForwardKinematics(m, d, q)
        Jv = pin.getFrameJacobian(m, d, m.getFrameId(f'end_effector_{a.side}_link'), pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, jt.idx_v]
        Jw = pin.getFrameJacobian(m, d, m.getFrameId(f'end_effector_{a.side}_link'), pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[3:, jt.idx_v]
        self.dth = a.dx / np.linalg.norm(Jv)                   # δθ_j = δx/‖J_v,j‖
        self.M = pin.crba(m, d, q)[jt.idx_v, jt.idx_v]
        if a.stiff == 'kmin':
            self.kp = B_S / self.dth                           # K_p = b_static/δθ_j
        else:
            G = yaml.safe_load(open(os.path.join(HERE, '../gains.yaml')))
            self.kp = float(Jv @ np.diag(G['K_pos']) @ Jv + Jw @ np.diag(G['K_rot']) @ Jw)   # J_jᵀ K J_j
        b_plant = 0.0 if a.frictionless else B_V
        self.kd = max(0.0, 2 * math.sqrt(self.kp * self.M) - b_plant)   # ζ = 1 (11.26), b = 플랜트 점성
        self.wn = math.sqrt(self.kp / self.M)
        z = (b_plant + self.kd) / (2 * math.sqrt(self.kp * self.M))   # 실제 ζ (11.26) — K_d가 0으로 잘리면 ζ > 1
        s_slow = self.wn * (z - math.sqrt(z * z - 1)) if z > 1 else z * self.wn
        self.ts = 4.0 / s_slow                                 # 2 % 정착 시간 ≈ 4t, 과감쇠면 느린 근 기준 (MR 412쪽)
        lo, hi = m.lowerPositionLimit[jt.idx_q], m.upperPositionLimit[jt.idx_q]
        self.sgn = 1.0 if qj + DELTA <= hi else -1.0
        self.q0, self.tcase = qj, 2 * self.ts + 2 * T
        self.get_logger().info(f'[{a.stiff}] {JN} θ0 {qj:+.3f}, b_static {B_S:.3f}, b_viscous {B_V:.3f} | δθ {self.dth * 1e3:.2f} mrad, '
                               f'M {self.M:.4f}, K_p {self.kp:.1f}, K_d {self.kd:.2f}, ω_n {self.wn:.1f}, t_s {self.ts:.3f} s | '
                               f'Δ {DELTA:.3f} rad, v_max {VMAX:g}, T {T:.3f} s')

    def ref(self, tt):
        """3차 시간 스케일링 θ0 → θ0+sΔ (0~T) → θ0 (T~2T)  MR (9.11)~(9.13)"""
        if tt < 0 or tt > 2 * T:
            return self.q0, 0.0, 0.0
        s0, d = (self.q0, self.sgn * DELTA) if tt <= T else (self.q0 + self.sgn * DELTA, -self.sgn * DELTA)
        u = tt if tt <= T else tt - T
        return (s0 + (3 * u ** 2 / T ** 2 - 2 * u ** 3 / T ** 3) * d, (6 * u / T ** 2 - 6 * u ** 2 / T ** 3) * d,
                (6 / T ** 2 - 12 * u / T ** 3) * d)

    def on_state(self, msg):
        if self.model is None:
            return
        m, d = self.model, self.data
        q, v = pin.neutral(m), np.zeros(m.nv)
        for n, name in enumerate(msg.name):
            name = name.split('::')[-1]
            if m.existJointName(name):
                jt = m.joints[m.getJointId(name)]
                if jt.nq == 1:
                    q[jt.idx_q] = msg.position[n]
                    if len(msg.velocity) > n:
                        v[jt.idx_v] = msg.velocity[n]
        jt = m.joints[m.getJointId(JN)]
        qj, vj = q[jt.idx_q], v[jt.idx_v]
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = ts
            self.setup(q, qj)
        t = ts - self.t0
        k = int(t // self.tcase)
        if k >= len(a.cases):
            raise SystemExit
        case, tt = a.cases[k], t - k * self.tcase - self.ts
        qd, vd, ad = self.ref(tt)
        seg = 'pre' if tt < 0 else ('traj' if tt <= 2 * T else 'hold')
        qq = q.copy(); qq[jt.idx_q] = qd
        Md = pin.crba(m, d, qq)[jt.idx_v, jt.idx_v]
        g = pin.computeGeneralizedGravity(m, d, qq)[jt.idx_v]
        f_ff = (B_S * np.sign(vd) if case in ('coulomb', 'both') else 0.0) + (B_V * vd if case in ('viscous', 'both') else 0.0)
        tau = Md * ad + g + f_ff + self.kp * (qd - qj) + self.kd * (vd - vj)
        out = [math.nan] * len(COMMANDED)
        out[COMMANDED.index(JN)] = tau
        cmd = Actuators()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.normalized = out
        cmd.position = [math.nan] * len(COMMANDED)
        self.pub.publish(cmd)
        self.rows.append([f'{t:.4f}', case, seg, f'{qd:.6f}', f'{qj:.6f}', f'{vd:.5f}', f'{vj:.5f}', f'{tau:.4f}', f'{f_ff:.4f}',
                          f'{self.kp:.3f}', f'{self.dth:.6f}'])


def main():
    rclpy.init()
    n = Run()
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    n.pub.publish(Actuators(normalized=[math.nan] * len(COMMANDED), position=[math.nan] * len(COMMANDED)))
    os.makedirs(a.out, exist_ok=True)
    fn = os.path.join(a.out, f'ff_{a.tag}_{a.stiff}_{a.robot}_{JN}.csv')
    with open(fn, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'case', 'seg', 'qd', 'q', 'vd', 'v', 'tau', 'f_ff', 'kp', 'dth'])
        w.writerows(n.rows)
    print(f'저장 {fn} ({len(n.rows)}행)')
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
