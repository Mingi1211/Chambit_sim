#!/usr/bin/env python3
"""시뮬 관절 마찰 확인 — Gazebo(DART)에 넣은 <friction>·<damping>이 그대로 재현되는가. 김민기 (11/16).

실물 11/10 측정과 같은 방법을 시뮬에서: 관절 하나만 등속 왕복, 나머지 관절은 위치 서보로 고정.
시뮬은 명령 토크를 정확히 알기 때문에 K_t 없이 바로
  MR (11.48) 등속(q̈≈0)·저속:  τ_cmd − g(q) = τ_f = f_c·sgn(q̇) + b·q̇      (MR 8.9.4 그림 8.13(c))
  → 등속 구간 데이터로 [sgn(q̇), q̇] 최소제곱 → f_c, b 추정 → robots.yaml joint_friction 값과 비교 (통과: ±10 % △ 내 기준)
등속 추종은 관절 PD + 중력 보상 (MR 11.4.2 식 (11.38), 게인 K_p = M_jj ω², K_d = 2 M_jj ω — MR (11.26) ζ=1) — 이 PD는 측정용 입력일 뿐.

사용 (Gazebo 실행 중, impedance_node는 끈 상태 — 둘 다 /sim/<robot>/effort 에 명령을 보내면 충돌):
  python3 sim_friction_check.py --robot robot1 --side r --joint 4 --amp 0.2 --speeds 0.05 0.1 0.2 0.4
출력: 속도별·전체 f_c, b 추정치. CSV는 --out 폴더(기본 logs/)
"""
import argparse
import csv
import math
import os
import time

import numpy as np
import pinocchio as pin
import rclpy
from actuator_msgs.msg import Actuators
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

COMMANDED = [f'arm_r_joint{i}' for i in range(1, 8)] + ['gripper_r_joint1'] + \
            [f'arm_l_joint{i}' for i in range(1, 8)] + ['gripper_l_joint1']

ap = argparse.ArgumentParser()
ap.add_argument('--robot', default='robot1')
ap.add_argument('--side', default='r')
ap.add_argument('--joint', type=int, required=True)
ap.add_argument('--amp', type=float, default=0.2, help='왕복 반폭 [rad] (현재 각도 기준)')
ap.add_argument('--speeds', type=float, nargs='+', default=[0.05, 0.1, 0.2, 0.4], help='등속 [rad/s]')
ap.add_argument('--cycles', type=int, default=2)
ap.add_argument('--omega', type=float, default=20.0, help='추종 PD 고유진동수 [rad/s]')
ap.add_argument('--out', default='logs')
a = ap.parse_args()
JN = f'arm_{a.side}_joint{a.joint}'


class Check(Node):
    def __init__(self):
        super().__init__('sim_friction_check')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.model = None
        self.create_subscription(String, f'/{a.robot}/robot_description', self.on_urdf, qos)
        self.create_subscription(JointState, f'/sim/{a.robot}/joint_states', self.on_state, qos_profile_sensor_data)
        self.pub = self.create_publisher(Actuators, f'/sim/{a.robot}/effort', 1)
        self.rows, self.q0, self.t0, self.plan = [], None, None, None

    def on_urdf(self, msg):
        if self.model is None:
            self.model = pin.buildModelFromXML(msg.data)
            self.data = self.model.createData()

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
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0, self.q0 = stamp, qj
            self.plan = make_plan(self.q0)
            lo, hi = m.lowerPositionLimit[jt.idx_q], m.upperPositionLimit[jt.idx_q]
            if not (lo + 0.05 <= qj - a.amp and qj + a.amp <= hi - 0.05):
                self.get_logger().error(f'{JN} q0 {qj:+.3f} ± {a.amp} 가 한계 [{lo:+.2f}, {hi:+.2f}] 밖 — --amp 줄이거나 자세 변경')
                raise SystemExit
            self.get_logger().info(f'{JN} 시작 q0 = {qj:+.3f} rad, 계획 {self.plan[-1][0]:.0f} s')
        t = stamp - self.t0
        g = pin.computeGeneralizedGravity(m, d, q)[jt.idx_v]
        M = pin.crba(m, d, q)
        Mjj = M[jt.idx_v, jt.idx_v]
        qr, vr, seg = reference(self.plan, t)
        kp, kd = Mjj * a.omega ** 2, 2 * Mjj * a.omega
        tau = g + kp * (qr - qj) + kd * (vr - vj) if seg != 'done' else math.nan
        out = [math.nan] * len(COMMANDED)
        out[COMMANDED.index(JN)] = tau
        cmd = Actuators()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.normalized = out
        cmd.position = [math.nan] * len(COMMANDED)
        self.pub.publish(cmd)
        if seg != 'done':
            self.rows.append([t, seg, vr, qj, vj, tau, g, Mjj])
        else:
            raise SystemExit


def make_plan(q0):
    """(끝 시각, 구간 이름, 속도, 시작각): 속도마다 cycles 회 +/− 등속 왕복 (q0 ± amp)"""
    plan, t, q = [], 0.0, q0
    plan.append((2.0, 'settle', 0.0, q))                       # 시작: 2 s 정지
    t = 2.0
    for s in a.speeds:
        T = a.amp / s
        for _ in range(a.cycles):
            for sign, k in ((+1, 1), (-1, 2), (+1, 1)):        # q0 → +amp → −amp → q0
                dur = T * k
                plan.append((t + dur, f'v{s:+}' if sign > 0 else f'v{-s:+}', sign * s, q))
                q += sign * s * dur
                t += dur
        plan.append((t + 1.0, 'settle', 0.0, q))
        t += 1.0
    plan.append((t, 'done', 0.0, q))
    return plan


def reference(plan, t):
    t_prev = 0.0
    for t_end, seg, vel, q_start in plan:
        if t <= t_end:
            return q_start + vel * (t - t_prev), vel, seg
        t_prev = t_end
    return plan[-1][3], 0.0, 'done'


def analyse(rows):
    """real/fit_friction.py와 같은 규칙 (MR 식·개념만)
    τ_fric = τ_cmd − g − M_jj θ̈ (MR 8장 운동방정식, 시뮬은 명령 토크 = 실제 토크)
    샘플: 지령 속도가 일정한 구간 + 관절이 지령 방향으로 움직이는 점 (θ̇ = 0이면 마찰 미정, MR 그림 8.13(b))
    F(v) = 평균[sgn(θ̇)·τ_fric] (양방향 평균) → 직선 F = b_static + b_viscous·v (그림 8.13(c))"""
    r = np.array([[x[0], x[2], x[3], x[4], x[5], x[6], x[7]] for x in rows])
    t, vr, q, v, tau, g, Mjj = r.T
    y = tau - g - Mjj * np.gradient(v, t)
    keep = (vr != 0) & (np.sign(v) == np.sign(vr))
    V = np.array(sorted(set(np.abs(vr[vr != 0]))))
    F = np.array([np.mean(np.sign(v[keep & (np.abs(vr) == s)]) * y[keep & (np.abs(vr) == s)]) for s in V])
    b_s, b_v = np.linalg.lstsq(np.c_[np.ones(len(V)), V], F, rcond=None)[0]
    print(f'\n{JN}: 속도별 F(v) ' + ' '.join(f'{s:g}:{f:.3f}' for s, f in zip(V, F)))
    print(f'  직선 맞춤 → b_static ≈ {b_s:.3f} N·m, b_viscous ≈ {b_v:.3f} N·m·s/rad   (robots.yaml에 넣은 값과 비교)')


def main():
    rclpy.init()
    node = Check()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    # 끝: 모든 관절 NaN → 위치 hold (플러그인)
    node.pub.publish(Actuators(normalized=[math.nan] * len(COMMANDED), position=[math.nan] * len(COMMANDED)))
    os.makedirs(a.out, exist_ok=True)
    fn = os.path.join(a.out, time.strftime('%m%d_%H%M%S') + f'_fric_{a.robot}_{JN}.csv')
    with open(fn, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'seg', 'v_ref', 'q', 'dq', 'tau_cmd', 'g', 'M_jj'])
        w.writerows(node.rows)
    print(f'저장 {fn} ({len(node.rows)}행)')
    if node.rows:
        analyse(node.rows)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
