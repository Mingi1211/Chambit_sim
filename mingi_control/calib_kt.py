#!/usr/bin/env python3
"""실물 토크 상수 보정 — 정지 자세에서 측정 전류 I와 모델 중력 토크 g(q)로 Kt_eff = g/I (관절별 최소제곱).

정지 상태면 모터가 내는 토크 = 중력 토크(마찰 무시) → τ = g(q) = Kt_eff · I   (MR 8.3, 11.8)
위치 제어 모드 그대로, 팔을 여러 자세(5개 이상, 관절마다 중력 토크가 크게 걸리게)에 세우고 Enter로 기록.
I는 /<robot>/joint_states 의 effort(= Present Current raw). raw_per_A로 A로 바꾼다.

사용: python3 calib_kt.py --robot robot1 --side r --raw_per_A <값>
"""
import argparse

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ap = argparse.ArgumentParser()
ap.add_argument('--robot', default='robot1')
ap.add_argument('--side', default='r')
ap.add_argument('--raw_per_A', type=float, required=True)
ap.add_argument('--min_torque', type=float, default=0.5, help='이보다 작은 g는 제외 [N·m] (잡음)')
a = ap.parse_args()
rclpy.init()
n = Node('calib_kt')
box = {}
n.create_subscription(JointState, f'/{a.robot}/joint_states', lambda m: box.__setitem__('js', m), 10)
n.create_subscription(String, f'/{a.robot}/robot_description', lambda m: box.__setitem__('urdf', m.data),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
while 'js' not in box or 'urdf' not in box:
    rclpy.spin_once(n, timeout_sec=0.1)
m = pin.buildModelFromXML(box['urdf'])
d = m.createData()
names = [f'arm_{a.side}_joint{i}' for i in range(1, 8)]
vi = [m.joints[m.getJointId(k)].idx_v for k in names]
G, I = [], []
while True:
    if input('자세 고정 후 Enter (끝: q) ') == 'q':
        break
    samples = []
    for _ in range(50):                                  # 0.5 s 평균
        rclpy.spin_once(n, timeout_sec=0.01)
        samples.append(box['js'])
    q = pin.neutral(m)
    cur = np.zeros(7)
    for js in samples:
        idx = dict(zip(js.name, range(len(js.name))))
        for k, name in enumerate(names):
            cur[k] += js.effort[idx[name]] / len(samples)
    for nm, i in idx.items():
        if m.existJointName(nm) and m.joints[m.getJointId(nm)].nq == 1:
            q[m.joints[m.getJointId(nm)].idx_q] = samples[-1].position[i]
    g = pin.computeGeneralizedGravity(m, d, q)[vi]
    G.append(g); I.append(cur / a.raw_per_A)
    print('  g[N·m]', np.round(g, 2), ' I[A]', np.round(cur / a.raw_per_A, 3))
G, I = np.array(G), np.array(I)
print('\n관절  Kt_eff[N·m/A]  (사용 자세 수)')
for k in range(7):
    use = np.abs(G[:, k]) > a.min_torque
    kt = (G[use, k] @ I[use, k]) / (I[use, k] @ I[use, k]) if use.sum() else float('nan')   # τ = Kt·I 최소제곱
    print(f'j{k+1}  {kt:8.3f}  ({use.sum()})')
