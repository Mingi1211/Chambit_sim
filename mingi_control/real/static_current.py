#!/usr/bin/env python3
"""실물 위치 모드 정지 전류 측정 — Kt·정지마찰 τ_s 실측용 (gain_design ④, motor_params.yaml). 김민기.

위치 모드 그대로(전류 모드 패치 불필요). 정지·등속일 때 Present Current ≈ (중력 + 마찰)/Kt 이므로
  같은 자세에 위쪽(q+δ)·아래쪽(q−δ)에서 접근해 멈춘 뒤 전류를 평균 →
  평균(두 방향)  ↔ 중력 토크 g(q)   → Kt  (PC에서 Pinocchio로 g 계산, fit_static.py)
  차이/2         ↔ 정지마찰 τ_s     → 문서 3·4쪽 K_min ≥ J̄ᵀτ_s/δx
한 관절만 움직이고 나머지는 home(0) 고정. 팔 하나씩.

모드
  noise : 지금 자세 그대로 duration초 기록 (움직이지 않음) — 제어 주기·전류 잡음
  sweep : --joint 관절을 --poses 각마다 위·아래 접근 → 각 hold초 기록

출력: /workspace/mingi/<시각>_<mode>_<arm>.csv (+ robot_description.urdf 한 번 저장)
  열: t, phase, target, dir, 관절별 q·dq·I (I = /dynamic_joint_states 'Present Current', 단위 미확인 — 그대로 저장)

사용 (로봇 컨테이너 안, bringup 켜진 상태):
  python3 static_current.py noise --arm l --duration 30
  python3 static_current.py sweep --arm l --joint 4 --poses 0 -0.4 -0.8 -1.2 --delta 0.1
"""
import argparse
import csv
import os
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import DynamicJointState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

ap = argparse.ArgumentParser()
ap.add_argument('mode', choices=['noise', 'sweep'])
ap.add_argument('--arm', choices=['r', 'l'], default='l')
ap.add_argument('--duration', type=float, default=30.0, help='noise 기록 시간 [s]')
ap.add_argument('--joint', type=int, default=4, help='sweep 관절 번호 1~7')
ap.add_argument('--poses', type=float, nargs='+', default=[0.0, -0.4, -0.8, -1.2], help='sweep 목표각 [rad]')
ap.add_argument('--delta', type=float, default=0.1, help='위·아래 접근 거리 [rad]')
ap.add_argument('--move', type=float, default=4.0, help='각 이동 시간 [s] (천천히)')
ap.add_argument('--hold', type=float, default=4.0, help='정지 후 기록 시간 [s] (앞 1 s는 안정화로 표시)')
ap.add_argument('--out', default='/workspace/mingi')
a = ap.parse_args()

JOINTS = [f'arm_{a.arm}_joint{i}' for i in range(1, 8)]
GRIP = f'gripper_{a.arm}_joint1'
DXL = [f'dxl{i}' for i in (range(1, 8) if a.arm == 'r' else range(31, 38))]   # 팔 관절 i ↔ DXL ID (bringup 설정)

rclpy.init()
n = Node('mingi_static_current')
state = {'q': {}, 'dq': {}, 'I': {}}
rows = []
tag = {'phase': 'idle', 'target': '', 'dir': ''}


def on_js(m):
    for name, p, v in zip(m.name, m.position, m.velocity if len(m.velocity) == len(m.name) else [float('nan')] * len(m.name)):
        state['q'][name], state['dq'][name] = p, v
    if all(j in state['q'] for j in JOINTS) and all(d in state['I'] for d in DXL):
        rows.append([f'{time.time():.4f}', tag['phase'], tag['target'], tag['dir']]
                    + [f"{state['q'][j]:.5f}" for j in JOINTS] + [f"{state['dq'][j]:.5f}" for j in JOINTS]
                    + [f"{state['I'][d]:.4f}" for d in DXL])


def on_dyn(m):
    # 관절 이름 + 인터페이스 이름으로 찾음 (관절 순서·인터페이스 순서가 제각각 — 11/8 확인)
    for name, iv in zip(m.joint_names, m.interface_values):
        if name in DXL and 'Present Current' in iv.interface_names:
            state['I'][name] = iv.values[iv.interface_names.index('Present Current')]


n.create_subscription(JointState, '/joint_states', on_js, 50)
n.create_subscription(DynamicJointState, '/dynamic_joint_states', on_dyn, 50)
urdf = {}
n.create_subscription(String, '/robot_description', lambda m: urdf.update(x=m.data),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))


def spin(sec):
    t = time.time() + sec
    while time.time() < t:
        rclpy.spin_once(n, timeout_sec=0.01)


def go(q_arm, T):
    """관절 공간 이동 (arm 제어기 JTC). 그리퍼는 현재값 유지"""
    cli = go.cli
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS + [GRIP]
    pt = JointTrajectoryPoint()
    pt.positions = [float(v) for v in q_arm] + [float(state['q'].get(GRIP, 0.0))]
    pt.time_from_start.sec, pt.time_from_start.nanosec = int(T), int((T % 1) * 1e9)
    goal.trajectory.points = [pt]
    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
    h = fut.result()
    if h is None or not h.accepted:
        raise SystemExit('궤적 목표 거부 — 다른 노드(MoveL 등)가 팔을 쓰는 중인지 확인')
    res = h.get_result_async()
    rclpy.spin_until_future_complete(n, res, timeout_sec=T + 10.0)


spin(2.0)
missing = [d for d in DXL if d not in state['I']] + [j for j in JOINTS if j not in state['q']]
if missing:
    raise SystemExit(f'상태 없음: {missing} — bringup 켜졌는지, 토크 ON인지 확인')
os.makedirs(a.out, exist_ok=True)
stamp = time.strftime('%m%d_%H%M%S')
if urdf.get('x'):
    open(os.path.join(a.out, 'robot_description.urdf'), 'w').write(urdf['x'])

if a.mode == 'noise':
    tag['phase'] = 'noise'
    print(f'정지 기록 {a.duration} s — 팔을 건드리지 마세요', flush=True)
    spin(a.duration)
else:
    go.cli = ActionClient(n, FollowJointTrajectory, f'/arm_{a.arm}_controller/follow_joint_trajectory')
    if not go.cli.wait_for_server(timeout_sec=5.0):
        raise SystemExit('arm 제어기 action 없음')
    k = a.joint - 1
    home = [0.0] * 7
    print(f'{a.arm}팔 관절{a.joint} sweep {a.poses} (δ {a.delta} rad, 이동 {a.move} s, 정지 {a.hold} s). 나머지 관절 0', flush=True)
    tag['phase'] = 'move'
    go(home, a.move + 1.0)
    for target in a.poses:
        for d, label in ((+a.delta, 'from_above'), (-a.delta, 'from_below')):
            q = list(home)
            q[k] = target + d
            tag.update(phase='move', target=f'{target}', dir=label)
            go(q, a.move)                       # 지나쳐 간 뒤
            q[k] = target
            go(q, a.move)                       # 목표로 되돌아옴 → 마지막 운동 방향이 접근 방향
            tag['phase'] = 'settle'
            spin(1.0)
            tag['phase'] = 'hold'
            spin(a.hold - 1.0)
            print(f'  목표 {target:+.2f} rad {label} 기록', flush=True)
    tag.update(phase='move', target='', dir='')
    go(home, a.move + 1.0)

path = os.path.join(a.out, f'{stamp}_{a.mode}_{a.arm}' + (f'_j{a.joint}' if a.mode == 'sweep' else '') + '.csv')
with open(path, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t', 'phase', 'target', 'dir'] + [f'q{i}' for i in range(1, 8)] + [f'dq{i}' for i in range(1, 8)] + [f'I{i}' for i in range(1, 8)])
    w.writerows(rows)
print(f'저장: {path}  ({len(rows)}행)')
n.destroy_node()
rclpy.shutdown()
