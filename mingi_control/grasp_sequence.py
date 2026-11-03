#!/usr/bin/env python3
"""두 로봇 4팔 파지 → 임피던스 전환.

1 그리퍼 열기 → 2 접근 자세(그리퍼 90° 돌려 박스 쪽, 손잡이 앞) → 3 전진(손잡이가 손가락 사이로)
→ 4 닫기 + 접촉 감지(손가락이 목표 전에 멈추면 접촉) → 5 접촉 위치 + squeeze로 고정 → 6 토크(임피던스) 모드 전환
1~3은 현대 QP(MoveL) 위치 제어. 전제: 로봇마다 impedance_node가 mode:=position으로 실행 중.

사용: python3 grasp_sequence.py [--robots robot1 robot2]
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robotis_interfaces.msg import MoveL
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

ap = argparse.ArgumentParser()
ap.add_argument('--robots', nargs='+', default=['robot1', 'robot2'])
ap.add_argument('--y', type=float, default=0.12, help='손잡이 y (base_link)')
ap.add_argument('--z', type=float, default=0.85, help='손잡이 높이 (base_link)')
ap.add_argument('--x_pre', type=float, default=0.18, help='접근 자세 손끝 x')
ap.add_argument('--x_grasp', type=float, default=0.29, help='파지 손끝 x (박스 끝면 0.30 바로 앞)')
ap.add_argument('--squeeze', type=float, default=0.05, help='접촉 위치에서 더 닫는 양 [rad]')
a = ap.parse_args()
QUAT = (-0.5, -0.5, 0.5, 0.5)       # 손끝이 +x(박스 쪽), 손가락은 상하로 닫힘 — 두 팔 모두 IK 도달 확인
OPEN, CLOSED = 0.0, 1.1

rclpy.init()
n = Node('grasp_sequence')
state = {}
pub_goal, pub_grip, pub_mode = {}, {}, {}
for r in a.robots:
    n.create_subscription(JointState, f'/sim/{r}/joint_states',
                          lambda m, r=r: state.__setitem__(r, dict(zip(m.name, m.position))), qos_profile_sensor_data)
    pub_mode[r] = n.create_publisher(String, f'/{r}/impedance/mode', 10)
    for s in 'rl':
        pub_goal[r, s] = n.create_publisher(MoveL, f'/{r}/{s}_goal_move', 10)
        pub_grip[r, s] = n.create_publisher(Float64, f'/{r}/impedance/{s}/gripper', 10)


def spin(sec):
    t = time.time()
    while time.time() - t < sec:
        rclpy.spin_once(n, timeout_sec=0.01)


def move(x, T):
    for (r, s), p in pub_goal.items():
        g = MoveL()
        g.pose.header.frame_id = 'base_link'
        g.pose.pose.position.x, g.pose.pose.position.y, g.pose.pose.position.z = x, (-a.y if s == 'r' else a.y), a.z
        g.pose.pose.orientation.x, g.pose.pose.orientation.y, g.pose.pose.orientation.z, g.pose.pose.orientation.w = QUAT
        g.time_from_start.sec = int(T)
        p.publish(g)
    spin(T + 1.5)


def grip(values):
    for k, v in values.items():
        pub_grip[k].publish(Float64(data=v))


def say(msg):
    print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)


spin(1.5)                                                   # 탐색 대기
for r in a.robots:
    pub_mode[r].publish(String(data='position'))
say('1 그리퍼 열기'); grip({k: OPEN for k in pub_grip}); spin(1.5)
say(f'2 접근 자세: 손끝 x={a.x_pre}, y=±{a.y}, z={a.z}'); move(a.x_pre, 4)
say(f'3 전진: 손끝 x={a.x_grasp}'); move(a.x_grasp, 3)
say('4 닫기 + 접촉 감지'); grip({k: CLOSED for k in pub_grip})
hist = {k: [] for k in pub_grip}
contact = {}
t0 = time.time()
while len(contact) < len(pub_grip) and time.time() - t0 < 5.0:
    spin(0.05)
    for (r, s) in pub_grip:
        q = state.get(r, {}).get(f'gripper_{s}_joint1')
        if q is None or (r, s) in contact:
            continue
        hist[r, s].append((time.time(), q))
        old = [v for t, v in hist[r, s] if time.time() - t > 0.3]
        if old and abs(q - old[-1]) < 0.003 and time.time() - t0 > 0.5:   # 0.3 s 동안 거의 안 움직임 = 멈춤
            contact[r, s] = q
            say(f'  {r}/{s}: 멈춤 q={q:.3f} rad → ' + ('접촉(손잡이 잡음)' if q < CLOSED - 0.1 else '끝까지 닫힘 = 놓침'))
miss = [k for k, q in contact.items() if q >= CLOSED - 0.1] + [k for k in pub_grip if k not in contact]
if miss:
    say(f'파지 실패 {miss} — 위치를 확인하고 다시 (--x_grasp, --z). 토크 전환 안 함')
else:
    grip({k: q + a.squeeze for k, q in contact.items()})       # 접촉 위치 + 조금 더 → 쥐는 힘 제한
    spin(1.0)
    say('5 고정 완료 → 6 토크(임피던스) 모드 전환')
    for r in a.robots:
        pub_mode[r].publish(String(data='task'))
    spin(1.0)
    say('완료: 임피던스로 현재 자세 유지 중')
n.destroy_node()
rclpy.shutdown()
