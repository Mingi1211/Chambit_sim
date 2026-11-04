#!/usr/bin/env python3
"""베이스 간 거리 오차 Δ 측정 — 리더–팔로워 xy 강성 설계 입력 (gain_design.py). 김민기.

현대 스워브 주행(base_move_node)을 박스 없이 실행하고 두 로봇 base_link의 월드 위치(/sim/<robot>/base_odometry,
Gazebo 실측)를 기록한다. 두 베이스가 하나의 강체처럼 움직이면 상대 위치 r = p2 − p1이 일정해야 하므로
  Δ(t) = |r(t) − r(0)|  (xy)   ← 박스를 잡고 있으면 이 변위를 팔이 흡수해야 함, 내력 ≈ K·Δ (문서 3p)
그리고 명령 거리 대비 실제 이동 거리(주행 정확도)도 함께 출력한다.

전제: 시뮬 실행 중, 현대 베이스 노드 실행 중 (ros2 launch coop_carry_bringup base_move.launch.py), 박스를 잡지 않은 상태
사용: python3 base_error.py [--timeout 300]
"""
import argparse
import json
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

ap = argparse.ArgumentParser()
ap.add_argument('--timeout', type=float, default=300.0)
a = ap.parse_args()

rclpy.init()
n = Node('base_error')
STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
lift_pub = n.create_publisher(Bool, '/carry/lift_done', STATE_QOS)
st = {'done': False, 'status': ''}
n.create_subscription(Bool, '/carry/base_move_done', lambda m: st.update(done=m.data), STATE_QOS)
n.create_subscription(String, '/carry/base_move_status', lambda m: st.update(status=m.data), STATE_QOS)
pose = {}
rows = []


def on_odom(r):
    def cb(m):
        p, v = m.pose.pose.position, m.twist.twist.linear
        pose[r] = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, v.x, v.y)
        if len(pose) == 2:
            rows.append((pose['robot1'], pose['robot2']))
    return cb


steer = []   # (sim t, left/right/rear 조향각) robot1 — 조향 정렬이 끝나야 구동 시작(스워브 제어기 정렬 게이트)


def on_js(m):
    d = dict(zip(m.name, m.position))
    if 'left_wheel_steer' in d:
        steer.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                      d['left_wheel_steer'], d['right_wheel_steer'], d['rear_wheel_steer']))


n.create_subscription(JointState, '/sim/robot1/joint_states', on_js, qos_profile_sensor_data)
for r in ('robot1', 'robot2'):
    n.create_subscription(Odometry, f'/sim/{r}/base_odometry', on_odom(r), qos_profile_sensor_data)


def spin(sec):
    t = time.time() + sec
    while time.time() < t:
        rclpy.spin_once(n, timeout_sec=0.02)


def call(name):
    cli = n.create_client(Trigger, name)
    if not cli.wait_for_service(timeout_sec=5.0):
        raise SystemExit(f'{name} 없음 — 현대 베이스 노드 실행 필요')
    fut = cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
    print(f'{name}: {fut.result().success} {fut.result().message}', flush=True)
    return fut.result().success


spin(2.0)
lift_pub.publish(Bool(data=False)); spin(0.5)
call('/carry/reset_base_move')
lift_pub.publish(Bool(data=True)); spin(1.0)
rows.clear()
if not call('/carry/start_base_move'):
    raise SystemExit(1)
st['done'] = False
t0 = time.time()
while not st['done'] and time.time() - t0 < a.timeout and '"FAULT"' not in st['status']:
    spin(0.2)
spin(1.0)
lift_pub.publish(Bool(data=False)); spin(0.3)
if len(rows) < 2:
    raise SystemExit('odometry 기록 없음')
(t10, x10, y10, _, _), (t20, x20, y20, _, _) = rows[0]
r0 = (x20 - x10, y20 - y10)
d_max, k_max = 0.0, 0
for k, ((_, x1, y1, _, _), (_, x2, y2, _, _)) in enumerate(rows):
    d = math.hypot((x2 - x1) - r0[0], (y2 - y1) - r0[1])
    if d > d_max:
        d_max, k_max = d, k
(_, x1f, y1f, _, _), (_, x2f, y2f, _, _) = rows[-1]
d_end = math.hypot((x2f - x1f) - r0[0], (y2f - y1f) - r0[1])
cfg = json.loads(st['status']) if st['status'].startswith('{') else {}
print(f"상태: {cfg.get('state')} {cfg.get('reason', '')}")
print(f'robot1 이동 ({x1f - x10:+.3f}, {y1f - y10:+.3f}) m,  robot2 이동 ({x2f - x20:+.3f}, {y2f - y20:+.3f}) m')
print(f'베이스 간 거리 오차 Δ: 최대 {d_max * 1000:.1f} mm (주행 중), 도착 후 {d_end * 1000:.1f} mm   [샘플 {len(rows)}]')
out = os.path.expanduser(f'~/impedance_logs/{time.strftime("%m%d_%H%M%S")}_base_error.csv')
with open(out, 'w') as f:
    f.write('t1,x1,y1,vx1,vy1,t2,x2,y2,vx2,vy2\n')
    for p1, p2 in rows:
        f.write(','.join(f'{v:.5f}' for v in p1 + p2) + '\n')
print('기록:', out)
if steer:
    t_s0 = rows[0][0][0]
    moving = next((p1[0] for p1, _ in rows if math.hypot(p1[3], p1[4]) > 0.01), None)
    print(f'구동 시작(속도 > 1 cm/s): 시작 후 {moving - t_s0:.2f} s (시뮬 시간)' if moving else '구동 시작 없음')
    for t, l, rr, re in steer[::max(1, len(steer) // 25)]:
        if t - t_s0 > 8:
            break
        print(f'  t={t - t_s0:5.2f} s  조향 L/R/Rear = {math.degrees(l):7.2f} {math.degrees(rr):7.2f} {math.degrees(re):7.2f} deg')
n.destroy_node()
rclpy.shutdown()
