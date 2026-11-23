#!/usr/bin/env python3
"""곡선 운반 1회 기록 노드. 김민기 (11/23, 11/24 바퀴 관절 추가).
기록: 두 베이스 Gazebo 자세 (/sim/robotN/base_odometry), 바퀴 조향·구동 관절 (/sim/robotN/joint_states),
      손 렌치 원신호 (/robotN/hand_wrench_{r,l}, F/T 시뮬 시각, 전부),
      베이스 노드 상태 (/carry/base_move_status, 상태 바뀔 때), 손 렌치·내력 (/coop/hand_state).
      ※ /robotN/cmd_vel은 구독하지 않음 — 현대 베이스 노드가 구독자 수를 검사해 FAULT를 냄.
옵션: MOVING ~ DONE 구간 창 녹화 (ffmpeg x11grab).
사용: curve_rec.py <출력 접두어> [창 id|-] [crop 필터]
"""
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

out = sys.argv[1]
win = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] != '-' else ''
crop = sys.argv[3] if len(sys.argv) > 3 else ''
WJ = [f'{s}_wheel_{k}' for k in ('steer', 'drive') for s in ('left', 'right', 'rear')]

rclpy.init()
n = rclpy.create_node('curve_trial_rec')
fb = open(out + '_base.csv', 'w'); wb = csv.writer(fb)
wb.writerow(['t_wall', 't_sim', 'robot', 'x', 'y', 'yaw', 'state'])
fw = open(out + '_wheels.csv', 'w'); ww = csv.writer(fw)
ww.writerow(['t_wall', 't_sim', 'robot', 'state'] + [f'q_{j}' for j in WJ] + [f'v_{j}' for j in WJ])
fh = open(out + '_hand.csv', 'w'); wh = csv.writer(fh)
fs = open(out + '_status.jsonl', 'w')
fr = open(out + '_wrench.csv', 'w'); wr = csv.writer(fr)
wr.writerow(['t_sim', 'hand', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz', 'state'])
st = {'s': None, 'last': {}, 'tsim': 0.0}
rec = {'p': None}


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def throttled(key, t):
    if t - st['last'].get(key, -1.0) < 0.02:          # 50 Hz
        return True
    st['last'][key] = t
    return False


def on_odom(robot, m):
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    st['tsim'] = max(st['tsim'], t)
    if throttled(robot, t):
        return
    p = m.pose.pose
    wb.writerow([f'{time.time():.3f}', f'{t:.3f}', robot, f'{p.position.x:.5f}', f'{p.position.y:.5f}',
                 f'{yaw(p.orientation):.5f}', st['s']])


def on_wheels(robot, m):
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if throttled(robot + '_w', t):
        return
    vel = m.velocity if len(m.velocity) == len(m.name) else [math.nan] * len(m.name)
    d = {k.split('::')[-1]: (q, v) for k, q, v in zip(m.name, m.position, vel)}
    if not all(j in d for j in WJ):
        return
    ww.writerow([f'{time.time():.3f}', f'{t:.3f}', robot, st['s']] + [f'{d[j][0]:.4f}' for j in WJ] + [f'{d[j][1]:.4f}' for j in WJ])


def on_wrench(hand, m):
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9          # F/T 메시지 시뮬 시각 (전부 기록, 스펙트럼용)
    f, q = m.wrench.force, m.wrench.torque
    wr.writerow([f'{t:.4f}', hand, f'{f.x:.3f}', f'{f.y:.3f}', f'{f.z:.3f}', f'{q.x:.4f}', f'{q.y:.4f}', f'{q.z:.4f}', st['s']])


def on_hand(m):
    wh.writerow([f'{time.time():.3f}', f'{st["tsim"]:.3f}', st['s']] + [f'{v:.3f}' for v in m.data])


def stop_rec():
    if rec['p'] is not None and rec['p'].poll() is None:
        rec['p'].stdin.write(b'q'); rec['p'].stdin.flush(); rec['p'].wait(20)


def on_status(m):
    d = json.loads(m.data)
    s = d.get('state')
    if s == st['s']:
        return
    d['t_wall'] = time.time(); d['t_sim_rec'] = st['tsim']
    fs.write(json.dumps(d, ensure_ascii=False) + '\n'); fs.flush()
    n.get_logger().info(f'state {st["s"]} -> {s}')
    if win and s == 'MOVING' and rec['p'] is None:
        vf = (crop + ',' if crop else '') + 'crop=trunc(iw/2)*2:trunc(ih/2)*2'
        rec['p'] = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'x11grab', '-window_id', win, '-framerate', '10',
                                     '-i', os.environ.get('DISPLAY', ':0'), '-vf', vf, '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                                     out + '_raw.mp4'], stdin=subprocess.PIPE)
    if rec['p'] is not None and s in ('DONE', 'FAULT', 'WAIT_LIFT'):
        n.create_timer(3.0, stop_rec)
    st['s'] = s


for r in ('robot1', 'robot2'):
    n.create_subscription(Odometry, f'/sim/{r}/base_odometry', lambda m, r=r: on_odom(r, m), qos_profile_sensor_data)
    n.create_subscription(JointState, f'/sim/{r}/joint_states', lambda m, r=r: on_wheels(r, m), qos_profile_sensor_data)
n.create_subscription(String, '/carry/base_move_status', on_status,
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
n.create_subscription(Float64MultiArray, '/coop/hand_state', on_hand, 50)
for r in ('robot1', 'robot2'):
    for sd in ('r', 'l'):
        n.create_subscription(WrenchStamped, f'/{r}/hand_wrench_{sd}', lambda m, h=f'{r}_{sd}': on_wrench(h, m), 200)
signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt))
try:
    rclpy.spin(n)
except (KeyboardInterrupt, Exception):
    pass
stop_rec()
for f in (fb, fw, fh, fs, fr):
    f.close()
