#!/usr/bin/env python3
"""박스 들고 베이스 이동 → 옆 받침대에 내려놓기. 전제: 파지 완료, 두 노드 mode task, robots.yaml base_motion: true.

1 리더·팔로워 강성 [문서 #7]   2 무게 피드포워드 [문서 #9]   3 들기 +lift
4 베이스 이동(월드 y로 dy, 5차 궤적 MR 9.2) — 팔로워 베이스는 리더보다 lag초 늦게 따라감
  → 두 베이스 사이 위치 오차가 생기고, 팔로워의 부드러운 xy 강성이 흡수 (내력 ≈ K_F·오차, 문서 #7)
5 내려놓기 −lift → 피드포워드 끄기 → 그리퍼 열기 → 위치 제어 복귀
베이스 이동은 현대 주행 파트의 대역: Gazebo 물리 모델의 베이스 슬라이드 관절 목표를 /sim/<robot>/base로 보냄

사용: python3 transport_scenario.py [--dy 2.0] [--T 20] [--lag 0.5]
"""
import argparse
import math
import os
import re
import subprocess
import threading
import time

import rclpy
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float64, String

ap = argparse.ArgumentParser()
ap.add_argument('--leader', default='robot1')
ap.add_argument('--follower', default='robot2')
ap.add_argument('--yaw', type=float, nargs=2, default=[0.0, math.pi], help='robots.yaml의 leader, follower yaw')
ap.add_argument('--dy', type=float, default=2.0, help='월드 y 이동 [m] (반대 방향이면 음수)')
ap.add_argument('--T', type=float, default=30.0, help='베이스 이동 시간 [s]')
ap.add_argument('--lag', type=float, default=0.2, help='팔로워 베이스 지연 [s]')
ap.add_argument('--lift', type=float, default=0.05)
ap.add_argument('--box_mass', type=float, default=2.0)
ap.add_argument('--K_leader', type=float, nargs=3, default=[800.0, 800.0, 1000.0])
ap.add_argument('--K_follower', type=float, nargs=3, default=[300.0, 300.0, 1000.0])
ap.add_argument('--move', type=float, default=4.0)
a = ap.parse_args()
robots = [a.leader, a.follower]
yaw = dict(zip(robots, a.yaw))

rclpy.init()
n = Node('transport_scenario')
pub = {(r, s, k): n.create_publisher(Vector3 if k in ('offset', 'stiffness') else Float64, f'/{r}/impedance/{s}/{k}', 10)
       for r in robots for s in 'rl' for k in ('offset', 'stiffness', 'weight', 'gripper')}
base_pub = {r: n.create_publisher(Actuators, f'/sim/{r}/base', 10) for r in robots}
mode_pub = {r: n.create_publisher(String, f'/{r}/impedance/mode', 10) for r in robots}
time.sleep(1.5)
log_dir = os.path.expanduser('~/impedance_logs')
stamp = time.strftime('%m%d_%H%M%S')
box_log, ev_log = f'{log_dir}/{stamp}_carry_box.csv', f'{log_dir}/{stamp}_carry_events.csv'
events, stop = [], threading.Event()


def box_monitor():
    """[기록용] Gazebo 박스 위치"""
    env = dict(os.environ, GZ_PARTITION=os.environ.get('GZ_PARTITION', 'internship_dual'))
    with open(box_log, 'w') as f:
        f.write('t,x,y,z\n')
        while not stop.is_set():
            out = subprocess.run(['gz', 'model', '-m', 'box', '-p'], capture_output=True, text=True, env=env).stdout
            m = re.search(r'Pose \[ XYZ \(m\) \].*?\n\s*\[([-\d.e ]+)\]', out)
            if m:
                f.write(f'{time.time():.3f},' + ','.join(m.group(1).split()) + '\n')
                f.flush()
            time.sleep(0.2)


def put(r, k, value):
    for s in 'rl':
        msg = Vector3(x=value[0], y=value[1], z=value[2]) if k in ('offset', 'stiffness') else Float64(data=value)
        pub[r, s, k].publish(msg)


def step(name, wait):
    events.append((name, time.time()))
    print(f'[{time.strftime("%H:%M:%S")}] {name}', flush=True)
    time.sleep(wait)


def base_to(y_world, r):
    """월드 y → 그 로봇 슬라이드 관절 좌표(관절 축 = 로봇 자신의 y축, yaw π면 부호 반대)"""
    base_pub[r].publish(Actuators(position=[math.cos(yaw[r]) * y_world]))


def quintic(tau):
    tau = min(max(tau, 0.0), 1.0)
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5


th = threading.Thread(target=box_monitor, daemon=True)
th.start()
for r in robots:
    base_to(0.0, r)
put(a.leader, 'stiffness', a.K_leader)
put(a.follower, 'stiffness', a.K_follower)
step(f'1 강성: 리더 {a.K_leader}, 팔로워 {a.K_follower}', 2.0)
for r in robots:
    put(r, 'weight', a.box_mass / 2)
step(f'2 무게 피드포워드 로봇당 {a.box_mass/2} kg', 2.0)
for r in robots:
    put(r, 'offset', (0.0, 0.0, a.lift))
step(f'3 들기 +{a.lift*100:.0f} cm', a.move + 2.0)
v_max = 1.875 * abs(a.dy) / a.T
print(f'   예측: 팔로워 지연 {a.lag} s × 최대 속도 {v_max:.2f} m/s ≈ 두 베이스 오차 {v_max*a.lag*100:.1f} cm '
      f'→ 팔로워 손당 내력 ≈ K_F·오차 = {a.K_follower[1]*v_max*a.lag:.0f} N (팔로워가 다 흡수한다고 볼 때 상한)')
events.append(('4 베이스 이동 시작', time.time()))
print(f'[{time.strftime("%H:%M:%S")}] 4 베이스 이동 월드 y {a.dy:+.1f} m, {a.T:.0f} s', flush=True)
t0 = time.time()
while True:
    t = time.time() - t0
    base_to(a.dy * quintic(t / a.T), a.leader)
    base_to(a.dy * quintic((t - a.lag) / a.T), a.follower)
    if t > a.T + a.lag + 0.5:
        break
    time.sleep(0.02)
step('   베이스 도착', 3.0)
for r in robots:
    put(r, 'offset', (0.0, 0.0, -a.lift))
step('5 내려놓기', a.move + 2.0)
for r in robots:
    put(r, 'weight', 0.0)
step('   피드포워드 끄기', 2.0)
for r in robots:
    put(r, 'gripper', 0.0)
step('   그리퍼 열기', 2.0)
for r in robots:
    mode_pub[r].publish(String(data='position'))
step('6 위치 제어로 복귀', 1.0)
stop.set()
th.join()
open(ev_log, 'w').write('name,t\n' + ''.join(f'{k},{t:.3f}\n' for k, t in events))
print(f'기록: {box_log}\n분석: python3 analyze.py carry {box_log}')
n.destroy_node()
rclpy.shutdown()
