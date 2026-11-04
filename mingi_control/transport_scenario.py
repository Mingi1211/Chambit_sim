#!/usr/bin/env python3
"""박스 들고 베이스 이동 → 옆 받침대에 내려놓기. 전제: 파지 완료, 두 노드 mode task, robots.yaml base_motion: true.

1 리더·팔로워 강성 [문서 #7]   2 무게 피드포워드 [문서 #9]   3 들기 +lift
4 베이스 이동(월드 y로 dy, 5차 궤적 MR 9.2) — 팔로워 베이스는 리더보다 lag초 늦게 따라감
  → 두 베이스 사이 위치 오차가 생기고, 팔로워의 부드러운 xy 강성이 흡수 (내력 ≈ K_F·오차, 문서 #7)
5 내려놓기 −lift → 피드포워드 끄기 → 그리퍼 열기
6 팔 빼기(손잡이에서 −x로 빼고 조금 올림) — 기본 위치 제어(QP), --retract_mode impedance면 토크 모드로 뺀 뒤 위치 복귀
8 판정: 박스가 받침대 위에 있고, 팔 뺄 때 끌려오지 않았는지(Gazebo 박스 위치, 기록용)
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

import numpy as np
import rclpy
import yaml
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.time import Time
from robotis_interfaces.msg import MoveL
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformListener

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
ap.add_argument('--retract', type=float, default=0.12, help='놓은 뒤 손을 박스에서 빼는 거리 [m] (각 base_link −x)')
ap.add_argument('--retract_up', type=float, default=0.03, help='빼면서 함께 올리는 높이 [m] (손잡이에 손가락이 걸리지 않게)')
ap.add_argument('--retract_time', type=float, default=3.0, help='팔 빼기 이동 시간 [s]')
ap.add_argument('--retract_mode', choices=['qp', 'impedance'], default='qp',
                help='qp: 위치 제어로 복귀 후 현대 QP(MoveL)로 빼기 / impedance: 토크 모드 유지한 채 목표 이동으로 빼고 나서 위치 복귀')
a = ap.parse_args()
robots = [a.leader, a.follower]
yaw = dict(zip(robots, a.yaw))

rclpy.init()
n = Node('transport_scenario')
pub = {(r, s, k): n.create_publisher(Vector3 if k in ('offset', 'stiffness') else Float64, f'/{r}/impedance/{s}/{k}', 10)
       for r in robots for s in 'rl' for k in ('offset', 'stiffness', 'weight', 'gripper')}
base_pub = {r: n.create_publisher(Actuators, f'/sim/{r}/base', 10) for r in robots}
mode_pub = {r: n.create_publisher(String, f'/{r}/impedance/mode', 10) for r in robots}
goal_pub = {(r, s): n.create_publisher(MoveL, f'/{r}/{s}_goal_move', 10) for r in robots for s in 'rl'}   # 현대 QP 입력
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, n)
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


def spin(sec):
    """[문서 밖] TF를 받으려면 노드를 돌려야 함 (time.sleep으로는 콜백이 안 돎)"""
    t_end = time.time() + sec
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.02)


def ee(r, s):
    """현대 robot_state_publisher TF: 각 로봇 base_link 기준 손끝 위치·방향"""
    tr = tf_buffer.lookup_transform(f'{r}/base_link', f'{r}/end_effector_{s}_link', Time()).transform
    return (np.array([tr.translation.x, tr.translation.y, tr.translation.z]),
            (tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w))


# 6 팔 빼기 — 손잡이에서 손가락을 빼고(각 base_link −x = 박스 반대쪽) 조금 올림.
# 박스는 받침대 위에 남아야 함. 놓은 뒤 토크 모드를 오래 두면 손목이 떨리므로(11/4) 기본은 위치 제어(QP)로 뺀다.
if a.retract_mode == 'impedance':
    for r in robots:
        put(r, 'offset', (-a.retract, 0.0, a.retract_up))
    step(f'6 팔 빼기(임피던스) −x {a.retract*100:.0f} cm, +z {a.retract_up*100:.0f} cm', a.move + 1.0)
    for r in robots:
        mode_pub[r].publish(String(data='position'))
    step('7 위치 제어로 복귀', 1.0)
else:
    for r in robots:
        mode_pub[r].publish(String(data='position'))
    step('6 위치 제어로 복귀', 1.0)
    spin(1.0)
    try:
        start = {(r, s): ee(r, s) for r in robots for s in 'rl'}
    except Exception as exc:                                   # TF가 아직 안 들어온 경우
        spin(2.0)
        start = {(r, s): ee(r, s) for r in robots for s in 'rl'}
        print(f'   (TF 재시도: {exc})')
    goal = {k: (p + np.array([-a.retract, 0.0, a.retract_up]), q) for k, (p, q) in start.items()}
    for (r, s), (p, q) in goal.items():
        g = MoveL()
        g.pose.header.frame_id = 'base_link'
        g.pose.pose.position.x, g.pose.pose.position.y, g.pose.pose.position.z = map(float, p)
        g.pose.pose.orientation.x, g.pose.pose.orientation.y, g.pose.pose.orientation.z, g.pose.pose.orientation.w = q
        g.time_from_start.sec = int(a.retract_time)
        g.time_from_start.nanosec = int((a.retract_time % 1) * 1e9)
        goal_pub[r, s].publish(g)
    events.append(('7 팔 빼기(QP)', time.time()))
    print(f'[{time.strftime("%H:%M:%S")}] 7 팔 빼기(QP) −x {a.retract*100:.0f} cm, +z {a.retract_up*100:.0f} cm', flush=True)
    spin(a.retract_time + 2.0)
    err = {k: float(np.linalg.norm(goal[k][0] - ee(*k)[0])) for k in goal}
    print('   팔 빼기 위치 오차 [mm]: ' + ' '.join(f'{r[-1]}{s}:{e*1000:.0f}' for (r, s), e in err.items()), flush=True)
    if max(err.values()) > 0.02:
        print('   ⚠ 2 cm 넘게 남음 — 손가락이 손잡이에 걸렸거나 QP 목표가 안 들어감(트러블슈팅: README 9절)', flush=True)
step('   정리', 1.0)
stop.set()
th.join()
open(ev_log, 'w').write('name,t\n' + ''.join(f'{k},{t:.3f}\n' for k, t in events))

# 8 결과 판정 [기록용 — Gazebo 박스 위치, 실물에서는 영상·마커로 대체]
rows = [list(map(float, l.split(','))) for l in open(box_log).read().splitlines()[1:] if l.count(',') == 3]
if len(rows) >= 2:
    x0, y0, z0 = rows[0][1:]
    xf, yf, zf = rows[-1][1:]
    t_rel = next((t for k, t in events if k.startswith('   그리퍼 열기')), None)
    at_rel = min(rows, key=lambda r: abs(r[0] - t_rel))[1:] if t_rel else rows[-1][1:]
    moved = float(np.linalg.norm(np.array([xf, yf, zf]) - np.array(at_rel)))
    cfg = yaml.safe_load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src/coop_carry_bringup/config/robots.yaml')))
    tw = cfg.get('box', {}).get('table_size', [0.30, 0.40])   # 받침대 x, y 크기
    on_stand = abs(xf - x0) < tw[0] / 2 and abs((yf - y0) - a.dy) < tw[1] / 2 and abs(zf - z0) < 0.02
    print(f'박스 시작 ({x0:.3f}, {y0:.3f}, {z0:.3f}) → 최종 ({xf:.3f}, {yf:.3f}, {zf:.3f}) m, '
          f'놓은 뒤 이동 {moved*1000:.0f} mm')
    print('판정:', '받침대 위에 놓고 팔 뺌 — 성공' if on_stand and moved < 0.01 else
          '확인 필요 (받침대 밖이거나, 팔 뺄 때 박스가 끌려옴)')
print(f'기록: {box_log}\n분석: python3 analyze.py carry {box_log}')
n.destroy_node()
rclpy.shutdown()
