#!/usr/bin/env python3
"""박스 들고 베이스 이동 → 옆 받침대에 내려놓기. 전제: 파지 완료, 두 노드 mode task, robots.yaml base_mode: swerve,
현대 베이스 노드 실행 중(ros2 launch coop_carry_bringup base_move.launch.py).

1 리더·팔로워 강성 [문서 #7]   2 무게 피드포워드 [문서 #9]   3 들기 +lift
4 베이스 이동 [문서 밖, 현대 파트]: /carry/lift_done=True → /carry/start_base_move 서비스 → /carry/base_move_done=True
  (현대 base_move_node가 두 로봇 cmd_vel로 스워브 주행, 방향·거리·시간은 현대 base_move.yaml — 월드 y +2 m, 5차 속도 MR 9.2)
  → 두 베이스가 따로 주행하며 생기는 위치 오차를 팔로워의 부드러운 xy 강성이 흡수 (내력 ≈ K_F·오차, 문서 #7)
5 내려놓기: −(lift − 1 cm) → 마지막 1 cm → 피드포워드 끄기(무게를 받침대로 천천히 넘김) → 그리퍼 열기
6 팔 빼기(손잡이에서 −x로 빼고 조금 올림) — 기본 위치 제어(QP), --retract_mode impedance면 토크 모드로 뺀 뒤 위치 복귀
8 판정: 박스가 받침대 위에 있고, 팔 뺄 때 끌려오지 않았는지(Gazebo 박스 위치, 기록용)

사용: python3 transport_scenario.py [--lift 0.05] [--retract_mode qp|impedance]
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
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.time import Time
from robotis_interfaces.msg import MoveL
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

ap = argparse.ArgumentParser()
ap.add_argument('--leader', default='robot1')
ap.add_argument('--follower', default='robot2')
ap.add_argument('--dy', type=float, default=2.0, help='판정용: robots.yaml box.place.dy (받침대 위치) [m]')
ap.add_argument('--base_timeout', type=float, default=300.0, help='베이스 이동 완료 대기 [s, wall]')
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

rclpy.init()
n = Node('transport_scenario')
pub = {(r, s, k): n.create_publisher(Vector3 if k in ('offset', 'stiffness') else Float64, f'/{r}/impedance/{s}/{k}', 10)
       for r in robots for s in 'rl' for k in ('offset', 'stiffness', 'weight', 'gripper')}
# [문서 밖] 현대 base_move_node 인터페이스 (이름·QoS 그대로)
STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
lift_pub = n.create_publisher(Bool, '/carry/lift_done', STATE_QOS)
base_done = {'v': False, 'status': ''}
n.create_subscription(Bool, '/carry/base_move_done', lambda m: base_done.update(v=m.data), STATE_QOS)
n.create_subscription(String, '/carry/base_move_status', lambda m: base_done.update(status=m.data), STATE_QOS)
start_cli = n.create_client(Trigger, '/carry/start_base_move')
reset_cli = n.create_client(Trigger, '/carry/reset_base_move')
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
        f.write('t,x,y,z,roll,pitch,yaw\n')
        while not stop.is_set():
            out = subprocess.run(['gz', 'model', '-m', 'box', '-p'], capture_output=True, text=True, env=env).stdout
            m = re.search(r'Pose \[ XYZ \(m\) \].*?\n\s*\[([-\d.e ]+)\].*?\n\s*\[([-\d.e ]+)\]', out, re.S)
            if m:
                f.write(f'{time.time():.3f},' + ','.join(m.group(1).split() + m.group(2).split()) + '\n')
                f.flush()
            time.sleep(0.2)


def put(r, k, value):
    for s in 'rl':
        msg = Vector3(x=value[0], y=value[1], z=value[2]) if k in ('offset', 'stiffness') else Float64(data=value)
        pub[r, s, k].publish(msg)


def spin(sec):
    """[문서 밖] 콜백(TF·베이스 상태)을 받으려면 노드를 돌려야 함 (time.sleep으로는 콜백이 안 돎)"""
    t_end = time.time() + sec
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.02)


def step(name, wait):
    events.append((name, time.time()))
    print(f'[{time.strftime("%H:%M:%S")}] {name}', flush=True)
    spin(wait)


def call(cli, label):
    if not cli.wait_for_service(timeout_sec=5.0):
        raise SystemExit(f'{label} 서비스 없음 — 현대 베이스 노드 실행: ros2 launch coop_carry_bringup base_move.launch.py')
    fut = cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
    res = fut.result()
    print(f'   {label}: {res.success} {res.message}', flush=True)
    return res.success


th = threading.Thread(target=box_monitor, daemon=True)
th.start()
put(a.leader, 'stiffness', a.K_leader)
put(a.follower, 'stiffness', a.K_follower)
step(f'1 강성: 리더 {a.K_leader}, 팔로워 {a.K_follower}', 2.0)
for r in robots:
    put(r, 'weight', a.box_mass / 2)
step(f'2 무게 피드포워드 로봇당 {a.box_mass/2} kg', 2.0)
for r in robots:
    put(r, 'offset', (0.0, 0.0, a.lift))
step(f'3 들기 +{a.lift*100:.0f} cm', a.move + 2.0)
# 4 베이스 이동 — 현대 base_move_node에 맡김 (한 번 쓴 lift 허가는 재사용 안 됨: False → reset → True)
lift_pub.publish(Bool(data=False))
spin(0.5)
call(reset_cli, 'reset_base_move')
lift_pub.publish(Bool(data=True))
spin(1.0)
events.append(('4 베이스 이동 시작', time.time()))
print(f'[{time.strftime("%H:%M:%S")}] 4 베이스 이동 (현대 스워브, base_move.yaml)', flush=True)
if not call(start_cli, 'start_base_move'):
    raise SystemExit('베이스 시작 거부 — 위 메시지 확인 (두 베이스 정지·피드백 ready 필요). 박스는 들린 채 유지 중')
t0 = time.time()
base_done['v'] = False
while not base_done['v']:
    spin(0.5)
    if '"FAULT"' in base_done['status'] or time.time() - t0 > a.base_timeout:
        print('   베이스 이동 실패/시간 초과: ' + base_done['status'][:300], flush=True)
        break
lift_pub.publish(Bool(data=False))
step('   베이스 도착', 3.0)
# 5 내려놓기 — 받침대 1 cm 위까지 내린 뒤 마지막 1 cm만 따로: 닿는 순간 손 목표가 거의 접촉 높이라
#   K_z·오차로 누르는 힘이 작음(문서 #4 e = F/K) → 한쪽 모서리만 눌려 박스가 기우는 것을 막음
for r in robots:
    put(r, 'offset', (0.0, 0.0, -(a.lift - 0.01)))
step(f'5 내려놓기 −{(a.lift-0.01)*100:.0f} cm (받침대 1 cm 위)', a.move + 1.0)
for r in robots:
    put(r, 'offset', (0.0, 0.0, -0.01))
step('   마지막 1 cm', a.move + 1.0)
for r in robots:
    put(r, 'weight', 0.0)
step('   피드포워드 끄기 (무게를 받침대로)', 3.0)
for r in robots:
    put(r, 'gripper', 0.0)
step('   그리퍼 열기', 2.0)


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
    # QP 이동 시간은 시뮬 시간 — RTF < 1이면 벽시계로 더 걸림 → 목표 2 cm 안에 들어오거나 30 s까지 기다림
    t_end = time.time() + 30.0
    while True:
        spin(1.0)
        err = {k: float(np.linalg.norm(goal[k][0] - ee(*k)[0])) for k in goal}
        if max(err.values()) < 0.02 or time.time() > t_end:
            break
    print('   팔 빼기 위치 오차 [mm]: ' + ' '.join(f'{r[-1]}{s}:{e*1000:.0f}' for (r, s), e in err.items()), flush=True)
    if max(err.values()) > 0.02:
        print('   ⚠ 2 cm 넘게 남음 — 손가락이 손잡이에 걸렸거나 QP 목표가 안 들어감(트러블슈팅: README 9절)', flush=True)
step('   정리', 1.0)
stop.set()
th.join()
open(ev_log, 'w').write('name,t\n' + ''.join(f'{k},{t:.3f}\n' for k, t in events))

# 8 결과 판정 [기록용 — Gazebo 박스 위치, 실물에서는 영상·마커로 대체]
rows = [list(map(float, l.split(','))) for l in open(box_log).read().splitlines()[1:] if l.count(',') == 6]
if len(rows) >= 2:
    x0, y0, z0 = rows[0][1:4]
    xf, yf, zf, rf, pf, _ = rows[-1][1:]
    t_rel = next((t for k, t in events if '그리퍼 열기' in k), None)
    at_rel = min(rows, key=lambda r: abs(r[0] - t_rel))[1:4] if t_rel else rows[-1][1:4]
    moved = float(np.linalg.norm(np.array([xf, yf, zf]) - np.array(at_rel)))
    tilt = math.degrees(max(abs(rf), abs(pf)))
    tilt_max = math.degrees(max(max(abs(r[4]), abs(r[5])) for r in rows))
    cfg = yaml.safe_load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src/coop_carry_bringup/config/robots.yaml')))
    tw = cfg.get('box', {}).get('table_size', [0.30, 0.40])   # 받침대 x, y 크기
    on_stand = abs(xf - x0) < tw[0] / 2 and abs((yf - y0) - a.dy) < tw[1] / 2 and abs(zf - z0) < 0.02
    print(f'박스 시작 ({x0:.3f}, {y0:.3f}, {z0:.3f}) → 최종 ({xf:.3f}, {yf:.3f}, {zf:.3f}) m, '
          f'놓은 뒤 이동 {moved*1000:.0f} mm, 최종 기울기 {tilt:.1f}° (운반 중 최대 {tilt_max:.1f}°)')
    print('판정:', '받침대 위에 수평으로 놓고 팔 뺌 — 성공' if on_stand and moved < 0.01 and tilt < 2.0 else
          '확인 필요 (받침대 밖·기울어짐·팔 뺄 때 박스가 끌려옴 중 하나)')
print(f'기록: {box_log}\n분석: python3 analyze.py carry {box_log}')
n.destroy_node()
rclpy.shutdown()
