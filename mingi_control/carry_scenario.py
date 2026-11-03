#!/usr/bin/env python3
"""파지 이후 운반 시나리오 — 두 로봇 동시. 전제: 파지 완료, 두 노드 mode task.

1 무게 피드포워드 [문서 #9]  τ += J_vᵀ[0,0,m g/2]ᵀ, 로봇당 m = 박스/2
2 들기 +lift (5차 궤적, MR 9.2)   3 유지
4 리더–팔로워 [문서 #7]: 리더(robot1)만 목표를 옆(월드 y)으로 shift. 팔로워 K_xy 부드럽게 → 단단하게 두 번
  예측(직렬 스프링, 마찰 무시): 박스 이동 = shift·K_L/(K_L+K_F),  손당 내력 ≈ K_F·(박스 이동)
5 리더 복귀 → 내리기 → 피드포워드 끄기 → 그리퍼 열기
평가: 노드 로그의 F_cmd(내력), Gazebo 박스 위치는 기록용

사용: python3 carry_scenario.py [--lift 0.05] [--shift 0.05] [--hold 10] [--box_mass 2.0]
"""
import argparse
import os
import re
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float64

ap = argparse.ArgumentParser()
ap.add_argument('--leader', default='robot1')
ap.add_argument('--follower', default='robot2')
ap.add_argument('--lift', type=float, default=0.05)
ap.add_argument('--shift', type=float, default=0.05, help='리더 목표 옆 이동 [m] (월드 y)')
ap.add_argument('--hold', type=float, default=10.0)
ap.add_argument('--box_mass', type=float, default=2.0)
ap.add_argument('--K_leader', type=float, nargs=3, default=[800.0, 800.0, 1000.0])
ap.add_argument('--K_soft', type=float, nargs=3, default=[300.0, 300.0, 1000.0])
ap.add_argument('--K_stiff', type=float, nargs=3, default=[800.0, 800.0, 1000.0])
ap.add_argument('--move', type=float, default=2.0, help='노드 move_time과 같게 [s]')
a = ap.parse_args()
robots = [a.leader, a.follower]

rclpy.init()
n = Node('carry_scenario')
pub = {(r, s, k): n.create_publisher(Vector3 if k in ('offset', 'stiffness') else Float64, f'/{r}/impedance/{s}/{k}', 10)
       for r in robots for s in 'rl' for k in ('offset', 'stiffness', 'weight', 'gripper')}
time.sleep(1.5)
log_dir = os.path.expanduser('~/impedance_logs')
stamp = time.strftime('%m%d_%H%M%S')
box_log, ev_log = f'{log_dir}/{stamp}_carry_box.csv', f'{log_dir}/{stamp}_carry_events.csv'
events, stop = [], threading.Event()


def box_monitor():
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


def put(r, k, value, sides='rl'):
    for s in sides:
        msg = Vector3(x=value[0], y=value[1], z=value[2]) if k in ('offset', 'stiffness') else Float64(data=value)
        pub[r, s, k].publish(msg)


def step(name, wait):
    events.append((name, time.time()))
    print(f'[{time.strftime("%H:%M:%S")}] {name}', flush=True)
    time.sleep(wait)


def lf_phase(K_F, label):
    """리더 목표만 옆으로 → 팔로워 강성 K_F에서 따라오는지·내력"""
    put(a.follower, 'stiffness', K_F)
    KL, KF = a.K_leader[1], K_F[1]
    follow = a.shift * KL / (KL + KF)
    print(f'   예측({label}, K_L={KL:.0f}, K_F={KF:.0f}): 박스 옆 이동 {follow*100:.1f} cm, 손당 내력 ≈ {KF*follow:.1f} N')
    put(a.leader, 'offset', (0.0, a.shift, 0.0))          # 리더는 월드 y = 로컬 y (yaw 0)
    step(f'4 리더 옆 이동 {a.shift*100:.0f} cm — 팔로워 {label}', a.move + 3.0)
    put(a.leader, 'offset', (0.0, -a.shift, 0.0))
    step('  리더 복귀', a.move + 2.0)


th = threading.Thread(target=box_monitor, daemon=True)
th.start()
put(a.leader, 'stiffness', a.K_leader)
put(a.follower, 'stiffness', a.K_soft)
step('0 시작 (리더·팔로워 강성 설정)', 2.0)
for r in robots:
    put(r, 'weight', a.box_mass / 2)
step(f'1 무게 피드포워드 로봇당 {a.box_mass/2} kg', 2.0)
for r in robots:
    put(r, 'offset', (0.0, 0.0, a.lift))
step(f'2 들기 +{a.lift*100:.0f} cm', a.move + 1.0)
step(f'3 유지 {a.hold:.0f} s', a.hold)
lf_phase(a.K_soft, '부드러움')
lf_phase(a.K_stiff, '단단함')
put(a.follower, 'stiffness', a.K_soft)
for r in robots:
    put(r, 'offset', (0.0, 0.0, -a.lift))
step('5 내리기', a.move + 1.0)
for r in robots:
    put(r, 'weight', 0.0)
step('  피드포워드 끄기', 2.0)
for r in robots:
    put(r, 'gripper', 0.0)
step('  그리퍼 열기', 2.0)
stop.set()
th.join()
open(ev_log, 'w').write('name,t\n' + ''.join(f'{k},{t:.3f}\n' for k, t in events))
print(f'기록: {box_log}\n분석: python3 analyze.py carry {box_log}')
n.destroy_node()
rclpy.shutdown()
