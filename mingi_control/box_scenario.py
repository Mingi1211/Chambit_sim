#!/usr/bin/env python3
"""S5 박스 잡기 시나리오 — 현대 QP로 박스 옆까지 온 뒤, 토크(임피던스)로 쥐고 → 들고 → 유지 → 내려놓기. 김민기.

전제: ① 현대 QP로 pregrasp.yaml 자세 도착 ② impedance_node(sides [r, l], mode task)가 켜져 현재 자세를 유지 중

단계와 근거:
  1 쥐기   양손 목표를 안쪽으로 (gap + δ) → 접촉 후 남는 오차 e_y만큼 쥐는 힘 F_sq = K_y e_y   [문서 #2·#7]
  2 무게   τ += J_vᵀ[0,0,m g/2]ᵀ 를 weight_ramp초 동안 램프 온                              [문서 #9]
  3 들기   목표 z +lift (5차 궤적)                                                          [MR 9.2]
  4 유지   hold초 — 박스 처짐·미끄러짐·쥐는 힘 기록
  5 내리기 z −lift → 무게 램프 오프 → 양손 바깥으로 (풀기)
평가(센서 없이): 쥐는 힘 = 제어기 값 F_cmd,y (노드 로그),  박스 자세는 Gazebo에서 [기록용]으로만 (box_log)

사용: python3 box_scenario.py --robot robot1 --gap 0.08 --delta 0.01 --lift 0.05 --hold 10 --mass 1.0
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
ap.add_argument('--robot', default='robot1')
ap.add_argument('--gap', type=float, default=0.08, help='pregrasp에서 손(EE)~박스 옆면 거리 [m]')
ap.add_argument('--delta', type=float, default=0.01, help='쥐는 오프셋 δ [m]')
ap.add_argument('--lift', type=float, default=0.05)
ap.add_argument('--hold', type=float, default=10.0)
ap.add_argument('--mass', type=float, default=1.0, help='피드포워드에 쓰는 박스 질량(사전 측정값) [kg]')
ap.add_argument('--move', type=float, default=2.0, help='각 이동 시간 [s] (노드 move_time과 같게)')
a = ap.parse_args()

rclpy.init()
n = Node('box_scenario', namespace=a.robot)
pub = {(s, k): n.create_publisher(Vector3 if k == 'offset' else Float64, f'impedance/{s}/{k}', 10)
       for s in 'rl' for k in ('offset', 'weight')}
time.sleep(1.0)                                     # [문서 밖] 탐색(discovery) 대기
log_dir = os.path.expanduser('~/impedance_logs')
os.makedirs(log_dir, exist_ok=True)
box_log = os.path.join(log_dir, time.strftime('%m%d_%H%M%S') + '_box.csv')
events = []
stop = threading.Event()


def box_monitor():
    """[기록용] Gazebo 박스 위치 — 제어에는 안 쓴다(실물은 영상·마커로 대체)"""
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


def offset(dy_r, dy_l, dz=0.0):
    pub['r', 'offset'].publish(Vector3(x=0.0, y=dy_r, z=dz))
    pub['l', 'offset'].publish(Vector3(x=0.0, y=dy_l, z=dz))


def weight(m):
    for s in 'rl':
        pub[s, 'weight'].publish(Float64(data=m))


def step(name, wait):
    events.append((name, time.time()))
    print(f'[{time.strftime("%H:%M:%S")}] {name}')
    time.sleep(wait)


th = threading.Thread(target=box_monitor, daemon=True)
th.start()
step('0 시작 (유지)', 2.0)
s = a.gap + a.delta
offset(+s, -s); step(f'1 쥐기: 양손 안쪽으로 {s*100:.0f} cm (gap {a.gap*100:.0f} + δ {a.delta*100:.0f})', a.move + 2.0)
weight(a.mass); step(f'2 무게 피드포워드 {a.mass} kg 램프 온', 2.0)
offset(0.0, 0.0, a.lift); step(f'3 들기 +{a.lift*100:.0f} cm', a.move + 1.0)
step(f'4 유지 {a.hold:.0f} s', a.hold)
offset(0.0, 0.0, -a.lift); step('5 내리기', a.move + 1.0)
weight(0.0); step('6 무게 피드포워드 램프 오프', 2.0)
offset(-s, +s); step('7 풀기', a.move + 1.0)
stop.set()
th.join()
with open(box_log.replace('_box.csv', '_events.csv'), 'w') as f:
    f.write('name,t\n' + ''.join(f'{k},{t:.3f}\n' for k, t in events))
print(f'박스 기록: {box_log}')
print(f'분석: python3 analyze.py box <노드 로그 task_{a.robot}.csv> {box_log}')
n.destroy_node()
rclpy.shutdown()
