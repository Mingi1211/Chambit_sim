#!/usr/bin/env python3
"""T5 계단 입력 — 목표를 한 축으로 d [m] 순간 이동(move_time을 잠시 0.01 s로). 김민기.

  [문서 #5·#6] ζ=1이면 오버슈트 ≈ 0, T_s ≈ 5.8/ωn, ωn = √(K/Λ)   (MR 11.2.2)

사용: python3 step.py --robot robot1 --side r --axis y --d 0.03
"""
import argparse
import subprocess
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node

ap = argparse.ArgumentParser()
ap.add_argument('--robot', default='robot1')
ap.add_argument('--side', default='r')
ap.add_argument('--axis', default='y', choices=list('xyz'))
ap.add_argument('--d', type=float, default=0.03)
a = ap.parse_args()
node_name = f'/{a.robot}/impedance_controller'
old = subprocess.run(['ros2', 'param', 'get', node_name, 'move_time'], capture_output=True, text=True).stdout
old = old.split(':')[-1].strip() or '2.0'
subprocess.run(['ros2', 'param', 'set', node_name, 'move_time', '0.01'], check=True)   # [문서 밖] 계단에 가깝게
rclpy.init()
n = Node('step_input', namespace=a.robot)
pub = n.create_publisher(Vector3, f'impedance/{a.side}/offset', 10)
time.sleep(1.0)
v = Vector3()
setattr(v, a.axis, a.d)
t_step = time.time()
pub.publish(v)
time.sleep(4.0)
subprocess.run(['ros2', 'param', 'set', node_name, 'move_time', old], check=True)
print(f'계단 {a.axis} {a.d*1000:+.0f} mm at {t_step:.2f}')
print(f'분석: python3 analyze.py t5 <로그.csv> --side {a.side} --t_step {t_step:.2f} --axis {a.axis}')
n.destroy_node()
rclpy.shutdown()
