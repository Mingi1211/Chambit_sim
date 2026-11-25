#!/usr/bin/env python3
"""벽시계 ↔ 시뮬 시간 기록 (풀 녹화 영상을 시뮬 시간 기준 실속도로 바꾸는 데 씀). 김민기 11/25
사용: clock_log.py <출력 csv>"""
import sys
import time

import rclpy
from rosgraph_msgs.msg import Clock

rclpy.init()
n = rclpy.create_node('clock_log')
f = open(sys.argv[1], 'w'); f.write('t_wall,t_sim\n')
last = [0.0]


def on_clock(m):
    w = time.time()
    if w - last[0] < 0.2:
        return
    last[0] = w
    f.write(f'{w:.3f},{m.clock.sec + m.clock.nanosec * 1e-9:.3f}\n'); f.flush()


n.create_subscription(Clock, '/clock', on_clock, 10)
try:
    rclpy.spin(n)
except (KeyboardInterrupt, Exception):
    pass
f.close()
