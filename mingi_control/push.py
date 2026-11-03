#!/usr/bin/env python3
"""T4 외력 시험 — 손목 링크에 알려진 힘 F를 hold초 가했다 뗀다. 김민기.

  [문서 #4] Λë + Dė + Ke ≈ F_ext → 정상상태(ë=ė=0)에서 e_ss = F/K (축별)   (MR 11.7.1)
  외력은 실물의 "무게추 걸기"에 해당하는 시험 입력이다(센서 아님) — [문서 밖] Gazebo ApplyLinkWrench로 인가
  주의: 힘은 arm_<s>_link7 원점에 걸린다(EE보다 0.215 m 위). 그래서 손 회전 오차가 조금 생기고, 위치 e_ss는 근사.

사용: python3 push.py --robot robot1 --side r --F 0 10 0 --hold 4
"""
import argparse
import os
import subprocess
import time

ap = argparse.ArgumentParser()
ap.add_argument('--robot', default='robot1')
ap.add_argument('--side', default='r')
ap.add_argument('--F', type=float, nargs=3, required=True)
ap.add_argument('--hold', type=float, default=4.0)
a = ap.parse_args()
env = dict(os.environ, GZ_PARTITION=os.environ.get('GZ_PARTITION', 'internship_dual'))
ent = f'name: "{a.robot}::arm_{a.side}_link7", type: LINK'
fx, fy, fz = a.F
t_on = time.time()
subprocess.run(['gz', 'topic', '-t', '/world/empty/wrench/persistent', '-m', 'gz.msgs.EntityWrench', '-p',
                f'entity: {{{ent}}}, wrench: {{force: {{x: {fx}, y: {fy}, z: {fz}}}}}'], check=True, env=env)
time.sleep(a.hold)
t_off = time.time()
subprocess.run(['gz', 'topic', '-t', '/world/empty/wrench/clear', '-m', 'gz.msgs.Entity', '-p', ent], check=True, env=env)
print(f'외력 {a.F} N: 켬 {t_on:.2f}, 끔 {t_off:.2f}')
print(f'분석: python3 analyze.py t4 <로그.csv> --side {a.side} --t_on {t_on:.2f} --t_off {t_off:.2f} --F {fx} {fy} {fz}')
