#!/usr/bin/env python3
"""팔로워 수평 강성 상한 — 손잡이 접촉 마찰 원뿔 (MR만). 김민기 (11/23, 11/24 MR 기준으로 재작성).

원리: 두 팔 간격은 박스 크기에 맞춰져 있고, 베이스 간 오차 Δ만큼 팔이 박스 안·밖으로 밀림. 팔로워가 따라가 주면 내력이 없고,
      버티면 그 힘이 내력이 됨. 팔로워 임피던스는 스프링 F = K e (MR (11.64)의 Kx 항) → 내력 ≈ F_s + K_F Δ
  ① 손끝 마찰력 F_fric = |J̄ᵀ| b_static (파지 자세, 관절 부호 최악; 11/17 k_lower_bound와 같은 식)
     → 보상 없이 팔로워가 움직이려면 손 힘이 이것을 넘어야 함
  ② 허용 내력 = 손잡이 접촉의 쿨롱 마찰 원뿔 (MR 12.2.1, 내력의 정의는 MR (12.26)(12.27)):
     손가락이 손잡이를 위아래로 쥠(닫힘축 ≈ 수직, 파지 FK) → 무게는 수직력에 더해지고 x·y 내력이 접선
     수평 한계 F_cone = μ (2N + m g/4),  N = 2 τ_grip / |dgap/dq| (가상일)
  ③ 팔로워 상한: 베이스 오차가 가장 클 때도 원뿔 안 — √((F_s,x + K_F Δx)² + (F_s,y + K_F Δy)²) ≤ F_cone  (K_F x = y)
     F_s = 마찰 보상 후 박스 들고 정지 중 남는 손 내력 (실측), Δ = 곡선 주행 실측 최대 (벌어짐·좁아짐 중 큰 값)
     변위(베이스 오차)가 주어지고 힘을 제한 → 상한. 팔로워의 베이스 오차 방향에는 위치를 지킬 요구가 없어 하한 없음
사용: python3 k_window.py --urdf ../../real_logs/robot_description.urdf --delta 0.0648 0.0016 --F_s 9.4 6.1
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
import gain_design as gd  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--urdf', required=True)
ap.add_argument('--delta', type=float, nargs=2, required=True, help='베이스 간 최대 오차 Δx Δy [m] (곡선 주행 실측)')
ap.add_argument('--F_s', type=float, nargs=2, required=True, help='보상 후 정지 중 남는 손 내력 x y [N] (실측)')
ap.add_argument('--tau_grip', type=float, default=None, help='그리퍼 토크 [N·m] (기본 gains.yaml gripper_effort)')
a = ap.parse_args()

G = yaml.safe_load(open(os.path.join(HERE, '../gains.yaml')))
CFG = yaml.safe_load(open(os.path.join(HERE, '../../src/coop_carry_bringup/config/robots.yaml')))
FR = CFG['joint_friction']
md = gd.model_at_grasp(a.urdf)

print('① 손끝 마찰력 F_fric = |J̄ᵀ| b_static (파지 자세, 관절 부호 최악) [N]   (base_link: x = 상대 로봇 방향, y = 주행 방향)')
F = {}
for s in 'rl':
    Jv, Mi = md[s]['J'][:3], np.linalg.inv(md[s]['M'])
    F[s] = np.abs(np.linalg.inv(Jv @ Mi @ Jv.T) @ Jv @ Mi) @ np.array(FR[s]['friction'])
    print(f'   {s}팔: x {F[s][0]:.1f}, y {F[s][1]:.1f}, z {F[s][2]:.1f}')
Fw = np.maximum(F['r'], F['l'])

tau = a.tau_grip if a.tau_grip is not None else G['gripper_effort']
dgap = abs(md['grip']['dgap_dq'])
N = 2 * tau / dgap
mu = float(CFG['box'].get('mu', 1.0))
m = float(CFG['box']['mass'])
w = m * 9.81 / 4
cap = mu * (2 * N + w)
print('\n② 허용 내력 = 손잡이 마찰 원뿔 (MR 12.2.1)')
print(f'   손가락 수직력 N = 2τ/|dgap/dq| = 2·{tau}/{dgap:.4f} = {N:.1f} N,  μ = {mu},  무게 몫 m g/4 = {w:.1f} N (닫힘축 = 수직 방향)')
print(f'   수평 한계 F_cone = μ(2N + m g/4) = {cap:.1f} N  (손 하나, x·y 합)')
print(f'   보상 없음: 손끝 마찰력 x {Fw[0]:.1f}, y {Fw[1]:.1f} N → 합 {np.hypot(Fw[0], Fw[1]):.1f} N '
      f'{"> F_cone → 어떤 K_F로도 원뿔 밖 (마찰 보상 필요)" if np.hypot(Fw[0], Fw[1]) > cap else "≤ F_cone"}')

dx, dy = a.delta
fsx, fsy = a.F_s
lo, hi = 0.0, 1e5
for _ in range(200):
    mid = 0.5 * (lo + hi)
    lo, hi = (mid, hi) if math.hypot(fsx + mid * dx, fsy + mid * dy) <= cap else (lo, mid)
print('\n③ 팔로워 강성 상한 (K_F x = y, 베이스 오차 최대일 때도 원뿔 안)')
print(f'   Δx = {dx * 1000:.1f} mm, Δy = {dy * 1000:.1f} mm (곡선 주행 실측 최대),  F_s = x {fsx}, y {fsy} N (보상 후 정지 중 실측)')
print(f'   √((F_s,x + K_F Δx)² + (F_s,y + K_F Δy)²) ≤ {cap:.1f}  →  K_F ≤ {lo:.0f} N/m')
print(f'   (x만: (F_cone − F_s,x)/Δx = {(cap - fsx) / dx:.0f},  y만: (F_cone − F_s,y)/Δy = {(cap - fsy) / dy:.0f} N/m → x가 결정)')
print('   하한: 팔로워의 베이스 오차 방향은 위치를 지키지 않고 따라가는 축 → 위치 정밀도에서 오는 하한 없음')
print('   예측 내력 (팔로워가 오차를 혼자 흡수한다고 볼 때, 보수적):')
for KF in (40, 110, 240, 406, 500):
    fx, fy = fsx + KF * dx, fsy + KF * dy
    print(f'     K_F {KF:4d}: x {fx:5.1f}  y {fy:4.1f}  합 {math.hypot(fx, fy):5.1f} N  {"≤" if math.hypot(fx, fy) <= cap else ">"} {cap:.1f}')
