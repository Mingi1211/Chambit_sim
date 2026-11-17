#!/usr/bin/env python3
"""11/10 실측 마찰 → Gazebo 관절 마찰 값 (URDF <dynamics friction damping>). 김민기 (11/16). numpy만 사용.

근거
  MR 8.9.4 그림 8.13(c): 쿨롱 + 점성  τ_f = f_c·sgn(q̇) + b·q̇
  MR (11.48) 등속·저속: c·I = τ_L + τ_f  →  +/− 방향 반차 = 마찰 크기 (11/10 Ubuntu 분석, kt_friction_table_1110.csv)
  쿨롱 크기가 하중 토크 |τ_L|에 따라 커짐 (Qian 2024 식 (10)(11), BAM III.B)
    → 박스 없음/있음 두 점으로 직선  τ_f(|τ_L|) = a + f_l·|τ_L|   (0.05 rad/s 데이터, 쿨롱만 — 점성 적합 안 함)
    → Gazebo <friction>는 상수 하나 → 운반(파지) 자세의 하중에서 계산한 값 하나로  f_c,eff = a + f_l·|τ_L,grasp|   [AI 제안: 합치는 방식]
  점성 b(<damping>): 11/10에서 0.05·0.1 rad/s 두 속도만 재 음수로 나옴 → 실측값 없음 → --damping 으로 후보값 지정(민감도 시험)

사용
  python3 friction_params.py kt_friction_table_1110.csv --urdf robot_description.urdf --box_mass 2.0 --hand_share 0.5 --damping 0.1
  출력 마지막의 YAML 블록을 coop_carry_bringup/config/robots.yaml 에 붙여 넣음 (joint_friction)
  --hand_share: 손 하나가 받는 박스 무게 비율 (impedance_node 박스 피드포워드와 같게: 한 로봇 한 손이면 0.5)
"""
import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from urdf_grav import Arm  # noqa: E402

# 파지 자세 (mingi_control/gain_design.py Q_GRASP, 11/4 시뮬 파지 순간)
Q_GRASP = {'r': [-0.3888, -0.7111, 1.9041, -1.37, -0.3074, 0.6082, -1.5823],
           'l': [0.5106, 0.1599, -0.0832, -1.266, 1.7479, -0.0301, -0.8224]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('table', help='kt_friction_table_1110.csv (실측_1110_Ubuntu)')
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--box_mass', type=float, default=2.0, help='시뮬 박스 질량 [kg] (robots.yaml box.mass)')
    ap.add_argument('--hand_share', type=float, default=0.5)
    ap.add_argument('--speed', type=float, default=0.05, help='사용할 등속 데이터 [rad/s]')
    ap.add_argument('--damping', type=float, nargs='+', default=[0.0],
                    help='<damping> 점성 b [N·m·s/rad]: 값 하나(전 관절) 또는 7개')
    ap.add_argument('--scale', type=float, default=1.0, help='friction 배율 (민감도 시험: 0, 0.5, 1, 1.5)')
    a = ap.parse_args()

    rows = [r for r in csv.DictReader(open(a.table, encoding='utf-8')) if abs(float(r['speed_rad_s']) - a.speed) < 1e-9]
    if len(rows) != 14:
        raise SystemExit(f'{a.speed} rad/s 행이 14개가 아님 ({len(rows)})')
    damp = a.damping * 7 if len(a.damping) == 1 else a.damping
    if len(damp) != 7:
        raise SystemExit('--damping 은 1개 또는 7개')
    np.set_printoptions(precision=3, suppress=True)
    out = {}
    print(f'박스 {a.box_mass} kg × 손 몫 {a.hand_share}, 파지 자세 Q_GRASP, 등속 {a.speed} rad/s 데이터\n')
    print('팔 관절 | 실측 (|τ_L|, τ_f) 박스 없음 → 있음 | 직선 a + f_l·|τ_L| | 파지 |τ_L| | friction(f_c,eff)')
    for side in ('r', 'l'):
        arm = Arm(a.urdf, side)
        q = Q_GRASP[side]
        tau_L = arm.gravity(q) + arm.box_torque(q, a.box_mass * a.hand_share)
        fr = []
        for j in range(1, 8):
            r = next(x for x in rows if x['arm'] == side and int(x['joint']) == j)
            L0, f0 = float(r['tauL_nobox_Nm']), float(r['tauf_nobox_Nm'])
            L1, f1 = float(r['tauL_load_Nm']), float(r['tauf_load_Nm'])
            fl = (f1 - f0) / (L1 - L0)
            a0 = f0 - fl * L0
            fc = max(0.0, a0 + fl * abs(tau_L[j - 1])) * a.scale
            fr.append(round(float(fc), 3))
            print(f' {side}  j{j}  | ({L0:5.1f}, {f0:5.2f}) → ({L1:5.1f}, {f1:5.2f}) | {a0:+6.2f} + {fl:4.2f}·|τ_L| '
                  f'| {abs(tau_L[j - 1]):5.2f} | {fc:5.2f}')
        out[side] = dict(friction=fr, damping=[float(x) for x in damp])
    print('\n# robots.yaml 에 붙여 넣기 (control_mode: effort 일 때만 적용, 비우면 마찰 0 = 기존과 같음)')
    print('joint_friction:')
    for side in ('r', 'l'):
        print(f'  {side}:')
        print(f'    friction: {out[side]["friction"]}   # [N·m] 쿨롱 (파지 자세 하중 기준)')
        print(f'    damping:  {out[side]["damping"]}   # [N·m·s/rad] 점성 (실측 없음 → 후보값)')


if __name__ == '__main__':
    main()
