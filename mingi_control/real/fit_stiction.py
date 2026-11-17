#!/usr/bin/env python3
"""정지 마찰 — 대표 자세 2개(hang, grasp) 비교 → 비슷하면 평균 채택 (11/17 선배 조언). 김민기.

근거
  BAM 2025 II.C 정지 영역: 정지 조건 |τ_m + τ_e| ≤ τ_f^m → 시행마다 τ_f^m = (τ₊ − τ₋)/2, −τ_e = (τ₊ + τ₋)/2
  MR 8.9.4 그림 8.13(b): τ_f^m = b_static (정지 마찰)
  MR 8장 g(q): −τ_e(측정)와 URDF 중력 g(q)를 나란히 표시 — 그 자세에서 외부 토크가 중력뿐인지 확인용
  자세에 따라 다를 수 있는 이유: 하중 의존 마찰 (BAM 식 (7) M3, Qian 식 (10)(11))
비교: 두 자세 평균의 차이 [%]와 시행값을 그대로 보여줌 — "비슷한지"는 사람이 판단 (선배 조언: 비슷하면 평균 채택).
  크게 다르면 하중 의존 마찰(BAM M3) 영향 → 시뮬은 파지 하중 기준이라 grasp 값이 맞음

사용: source /opt/ros/jazzy/setup.bash
  python3 fit_stiction.py --arm l --urdf ../robot_description.urdf [--visc friction_l.csv] --out .
  (현재 폴더의 l_stic_hang_j*.csv, l_stic_grasp_j*.csv 를 읽음)
출력: 표(화면·stiction_<팔>.csv), stiction_pose_<팔>.png (자세별 정지 마찰 비교), stiction_time_<팔>_j<n>.png (토크 램프와 출발)
"""
import argparse
import csv
import glob
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from matplotlib import font_manager

for f in ('NanumGothic', 'Noto Sans CJK KR'):
    if any(f in x.name for x in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = f
        break
plt.rcParams['axes.unicode_minus'] = False

ap = argparse.ArgumentParser()
ap.add_argument('--arm', required=True)
ap.add_argument('--urdf', required=True)
ap.add_argument('--visc', default=None, help='fit_friction.py 결과 friction_<팔>.csv (운동 마찰 F_c 같이 표시)')
ap.add_argument('--dir', default='.')
ap.add_argument('--out', default='.')
ap.add_argument('--time_joint', type=int, default=4)
a = ap.parse_args()
C = {'l': [0.8099, 0.6729, 0.5772, 0.7537, 0.8187, 0.807, 0.0055], 'r': [0.7922, 0.5475, 0.7601, 0.7183, 0.8034, 0.8266, 0.0063]}[a.arm]
NOM = [0.07692307692] * 3 + [0.04] * 3 + [1.0]
POSE = {'hang': {'l': [0] * 7, 'r': [0] * 7},
        'grasp': {'l': [0.5106, 0.1599, -0.0832, -1.266, 1.7479, -0.0301, -0.8224],
                  'r': [-0.3888, -0.7111, 1.9041, -1.37, -0.3074, 0.6082, -1.5823]}}
FC = {}
if a.visc:
    FC = {int(r['joint']): float(r['Fc']) for r in csv.DictReader(open(a.visc))}

text = open(a.urdf).read()
text = text[text.index('<?xml'):] if '<?xml' in text else text[text.index('<robot'):]
M = pin.buildModelFromXML(text)
Dm = M.createData()
IQ = [M.joints[M.getJointId(f'arm_{a.arm}_joint{j}')].idx_q for j in range(1, 8)]
IV = [M.joints[M.getJointId(f'arm_{a.arm}_joint{j}')].idx_v for j in range(1, 8)]


def g_urdf(pose, j):
    q7 = list(POSE[pose][a.arm])
    if pose == 'hang' and j == 2:
        q7[1] = 0.3 if a.arm == 'l' else -0.3
    q = pin.neutral(M)
    for k, v in zip(IQ, q7):
        q[k] = v
    return pin.computeGeneralizedGravity(M, Dm, q)[IV[j - 1]]


def load(path, j):
    d = np.genfromtxt(path, delimiter=',', names=True)
    tau = C[j - 1] * d['I']
    tp = tau[d['onset'] > 0]; tn = tau[d['onset'] < 0]
    n = min(len(tp), len(tn))
    return d, tau, (tp[:n] - tn[:n]) / 2, (tp[:n] + tn[:n]) / 2       # 시행별 τ_f^m, −τ_e (BAM II.C)


R = {}
for pose in ('hang', 'grasp'):
    for path in glob.glob(os.path.join(a.dir, f'{a.arm}_stic_{pose}_j*.csv')):
        j = int(re.search(r'_j(\d)', path).group(1))
        R[(pose, j)] = load(path, j)

rows = []
print(f'\n{a.arm}팔 정지 마찰 τ_f^m [N·m] (BAM II.C) — 자세별 시행값, 평균 / −τ_e(측정) vs g(q)(URDF)')
print(' 관절 | hang 시행값        | grasp 시행값       | 자세 차이 | 전체 평균 | 운동 F_c | hang −τ_e/g  | grasp −τ_e/g')
J = sorted({j for _, j in R})
for j in J:
    h = R.get(('hang', j)); g = R.get(('grasp', j))
    hv = h[2] if h else np.array([]); gv = g[2] if g else np.array([])
    adopt = np.r_[hv, gv].mean() if len(hv) + len(gv) else np.nan
    diff = abs(hv.mean() - gv.mean()) / adopt * 100 if len(hv) and len(gv) else np.nan
    he = f'{h[3].mean():+.2f}/{g_urdf("hang", j):+.2f}' if h is not None and len(h[3]) else '-'
    ge = f'{g[3].mean():+.2f}/{g_urdf("grasp", j):+.2f}' if g is not None and len(g[3]) else '-'
    print(f'  j{j}  | {" ".join(f"{x:5.2f}" for x in hv):18s} | {" ".join(f"{x:5.2f}" for x in gv):18s} | '
          f'{diff:5.1f} %   | {adopt:6.2f}    | {FC.get(j, np.nan):6.2f}   | {he:12s} | {ge}')
    rows.append(dict(arm=a.arm, joint=j, hang_mean=round(hv.mean(), 3) if len(hv) else '', grasp_mean=round(gv.mean(), 3) if len(gv) else '',
                     diff_pct=round(float(diff), 1), tau_s_mean=round(float(adopt), 3), Fc_kinetic=FC.get(j, '')))
with open(os.path.join(a.out, f'stiction_{a.arm}.csv'), 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)
print('비슷하면 전체 평균, 크게 다르면 grasp 평균을 → 시뮬 <friction> (MR 그림 8.13(b) b_static). 운동 F_c와 비교: τ_s ≈ F_c 면 MR (c) 모델로 충분, τ_s > F_c 면 (d) 형태')

# 그림 1: 자세별 정지 마찰 비교 (보고용)
fig, ax = plt.subplots(figsize=(10, 4.8))
x = np.arange(len(J)); w = 0.35
for o, (pose, col) in enumerate((('hang', 'C0'), ('grasp', 'C1'))):
    means = [R[(pose, j)][2].mean() if (pose, j) in R and len(R[(pose, j)][2]) else np.nan for j in J]
    ax.bar(x + (o - 0.5) * w, means, w, color=col, alpha=0.7, label={'hang': '팔 늘어뜨림', 'grasp': '박스 파지 직후 자세'}[pose])
    for k, j in enumerate(J):
        if (pose, j) in R:
            ax.plot([x[k] + (o - 0.5) * w] * len(R[(pose, j)][2]), R[(pose, j)][2], 'k.', ms=5)
if FC:
    ax.plot(x, [FC.get(j, np.nan) for j in J], 'r_', ms=22, mew=2, label='운동 마찰 F_c (등속 왕복)')
ax.set_xticks(x); ax.set_xticklabels([f'j{j}' for j in J]); ax.set_ylabel('정지 마찰 τ_s [N·m]')
ax.set_title(f'{a.arm}팔 정지 마찰: 자세 2개 비교 (막대 = 평균, 점 = 시행값)'); ax.legend(fontsize=8)
fig.tight_layout(); p1 = os.path.join(a.out, f'stiction_pose_{a.arm}.png'); fig.savefig(p1, dpi=130)

# 그림 2: 한 관절의 토크 램프와 출발 (자세 2개)
j = a.time_joint
if ('hang', j) in R or ('grasp', j) in R:
    fig, ax = plt.subplots(2, 2, figsize=(13, 5.5), sharex='col')
    for c_, pose in enumerate(('hang', 'grasp')):
        if (pose, j) not in R:
            continue
        d, tau, *_ = R[(pose, j)]
        t = d['t'] - d['t'][0]
        ax[0, c_].plot(t, tau, lw=0.8, label='측정 토크 c·I')
        ax[0, c_].plot(t, C[j - 1] * NOM[j - 1] * d['goal_raw'], '--', lw=0.8, label='명령 (램프)')
        on = d['onset'] != 0
        ax[0, c_].plot(t[on], tau[on], 'r^', label='출발 순간')
        ax[1, c_].plot(t, d['q'] - d['q'][0])
        ax[0, c_].set_title({'hang': '팔 늘어뜨림', 'grasp': '박스 파지 직후 자세'}[pose])
        ax[0, c_].set_ylabel('토크 [N·m]'); ax[1, c_].set_ylabel('관절 이동 [rad]'); ax[1, c_].set_xlabel('t [s]')
    ax[0, 0].legend(fontsize=8)
    fig.suptitle(f'{a.arm}팔 j{j} 정지 마찰: 토크를 천천히 바꾸다 움직이기 시작하는 순간 (BAM II.C 정지 영역 경계)')
    fig.tight_layout(); p2 = os.path.join(a.out, f'stiction_time_{a.arm}_j{j}.png'); fig.savefig(p2, dpi=130)
    print(f'그림: {p1}\n      {p2}')
