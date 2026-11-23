#!/usr/bin/env python3
"""곡선 주행 결과 그림. 김민기 (11/23).
  base  : 베이스 간 오차 Δx, Δy (반복 런) + 지령 상대 변위 (현대 base_curve: 로봇1만 옆으로 side_offset·sin²(πs), 옆 = −X)
  force : 조건별 손 내력 (4손 × 1 s 창 중앙값) 시간 그래프
  sweep : 팔로워 강성별 손 내력·리더 오차 (마찰 보상 유무)
사용: python3 plot_curve.py base --tags b1 b2 ... --out x.png
      python3 plot_curve.py force --tags A B --labels ... --out x.png
      python3 plot_curve.py sweep --on K10:tag K40:tag ... --off K40:tag ... --out x.png
"""
import argparse
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from curve_analysis import analyze  # noqa: E402

f = '/usr/share/fonts/truetype/nanum/NanumGothic.ttf'
font_manager.fontManager.addfont(f)
plt.rcParams['font.family'] = font_manager.FontProperties(fname=f).get_name()
plt.rcParams['axes.unicode_minus'] = False

ap = argparse.ArgumentParser()
ap.add_argument('kind', choices=['base', 'force', 'sweep'])
ap.add_argument('--dir', default=os.path.join(HERE, '../../real_logs/1123_curve_trials'))
ap.add_argument('--tags', nargs='*', default=[])
ap.add_argument('--labels', nargs='*', default=[])
ap.add_argument('--on', nargs='*', default=[])
ap.add_argument('--off', nargs='*', default=[])
ap.add_argument('--side_offset', type=float, default=0.20)
ap.add_argument('--out', required=True)
a = ap.parse_args()


def s_of(t):
    tau = np.clip(t / 10.0, 0, 1)
    return tau ** 3 * (10 - 15 * tau + 6 * tau ** 2)


def tsim(r, t_wall):
    b = r['base']
    return np.interp(t_wall, b['t'], b['ts']) - b['ts'][b['i0']]


def rolling_median(t, v, w=1.0):
    j0 = np.searchsorted(t, t - w)
    return np.array([np.median(v[j:i + 1]) for i, j in enumerate(j0)])


if a.kind == 'base':
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    tt = np.linspace(0, 13, 400)
    cmd_x = np.where(tt < 10, a.side_offset * np.sin(np.pi * s_of(tt)) ** 2, 0)
    ax[0].plot(tt, cmd_x * 1e3, 'k--', lw=1.5, label='지령 (로봇1 옆 이탈)')
    ax[1].plot(tt, 0 * tt, 'k--', lw=1.5, label='지령 (두 로봇 같은 전진)')
    ax[2].plot(tt, 2000 * s_of(tt), 'k--', lw=1.5, label='지령 2 m')
    for k, tag in enumerate(a.tags):
        r = analyze(a.dir, tag)
        b = r['base']; m = b['m']
        ts = b['ts'][m] - b['ts'][b['i0']]
        c = f'C{k}'
        ax[0].plot(ts, b['d'][m, 0] * 1e3, c, lw=1, label=tag)
        ax[1].plot(ts, b['d'][m, 1] * 1e3, c, lw=1, label=tag)
        p0 = (b['p1'][b['i0']], b['p2'][b['i0']])
        ax[2].plot(ts, (b['p1'][m, 1] - p0[0][1]) * 1e3, c, lw=1, label=f'{tag} R1')
        ax[2].plot(ts, (b['p2'][m, 1] - p0[1][1]) * 1e3, c, lw=1, ls=':')
    ax[0].set_title('Δx: 두 로봇 사이 거리 변화 (+ = 멀어짐)')
    ax[1].set_title('Δy: 주행 방향 어긋남 (+ = 로봇2가 앞섬)')
    ax[2].set_title('각 베이스 전진 거리 (실선 R1, 점선 R2)')
    for x in ax:
        x.set_xlabel('베이스 패턴 시뮬 시간 [s]'); x.grid(alpha=0.3); x.axvline(10, color='0.6', lw=0.8)
    ax[0].set_ylabel('[mm]'); ax[0].legend(fontsize=8); ax[2].legend(fontsize=7, ncol=2)
    fig.tight_layout(); fig.savefig(a.out, dpi=130)

elif a.kind == 'force':
    fig, ax = plt.subplots(figsize=(9, 4))
    for k, tag in enumerate(a.tags):
        r = analyze(a.dir, tag)
        h = r['hand']
        m = (h['t'] > r['t0'] - 20) & (h['t'] < r['t1'] + 5)
        ts = tsim(r, h['t'][m])
        v = rolling_median(ts, h['nint'][m].ravel() if False else np.median(h['nint'][m], axis=1))
        ax.plot(ts, v, lw=1.3, label=a.labels[k] if k < len(a.labels) else tag)
    ax.axhline(15, color='k', ls='--', lw=0.8, label='허용 15 N (문서 3쪽)')
    ax.axvspan(0, 10, color='0.93', zorder=0)
    ax.set_xlabel('베이스 패턴 시뮬 시간 [s] (회색 = 곡선 주행, 0 이전 = 박스 든 채 정지)')
    ax.set_ylabel('손 내력 ‖f_int‖ [N] (4손 중앙값, 1 s 이동 중앙값)')
    ax.grid(alpha=0.3); ax.legend(fontsize=9)
    fig.tight_layout(); fig.savefig(a.out, dpi=130)

else:
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for name, items, mk in (('보상 있음', a.on, 'o-'), ('보상 없음', a.off, 's--')):
        K, F, E = [], [], []
        for it in items:
            k, tag = it.split(':')
            r = analyze(a.dir, tag); h = r['hand']
            K.append(float(k)); F.append(np.median(h['nint'][h['m']]))
            E.append(max(np.hypot(*r['leader'][s]) for s in 'rl') * 1e3)
        o = np.argsort(K)
        ax[0].plot(np.array(K)[o], np.array(F)[o], mk, label=name)
        ax[1].plot(np.array(K)[o], np.array(E)[o], mk, label=name)
    ax[0].axhline(15, color='k', ls='--', lw=0.8, label='허용 15 N')
    ax[0].set_ylabel('곡선 주행 중 손 내력 중앙값 [N]')
    ax[1].set_ylabel('리더 손 수평 오차 최대 [mm]')
    for x in ax:
        x.set_xscale('log'); x.set_xlabel('팔로워 수평 강성 K_F [N/m]'); x.grid(alpha=0.3, which='both'); x.legend()
    fig.tight_layout(); fig.savefig(a.out, dpi=130)
print('저장:', a.out)
