#!/usr/bin/env python3
"""ff_compare.py 결과 → 보고용 그림 2종. 김민기 (11/17).

  ① 보상 비교 (기존 설계 강성): 보상 없음 / 쿨롱 / 점성 / 쿨롱+점성 — 추종 오차 시간 그래프 + RMS 막대
  ② 강성 하한 before/after: 기존 강성 vs 하한 강성, 보상 없음·쿨롱+점성 — 정상상태 오차 vs 허용 오차 δθ
지표: 이동 중 RMS 오차, 정상상태 오차 = 이동 끝 + 2 % 정착 시간 t_s 시점 오차 (MR 412쪽)
사용: python3 plot_ff_compare.py --dir ../../real_logs/1117_fric/sim --joints 4 1
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

for f in ('NanumGothic', 'Noto Sans CJK KR'):
    if any(f in x.name for x in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = f
        break
plt.rcParams['axes.unicode_minus'] = False

ap = argparse.ArgumentParser()
ap.add_argument('--dir', required=True)
ap.add_argument('--robot', default='robot1')
ap.add_argument('--side', default='r')
ap.add_argument('--joints', type=int, nargs='+', default=[4, 1])
ap.add_argument('--out', default=None)
a = ap.parse_args()
out = a.out or a.dir
CASES = [('none', '보상 없음', '#c0392b'), ('coulomb', '쿨롱 보상', '#e67e22'), ('viscous', '점성 보상', '#27ae60'), ('both', '쿨롱+점성 보상', '#2c6fbb')]


def load(stiff, j, case):
    p = os.path.join(a.dir, f'ff_fric_{stiff}_{a.robot}_arm_{a.side}_joint{j}.csv')
    R = [r for r in csv.DictReader(open(p)) if r['case'] == case]
    tr = [r for r in R if r['seg'] == 'traj']; ho = [r for r in R if r['seg'] == 'hold']
    t = np.array([float(r['t']) for r in tr]); t -= t[0]
    e = np.array([float(r['qd']) - float(r['q']) for r in tr]) * 1e3
    return dict(t=t, e=e, rms=float(np.sqrt(np.mean(e ** 2))), ess=abs(float(ho[-1]['qd']) - float(ho[-1]['q'])) * 1e3,
                kp=float(R[0]['kp']), dth=float(R[0]['dth']) * 1e3)


rows = []
for j in a.joints:
    # ① 보상 비교
    S = {c: load('design', j, c) for c, _, _ in CASES}
    fig, ax = plt.subplots(1, 2, figsize=(12, 3.8), gridspec_kw=dict(width_ratios=[2.2, 1]))
    for c, lb, col in CASES:
        ax[0].plot(S[c]['t'], S[c]['e'], color=col, lw=1.2, label=lb)
    ax[0].axhline(0, color='gray', lw=0.5)
    ax[0].set_xlabel('시간 [s]'); ax[0].set_ylabel('추종 오차 [mrad]'); ax[0].legend(fontsize=8, ncol=2, frameon=False)
    ax[0].set_title('이동 중 추종 오차')
    v = [S[c]['rms'] for c, _, _ in CASES]
    ax[1].bar(range(4), v, color=[col for _, _, col in CASES])
    for k, x in enumerate(v):
        ax[1].text(k, x, f'{x:.1f}', ha='center', va='bottom', fontsize=9)
    ax[1].set_xticks(range(4)); ax[1].set_xticklabels(['없음', '쿨롱', '점성', '쿨롱+점성'], fontsize=9)
    ax[1].set_ylabel('RMS 오차 [mrad]'); ax[1].set_title('RMS')
    for x in ax:
        x.spines[['top', 'right']].set_visible(False)
    fig.suptitle(f'오른팔 j{j} 마찰 보상 비교 (기존 설계 강성 K_p = {S["none"]["kp"]:.0f} N·m/rad)', fontsize=11)
    fig.tight_layout(); fig.savefig(os.path.join(out, f'comp_r_j{j}.png'), dpi=150); plt.close(fig)
    # ② 강성 하한 before/after
    B = {('design', c): S[c] for c in ('none', 'both')}
    for c in ('none', 'both'):
        B[('kmin', c)] = load('kmin', j, c)
    dth = B[('kmin', 'none')]['dth']
    labels = ['기존 강성\n보상 없음', '기존 강성\n쿨롱+점성', '하한 강성\n보상 없음', '하한 강성\n쿨롱+점성']
    keys = [('design', 'none'), ('design', 'both'), ('kmin', 'none'), ('kmin', 'both')]
    fig, ax = plt.subplots(1, 2, figsize=(12, 3.8))
    vals = [B[k]['ess'] for k in keys]
    ax[0].bar(range(4), vals, color=['#c0392b', '#2c6fbb', '#e8a29a', '#9dbbe0'], edgecolor=['#c0392b', '#2c6fbb', '#c0392b', '#2c6fbb'])
    ax[0].axhline(dth, color='k', ls='--', lw=1); ax[0].text(3.45, dth, f' 허용 δθ = {dth:.1f}', va='bottom', ha='right', fontsize=9)
    bd = 1e3 * float(load('design', j, 'none')['kp'] and 0) if False else None
    for k, x in enumerate(vals):
        ax[0].text(k, x, f'{x:.1f}', ha='center', va='bottom', fontsize=9)
    ax[0].set_xticks(range(4)); ax[0].set_xticklabels(labels, fontsize=8.5); ax[0].set_ylabel('정상상태 오차 [mrad]')
    ax[0].set_title('멈춘 뒤 남은 오차')
    vals2 = [B[k]['rms'] for k in keys]
    ax[1].bar(range(4), vals2, color=['#c0392b', '#2c6fbb', '#e8a29a', '#9dbbe0'], edgecolor=['#c0392b', '#2c6fbb', '#c0392b', '#2c6fbb'])
    for k, x in enumerate(vals2):
        ax[1].text(k, x, f'{x:.1f}', ha='center', va='bottom', fontsize=9)
    ax[1].set_xticks(range(4)); ax[1].set_xticklabels(labels, fontsize=8.5); ax[1].set_ylabel('RMS 오차 [mrad]'); ax[1].set_title('이동 중 RMS')
    for x in ax:
        x.spines[['top', 'right']].set_visible(False)
    fig.suptitle(f'오른팔 j{j} 강성 하한 적용 전후 (기존 K_p {B[("design", "none")]["kp"]:.0f} → 하한 K_p {B[("kmin", "none")]["kp"]:.0f} N·m/rad)', fontsize=11)
    fig.tight_layout(); fig.savefig(os.path.join(out, f'kmin_r_j{j}.png'), dpi=150); plt.close(fig)
    for k in keys + [('design', 'coulomb'), ('design', 'viscous')]:
        X = B.get(k) or S[k[1]]
        rows.append(dict(joint=j, stiffness=k[0], case=k[1], kp=round(X['kp'], 1), rms_mrad=round(X['rms'], 2), ess_mrad=round(X['ess'], 2), dtheta_mrad=round(X['dth'], 2)))
with open(os.path.join(out, 'comp_table.csv'), 'w', newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)
for r in rows:
    print(r)
