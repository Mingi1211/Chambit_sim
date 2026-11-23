#!/usr/bin/env python3
"""런별 요약 그림 — 곡선 주행 중 손 내력(중앙값, 1 s 중앙값 최대), 리더 오차, 박스 결과. 베이스 양상(정상/출발 지연)별로 나눔. 김민기 (11/23)."""
import csv
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
D = os.path.join(HERE, '../../real_logs/1123_curve_trials')
RUNS = [  # (tag, 라벨)
    ('b1_K40_off', 'K_F 40\n보상 없음 (b1)'), ('b3_K40_off', 'K_F 40\n보상 없음 (b3)'), ('f_K500_off', 'K_F 500\n보상 없음'),
    ('f2_K10_on', 'K_F 10\n보상'), ('f_K40_on', 'K_F 40\n보상'), ('o1_opt_on', 'K_F (40,10)\n보상 ①'), ('o2_opt_on', 'K_F (40,10)\n보상 ②'),
    ('b5_K40_off', 'K_F 40\n보상 없음 (b5)'), ('f_K150_on', 'K_F 150\n보상'), ('f_K500_on', 'K_F 500\n보상 ①'), ('f2_K500_on', 'K_F 500\n보상 ②')]


def rmed(t, v, w=1.0):
    j0 = np.searchsorted(t, t - w)
    return np.array([np.median(v[j:i + 1]) for i, j in enumerate(j0)])


rows = []
for tag, lab in RUNS:
    r = analyze(D, tag); b = r['base']; h = r['hand']
    B = np.array([[float(x[k]) for k in ('t', 'roll', 'pitch')] for x in csv.DictReader(open(f'{D}/{tag}_box.csv'))])
    tilt = np.degrees(np.hypot(B[:, 1], B[:, 2]))
    tsb = np.interp(B[:, 0], b['t'], b['ts']) - b['ts'][b['i0']]
    k = np.where((tsb >= 0) & (tsb <= 10) & (tilt > 5))[0]
    t_end = tsb[k[0]] if len(k) else 10.0               # 박스가 기울기 5°를 넘으면 그 앞까지만 (손에서 빠진 뒤 내력은 의미 없음)
    tsh = np.interp(h['t'], b['t'], b['ts']) - b['ts'][b['i0']]
    m = (tsh >= 0) & (tsh <= t_end)
    n = np.median(h['nint'], axis=1)
    d = b['d'][b['m']]
    rows.append(dict(tag=tag, lab=lab, med=np.median(h['nint'][m]), peak=rmed(tsh[m], n[m]).max(),
                     eL=max(np.hypot(*r['leader'][s]) for s in 'rl') * 1e3, drop=len(k) > 0, t_drop=t_end,
                     nominal=abs(d[:, 1]).max() < 0.05, dy=abs(d[:, 1]).max() * 1e3, dx=abs(d[:, 0]).max() * 1e3,
                     tilt=tilt[(tsb >= 0) & (tsb <= 10)].max()))
fig, ax = plt.subplots(2, 1, figsize=(13, 7.5), sharex=True)
x = np.arange(len(rows))
for i, r in enumerate(rows):
    c = 'tab:red' if r['drop'] else ('tab:blue' if 'on' in r['tag'] else 'tab:gray')
    ax[0].bar(i - 0.2, r['med'], 0.4, color=c, alpha=0.9)
    ax[0].bar(i + 0.2, r['peak'], 0.4, color=c, alpha=0.45)
    ax[1].bar(i, r['eL'], 0.6, color=c)
    note = f"낙하 {r['t_drop']:.1f} s" if r['drop'] else f"유지 {r['tilt']:.1f}°"
    ax[0].text(i, max(r['med'], r['peak']) + 0.8, note, ha='center', fontsize=8)
    ax[1].text(i, r['eL'] + 3, f"Δx {r['dx']:.0f}\nΔy {r['dy']:.0f}", ha='center', fontsize=7)
ax[0].axhline(15, color='k', ls='--', lw=0.8); ax[0].set_ylim(0, 58)
ax[0].set_ylabel('손 내력 [N]\n진한 = 주행 중 중앙값, 연한 = 1 s 중앙값 최대')
ax[1].set_ylabel('리더 손 수평 오차 최대 [mm]')
sep = sum(1 for r in rows if not r['nominal']) - 0.5
for a in ax:
    a.axvline(sep, color='0.3', lw=1); a.grid(axis='y', alpha=0.3)
ax[0].text(sep / 2, 55, '베이스 출발 지연 (Δy 0.18 ~ 0.45 m 어긋남)', ha='center')
ax[0].text((sep + len(rows)) / 2, 55, '베이스 정상 (Δx 0.2 m 벌어짐)', ha='center')
ax[1].set_xticks(x); ax[1].set_xticklabels([r['lab'] for r in rows], fontsize=8)
fig.suptitle('곡선 주행 중 손 내력·리더 오차 (빨강 = 박스 낙하, 파랑 = 마찰 보상, 회색 = 보상 없음)')
fig.tight_layout()
out = os.path.join(HERE, '../../real_logs/1123_report/stiffness_summary.png')
fig.savefig(out, dpi=130)
for r in rows:
    print(f"{r['tag']:12s} {'정상' if r['nominal'] else '지연'} Δx {r['dx']:4.0f} Δy {r['dy']:4.0f} | 내력 중앙 {r['med']:5.1f} 최대 {r['peak']:5.1f} N | 리더 {r['eL']:4.0f} mm | {'낙하 %.1fs' % r['t_drop'] if r['drop'] else '유지 %.1f°' % r['tilt']}")
print('저장:', out)
