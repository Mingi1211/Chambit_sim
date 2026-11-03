#!/usr/bin/env python3
"""결과 분석 — 예측(문서 식) vs 측정. 김민기.

  t12  python3 analyze.py t12 <로그> [--side r]                 중력만(mode gravity): 손 위치 드리프트 [문서 #12-③]
  t4   python3 analyze.py t4 <로그> --t_on T --t_off T --F fx fy fz   e_ss ≈ F/K  [문서 #4]
  t5   python3 analyze.py t5 <로그> --t_step T --axis y            오버슈트 ≈ 0, T_s ≈ 5.8/ωn, ωn = √(K/Λ)  [문서 #5·#6]
  box  python3 analyze.py box <로그> <box.csv>                      쥐는 힘 F_sq = |F_cmd,y|, 박스 처짐·미끄러짐  [S5]
그래프는 로그 옆에 PNG로 저장.
"""
import argparse
import csv
import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

for f in ('NanumGothic', 'Noto Sans CJK KR'):
    if any(f in x.name for x in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = f
        break
plt.rcParams['axes.unicode_minus'] = False


def load(path, side):
    rows = [r for r in csv.DictReader(open(path)) if r['side'] == side]
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k != 'side'}


def t12(a):
    L = load(a.log, a.side)
    t = L['t'] - L['t'][0]
    p = np.c_[L['px'], L['py'], L['pz']]
    drift = np.linalg.norm(p - p[0], axis=1) * 1000
    m = t <= 10
    print(f'== T12 중력만: 10 s 손 위치 드리프트 최대 {drift[m].max():.1f} mm (작으면 g(q) 부호·크기 맞음)')
    plt.plot(t, drift); plt.xlabel('t [s]'); plt.ylabel('손 위치 변화 [mm]'); plt.title('T12: τ = g(q)만')
    plt.savefig(a.log.replace('.csv', f'_t12_{a.side}.png'), dpi=120); print('그래프 저장')


def t4(a):
    L = load(a.log, a.side)
    t = L['t']
    on = (t > a.t_off - 1.0) & (t < a.t_off - 0.05)          # 힘 끄기 직전 1 s = 정상상태
    e = np.array([L[f'e{c}'][on].mean() for c in 'xyz'])
    K = np.array([L[f'K{c}'][on].mean() for c in 'xyz'])
    F = np.array(a.F)
    # e = p_d − p. 외력 F가 손을 F 방향으로 밀면 p가 F쪽으로 → e = −F/K
    pred = -F / K
    print('== T4 정상상태 오차 [문서 #4] e_ss = −F/K (e = p_d − p)')
    for i, c in enumerate('xyz'):
        if abs(F[i]) > 0:
            print(f'  {c}: F={F[i]:+.1f} N, K={K[i]:.0f} N/m → 예측 {pred[i]*1000:+.2f} mm, 측정 {e[i]*1000:+.2f} mm '
                  f'(비 {e[i]/pred[i]:.2f})')
    win = (t > a.t_on - 1) & (t < a.t_off + 3)
    tt = t[win] - a.t_on
    for c, col in zip('xyz', ('C0', 'C1', 'C2')):
        plt.plot(tt, L[f'e{c}'][win] * 1000, col, label=f'e_{c} 측정')
        i = 'xyz'.index(c)
        if abs(F[i]) > 0:
            plt.axhline(pred[i] * 1000, color=col, ls='--', label=f'e_{c} 예측 −F/K')
    plt.axvline(0, color='gray', lw=0.5); plt.axvline(a.t_off - a.t_on, color='gray', lw=0.5)
    plt.xlabel('외력 켠 뒤 t [s]'); plt.ylabel('e = p_d − p [mm]'); plt.legend(); plt.title(f'T4: 외력 {F} N')
    plt.savefig(a.log.replace('.csv', f'_t4_{a.side}.png'), dpi=120); print('그래프 저장')


def t5(a):
    L = load(a.log, a.side)
    t = L['t']
    i = 'xyz'.index(a.axis)
    after = t >= a.t_step
    p, pd = L[f'p{a.axis}'], L[f'pd{a.axis}']
    k0 = np.argmax(after)
    p0, pf = p[k0 - 1], pd[after][-1]
    step = pf - p0
    seg_t, seg_p = t[after] - a.t_step, p[after]
    over = max(0.0, ((seg_p - pf) * np.sign(step)).max()) / abs(step) * 100
    outside = np.abs(seg_p - pf) > 0.02 * abs(step)
    Ts = seg_t[np.where(outside)[0][-1]] if outside.any() else 0.0
    K, Lam = L[f'K{a.axis}'][after].mean(), L[f'L{a.axis}'][after].mean()
    wn = math.sqrt(K / Lam)
    print(f'== T5 계단 응답 {a.axis}축 {step*1000:+.1f} mm [문서 #5·#6]')
    print(f'  K={K:.0f} N/m, Λ_{a.axis}{a.axis}={Lam:.2f} kg → ωn=√(K/Λ)={wn:.1f} rad/s, 예측 T_s≈5.8/ωn={5.8/wn:.3f} s')
    print(f'  측정: 오버슈트 {over:.1f} % (예측 ≈ 0), T_s(2%) {Ts:.3f} s')
    plt.plot(seg_t, (seg_p - p0) * 1000, label='측정'); plt.plot(seg_t, (pd[after] - p0) * 1000, '--', label='목표')
    plt.axvline(5.8 / wn, color='gray', ls=':', label='예측 T_s')
    plt.xlabel('t [s]'); plt.ylabel(f'{a.axis} 변위 [mm]'); plt.legend(); plt.title('T5 계단 응답')
    plt.savefig(a.log.replace('.csv', f'_t5_{a.axis}.png'), dpi=120); print('그래프 저장')


def box(a):
    R, Lf = load(a.log, 'r'), load(a.log, 'l')
    B = np.genfromtxt(a.box_log, delimiter=',', names=True)
    ev = {}
    try:
        for r in csv.DictReader(open(a.box_log.replace('_box.csv', '_events.csv'))):
            ev[r['name'][0]] = float(r['t'])
    except FileNotFoundError:
        pass
    t0 = R['t'][0]
    fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    for L, s in ((R, 'r'), (Lf, 'l')):
        ax[0].plot(L['t'] - t0, np.abs(L['Fy']), label=f'{s}: |F_cmd,y| = 쥐는 힘')
        ax[2].plot(L['t'] - t0, L['pz'], label=f'손 {s} z')
    ax[0].axhline(30, color='r', ls='--', label='내력 상한 30 N')
    ax[0].set_ylabel('[N]'); ax[0].legend(fontsize=8)
    ax[1].plot(R['t'] - t0, R['m_ff'], label='무게 피드포워드 m [kg]'); ax[1].legend(fontsize=8)
    ax[2].plot(B['t'] - t0, B['z'] - (B['z'][0] - R['pz'][0] if len(B) else 0), 'k', label='박스 z (손 높이에 맞춤, 기록용)')
    ax[2].set_ylabel('z [m]'); ax[2].set_xlabel('t [s]'); ax[2].legend(fontsize=8)
    for k, tv in ev.items():
        for x in ax:
            x.axvline(tv - t0, color='gray', lw=0.5)
        ax[0].text(tv - t0, ax[0].get_ylim()[1] * 0.9, k, fontsize=8)
    fig.savefig(a.log.replace('.csv', '_box.png'), dpi=120)
    if '4' in ev and '5' in ev:
        hold = lambda L: (L['t'] > ev['4']) & (L['t'] < ev['5'])  # noqa: E731
        fsq = (np.abs(R['Fy'][hold(R)]).mean() + np.abs(Lf['Fy'][hold(Lf)]).mean()) / 2
        hz = (R['pz'][hold(R)].mean() + Lf['pz'][hold(Lf)].mean()) / 2
        bh = (B['t'] > ev['4']) & (B['t'] < ev['5'])
        rel = B['z'][bh] - np.interp(B['t'][bh], R['t'], R['pz'])
        print(f'== S5 유지 구간: 쥐는 힘 평균 {fsq:.1f} N (상한 30 N), 최대 {max(np.abs(R["Fy"][hold(R)]).max(), np.abs(Lf["Fy"][hold(Lf)]).max()):.1f} N')
        print(f'   박스-손 상대 높이 변화(미끄러짐) {(rel.max() - rel.min())*1000:.1f} mm (기준 < 5 mm)')
        z_lift = B['z'][bh].mean() - B['z'][0]
        print(f'   박스 들린 높이 {z_lift*1000:.1f} mm (목표 {(hz - R["pz"][0])*1000:.1f} mm)')
    print('그래프 저장:', a.log.replace('.csv', '_box.png'))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('kind', choices=['t12', 't4', 't5', 'box'])
    ap.add_argument('log')
    ap.add_argument('box_log', nargs='?')
    ap.add_argument('--side', default='r')
    ap.add_argument('--t_on', type=float); ap.add_argument('--t_off', type=float)
    ap.add_argument('--F', type=float, nargs=3)
    ap.add_argument('--t_step', type=float); ap.add_argument('--axis', default='y')
    a = ap.parse_args()
    {'t12': t12, 't4': t4, 't5': t5, 'box': box}[a.kind](a)
