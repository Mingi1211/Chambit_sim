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
import os

import matplotlib
matplotlib.use('Agg')                      # 화면 없이 PNG만 저장 (Authorization 경고 방지)
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
    if not rows:
        raise SystemExit(f'{path}: 토크 모드({side}) 데이터 없음 — 아직 task/gravity로 전환 안 됐거나, 노드가 새로 시작돼 빈 로그')
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k != 'side'}


def t12(a):
    L = load(a.log, a.side)
    k0 = np.argmax(L['t'] >= a.t0) if a.t0 else 0               # --t0: 모드 전환 시각부터
    L = {k: v[k0:] for k, v in L.items()}
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
    after = (t >= a.t_step) & (t < (a.t_end if a.t_end else np.inf))     # 다음 입력 전까지만
    p, pd = L[f'p{a.axis}'], L[f'pd{a.axis}']
    k0 = np.argmax(after)
    p0, pf = p[k0 - 1], pd[after][-1]
    step = pf - p0
    seg_t, seg_p = t[after] - a.t_step, p[after]
    if 'ts' in L:                                  # 시뮬 시간 기준 (RTF < 1이면 벽시계는 실제보다 김)
        seg_t = L['ts'][after] - L['ts'][k0]
        print(f'  시간축: 시뮬 시간 (측정 루프 주기 T = {np.median(L["T"][after])*1000:.1f} ms)')
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


def carry(a):
    """운반 시나리오: 단계별 박스 위치(기록용)·손 z 처짐·손 힘(F_cmd). 노드 로그는 같은 시각대 최신 것을 자동 선택"""
    import glob
    B = np.genfromtxt(a.log, delimiter=',', names=True)
    ev = [(r['name'], float(r['t'])) for r in csv.DictReader(open(a.log.replace('_box.csv', '_events.csv')))]
    T = dict((k.split()[0], t) for k, t in ev)
    logs = {r: sorted(glob.glob(os.path.expanduser(f'~/impedance_logs/*_task_{r}.csv')))[-1] for r in ('robot1', 'robot2')}
    D = {(r, s): load(p, s) for r, p in logs.items() for s in 'rl'}
    t0 = ev[0][1]
    win = lambda L, a0, a1: (L['t'] > a0) & (L['t'] < a1)  # noqa: E731
    print('== 단계별 (손 4개 평균)')
    marks = [t for _, t in ev] + [ev[-1][1] + 2]
    for (name, ta), tb in zip(ev, marks[1:]):
        ez = np.mean([D[k]['ez'][win(D[k], tb - 1.0, tb)].mean() for k in D]) * 1000
        fy = {r: np.mean([np.abs(D[r, s]['Fy'][win(D[r, s], tb - 1.0, tb)]).mean() for s in 'rl']) for r in ('robot1', 'robot2')}
        bw = (B['t'] > tb - 1.0) & (B['t'] < tb)
        by, bz = (B['y'][bw].mean() - B['y'][0]) * 100, (B['z'][bw].mean() - B['z'][0]) * 100
        print(f'  {name:32s} 손 z오차 {ez:5.1f} mm | |F_y| 리더 {fy["robot1"]:5.1f} N, 팔로워 {fy["robot2"]:5.1f} N | 박스 Δy {by:5.1f} cm, Δz {bz:5.1f} cm')
    # 베이스 이동 구간 내력 검증 (gain_design ⑥: 손당 F_int ≤ F_int,max, 예측 Δ·K_L K_F/(K_L+K_F))
    t_b0 = next((t for k, t in ev if k.startswith('4')), None)
    t_b1 = next((t for k, t in ev if '도착' in k), None)
    if t_b0 and t_b1:
        for r in ('robot1', 'robot2'):
            fxy = max(np.hypot(D[r, s]['Fx'], D[r, s]['Fy'])[win(D[r, s], t_b0, t_b1)].max() for s in 'rl')
            print(f'  베이스 이동 중 {r} 손 수평 힘 |F_cmd,xy| 최대 {fxy:.1f} N (허용 15 N)')
    if 'roll' in B.dtype.names:
        print(f'  박스 기울기 최대 {np.degrees(np.maximum(np.abs(B["roll"]), np.abs(B["pitch"]))).max():.1f}° (판정 2°)')
    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for (r, s), L in D.items():
        ax[0].plot(L['t'] - t0, L['Fy'], label=f'{r}/{s} F_y')
        ax[1].plot(L['t'] - t0, L['ez'] * 1000, label=f'{r}/{s} e_z')
    ax[2].plot(B['t'] - t0, (B['y'] - B['y'][0]) * 100, label='박스 Δy [cm]')
    ax[2].plot(B['t'] - t0, (B['z'] - B['z'][0]) * 100, label='박스 Δz [cm]')
    ax[0].set_ylabel('F_cmd,y [N] (내력)'); ax[1].set_ylabel('손 z 오차 [mm]'); ax[2].set_ylabel('[cm]'); ax[2].set_xlabel('t [s]')
    for x in ax:
        for name, t in ev:
            x.axvline(t - t0, color='gray', lw=0.5)
        x.legend(fontsize=7)
    for name, t in ev:
        ax[0].text(t - t0, ax[0].get_ylim()[1] * 0.85, name.split()[0], fontsize=8)
    out = a.log.replace('_box.csv', '.png')
    fig.tight_layout(); fig.savefig(out, dpi=120); print('그래프:', out)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('kind', choices=['t12', 't4', 't5', 'box', 'carry'])
    ap.add_argument('log')
    ap.add_argument('box_log', nargs='?')
    ap.add_argument('--side', default='r')
    ap.add_argument('--t_on', type=float); ap.add_argument('--t_off', type=float)
    ap.add_argument('--F', type=float, nargs=3)
    ap.add_argument('--t_step', type=float); ap.add_argument('--t_end', type=float); ap.add_argument('--axis', default='y')
    ap.add_argument('--t0', type=float, help='t12: 이 시각(로그 t)부터 처짐 계산')
    a = ap.parse_args()
    {'t12': t12, 't4': t4, 't5': t5, 'box': box, 'carry': carry}[a.kind](a)
