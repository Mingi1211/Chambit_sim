#!/usr/bin/env python3
"""손 힘 떨림 스펙트럼 — 마찰 보상(측정 속도 sgn) 유무 비교. 김민기 (11/24).
구간: 박스를 든 채 베이스 바퀴 정렬 중(ALIGNING = 상태 ARMED, 베이스 정지) — 팔 외란이 가장 적은 정지 파지 구간.
신호: /robotN/hand_wrench_{r,l} 손 → 박스 힘 (F/T 시뮬 시각 스탬프), 4손 ‖f‖에서 평균을 빼고 시뮬 시간 균일 재표본 → 해닝 창 FFT.
지표: RMS는 접촉 충격 스파이크가 좌우 → 연속 떨림은 로버스트 σ = 1.4826·MAD, 스파이크는 10σ 넘는 표본 비율로 따로
사용: python3 hand_spectrum.py --dir ../../real_logs/1124_final --tags A1_K40_off B1_K40_on --out spectrum.png
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

f = '/usr/share/fonts/truetype/nanum/NanumGothic.ttf'
font_manager.fontManager.addfont(f)
plt.rcParams['font.family'] = font_manager.FontProperties(fname=f).get_name()
plt.rcParams['axes.unicode_minus'] = False


def load(d, tag, state='ARMED'):
    rows = [r for r in csv.DictReader(open(os.path.join(d, tag + '_wrench.csv'))) if r['state'] == state]
    by = {}
    for r in rows:
        by.setdefault(float(r['t_sim']), {})[r['hand']] = np.linalg.norm([float(r['fx']), float(r['fy']), float(r['fz'])])
    t = np.array(sorted(k for k, v in by.items() if len(v) == 4))
    x = np.array([np.mean(list(by[k].values())) for k in t])
    return t, x


def spectrum(t, x):
    dt = np.median(np.diff(t))
    g = np.arange(t[0], t[-1], dt)
    y = np.interp(g, t, x); y = y - y.mean()
    P = np.abs(np.fft.rfft(y * np.hanning(len(y)))) ** 2 / len(y)
    return np.fft.rfftfreq(len(y), dt), P, 1 / dt, y


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', required=True)
    ap.add_argument('--tags', nargs='+', required=True)
    ap.add_argument('--labels', nargs='*', default=[])
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    fig, ax = plt.subplots(figsize=(9, 4)) if a.out else (None, None)
    for i, tag in enumerate(a.tags):
        t, x = load(a.dir, tag)
        fr, P, fs, y = spectrum(t, x)
        m = fr > 0.5
        k = np.argsort(P[m])[-3:][::-1]
        mad = np.median(np.abs(y - np.median(y))) * 1.4826             # 로버스트 σ: 접촉 충격 스파이크에 영향 적음
        spike = float((np.abs(y) > 10 * mad).mean())
        print(f'{tag:14s} 구간 {t[-1] - t[0]:.1f} s (시뮬), 표본 {fs:.0f} Hz, RMS {y.std():.2f} N, 연속 떨림 σ(MAD) {mad:.2f} N, '
              f'스파이크(>10σ) {spike*100:.2f} %, 중앙값 {np.median(x):.1f} N, 피크 {", ".join(f"{fr[m][j]:.1f}" for j in k)} Hz')
        if ax is not None:
            ax.semilogy(fr[m], P[m], lw=1, label=(a.labels[i] if i < len(a.labels) else tag) + f' (RMS {y.std():.2f} N)')
    if ax is not None:
        ax.set_xlabel('주파수 [Hz] (시뮬 시간)'); ax.set_ylabel('손 힘 파워 스펙트럼')
        ax.set_title('박스 들고 정지 중 손 힘 떨림 (바퀴 정렬 구간)'); ax.grid(alpha=0.3, which='both'); ax.legend()
        fig.tight_layout(); fig.savefig(a.out, dpi=130); print('저장:', a.out)
