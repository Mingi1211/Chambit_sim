#!/usr/bin/env python3
"""두 조건의 곡선 주행 구간을 나란히 보여주는 GIF — 위: RViz 녹화(손 힘 파랑, 내력 빨강), 아래: 같은 시뮬 시각의 손 내력. 김민기 (11/23).

시간축: 베이스 10 s 패턴의 시뮬 시간 (두 조건을 같은 주행 시점끼리 맞춤). 녹화는 MOVING 시작(벽시계)부터 → hand.csv의 (벽시계, 시뮬) 짝으로 변환.
내력: /coop/hand_state의 손별 ‖f_int‖ (I − G†G)h, 4손 × 1 s 창의 중앙값 (F/T 원신호 접촉 충격 스파이크가 평균을 끌어올림; 창 1 s = RViz 표시 TAU_VIEW)
사용: python3 compare_gif.py --dir ../../real_logs/1123_curve_trials --tags A B --labels "라벨A" "라벨B" --out x.gif
"""
import argparse
import json
import os
import subprocess
import tempfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import font_manager  # noqa: E402

for f in ('/usr/share/fonts/truetype/nanum/NanumGothic.ttf', '/usr/share/fonts/truetype/nanum/NanumSquare_acR.ttf'):
    if os.path.exists(f):
        font_manager.fontManager.addfont(f)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=f).get_name()
        break
plt.rcParams['axes.unicode_minus'] = False

ap = argparse.ArgumentParser()
ap.add_argument('--dir', required=True)
ap.add_argument('--tags', nargs='+', required=True)
ap.add_argument('--labels', nargs='+', required=True)
ap.add_argument('--out', required=True)
ap.add_argument('--crop', default='1020:810:378:110', help='RViz 3D 화면 영역 (w:h:x:y)')
ap.add_argument('--drop', type=float, nargs='*', default=[], help='조건별 박스 낙하 시각 [s] (없으면 -1): 그 뒤 내력은 손이 빈 상태라 회색 처리')
ap.add_argument('--fps', type=float, default=8)
ap.add_argument('--secs', type=float, default=14, help='GIF 길이 [s]')
ap.add_argument('--hold', type=float, default=3.0, help='DONE 뒤 더 보여줄 시뮬 시간 [s]')
a = ap.parse_args()


def load(tag):
    pre = os.path.join(a.dir, tag)
    st = [json.loads(line) for line in open(pre + '_status.jsonl')]
    t0w = next(d['t_wall'] for d in st if d['state'] == 'MOVING')
    H = []
    for line in open(pre + '_hand.csv'):
        r = line.rstrip().split(',')
        if len(r) == 43:
            H.append([float(r[0]), float(r[1])] + [float(v) for v in r[27:39]])
    H = np.array(H)
    tw, ts = H[:, 0], H[:, 1]
    ts0 = np.interp(t0w, tw, ts)
    nint = np.linalg.norm(H[:, 2:].reshape(-1, 4, 3), axis=2)
    tsr = ts - ts0
    # 1 s 이동 중앙값 (4손 × 1 s 창, 시뮬 시간 기준) — F/T 원신호의 접촉 충격 스파이크가 평균을 끌어올려 중앙값 사용
    j0 = np.searchsorted(tsr, tsr - 1.0)
    avg = np.array([np.median(nint[j:i + 1]) for i, j in enumerate(j0)])
    return dict(pre=pre, t0w=t0w, tw=tw, tsr=tsr, avg=avg)


runs = [load(t) for t in a.tags]
T_end = 10.0 + a.hold
n = int(a.secs * a.fps)
tq = np.linspace(0, T_end, n)
tmp = tempfile.mkdtemp()
frames = []
for r in runs:                       # 시뮬 시각 tq → 벽시계 → 영상 시각 → 프레임 추출
    m = r['tsr'] >= -0.5
    vt = np.interp(tq, r['tsr'][m], r['tw'][m]) - r['t0w']
    dur = float(subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0',
                                r['pre'] + '_raw.mp4'], capture_output=True, text=True).stdout)
    vt = np.clip(vt, 0, dur - 0.3)
    imgs = []
    for k, t in enumerate(vt):
        p = os.path.join(tmp, f'{os.path.basename(r["pre"])}_{k:04d}.png')
        subprocess.run(['ffmpeg', '-loglevel', 'error', '-y', '-ss', f'{t:.2f}', '-i', r['pre'] + '_raw.mp4',
                        '-frames:v', '1', '-vf', f'crop={a.crop},scale=510:-1', p], check=True)
        imgs.append(p)
    frames.append(imgs)

ymax = max(np.nanmax(r['avg'][(r['tsr'] >= 0) & (r['tsr'] <= T_end)]) for r in runs) * 1.15
out_frames = []
for k, t in enumerate(tq):
    fig = plt.figure(figsize=(5.1 * len(runs), 6.4), dpi=100)
    gs = fig.add_gridspec(2, len(runs), height_ratios=[4, 1.6], hspace=0.12, wspace=0.04)
    for i, r in enumerate(runs):
        ax = fig.add_subplot(gs[0, i]); ax.imshow(plt.imread(frames[i][k])); ax.axis('off')
        mm = (r['tsr'] >= 0) & (r['tsr'] <= T_end)
        v = float(np.interp(t, r['tsr'], r['avg']))
        td0 = a.drop[i] if i < len(a.drop) else -1
        state = '박스 낙하 — 손이 빈 상태' if 0 <= td0 <= t else f'손 내력 (1 s 중앙값) {v:5.1f} N'
        ax.set_title(f'{a.labels[i]}\n{state}', fontsize=12, color='tab:red' if 0 <= td0 <= t else 'k')
        bx = fig.add_subplot(gs[1, i])
        bx.plot(r['tsr'][mm], r['avg'][mm], color='tab:red', lw=1.2)
        bx.axvline(t, color='k', lw=0.8)
        bx.axvspan(10.0, T_end, color='0.9')
        td = a.drop[i] if i < len(a.drop) else -1
        if td >= 0:
            bx.axvspan(td, T_end, color='tab:red', alpha=0.15)
            bx.axvline(td, color='tab:red', lw=1.2)
            bx.text(td + 0.15, ymax * 0.85, '박스 낙하', color='tab:red', fontsize=9)
        bx.set_xlim(0, T_end); bx.set_ylim(0, ymax)
        bx.set_xlabel('베이스 패턴 시뮬 시간 [s] (회색 = 정지 후)', fontsize=9)
        bx.axhline(15, color='0.4', ls='--', lw=0.8)
        if i == 0:
            bx.set_ylabel('손 내력 [N]', fontsize=9)
        else:
            bx.set_yticklabels([])
    p = os.path.join(tmp, f'out_{k:04d}.png')
    fig.savefig(p, bbox_inches='tight'); plt.close(fig)
    out_frames.append(p)
subprocess.run(['ffmpeg', '-loglevel', 'error', '-y', '-framerate', str(a.fps), '-i', os.path.join(tmp, 'out_%04d.png'),
                '-vf', 'scale=900:-2:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer',
                a.out], check=True)
print('저장:', a.out)
