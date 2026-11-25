#!/usr/bin/env python3
"""최종 설계값 검증표 — MR 부등식 항목 순서대로. 김민기 (11/24).
설계: 리더 K_L = MR 하한 (531, 180, 837) [K_L ≥ F_dist/e_allow], 팔로워 K_F = (40, 40, 837) [K_F ≤ 406, 마찰 원뿔]
판정 항목
  ① 베이스 합격 (출발 시각 차 < 0.2 s, 최종 위치 오차 < 0.05 m, Δy < 0.05 m, 추종 무효 없음 — 전달자료)
  ② 곡선 주행(패턴 0 ~ 10 s) 중 손별 수평 내력 (월드 x·y, (I − G†G)h) 1 s 중앙값의 최대 ≤ F_cone 36.4 N (MR 12.2.1)
  ③ 리더 손 오차 최대 ≤ 허용 오차 x 131, y 181 mm (받침대 반폭 − 베이스 최종 오차), 놓을 때 낙하 ≤ 10 mm (z 허용 = 마지막 단계)
  ④ 박스: 주행 중 낙하(높이 < 0.5 m) 없음, 기울기
  ⑤ 내려놓기 판정 (transport_scenario)
  파지 멈춤 q: 손가락이 손잡이에 닿아 멈춘 각 (설계 접촉각 0.855 = 손잡이 두께 30 mm, GAIN_RATIONALE ⑨). 제각각이면 모서리 접촉 의심
  보상 후 정지 중 남는 손 내력 F_s (출발 전 20 s, 성분 중앙값) — 팔로워 상한 식의 입력
사용: python3 final_check.py [--out 표.md]
"""
import argparse
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from curve_analysis import analyze, base_check, rolling_median  # noqa: E402

D = os.path.join(HERE, '../../real_logs/1124_final')
CONE, EX, EY, EZ = 36.4, 0.131, 0.181, 0.010
GROUPS = [('최종 설계 (보상 있음)', ['L1', 'L2', 'L3', 'L5', 'L6', 'L7', 'L8']),
          ('최종 강성, 보상 없음', ['M1', 'M2', 'M3']),
          ('팔로워 상한 초과 K_F 500 (보상 있음)', ['P1', 'P2']),
          ('최종 설계 직선 주행 (옆 이탈 0)', ['S1']),
          ('풀 녹화 (곡선 V1 / 직선 V2)', ['V1', 'V2'])]


def judge(tag):
    p = os.path.join(D, tag + '_transport.log')
    t = open(p).read() if os.path.exists(p) else ''
    m = re.search(r'판정: (.*)', t)
    d = re.search(r'놓을 때 낙하 (-?\d+) mm', t)
    return (('성공' if '성공' in m.group(1) else m.group(1).replace('확인 필요 — ', '✖ ')) if m else '판정 없음',
            int(d.group(1)) if d else None)


def grasp_q(tag):
    p = os.path.join(D, tag + '_grasp.log')
    q = [float(x) for x in re.findall(r'멈춤 q=([0-9.]+)', open(p).read())][-4:] if os.path.exists(p) else []
    return q


def row(tag):
    r = analyze(D, tag); c = base_check(r); b = r['base']; h = r['hand']; B = r['box']
    ok = c['dstart'] < 0.2 and c['final_err'] < 0.05 and c['dy_max'] < 0.05 and not c['invalid']
    tsh = np.interp(h['t'], b['t'], b['ts']) - b['ts'][b['i0']]
    te = B['t_drop'] if B['t_drop'] is not None else 10.0
    # 1 s 창이 늘 꽉 차게 패턴 1 s 전(박스 든 채 정지)부터 이동 중앙값을 구하고 0 ~ te만 봄 (11/25: 0 s 첫 샘플 경계 오류 수정)
    mw = (tsh >= -1.0) & (tsh <= te); m = (tsh[mw] >= 0)
    fxy = np.hypot(h['Fint'][mw][:, :, 0], h['Fint'][mw][:, :, 1])
    per = [rolling_median(tsh[mw], fxy[:, i])[m].max() for i in range(4)]
    pre = (h['t'] > r['t0'] - 20) & (h['t'] < r['t0'])
    fs = np.median(np.abs(h['Fint'][pre]), axis=(0, 1))
    tsb = np.interp(B['t'], b['t'], b['ts']) - b['ts'][b['i0']]
    tilt = B['tilt'][(tsb >= 0) & (tsb <= 10)].max()
    L = r['leader']
    ex = max(L['r'][0], L['l'][0]); ey = max(L['r'][1], L['l'][1])
    j, drop = judge(tag)
    q = grasp_q(tag)
    return dict(tag=tag, q=q, ok=ok, dx=c['dx_max'], dy=c['dy_max'], fmax=max(per), per=per, fs=fs, tilt=tilt,
                fall=B['t_drop'], ex=ex, ey=ey, drop=drop, j=j)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--out', default='')
    a = ap.parse_args()
    out = ['| 조건 | 런 | ① 베이스 | Δx / Δy 최대 [mm] | ② 손별 수평 내력 최대 [N] (≤ 36.4) | ③ 리더 오차 x / y [mm] (≤ 131 / 181) | ③ 놓을 때 낙하 [mm] (≤ 10) | ④ 주행 중 기울기 / 낙하 | ⑤ 내려놓기 | 정지 중 F_s x / y [N] | 파지 멈춤 q [rad] (설계 접촉 0.855) |',
           '|---|---|---|---|---|---|---|---|---|---|---|']
    for name, tags in GROUPS:
        for t in tags:
            if not os.path.exists(os.path.join(D, t + '_status.jsonl')):
                continue
            try:
                x = row(t)
            except Exception as e:  # noqa: BLE001
                out.append(f'| {name} | {t} | 분석 불가: {e} |'); continue
            out.append(f"| {name} | {t} | {'○' if x['ok'] else '✖'} | {x['dx']*1e3:.0f} / {x['dy']*1e3:.1f} | "
                       f"{x['fmax']:.1f} {'○' if x['fmax'] <= CONE else '✖'} ({' '.join(f'{v:.1f}' for v in x['per'])}) | "
                       f"{x['ex']*1e3:.0f} / {x['ey']*1e3:.0f} {'○' if x['ex'] <= EX and x['ey'] <= EY else '✖'} | "
                       f"{x['drop'] if x['drop'] is not None else '-'} | {x['tilt']:.1f}° / {'%.1f s' % x['fall'] if x['fall'] is not None else '없음'} | "
                       f"{x['j']} | {x['fs'][0]:.1f} / {x['fs'][1]:.1f} | {' '.join(f'{v:.2f}' for v in x['q'])} |")
    s = '\n'.join(out)
    print(s)
    if a.out:
        open(a.out, 'w').write(s + '\n')
