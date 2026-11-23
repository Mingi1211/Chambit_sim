#!/usr/bin/env python3
"""11/24 최종 재측정 요약표 — 조건별 런 중앙값(최소 ~ 최대). 김민기.
대표값(전달자료 P0-3): 4손 내력의 1 s 이동 중앙값 → 런 안 중앙값(med)·최대(peak) → 조건별 런 중앙값과 범위
박스: 곡선 주행 중(패턴 0 ~ 10 s) 기울기 최대, 낙하(5° 초과) 여부, 내려놓기 판정(transport 로그)"""
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from curve_analysis import analyze, base_check, force_metrics  # noqa: E402

D = os.path.join(HERE, '../../real_logs/1124_final')
D2 = os.path.join(D, '_reanchor_rejected')   # 내려놓기 x·y 재설정 시도 런(되돌림) — 주행 구간은 최종 코드와 같아 주행 지표에만 사용
GROUPS = [('A 기준: K_F 40, 보상 없음', ['A?_K40_off']), ('H 보상: K_F 40, 보상(계획+측정)', ['H?_K40_both']),
          ('C 재산정: K_F (110, 510), 보상', ['C?_K110_both', 'C??_K110_both']), ('D 과대: K_F 500, 보상', ['D?_K500_both']),
          ('E 0.2 m: K_F (110, 510)', ['E?_s020_K110_both']), ('E 0.2 m: K_F (30, 510)', ['E?_s020_K30_both', 'E??_s020_K30_both', 'G2*_s020_K30_both']),
          ('E 0.2 m: K_F 500', ['E?_s020_K500_both']), ('G Gazebo 영상 0.0575 m', ['G1_gz_s0575_K110_both'])]


def judge(d, tag):
    if d == D2:
        return '제외(재설정 시도)'
    t = open(os.path.join(d, tag + '_transport.log')).read()
    m = re.search(r'판정: (.*)', t)
    return ('성공' if '성공' in m.group(1) else m.group(1).replace('확인 필요 — ', '✖ ')) if m else '?'


def rng(v, f='{:.1f}'):
    v = np.array(v, float)
    return f'{f.format(np.median(v))} ({f.format(v.min())} ~ {f.format(v.max())})' if len(v) > 1 else f.format(v[0])


print('| 조건 | 런 | 베이스 합격 | Δx 최대 [mm] | 손 내력 중앙값 [N] | 손 내력 1 s 최대 [N] | 주행 중 박스 기울기 최대 [°] | 주행 중 낙하 | 내려놓기 |')
print('|---|---|---|---|---|---|---|---|---|')
for name, pat in GROUPS:
    tags = sorted({(d, os.path.basename(p)[:-len('_status.jsonl')]) for d in (D, D2) for pt in pat
                   for p in glob.glob(os.path.join(d, pt + '_status.jsonl'))}, key=lambda x: (x[0] == D2, x[1]))
    if not tags:
        continue
    rows = []
    for d, t in tags:
        r = analyze(d, t); c = base_check(r); fm = force_metrics(r)
        b = r['base']; B = r['box']
        tsb = np.interp(B['t'], b['t'], b['ts']) - b['ts'][b['i0']]
        m = (tsb >= 0) & (tsb <= 10.0)
        ok = c['dstart'] < 0.2 and c['final_err'] < 0.05 and c['dy_max'] < 0.05 and not c['invalid']
        rows.append(dict(t=t, ok=ok, dx=c['dx_max'] * 1e3, med=fm['med'], peak=fm['peak'],
                         tilt=B['tilt'][m].max(), drop=B['t_drop'], j=judge(d, t)))
    print(f"| {name} | {len(rows)} ({', '.join(x['t'].split('_')[0] for x in rows)}) | {sum(x['ok'] for x in rows)}/{len(rows)} | "
          f"{rng([x['dx'] for x in rows], '{:.0f}')} | {rng([x['med'] for x in rows])} | {rng([x['peak'] for x in rows])} | "
          f"{rng([min(x['tilt'], 90) for x in rows])} | {sum(x['drop'] is not None for x in rows)}/{len(rows)} | "
          f"{sum(x['j'] == '성공' for x in rows)}/{sum(not x['j'].startswith('제외') for x in rows)} 성공 |")
    for x in rows:
        print(f"|  └ {x['t']} | | {'○' if x['ok'] else '✖'} | {x['dx']:.0f} | {x['med']:.1f} | {x['peak']:.1f} | {min(x['tilt'], 90):.1f} | "
              f"{'%.1f s' % x['drop'] if x['drop'] is not None else '-'} | {x['j']} |")
