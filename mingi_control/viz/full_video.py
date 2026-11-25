#!/usr/bin/env python3
"""풀 녹화 원본(벽시계, 시뮬 RTF 3 ~ 13 %) → 시뮬 시간 기준 영상 + 단계 자막. 김민기 (11/25)
입력 (run1.bash FULLREC=1): <tag>_full_raw.mp4, _full_start.txt, _clock.csv, _grasp.log, _events.csv, _box.csv
자막: 오른쪽 위 단계 이름 + 배속 (grasp_sequence / transport_scenario 로그) — 힘 시각화 없음
배속 (시뮬 시간 기준): 들기·주행·내려놓기 ×0.5, 접근·파지·대기 등 움직임 적은 구간 ×2
사용: python3 full_video.py <데이터 폴더> <tag> <출력.mp4> [--max 최대 길이 s]
"""
import argparse
import datetime
import json
import os
import re

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT = '/usr/share/fonts/truetype/nanum/NanumSquare_acB.ttf'
if not os.path.exists(FONT):
    FONT = '/usr/share/fonts/truetype/nanum/NanumSquare_acR.ttf'
GRASP = [('1 ', '파지: 그리퍼 열기'), ('2a', '파지: 손잡이 위로 이동'), ('2b', '파지: 손 방향 맞추기'),
         ('2c', '파지: 손잡이 높이로 하강'), ('3 ', '파지: 손잡이로 전진'), ('4 ', '파지: 그리퍼 닫기'),
         ('5 ', '파지 완료 → 임피던스 모드')]
EVENTS = [('1 강성', '임피던스 강성 설정'), ('2 무게', '무게 보상 켜기'), ('3 들기', '박스 들기 +5 cm'),
          ('4 베이스', '베이스: 바퀴 방향 정렬'), ('베이스 도착', '베이스 도착'), ('팔로워 강성', '팔로워 강성 → 리더 값'),
          ('5 내려놓기', '내려놓기'), ('마지막 1 cm', '내려놓기: 마지막 1 cm'), ('피드포워드 끄기', '무게를 받침대로'),
          ('그리퍼 열기', '그리퍼 열기'), ('6 위치', '위치 제어로 복귀'), ('7 팔', '팔 빼기'), ('정리', '완료')]


def phases(d, tag, t0):
    out = []
    day = datetime.datetime.fromtimestamp(t0).date()
    for line in open(os.path.join(d, tag + '_grasp.log')):
        m = re.match(r'\[(\d\d):(\d\d):(\d\d)\] (.*)', line)
        if not m:
            continue
        for k, name in GRASP:
            if m.group(4).startswith(k):
                t = datetime.datetime.combine(day, datetime.time(*map(int, m.groups()[:3]))).timestamp()
                out.append((t, name))
    ev = os.path.join(d, tag + '_events.csv')
    for line in (open(ev).readlines()[1:] if os.path.exists(ev) else []):
        name, t = line.rsplit(',', 1)
        for k, lab in EVENTS:
            if k in name:
                out.append((float(t), lab)); break
    st = os.path.join(d, tag + '_status.jsonl')              # 베이스 노드 MOVING = 정렬 끝, 주행 패턴 시작
    for line in (open(st) if os.path.exists(st) else []):
        m = json.loads(line)
        if m.get('state') == 'MOVING':
            out.append((m['t_wall'], '베이스 주행'))
    return sorted(out)


# 시뮬 시간 배속: 들기·주행·내려놓기·놓기는 ×0.5 (느리게), 나머지(접근·파지·대기·팔 빼기)는 ×FAST
# (시뮬 시간으로 전체 동작이 약 85 s라 이렇게 하면 약 1.5분; --max 초를 넘으면 전체를 같은 비율로 빠르게)
KEY = {'박스 들기 +5 cm': 0.5, '베이스 주행': 0.5, '내려놓기': 0.5, '내려놓기: 마지막 1 cm': 0.5,
       '무게를 받침대로': 0.5, '그리퍼 열기': 0.5}
FAST = 2.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('d'); ap.add_argument('tag'); ap.add_argument('out')
    ap.add_argument('--max', type=float, default=170.0, help='출력 최대 길이 [s]')
    ap.add_argument('--fps', type=float, default=10.0)
    a = ap.parse_args()
    d, tag = a.d, a.tag
    t0 = float(open(os.path.join(d, tag + '_full_start.txt')).read())
    ck = np.genfromtxt(os.path.join(d, tag + '_clock.csv'), delimiter=',', names=True)
    ph = phases(d, tag, t0)
    raw = os.path.abspath(os.path.join(d, tag + '_full_raw.mp4'))
    cap = cv2.VideoCapture(raw, cv2.CAP_FFMPEG)                              # 1차: 프레임 수만 (메모리 아끼려고 두 번 읽음)
    n = 0
    while cap.grab():
        n += 1
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    tw = t0 + np.arange(n) / cap.get(cv2.CAP_PROP_FPS)     # x11grab 고정 10 fps (벽시계)
    ts = np.interp(tw, ck['t_wall'], ck['t_sim'])          # 프레임별 시뮬 시간
    ts_s, ts_e = ts[0], ts[-1]
    # 단계 구간 (시뮬 시간)
    seg = [(ts_s, '')] + [(float(np.interp(t, ck['t_wall'], ck['t_sim'])), n) for t, n in ph]
    seg = [(max(t, ts_s), n) for t, n in seg] + [(ts_e, None)]
    total = sum((seg[j + 1][0] - seg[j][0]) / KEY.get(seg[j][1], FAST) for j in range(len(seg) - 1))
    c = max(1.0, total / a.max)
    cap = cv2.VideoCapture(raw, cv2.CAP_FFMPEG); k, cur = -1, None
    vw = cv2.VideoWriter(a.out + '.tmp.mp4', cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (w, h))
    font = ImageFont.truetype(FONT, 15)
    s, j, nout = ts_s, 0, 0
    while s < ts_e:
        while seg[j + 1][0] <= s:
            j += 1
        lab = seg[j][1]
        sp = KEY.get(lab, FAST) * c
        i = min(np.searchsorted(ts, s), len(tw) - 1)
        while k < i:
            ok, f = cap.read(); k += 1
            if ok:
                cur = f
        img = Image.fromarray(cv2.cvtColor(cur, cv2.COLOR_BGR2RGB))
        dr = ImageDraw.Draw(img, 'RGBA')
        txt = (lab or '준비') + f'  ×{sp:g}'
        x0, y0, x1, y1 = dr.textbbox((0, 0), txt, font=font)
        X, Y = w - (x1 - x0) - 14, 8
        dr.rectangle((X - 6, Y - 4, X + (x1 - x0) + 6, Y + (y1 - y0) + 8), fill=(255, 255, 255, 190))
        dr.text((X, Y), txt, font=font, fill=(25, 25, 25))
        vw.write(cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)); nout += 1
        s += sp / a.fps
    vw.release()
    os.system(f'ffmpeg -v error -y -i "{a.out}.tmp.mp4" -c:v libx264 -pix_fmt yuv420p -crf 23 -movflags +faststart "{a.out}" && rm "{a.out}.tmp.mp4"')
    print(a.out, f'출력 {nout / a.fps:.0f} s (배율 {c:.2f}), 시뮬 {ts_e - ts_s:.0f} s, 원본 {tw[-1] - tw[0]:.0f} s')


if __name__ == '__main__':
    main()
