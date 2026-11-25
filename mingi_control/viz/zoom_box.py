#!/usr/bin/env python3
"""박스(주황)를 색으로 따라가며 오른쪽(팔로워 r2) 쪽을 2배 확대한 영상. 김민기 (11/25)
사용: python3 zoom_box.py <입력.mp4> <출력.mp4> [제목]"""
import os
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

src, out = sys.argv[1], sys.argv[2]
title = sys.argv[3] if len(sys.argv) > 3 else '확대 · 오른쪽 = 팔로워(r2)'
cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG); fr = []
while True:
    ok, f = cap.read()
    if not ok:
        break
    fr.append(f)
cs = []
for f in fr:                                                  # 박스 중심 = 주황 화소 중앙값 (윗 도구막대 제외)
    m = cv2.inRange(cv2.cvtColor(f, cv2.COLOR_BGR2HSV), (8, 80, 120), (25, 220, 255)); m[:110] = 0
    ys, xs = np.nonzero(m)
    cs.append((np.median(xs), np.median(ys)) if len(xs) > 200 else (np.nan, np.nan))
cs = np.array(cs, float)
idx = np.arange(len(cs))
for k in range(2):
    ok = ~np.isnan(cs[:, k]); cs[:, k] = np.interp(idx, idx[ok], cs[ok, k])
    cs[:, k] = np.convolve(np.pad(cs[:, k], 4, 'edge'), np.ones(9) / 9, 'valid')   # 0.9 s 이동 평균 (흔들림 줄임)
W, H, S = 320, 280, 2
font = ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumSquare_acB.ttf', 22)
vw = cv2.VideoWriter(out + '.tmp.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 10, (W * S, H * S))
h, w = fr[0].shape[:2]
for f, (cx, cy) in zip(fr, cs):
    x0 = int(np.clip(cx - 90, 0, w - W)); y0 = int(np.clip(cy - H / 2, 0, h - H))
    img = Image.fromarray(cv2.cvtColor(cv2.resize(f[y0:y0 + H, x0:x0 + W], (W * S, H * S), interpolation=cv2.INTER_CUBIC), cv2.COLOR_BGR2RGB))
    dr = ImageDraw.Draw(img, 'RGBA')
    bb = dr.textbbox((0, 0), title, font=font); X = W * S - (bb[2] - bb[0]) - 16
    dr.rectangle((X - 8, 6, W * S - 6, bb[3] - bb[1] + 20), fill=(255, 255, 255, 200)); dr.text((X, 10), title, font=font, fill=(25, 25, 25))
    vw.write(cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR))
vw.release()
os.system(f'ffmpeg -v error -y -i "{out}.tmp.mp4" -c:v libx264 -profile:v main -level 3.1 -pix_fmt yuv420p -crf 20 -movflags +faststart "{out}" && rm "{out}.tmp.mp4"')
