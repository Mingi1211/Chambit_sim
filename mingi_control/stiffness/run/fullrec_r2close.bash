#!/usr/bin/env bash
# 최종 강성 곡선 풀 녹화 (11/25): V7, 팔로워 robot2 팔 바로 위에서 낮게 따라가는 카메라 — r2 팔이 리더 쪽으로 뻗는 모습 확대
HERE=$(cd "$(dirname "$0")" && pwd)
FULLREC=1 CAMFOLLOW=robot2 CAMOFF="x: -0.3, y: -0.05, z: 1.1" bash "$HERE/queue.bash" "$HERE/q_fullrec_r2close.txt"
