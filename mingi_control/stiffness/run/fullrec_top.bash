#!/usr/bin/env bash
# 최종 강성 곡선 풀 녹화 (11/25): V6, 박스를 위에서 낮게 따라가는 카메라 — 팔로워 팔이 리더 쪽으로 뻗는 모습 (화면 배율 고정)
HERE=$(cd "$(dirname "$0")" && pwd)
FULLREC=1 CAMFOLLOW=box CAMOFF="x: 0.0, y: -0.05, z: 1.7" bash "$HERE/queue.bash" "$HERE/q_fullrec_top.txt"
