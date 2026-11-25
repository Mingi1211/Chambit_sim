#!/usr/bin/env bash
# 최종 강성 풀 녹화 (11/25): V1 곡선 (천장 시점), V2 직선 (옆 시점, 박스 들림 확인용)
HERE=$(cd "$(dirname "$0")" && pwd)
export FULLREC=1
CAM= bash "$HERE/queue.bash" "$HERE/q_fullrec_curve.txt"
CAM="position: {x: 0.0, y: -1.9, z: 1.25}, orientation: {w: 0.7054, x: -0.0484, y: 0.0484, z: 0.7054}" bash "$HERE/queue.bash" "$HERE/q_fullrec_straight.txt"
