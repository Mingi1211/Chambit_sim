#!/usr/bin/env bash
# 최종 강성 곡선 풀 녹화 (11/25): V5 (V3 카메라 오프셋 미적용, V4 베이스 피드백 시간 초과로 폐기), 팔로워 robot2 뒤를 따라가는 카메라 — 팔로워 팔이 리더 쪽으로 뻗는 모습 확인용
HERE=$(cd "$(dirname "$0")" && pwd)
FULLREC=1 CAMFOLLOW=robot2 CAMOFF="x: 0.1, y: -1.5, z: 1.4" bash "$HERE/queue.bash" "$HERE/q_fullrec_follow.txt"
