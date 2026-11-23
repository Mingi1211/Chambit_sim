#!/usr/bin/env bash
# 손끝 힘 시각화 실행 (깨끗한 셸에서): bash mingi_control/viz/run_viz.bash
cd "$(dirname "$0")/../.." && source scripts/environment.bash >/dev/null
python3 mingi_control/viz/hand_wrench_node.py &
NODE=$!
rviz2 -d mingi_control/viz/hand_wrench.rviz
kill $NODE
