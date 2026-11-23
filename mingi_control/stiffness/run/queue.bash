#!/usr/bin/env bash
# 실행 목록을 차례로. 파지 실패면 같은 조건으로 최대 3번 다시. 김민기 11/23
# 줄 형식: tag|K_follower|fric|rec|K_leader|베이스 yaml   (# 시작 줄은 건너뜀)
HERE=$(cd "$(dirname "$0")" && pwd)
LOG=${LOG:-/tmp/curve_queue}
OUT_DIR=${OUT:-$(cd "$HERE/../../.." && pwd)/real_logs/1124_final}
mkdir -p "$LOG"
while IFS='|' read -r tag kf fric rec kl basep; do
  [[ -z $tag || $tag == \#* ]] && continue
  for att in 1 2 3; do
    bash "$HERE/run1.bash" "$tag" "$kf" "$fric" "$rec" "${kl:-500 500 400}" "$basep" > "$LOG/run_$tag.log" 2>&1
    cat "$LOG/run_$tag.log"
    # 파지 실패 또는 베이스 시작 거부(시뮬 직후 스워브 제어기 미활성)면 같은 조건으로 다시
    grep -q "grasp failed" "$LOG/run_$tag.log" || grep -q "베이스 시작 거부" "$OUT_DIR/${tag}_transport.log" 2>/dev/null || break
    echo "--- $tag 파지 실패/베이스 시작 거부, 재시도 $att"
  done
done < "$1"
cd "$HERE/../../.." && docker compose -f docker/compose.yaml down >/dev/null 2>&1
echo QUEUE_DONE
