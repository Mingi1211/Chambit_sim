#!/usr/bin/env bash
# 곡선 운반 1회 (시뮬 새로 띄움 → 베이스 노드·임피던스 노드·손 힘 노드·기록 → 파지 → 운반 → 정리). 김민기 11/23·24
# 사용: run1.bash <tag> "<K_follower x y z>" <fric:on|off> <rec:none|rviz|gazebo> ["<K_leader x y z>"] [베이스 파라미터 yaml]
# 출력: $OUT (기본 real_logs/1124_final)/<tag>_*
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
OUT=${OUT:-$ROOT/real_logs/1124_final}
mkdir -p "$OUT"
tag=$1; KF=$2; fric=$3; recmode=$4; KL=${5:-"500 500 400"}; BASEP=${6:-}
CLEAN=(env -i HOME="$HOME" USER="$USER" PATH=/usr/bin:/bin DISPLAY=:0 XAUTHORITY="$XAUTHORITY" XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" QT_QPA_PLATFORM=xcb setsid bash --norc -c)
P="cd $ROOT && source scripts/environment.bash >/dev/null 2>&1 && cd mingi_control &&"
FR=""
if [[ $fric == on ]]; then
  # robots.yaml joint_friction (11/17 실측, 플랜트와 같은 값) → MR (11.65) η̃의 마찰 몫, 측정 속도 기준
  FR="-p fric_ff:=true -p fric_src:=${FRIC_SRC:-both} -p fric_c_r:=[6.387,5.115,7.869,4.29,3.157,3.047,1.091] -p fric_b_r:=[5.881,4.119,6.511,2.691,1.208,4.15,0.0] -p fric_c_l:=[6.419,5.667,8.272,3.215,3.298,3.175,0.762] -p fric_b_l:=[8.973,4.811,1.268,3.13,2.531,3.436,0.095]"
fi
pids=()
cleanup() {
  for p in "${pids[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 6
  for p in "${pids[@]}"; do kill -KILL -- -"$p" 2>/dev/null; done
}
echo "=== $tag start $(date +%T) KF=[$KF] KL=[$KL] fric=$fric rec=$recmode base=${BASEP:-기존}"
cd "$ROOT"
docker compose -f docker/compose.yaml down >/dev/null 2>&1
gui=false; [[ $recmode == gazebo ]] && gui=true && DISPLAY=:0 xhost +si:localuser:root >/dev/null
DISPLAY=:0 COOP_GUI=$gui docker compose -f docker/compose.yaml up -d --no-build >/dev/null 2>&1 || { echo up failed; exit 1; }
win=-; crop=""
# CAM: 카메라 자세 (기본 천장 시점, 11/25 직선 풀 녹화는 옆 시점)
CAMP=${CAM:-"position: {x: 0.0, y: 1.0, z: 3.6}, orientation: {w: 0.5004, x: -0.4996, y: 0.4996, z: 0.5004}"}
if [[ $recmode == gazebo ]]; then
  for k in $(seq 60); do win=$(DISPLAY=:0 xwininfo -root -tree 2>/dev/null | awk '/"gz-sim-gui" "Gazebo GUI"/{print $1; exit}'); [[ -n $win ]] && break; sleep 2; done
  sleep 15
  docker exec internship-gazebo-1 bash -c 'source /opt/ros/jazzy/setup.bash; gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 --req "pose: {'"$CAMP"'}"' >/dev/null
  crop="crop=582:796:0:48"
fi
"${CLEAN[@]}" "$P exec ros2 launch coop_carry_bringup base_move_curve.launch.py ${BASEP:+params:=$BASEP}" > "$OUT/${tag}_basenode.log" 2>&1 & pids+=($!)
"${CLEAN[@]}" "$P exec python3 impedance_node.py --ros-args -r __ns:=/robot1 -p robot:=robot1 -p sides:=[r,l] -p mode:=position $FR" > "$OUT/${tag}_imp1.log" 2>&1 & pids+=($!)
"${CLEAN[@]}" "$P exec python3 impedance_node.py --ros-args -r __ns:=/robot2 -p robot:=robot2 -p sides:=[r,l] -p mode:=position $FR" > "$OUT/${tag}_imp2.log" 2>&1 & pids+=($!)
"${CLEAN[@]}" "$P exec python3 viz/hand_wrench_node.py" > "$OUT/${tag}_hw.log" 2>&1 & pids+=($!)
if [[ $recmode == rviz ]]; then
  "${CLEAN[@]}" "cd $ROOT && source scripts/environment.bash >/dev/null 2>&1 && exec rviz2 -d mingi_control/viz/hand_wrench_rec.rviz" > "$OUT/${tag}_rviz.log" 2>&1 & pids+=($!)
  for k in $(seq 30); do win=$(DISPLAY=:0 xwininfo -root -tree 2>/dev/null | awk '/hand_wrench_rec.rviz/ && /RViz/{print $1; exit}'); [[ -n $win ]] && break; sleep 2; done
  echo "rviz window $win"
fi
sleep 12
# FULLREC=1: 파지 시작 ~ 정리까지 창 전체 녹화 + 벽시계↔시뮬 시간 기록 (11/25, 시뮬 시간 실속도 영상용)
if [[ ${FULLREC:-0} == 1 && $win != - ]]; then
  "${CLEAN[@]}" "$P exec python3 stiffness/run/clock_log.py $OUT/${tag}_clock.csv" > /dev/null 2>&1 & pids+=($!)
  date +%s.%N > "$OUT/${tag}_full_start.txt"
  ffmpeg -y -loglevel error -f x11grab -window_id $win -framerate 10 -i :0 -vf "$crop,crop=trunc(iw/2)*2:trunc(ih/2)*2" \
    -c:v libx264 -pix_fmt yuv420p "$OUT/${tag}_full_raw.mp4" < /dev/null > "$OUT/${tag}_ffmpeg.log" 2>&1 & recpid=$!
fi
"${CLEAN[@]}" "$P timeout -k 30 1500 python3 grasp_sequence.py" > "$OUT/${tag}_grasp.log" 2>&1
if grep -q "완료: 임피던스" "$OUT/${tag}_grasp.log"; then
  "${CLEAN[@]}" "$P exec python3 stiffness/run/curve_rec.py $OUT/$tag $([[ ${FULLREC:-0} == 1 ]] && echo - || echo $win) '$crop'" > "$OUT/${tag}_rec.log" 2>&1 & pids+=($!)
  sleep 3
  "${CLEAN[@]}" "$P timeout -k 30 2400 python3 transport_scenario.py --K_follower $KF --K_leader $KL" > "$OUT/${tag}_transport.log" 2>&1
  box=$(grep -o '/[^ ]*_carry_box.csv' "$OUT/${tag}_transport.log" | tail -1)
  [[ -n $box ]] && cp "$box" "$OUT/${tag}_box.csv" && cp "${box%_box.csv}_events.csv" "$OUT/${tag}_events.csv"
  for r in robot1 robot2; do t=$(ls -t ~/impedance_logs/*_task_$r.csv | head -1); cp "$t" "$OUT/${tag}_task_$r.csv"; done
  grep -E "박스 시작|판정" "$OUT/${tag}_transport.log"
  grep -o "정렬 [^→]*" "$OUT/${tag}_basenode.log" | head -1
  grep -o "추종 무효.*" "$OUT/${tag}_basenode.log" | head -1
else
  echo "$tag grasp failed"; tail -3 "$OUT/${tag}_grasp.log"
fi
if [[ -n ${recpid:-} ]]; then kill -INT $recpid; wait $recpid; date +%s.%N > "$OUT/${tag}_full_end.txt"; fi
cleanup
echo "=== $tag end $(date +%T)"
