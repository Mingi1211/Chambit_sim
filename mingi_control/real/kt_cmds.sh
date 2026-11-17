# 실물 Kt 측정 명령 줄임 (11/10) — 내 PC 터미널에서: source ~/Desktop/Chambit_Sim/mingi_control/real/kt_cmds.sh
#   pose <팔 l|r> "<q1..q7>"            기준 자세로 이동
#   vel  <팔> <관절> "<q1..q7>" "<lo hi>" <저장파일.csv>   등속 왕복 측정
#   grip <팔> "<q1..q7>" <0.0 열기|1.0 닫기>   팔은 그 자세 그대로, 그리퍼만
SC=~/Desktop/Chambit_Sim/mingi_control/real/static_current.py; ROB=robotis@192.168.6.2
pose(){ ssh $ROB "docker exec -i ai_worker bash -ic 'python3 - goto --arm $1 --base $2 --move 6'" < $SC; }
vel(){ ssh $ROB "docker exec -i ai_worker bash -ic 'python3 - vel --arm $1 --joint $2 --base $3 --range $4'" < $SC | sed -n '/^t,phase/,$p' > $5; wc -l $5; }
grip(){ local P=$(echo $2 | awk '{for(i=1;i<=NF;i++) printf "%s%.4f", (i>1?", ":""), $i}'); ssh $ROB "docker exec ai_worker bash -ic 'ros2 action send_goal /arm_$1_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{trajectory: {joint_names: [arm_$1_joint1, arm_$1_joint2, arm_$1_joint3, arm_$1_joint4, arm_$1_joint5, arm_$1_joint6, arm_$1_joint7, gripper_$1_joint1], points: [{positions: [$P, $3], time_from_start: {sec: 3}}]}}\"'"; }
# ===== 11/17 마찰 실측 (박스 없음) =====
#   visc_arm <팔>   점성: 7관절 자동, 관절당 ≈1분 (±0.25 rad, 속도 0.05 0.1 0.2 0.4 0.6 각 1왕복). 결과 <팔>_visc_j<n>.csv, 화면 기록 visc_<팔>.log
#   stic_check <팔> <관절>   정지: 서비스·현재 모드 확인만 (쓰기 없음)
#   stic <팔> <관절> "<q1..q7>" <저장.csv>   정지: 관절 하나 (선배 허락 후)
#   stic_arm <팔>   정지: 7관절 자동, 관절당 ≈40초 (홈 자세, 가벼운 관절부터). 복구 실패하면 즉시 멈춤
ST=~/Desktop/Chambit_Sim/mingi_control/real/stiction.py
_vpose(){ case $1$2 in   # 11/10과 같은 관절별 기준 자세 (오른팔은 j2·j3·j5 부호 반대)
  ?1|?4|?7) echo "0 0 0 -1.57 1.57 0 0";; l2) echo "0 0.5 0 -1.57 1.57 0 0";; r2) echo "0 -0.5 0 -1.57 1.57 0 0";;
  l3) echo "0 1.57 0 -1.57 0 0 0";; r3) echo "0 -1.57 0 -1.57 0 0 0";; l5) echo "0 0 0 -1.57 1.57 -1.0 0";; r5) echo "0 0 0 -1.57 1.57 1.0 0";;
  ?6) echo "0 0 0 -1.57 0 0 0";; esac; }
visc(){ ssh $ROB "docker exec -i ai_worker bash -ic 'python3 - vel --arm $1 --joint $2 --base $3 --range -0.25 0.25 --speeds 0.05 0.1 0.2 0.4 0.6 --cycles 1 --move 3'" < $SC 2> >(tee -a visc_$1.log >&2) | sed -n '/^t,phase/,$p' > $4; wc -l $4; }
visc_arm(){ for j in 1 4 7 2 3 5 6; do echo "== $1팔 j$j ($(date +%H:%M:%S))"; visc $1 $j "$(_vpose $1 $j)" $1_visc_j$j.csv; done; pose $1 "0 0 0 0 0 0 0"; }
stic(){ ssh $ROB "docker exec -i ai_worker bash -ic 'python3 - --arm $1 --joint $2 --base $3'" < $ST 2> >(tee -a stic_$1.log >&2) | sed -n '/^t,q/,$p' > $4; local rc=${PIPESTATUS[0]}; wc -l $4; return $rc; }
stic_check(){ ssh $ROB "docker exec -i ai_worker bash -ic 'python3 - --arm $1 --joint $2 --check'" < $ST; }
# 정지 마찰 대표 자세 (11/17 선배 조언: 2~3 자세, 비슷하면 평균)
#   hang  = 팔 늘어뜨림 (j2만 한계에서 ±0.3)
#   grasp = 박스 파지 직후 힘 제어 전환 자세 = 시뮬 파지 자세 Q_GRASP (gain_design.py, 11/4)
_spose(){ case $1$2 in
  lhang) [ $3 = 2 ] && echo "0 0.3 0 0 0 0 0" || echo "0 0 0 0 0 0 0";;  rhang) [ $3 = 2 ] && echo "0 -0.3 0 0 0 0 0" || echo "0 0 0 0 0 0 0";;
  lgrasp) echo "0.5106 0.1599 -0.0832 -1.266 1.7479 -0.0301 -0.8224";;  rgrasp) echo "-0.3888 -0.7111 1.9041 -1.37 -0.3074 0.6082 -1.5823";; esac; }
#   stic_arm <팔> <hang|grasp>   7관절 자동 → <팔>_stic_<자세>_j<n>.csv
stic_arm(){ for j in 6 7 5 4 3 1 2; do echo "== $1팔 $2 j$j ($(date +%H:%M:%S))"
  stic $1 $j "$(_spose $1 $2 $j)" $1_stic_$2_j$j.csv || { echo "!! j$j 실패/복구 문제 — 여기서 멈춤. bringup 상태 확인"; return 1; }; done; pose $1 "0 0 0 0 0 0 0"; }
#   (11/17 재측정) viscfast <팔> <관절>   Stribeck 구간(MR 그림 8.13(f)) 밖 빠른 속도: ±0.5 rad, 0.2 0.4 0.7 1.0 rad/s → <팔>_viscfast_j<n>.csv
#   viscfast_redo   b ≤ 0·불확실했던 관절 한 번에 (l3 l7 r7 l4 r4)
viscfast(){ ssh $ROB "docker exec -i ai_worker bash -ic 'python3 - vel --arm $1 --joint $2 --base $(_vpose $1 $2) --range -0.5 0.5 --speeds 0.2 0.4 0.7 1.0 --cycles 1 --move 3'" < $SC 2> >(tee -a viscfast_$1.log >&2) | sed -n '/^t,phase/,$p' > $1_viscfast_j$2.csv; wc -l $1_viscfast_j$2.csv; }
viscfast_redo(){ for aj in "l 3" "l 7" "l 4" "r 7" "r 4"; do set -- $aj; echo "== $1팔 j$2 ($(date +%H:%M:%S))"; viscfast $1 $2; done; pose l "0 0 0 0 0 0 0"; pose r "0 0 0 0 0 0 0"; }
