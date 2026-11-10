# 실물 Kt 측정 명령 줄임 (11/10) — 내 PC 터미널에서: source ~/Desktop/Chambit_Sim/mingi_control/real/kt_cmds.sh
#   pose <팔 l|r> "<q1..q7>"            기준 자세로 이동
#   vel  <팔> <관절> "<q1..q7>" "<lo hi>" <저장파일.csv>   등속 왕복 측정
#   grip <팔> "<q1..q7>" <0.0 열기|1.0 닫기>   팔은 그 자세 그대로, 그리퍼만
SC=~/Desktop/Chambit_Sim/mingi_control/real/static_current.py; ROB=robotis@192.168.6.2
pose(){ ssh $ROB "docker exec -i ai_worker bash -ic 'python3 - goto --arm $1 --base $2 --move 6'" < $SC; }
vel(){ ssh $ROB "docker exec -i ai_worker bash -ic 'python3 - vel --arm $1 --joint $2 --base $3 --range $4'" < $SC | sed -n '/^t,phase/,$p' > $5; wc -l $5; }
grip(){ local P=$(echo $2 | awk '{for(i=1;i<=NF;i++) printf "%s%.4f", (i>1?", ":""), $i}'); ssh $ROB "docker exec ai_worker bash -ic 'ros2 action send_goal /arm_$1_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{trajectory: {joint_names: [arm_$1_joint1, arm_$1_joint2, arm_$1_joint3, arm_$1_joint4, arm_$1_joint5, arm_$1_joint6, arm_$1_joint7, gripper_$1_joint1], points: [{positions: [$P, $3], time_from_start: {sec: 3}}]}}\"'"; }
