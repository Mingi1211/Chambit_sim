# 손끝 힘 시각화 (RViz) + VLA 입력 토픽 (11/22, 김민기)

손목 F/T로 각 손이 박스에 주는 힘과 박스 내력을 실시간으로 보이고, 같은 값을 토픽으로 내보낸다.

## 무엇을 하나
| 단계 | 내용 |
|---|---|
| 센서 | 손목–그리퍼 고정 관절 `gripper_{r,l}_joint`에 Gazebo force_torque 센서 (`simulation.py`, effort 모드) → `/sim/robotN/ft_{r,l}` |
| 그리퍼 무게 제거 | 정지(준정적) 힘 평형: 손 → 박스 힘 f = F_s + m_g·g, 모멘트 τ = T_s + r_c × m_g·g (m_g, r_c = URDF 그리퍼 링크) |
| 박스 내력 | 손 4개 렌치 h, 박스 기준점 = 손 위치 평균, grasp matrix G → 박스 합 렌치 Gh, 내력 (I − G†G)h (Liu et al., 노션의 −G†h_o) |
| 표시 | RViz: 파랑 = 손 힘, 빨강 = 내력, 로봇 번호와 손 힘 크기(예: `R1:7.6N`), 네 손을 잇는 선. 화살표·숫자는 1 s 지수 평균(표시만, 토픽은 원신호) |

## 실행
시뮬과 파지(README 1장: impedance_node 두 개 + `grasp_sequence.py`)를 띄운 뒤, 깨끗한 셸에서:
```bash
bash mingi_control/viz/run_viz.bash
```
(`hand_wrench_node.py` + `rviz2 -d hand_wrench.rviz`. 로봇 모델의 머리 카메라 메시 하나는 컨테이너 경로라 안 보임 — 팔·손 표시에는 영향 없음)

## 토픽 (VLA 입력용)
| 토픽 | 형식 | 내용 |
|---|---|---|
| `/robotN/hand_wrench_{r,l}` | WrenchStamped | 손 → 박스 렌치, 그리퍼 좌표 |
| `/coop/hand_state` | Float64MultiArray (40) | [손 4개 × (f 3, τ 3), 월드] 24 + [손 4개 내력 f_int 3] 12 + [‖f_int‖] 4. 순서 robot1_r, robot1_l, robot2_r, robot2_l |
| `/coop/hand_markers` | MarkerArray | RViz 표시용 |

## 확인 결과 (`real_logs/1122_handforce/`)
- 손에 아무것도 없을 때 손 힘 ≈ 0.01 N (그리퍼 무게 4.1 N이 빠짐)
- 박스(2 kg)를 잡고 있을 때 네 손이 위로 받치는 힘의 합 19.56 N (1 s 평균) = 박스 무게 19.6 N
- 내력 손당 약 10 N (1 s 평균), 임피던스 전환 순간 최대 380 N
- 파지 중 손 힘이 약 12 Hz(벽시계)로 떨림 — 11/17 임피던스 감쇠항 떨림과 같은 현상으로 추정, 미확인

## 한계
- 그리퍼 무게 보정은 정지 평형 기준 (빠른 가감속 중에는 관성력 몫이 섞임)
- 박스 자체는 Gazebo에만 있어 RViz에 그리지 않음 (손 위치만 사용)
- 실물에는 손목 F/T가 없음 → 실물에서는 관절 전류(토크) 기반 추정으로 바꿔야 함

## 비교 GIF (11/23)
- `hand_wrench_rec.rviz`: 녹화용 시점 (곡선 주행 2 m가 화면에 들어오게)
- `compare_gif.py`: 두 런을 나란히 — 위 RViz 녹화, 아래 같은 시뮬 시각의 손 내력(4손 × 1 s 중앙값), `--drop`으로 박스 낙하 뒤 회색 처리. 사용법은 `../stiffness/README.md`
