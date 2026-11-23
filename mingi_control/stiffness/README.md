# 곡선 주행 → 강성 범위 재산정 → 마찰 보상·강성 효과 (11/23, 김민기)

식은 MR·게인 문서만 사용. 결과 정리: Drive `실물테스트_1108/곡선주행_강성_1123/보고_곡선주행_강성_마찰보상_1123.md`

## 흐름
| 단계 | 파일 | 내용 |
|---|---|---|
| 1. 강성 범위 | `k_window.py` | 넣을 수 있는 상한 K_max (실물 100 Hz 이산 폐루프 ρ(Φ) < 1, GAIN_RATIONALE ⑦), 손끝 마찰력 \|J̄ᵀ\|b_static, 방향별 허용 오차 δ = F/K_max, 팔로워 K_F ≤ (15 − F_res)/Δ |
| 2. 곡선 운반 | `../transport_scenario.py` + 현대 `base_move_curve.launch.py` | 박스 파지 후 곡선 주행 (로봇1만 옆 0.2 m) |
| 3. 마찰 보상 | `../impedance_node.py` `-p fric_ff:=true -p fric_src:=actual` | MR (11.65) η̃(θ, ẋ)처럼 측정 속도로 b_static sgn(q̇) + b_viscous q̇ (기존 `desired`는 θ̇_d 기준 → 손 목표 고정이면 0) |
| 4. 분석 | `curve_analysis.py` | 베이스 간 오차 Δ = (p₂ − p₁) − (p₂ − p₁)₀, 손 내력 (I − G†G)h 중앙값, 리더 오차, 박스 |
| 5. 그림 | `plot_curve.py` (base / force / sweep), `summary_plot.py` | 베이스 오차 5회, 보상 유무 내력, 런 요약 |
| 6. GIF | `../viz/compare_gif.py` | 두 조건 나란히: RViz 녹화(`../viz/hand_wrench_rec.rviz`) + 같은 시뮬 시각의 손 내력 |

## 런 하나의 기록 (`real_logs/1123_curve_trials/<tag>_*`)
`_base.csv` 베이스 Gazebo 자세 · `_status.jsonl` 베이스 상태 (MOVING ~ DONE) · `_hand.csv` `/coop/hand_state` · `_task_robot{1,2}.csv` 임피던스 로그 · `_box.csv` 박스 자세 · `_raw.mp4` RViz 녹화
기록 노드·실행 스크립트는 세션용으로 따로 둠 (base_odometry·base_move_status·hand_state 구독, 실행 순서는 README 1장 + `hand_wrench_node.py`).

## 실행
```bash
python3 k_window.py --urdf ../../real_logs/robot_description.urdf --delta 0.206 0.764 --F_res 6.2 4.4
python3 curve_analysis.py --dir ../../real_logs/1123_curve_trials --tags b1_K40_off f_K40_on o2_opt_on
python3 plot_curve.py base --tags b1_K40_off b2_K40_off b3_K40_off b4_K40_off b5_K40_off --out base_error_5runs.png
python3 summary_plot.py
python3 ../viz/compare_gif.py --dir ../../real_logs/1123_curve_trials --tags b1_K40_off f_K40_on --labels "보상 없음" "보상 있음" --crop 860:700:470:220 --out gif1.gif
```

## 결과 요약
- K_max = 515 N/m (실측 점성 넣어도 518) → 11/17 하한 13,400 N/m는 불가. 상한에서 허용 오차 δx 130, δy 63 mm
- 마찰 보상 (K_F 40, 같은 곡선): 손 내력 중앙값 31 → 11.6 N (허용 15 N)
- 제안: K_F = (40, 10) N/m + 보상. K_F 150·500은 정상 곡선에서 3회 모두 박스 낙하

## 남은 문제
- (40, 10)을 정상 곡선(Δx 0.2 m)에서 미확인 — 베이스 출발 지연이 무작위(5회 중 정상 1회)
- 베이스 출발 지연 원인 미확인 (후보: `coop_carry_hardware/src/gazebo_swerve_actuator.cpp` 지령 timeout) → 현대 확인
- 손 F/T 원신호에 접촉 충격 스파이크 → 대표값은 중앙값
