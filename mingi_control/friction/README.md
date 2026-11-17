# 관절 마찰 — 실측값을 시뮬에 넣고 보상·강성 하한 확인 (11/17, 김민기)

식은 Modern Robotics(MR)만 사용. 결과 정리: Drive `실물테스트_1108/마찰보상_1117/보고_마찰실측_보상_강성하한_1117.md`

## 흐름
| 단계 | 파일 | 내용 |
|---|---|---|
| 1. 실측 | `mingi_control/real/kt_cmds.sh` `visc_arm`, `viscfast` | 위치 모드, 한 관절 등속 왕복 (실물) |
| 2. 식별 | `mingi_control/real/fit_friction.py` | τ_fric = c·I − g − M_jj θ̈ (MR 8장), 속도별 양방향 평균 F(v), 커지는 구간에서 F = b_static + b_viscous·v (MR 그림 8.13(c)), 없으면 (b) |
| 3. 시뮬 입력 | `src/coop_carry_bringup/config/robots.yaml` `joint_friction` | friction = b_static, damping = b_viscous → URDF → SDF → DART |
| 4. DART 재현 확인 | `sim_friction_check.py` | 시뮬 등속 왕복 → 같은 방법으로 역추정 |
| 5. 보상 비교 | `ff_compare.py` (`--stiff design` / `kmin`), `plot_ff_compare.py` | τ = M̃θ̈_d + h̃(θ_d, θ̇_d) + K_p θ_e + K_d θ̇_e (MR (11.32)+(11.24)), h̃ = g [+ b_static sgn θ̇_d] [+ b_viscous θ̇_d] |
| 6. 강성 하한 | `k_lower_bound.py` | K_p ≥ b_static/δθ_j, δθ_j = δx/‖J_v,j‖ (δx 5 mm, 게인 문서); 작업공간 K ≥ \|J̄ᵀ\|τ_s/δx |

## 실행 (시뮬: 깨끗한 셸 + `source scripts/environment.bash`, impedance_node 꺼짐)
```bash
python3 sim_friction_check.py --robot robot1 --side r --joint 4 --amp 0.5 --speeds 0.2 0.4 0.7 1.0 --out ../../real_logs/1117_fric/sim
python3 ff_compare.py --robot robot1 --side r --joint 4 --stiff design --out ../../real_logs/1117_fric/sim
python3 ff_compare.py --robot robot1 --side r --joint 4 --stiff kmin --cases none both --out ../../real_logs/1117_fric/sim
python3 plot_ff_compare.py --dir ../../real_logs/1117_fric/sim --joints 4 1 --out ../../real_logs/1117_fric/report
python3 k_lower_bound.py --urdf ../../real_logs/robot_description.urdf --dx 0.005
```
`ff_compare.py`는 실측 결과 `real_logs/1117_fric/friction_{l,r}.csv`와 왕복 범위 CSV를 읽음 (Drive에서 받아 같은 경로에 둘 것).

## 남은 문제
- 손목 j6: 시뮬 역추정이 넣은 값과 다름 (3.05/4.15 → 1.61/0.66) — 원인 미확인
- DART 마찰은 정지 마찰보다 작은 힘에서도 천천히 미끄러짐 → 정상상태는 MR 2 % 정착 시간 시점으로 판정
- `friction_params.py`, `urdf_grav.py`: 11/16 패치의 이전 방식(하중 의존 보정). 지금 흐름에서는 쓰지 않음
- `impedance_node.py`의 `fric_ff`, `dgear` 옵션: 아직 시뮬 미검증
