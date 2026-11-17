# 관절 마찰 시뮬 — 넣기 · 확인 · 보상 검증 (11/16, 김민기)

목표: 추가 실측 없이 Gazebo(DART)에 실물 관절 마찰을 넣고, 마찰 보상(피드포워드)과 감쇠 게인(D_gear)이 맞게 상쇄되는지 확인한다.

## 근거 (MR · 논문만)
| 식 | 어디 | 쓰는 곳 |
|---|---|---|
| τ_f = f_c·sgn(q̇) + b·q̇ (쿨롱 + 점성) | MR 8.9.4 그림 8.13(c) | 플랜트 마찰 모델 (URDF `<dynamics friction damping>`) |
| M q̈ + c + g + b(q̇) + JᵀF = τ, 등속·저속 → τ − g = τ_f | MR (11.48) | 실험 0 (넣은 값 확인), 11/10 실측 해석 |
| 하중이 클수록 쿨롱이 커짐 | Qian 2024 식 (10)(11), BAM(ICRA 2025) III.B | `<friction>` 값을 파지 자세 하중으로 계산 |
| 피드포워드 τ = M̃(θ_d)θ̈_d + h̃(θ_d, θ̇_d) (h에 마찰 포함, (11.21)) | MR (11.32) | `fric_ff`: f_c·sgn(q̇_d) + b·q̇_d |
| ζ = (b + K_d) / (2√(K_p M)) | MR (11.25)(11.26) | `dgear`: D_ctrl = 2ζ√(ΛK) − D_gear |
| D_gear = J̄ᵀ diag(b) J̄ (작업공간으로 옮김) | MR에 그대로 없음 — τ = −b q̇, q̇ = J̄ẋ, F = J̄ᵀτ에서 유도 △ | `dgear` |

- 쿨롱 f_c: 11/10 실측 (kt_friction_table_1110.csv) ✔
- 점성 b: 11/10은 두 속도(0.05, 0.1 rad/s)만이라 음수로 나옴 → **실측 없음 → 후보값으로 민감도 시험**
- SDF `<friction>`은 값 하나 → 정지 마찰 = 쿨롱 (그림 8.13(c) 형태로 가정 △, 실험 0에서 확인)

## 바뀐 파일
| 파일 | 내용 |
|---|---|
| `src/coop_carry_bringup/config/robots.yaml` | `joint_friction: {}` 추가 (비우면 마찰 0 = 기존과 같음) |
| `src/coop_carry_bringup/coop_carry_bringup/model_factory.py` | `joint_friction` 읽기 |
| `src/coop_carry_bringup/coop_carry_bringup/simulation.py` | effort 모드 물리 모델의 팔 관절에 `<dynamics friction damping>` 넣기 |
| `mingi_control/impedance_node.py` | 파라미터 `fric_ff`, `dgear`, `fric_c_r/l`, `fric_b_r/l` (기본 꺼짐) |
| `mingi_control/friction/friction_params.py` | 11/10 표 → `<friction>` 값 (파지 자세 하중 기준) |
| `mingi_control/friction/sim_friction_check.py` | 시뮬에서 등속 왕복 → 넣은 f_c, b가 그대로 나오는지 |
| `mingi_control/friction/urdf_grav.py` | URDF 중력·박스 토크 (numpy) |

## 0. 준비
```bash
cd ~/Desktop/Chambit_Sim
git apply friction_1116.patch          # Drive의 패치
docker compose -f docker/compose.yaml build      # robots.yaml·simulation.py가 이미지 안에 들어감 → 바꿀 때마다 다시 빌드
```

## 1. 마찰 값 계산 → robots.yaml
```bash
cd mingi_control/friction
docker compose -f ../../docker/compose.yaml exec robot1 cat /run/coop/robot1/control.urdf > /tmp/control.urdf   # (시뮬 실행 중일 때)
python3 friction_params.py ~/Desktop/Chambit_Sim/real_logs/kt_friction_table_1110.csv --urdf /tmp/control.urdf \
        --box_mass 2.0 --hand_share 0.5 --damping 0.0
```
- 출력 마지막 `joint_friction:` 블록을 `robots.yaml`에 붙여 넣음 → 다시 빌드
- 실험마다 `--damping`(점성 후보), `--scale`(쿨롱 배율)을 바꿔 새 블록을 만든다
- 확인: `docker compose -f docker/compose.yaml exec robot1 grep -A3 'name="arm_r_joint4"' /run/coop/robot1/physics.urdf` 에 `<dynamics friction=… damping=…>`

## 2. 실험 0 — 넣은 마찰이 DART에서 그대로 나오나 (먼저 꼭)
조건: impedance_node 끔 (같은 effort 토픽을 씀), 팔은 위치 hold 상태
```bash
python3 sim_friction_check.py --robot robot1 --side r --joint 4 --amp 0.2 --speeds 0.05 0.1 0.2 0.4
```
- 관절 하나만 등속 왕복(관절 PD + 중력 보상), 나머지는 위치 hold. 시뮬은 명령 토크를 정확히 알아서 τ − g − M q̈ = f_c·sgn + b·q̇ 최소제곱
- **통과:** 추정 f_c, b가 robots.yaml 값과 ±10 % (b가 작으면 ±0.1) 안 △ 내 기준. 합성 데이터로 코드 확인함 (참값 (2.0, 0.3) → (1.99, 0.36), (1.0, 1.5) → (1.00, 1.50))
- 관절 2개 이상 (어깨 j1 또는 j4, 손목 j6) · 한계 근처면 `--amp`를 줄임
- **안 맞으면:** DART 마찰 형태가 그림 8.13(c)와 다름 → 기록해 두고 플러그인으로 넣는 방법 검토 (BAM IV.E)

## 3. 실험 1 — 감쇠: 감속기 점성만큼 D를 줄여도 되나 (교수님 말씀)
쿨롱은 끄고 점성만: `friction_params.py ... --scale 0 --damping B` (B = 0.5, 1.0, 2.0 중 하나씩)
| 케이스 | robots.yaml | impedance_node | 기대 (MR (11.26)) |
|---|---|---|---|
| 기준 | `joint_friction: {}` | 기본 | ζ = 1: 오버슈트 ≈ 0, T_s ≈ 5.8/ω_n (11/5와 같음) |
| A | 점성 B | 기본 (`dgear` false) | 몸 점성이 더해져 ζ > 1 → 오버슈트 0, **더 느림** |
| B | 점성 B | `dgear:=true`, `fric_b_r:=[B,…]` | ζ ≈ 1 → 기준과 같은 응답 |

```bash
python3 ../impedance_node.py --ros-args -r __ns:=/robot1 -p robot:=robot1 -p "sides:=[r,l]" -p mode:=task \
  -p dgear:=true -p "fric_b_r:=[1.0,1.0,1.0,1.0,1.0,1.0,1.0]" -p "fric_b_l:=[1.0,1.0,1.0,1.0,1.0,1.0,1.0]"
python3 ../step.py --robot robot1 --side r --axis y --d 0.03      # 출력되는 analyze t5 명령 실행
```
- 판정: 기준·A·B의 오버슈트, T_s 비교 → B가 기준과 같으면 "D를 D_gear만큼 줄여도 된다"
- 주의: `omega_max`, `D_safety` 상한에 걸리면 D가 잘려서 비교가 흐려짐 → 로그 Dx/Dy/Dz 열 확인

## 4. 실험 2 — 쿨롱: 피드포워드가 상쇄하나
점성은 0, 쿨롱만: `friction_params.py ... --scale 1 --damping 0`
| 케이스 | impedance_node | 기대 |
|---|---|---|
| 기준 | 마찰 0 | 이동 중 오차 작음, 끝나면 e ≈ 0 |
| C | `fric_ff` false | 이동 중 뒤처짐, 끝나도 마찰에 걸린 오차 남음 (e_ss ≈ \|J̄ᵀ\|f_c / K 이하) |
| D | `fric_ff:=true`, `fric_c_r/l` = robots.yaml 값 | 이동 중 오차 → 기준 수준. **끝난 뒤(정지 목표)는 피드포워드 0** → 정지 마찰로 남는 오차는 그대로 → 강성 하한이 필요한 이유 |
| E (민감도) | `fric_c` = 0.5배 / 1.5배 | 부족: 오차 일부 남음 / 과보상: 진동·되돌림 △ |

```bash
ros2 param set /robot1/impedance_controller move_time 3.0
ros2 topic pub --once /robot1/impedance/r/offset geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.05, z: 0.0}"
```
- 지표: 로그 CSV의 ex, ey, ez — 이동 중 최대 \|e\|, 끝나고 3 s 뒤 \|e\|
- 외력 시험도 같이: `python3 ../push.py --robot robot1 --side r --F 0 5 0` → e_ss vs F/K (마찰이 있으면 작은 힘에 안 움직이는 구간)

## 5. 실험 3 — 둘 다 (실물 근사)
`--scale 1 --damping B` + `fric_ff:=true dgear:=true` → 기준(마찰 0)과 얼마나 같은가. 이 게인으로 운반 시나리오(외력·곡선 주행).

## 결과 기록 표 (보고용)
| 실험 | 플랜트 (f_c, b) | 보상 | 오버슈트 | T_s | 이동 중 최대 \|e\| | e_ss | 비고 |
|---|---|---|---|---|---|---|---|

표기: △ = 확인 필요 / 내 기준·유도
