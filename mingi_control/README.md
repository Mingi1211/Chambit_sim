# 2번 하위 제어 — 토크(작업공간 임피던스) · 김민기

현대 Docker 환경(QP 위치제어로 박스 옆까지 이동) 위에서, 잡은 뒤에는 **토크 제어로 전환**해
중력 보상으로 팔이 처지지 않게 유지하고, 임피던스로 박스를 **내력이 제한된 채** 쥐고 → 들고 → 유지한다.

이론 근거: 「작업공간 임피던스 제어 PD gain 설정」(인수인계 v2 9절 #1~#13), Modern Robotics 8.6 · 11.2 · 11.4.1 · 11.7.1, Khatib(1987).
코드의 `# [문서 #n]` = 그 식, `# [문서 밖]` = 이론이 아닌 ROS·Gazebo 연결.

## 1. 식 ↔ 코드 (`impedance_node.py`)

| 문서 | 식 | 줄 | 뜻 |
|---|---|---|---|
| #12-③ · MR 8.3 | g(q) = RNEA(q, 0, 0) | 174 | 지금 자세에서 팔 무게를 버티는 토크. 지탱 토크라 τ에 **+g** |
| #1 · MR 8.6 | M 대칭화, J(팔 7축, 리프트 제외) | 178, 190 | crba는 위 삼각만 채움. J는 base_link 축 기준 6×7 |
| #1 · Khatib | Λ = (J M⁻¹ Jᵀ)⁻¹ | 192–194 | 손에서 본 유효 질량. 손목 5·6·7 armature 0.05 추가(교수님 지시) |
| #4 | e = [p_d − p ; log3(R_d Rᵀ)], ė = V_d − J q̇ | 195–198 | 위치·자세 오차. 목표 가속도 0 가정 |
| #5 · MR 11.2.2 | D = 2ζ√(Λ_ii K_ii) | 199–201 | 축마다 임계감쇠(ζ=1). Λ가 자세마다 바뀌어 매 주기 재계산. 입력은 K, ζ뿐 |
| #2 · MR 11.7.1 | F_cmd = K e + D ė, τ_task = Jᵀ F_cmd | 202–204 | 손에 스프링·댐퍼를 단 힘 → 관절 토크 (MR 5.2 τ = JᵀF) |
| #3 · Khatib | J̄ᵀ = Λ J M⁻¹, N_T = I − Jᵀ J̄ᵀ, τ0 = K_q(q_nom − q) − D_q q̇ | 205–209 | 남는 1자유도(7축−6)로 자세 유지. 손에는 영향 없음(T3) |
| #9 | τ += J_vᵀ[0, 0, m g/2]ᵀ (램프) | 210–213 | 박스 무게 절반씩 미리 들어 줌 → K_z는 무게 오차만 감당 |
| #2 | τ_cmd = τ_task + g + τ_null (+ff), 포화 | 214–219 | 최종 토크. `mode gravity`면 g만 (T12) |
| #11 | F̂_ext = Λë + Dė + Ke | 221–224 | 센서 없는 외력 지표(기록용). 쥐는 힘은 F_cmd,y로 본다 |

적분항 없음(#13, 와인드업 방지). 문서에 없는 제어 방법(computed torque·어드미턴스·관측기)은 쓰지 않음.

## 2. 문서 밖 (ROS·Gazebo 연결) — 무엇을 왜

| 파일 | 무엇 | 왜 |
|---|---|---|
| `src/coop_carry_hardware/src/gazebo_joint_actuator.cpp` | 토크 토픽 `/sim/<robot>/effort` 추가. 관절 값이 숫자면 `JointForceCmd`, NaN이면 기존 위치 서보. 0.1 s 끊기면 그 자세로 위치 hold | 현대 플러그인은 속도 서보(무한한 힘처럼 동작) → 밀어도 안 밀리고 쥐면 내력 무한대. 임피던스는 **토크 입력**이 있어야 성립(MR 11.7) |
| `.../simulation.py` · `model_factory.py` | `control_mode: effort`일 때만 토크 토픽 브리지, 물리 1 ms, 외력 플러그인, 박스·받침대 | 기본값 position → **현대 기존 동작 그대로**. 물리 10 ms에서는 토크 루프가 지연으로 진동(10/28 확인) |
| `config/robots.yaml` | `control_mode`, `state_rate: 500`, `box` 절 | 문서 #8 감쇠 상한 D < 2Λ/T: 루프 주기 T를 줄일 수 있게 |
| 노드 입출력 | `/sim/<robot>/joint_states` 구독 → 계산 → `/sim/<robot>/effort` | 상태가 올 때마다 1주기 = 루프 500 Hz. Pinocchio가 현대 이미지에 없어 **PC(host)에서 실행** |

## 3. 실행 순서

```bash
# 0) 이미지 빌드 (C++ 플러그인·설정이 바뀌었으므로) — ~/KW_Internship
docker compose -f docker/compose.yaml build
# 1) 실행 (GUI)
xhost +si:localuser:root && COOP_GUI=true docker compose -f docker/compose.yaml up -d --no-build
```
이후 터미널은 모두 **깨끗한 셸**에서:
```bash
env -i HOME="$HOME" USER="$USER" DISPLAY="$DISPLAY" TERM="$TERM" PATH=/usr/bin:/bin bash --norc
cd ~/KW_Internship && source scripts/environment.bash && cd mingi_control
```
| 단계 | 명령 | 확인 |
|---|---|---|
| 상태 | `ros2 run coop_carry_bringup robot_command --status` | 두 로봇 `ready` |
| 접근(현대 QP) | `ros2 run coop_carry_bringup robot_command --goals pregrasp.yaml` | 양손이 박스 옆 |
| T12 중력 | `python3 impedance_node.py --ros-args -r __ns:=/robot1 -p robot:=robot1 -p "sides:=[r,l]" -p mode:=gravity` | 팔이 안 처짐. Ctrl+C 하면 위치 hold로 복귀 |
| 임피던스 | 위와 같되 `-p mode:=task -p move_time:=2.0` | 켜도 자세 유지 |
| T4 외력 | `python3 push.py --side r --F 0 10 0` | 출력된 analyze 명령 실행 |
| T5 계단 | `python3 step.py --side r --axis y --d 0.03` | 오버슈트≈0, T_s≈5.8/ωn |
| S5 박스 | `python3 box_scenario.py --gap 0.08 --delta 0.01 --lift 0.05 --hold 10 --mass 1.0` | 쥐는 힘 ≤ 30 N, 미끄러짐 < 5 mm |
| 시뮬 없이 | `python3 check_theory.py <control.urdf>` · `python3 gain_design.py --urdf <control.urdf>` | T3, T6 PASS |

`control.urdf`: `docker compose -f docker/compose.yaml exec robot1 cat /run/coop/robot1/control.urdf > /tmp/control.urdf`

## 4. 게인 (gain_design.py 결과, 박스 잡는 자세 (0.40, ±0.15, 0.85))

- diag(Λ)_xyz ≈ (5.6, 3.4, 6.8) kg → K ≤ Λ_min(2πf/10)² ≈ 3.4×10⁵ N/m (f=500 Hz), 여유 큼
- K_z = 1000 N/m ≥ 490 (피드포워드 없을 때 (m g/2)/1 cm) → 피드포워드 없이도 처짐 1 cm 이내
- K_y = 500 N/m ≤ F_int,max/Δx = 30/0.02 = 1500 → 손 위치 오차 2 cm여도 내력 30 N 이하
- 쥐기: F_sq = K_y·e_y. 미끄럼 방지 2μF_sq ≥ m g → F_sq ≥ 4.9 N (μ=1, 1 kg)
- K_y < K_z (수평 부드럽게·수직 단단하게 — 교수님 지시)

## 5. 아직 문서·실험으로 확인 안 된 것

- 손(EE) 기준 그리퍼 두께 → 실제 접촉 위치. 그래서 쥐는 힘은 **설정 δ가 아니라 로그의 F_cmd,y로 보고**
- 외력은 link7 원점에 걸림(EE보다 0.215 m 위) → T4는 근사
- Gazebo에는 관절 마찰·정지마찰이 없음 → 실물에서는 F̂_ext가 마찰만큼 틀림
