# Chambit_sim — AI Worker 2대 박스 협동 운반 (Gazebo)

광운대 DREAM Lab 참빛설계 · AI Worker2(FFW-SG2) 두 대가 박스를 손잡이로 잡고 → 들고 → 베이스 2 m 이동 → 옆 받침대에 내려놓는다.

- **상위 제어(김현대)**: Docker 환경, 로봇 2대 Gazebo, Cyclo QP(MoveL) 위치 제어 — 원본 [jacob020328/KW_Internship](https://github.com/jacob020328/KW_Internship), 설명은 [docs/README_hyundai.md](docs/README_hyundai.md)
- **하위 제어(김민기)**: 파지 이후 **토크 제어 — 중력 보상 + 작업공간 임피던스 + 리더–팔로워 강성 차등** (`mingi_control/`)

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic / Docker 기준.

---

## 1. 따라하기

### 1-1. 처음 한 번: 설치

```bash
# 코드 + 외부 소스 4개(ai_worker, cyclo_control, robotis_interfaces, realsense-ros — 고정 커밋)
git clone --recurse-submodules git@github.com:Mingi1211/Chambit_sim.git ~/Desktop/Chambit_Sim

# 시스템 패키지 (docker compose 플러그인, Zenoh 통신, Pinocchio)
sudo apt update && sudo apt install -y docker-compose-v2 ros-jazzy-rmw-zenoh-cpp ros-jazzy-pinocchio

# 빌드: 호스트(명령 도구) + Docker 이미지(로봇·Gazebo). 이미지 첫 빌드는 수십 분
cd ~/Desktop/Chambit_Sim && bash scripts/build_host.bash && docker compose -f docker/compose.yaml build
```

> `src/` 아래 코드나 `src/coop_carry_bringup/config/robots.yaml`을 바꾸면 `docker compose ... build`를 다시 한다. `mingi_control/` 파이썬은 빌드 불필요.

### 1-2. 실행: 터미널 4개

**터미널 1 — 시뮬레이션 (GUI)**
```bash
cd ~/Desktop/Chambit_Sim && docker compose -f docker/compose.yaml down && xhost +si:localuser:root && COOP_GUI=true docker compose -f docker/compose.yaml up -d --no-build
```
⚠️ Gazebo 창을 닫으면 시뮬 전체가 종료된다(최소화만).

**터미널 2·3·4 — 공통으로 먼저**: `~/.bashrc`의 다른 ROS 설정과 섞이지 않게 깨끗한 셸 + 현대 환경(Zenoh, domain 71)
```bash
env -i HOME="$HOME" USER="$USER" DISPLAY="$DISPLAY" TERM="$TERM" PATH=/usr/bin:/bin bash --norc
cd ~/Desktop/Chambit_Sim && source scripts/environment.bash && cd mingi_control
```

**터미널 2 — robot1 임피던스 노드**
```bash
python3 impedance_node.py --ros-args -r __ns:=/robot1 -p robot:=robot1 -p "sides:=[r,l]" -p mode:=position
```

**터미널 3 — robot2 임피던스 노드**
```bash
python3 impedance_node.py --ros-args -r __ns:=/robot2 -p robot:=robot2 -p "sides:=[r,l]" -p mode:=position
```

**터미널 4 — 시나리오**
```bash
python3 grasp_sequence.py          # 접근(현대 QP) → 그리퍼 접촉 감지 → 토크(임피던스) 전환
python3 transport_scenario.py      # 들기 → 베이스 2 m 이동 → 내려놓기 → 놓기 → 위치 제어 복귀
python3 analyze.py carry $(ls -t ~/impedance_logs/*_carry_box.csv | head -1)   # 단계별 처짐·내력·박스 이동 + 그래프
```

| 확인 | 기준 |
|---|---|
| `grasp_sequence.py` 끝 | `완료: 임피던스로 현재 자세 유지 중`, 4팔 `멈춤 q` 0.75~0.95 |
| 운반 끝 박스 위치 `gz model -m box -p` | 약 (0, 2.0, 1.05) = 옆 받침대 위 |

다른 시나리오: `carry_scenario.py`(베이스 고정, 리더만 옆 이동 — 리더–팔로워 내력 비교), 허공 시험 `step.py`(계단 응답), `push.py`(외력), `mode:=gravity`(중력 보상만).

### 1-3. 끝내기·다시 하기
- 노드 종료: 그 터미널에서 `Ctrl + C` (`clear`는 화면만 지움). 어디서 도는지 모르면 `pkill -f "impedance_node.py --ros-args"`
- 처음부터: 터미널 1의 명령(down → up)을 다시, 노드 2개 다시, 터미널 4부터 반복

---

## 2. 내 파트: 하위 제어 구현

### 2-1. 흐름

```
[현대] Cyclo QP 위치 제어로 손잡이 앞까지 접근 (mode:=position, 팔은 위치 서보)
   ↓ grasp_sequence: 그리퍼 닫기 → 손가락이 닫힘 위치 전에 멈추면 "접촉"(센서 없이)
[김민기] mode:=task 전환 — 그 순간 자세를 목표로 토크 제어
   ↓ 무게 피드포워드 → 들기 → 베이스 이동(리더–팔로워) → 내려놓기 → 놓기
[현대] mode:=position 복귀
```

토크 경로: `impedance_node`(PC) → `/sim/<robot>/effort` → Gazebo 플러그인이 관절에 **힘(JointForceCmd)** 으로 인가. 0.1 s 끊기면 그 자세로 위치 고정(안전).

### 2-2. 이론 → 코드 (`mingi_control/impedance_node.py`)

근거: 「작업공간 임피던스 제어 PD gain 설정」(문서 #1~#13), Modern Robotics(MR), Khatib(1987). 코드의 `# [문서 #n]` 주석이 아래 표와 대응.

| 문서 | 식 | 줄 | 뜻 |
|---|---|---|---|
| #12 · MR 8.3 | g(q) = RNEA(q, 0, 0) | 223 | 자세마다 팔 무게를 지탱하는 토크 → τ에 **+g** (중력 보상) |
| #1 · MR 8.6 | M 대칭화, J = 팔 7축(리프트 제외) | 227, 249 | crba는 위 삼각만 채움, J는 base_link 축 6×7 |
| #1 · Khatib | Λ = (J M⁻¹ Jᵀ)⁻¹ | 253 | 손에서 본 유효 질량 |
| #4 | e = [p_d − p ; log3(R_d Rᵀ)], ė = V_d − J q̇ | 254 | 위치·자세 오차 |
| #5 · MR 11.2 | D = 2ζ√(Λ_ii K_ii), ζ = 1 | 258 | 축마다 임계감쇠, Λ가 자세마다 바뀌어 매 주기 재계산 |
| #8 | K_ii ≤ Λ_ii ω_max², D_ii ≤ 2Λ_ii/T | 260, 263 | 이산 시간 안정 조건(실측 루프 주기 T) |
| #2 · MR 11.7.1 | F_cmd = K e + D ė, τ_task = Jᵀ F_cmd | 266 | 손에 스프링·댐퍼 → 관절 토크(MR 5.2 τ = JᵀF) |
| #3 · Khatib | J̄ᵀ = Λ J M⁻¹, N_T = I − Jᵀ J̄ᵀ, τ0 = K_q(q_nom − q) − D_q q̇ | 244, 269 | 7축의 남는 1자유도(팔꿈치) 자세 유지·관절 한계 회피, 손에는 영향 없음 |
| #9 | τ += J_vᵀ[0, 0, m g/2]ᵀ (램프) | 279 | 박스 무게 미리 보상 → K_z는 추정 오차만 |
| #2 | τ_cmd = τ_task + g + τ_null (+ff), 포화 | 283 | 최종 토크 |
| #11 | F̂_ext = Λë + Dė + Ke | 295 | 센서 없는 외력 지표(로그) |
| #7 | 리더 K = (800, 800, 1000), 팔로워 (300, 300, 1000) N/m | 114 | 수평 부드럽게·수직 단단하게, 리더 단단·팔로워 부드럽게 |

적분항 없음(#13). 문서에 없는 제어 방법은 쓰지 않음. 게인 범위는 `gain_design.py`(#5~#9), 식 검증은 `check_theory.py`(T3 영공간, T6 문서 예제).

### 2-3. 결과 (2026-11-04)

| 항목 | 예측(이론) | 측정 |
|---|---|---|
| 박스 무게 처짐 e = F/K (#4) | 4.9 mm | 3~4.5 mm |
| 무게 피드포워드 후 (#9) | ≈ 0 | 0.1 mm |
| 계단 응답 오버슈트 (#5, ζ=1) | ≈ 0 | 0.0 % |
| 리더 옆 이동 시 박스 추종 (#7) | 3.6 cm | 3.1 cm |
| 팔로워 내력: 부드러움 vs 단단함 (#7) | 부드러움 < 단단함 | 5.3 N < 8.7 N |
| 영공간 간섭 T3 (#3) | < 1e-9 | 2.9e-11 |
| 문서 예제 T6 (#6) | K 1000, D 126.5, Tₛ 0.37 s | 동일 |
| **전체 운반** | 받침대 (0, 2.0, 1.05) | **(−0.006, 2.018, 1.050)** |

### 2-4. 파일

| 파일 | 역할 |
|---|---|
| `mingi_control/impedance_node.py` | 토크 제어 노드 (mode position / gravity / task, 그리퍼 명령, 리더·팔로워, 실물 전류 출력 `output:=dxl`) |
| `mingi_control/grasp_sequence.py` | 접근 → 그리퍼 접촉 감지 → task 전환 |
| `mingi_control/transport_scenario.py` | 들기 → 베이스 이동(팔로워 지연) → 내려놓기 → 놓기 |
| `mingi_control/carry_scenario.py` | 베이스 고정, 리더–팔로워 내력 비교 |
| `mingi_control/analyze.py` · `gain_design.py` · `check_theory.py` · `push.py` · `step.py` | 분석·게인 설계·식 검증·외력·계단 시험 |
| `mingi_control/real/`, `calib_kt.py`, `motor_params.yaml` | 실물 DYNAMIXEL 전류 모드(코드만, **실물 미검증**, 값 비면 실행 거부) |
| `src/coop_carry_hardware/src/gazebo_joint_actuator.cpp` | Gazebo 플러그인: 토크·그리퍼 위치·베이스 목표 입력 추가 |
| `src/coop_carry_bringup/.../simulation.py`, `model_factory.py`, `config/robots.yaml` | `control_mode: effort`일 때만: 물리 1 ms, 상태 500 Hz, 박스·손잡이·받침대, 그리퍼 effort 한계, 베이스 슬라이드 관절 |

`control_mode: position`이면 현대 원본 동작 그대로.

---

## 3. 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `docker compose`가 help만 출력 | compose 플러그인 없음 | `sudo apt install docker-compose-v2` |
| `librmw_zenoh_cpp.so` 없음 | 호스트에 Zenoh RMW 미설치 | `sudo apt install ros-jazzy-rmw-zenoh-cpp` |
| `No module named 'pinocchio'` / 노드 0개로 보임 | 깨끗한 셸·`environment.bash` 생략 → 다른 ROS 설정(domain, RMW) | 1-2 공통 명령 먼저 |
| `--status`가 `waiting or disconnected` | Gazebo만 재시작되고 로봇은 옛 월드에 연결 | 전체 down → up |
| 노드가 "하나여야/0개" 에러, Ctrl+C 안 먹음 | 닫은 창에 노드가 남아 있음 | `pkill -f "impedance_node.py --ros-args"` |
| 토크 전환 시 손목 11~13° 떨림, 12.3 Hz 진동 | 관성 작은 손목에 고정 게인 과대(D·dt/M > 1) + 물리 10 ms 지연 | 물리 1 ms, 축별 ωn·D 상한(#8), 영공간 게인 관절 관성 비례 |
| 놓은 뒤·특이 자세에서 57 Hz 떨림 | 시뮬에 없는 모터 관성(armature)을 모델에 넣어 D 과대 | 시뮬은 armature 0(모델=플랜트), 실물에서만 사용 |
| 손 위치 일정한데 팔꿈치가 처짐 | 7축의 영공간(팔꿈치)은 τ0로만 버팀 | 영공간 게인 관절 관성 비례(#3, #5) |
| 박스가 전혀 안 들림, 시뮬이 굳은 듯 | 그리퍼 effort 1000 → 손잡이에 파고들어 접촉 계산 폭주, **RTF 0.007** | 물리 모델 그리퍼 effort 한계 100 |
| 그리퍼가 q ≈ 0.53에서 멈추고 운반 중 박스 낙하 | 손잡이와 손가락 닫힘 중심이 어긋나 한쪽만 닿음 | grasp_sequence 정렬 수정(접촉 q 0.84~0.93) |
| 들기·이동 시 한 손이 막혀 전체가 안 움직임 | 관절 가동 한계에 붙음 → 박스로 묶인 닫힌 고리 전체 잠김 | 영공간 자세 목표를 한계 9° 안쪽(#3), 한계 근접 경고 |
| 베이스가 안 움직임 | 현대 모델은 베이스를 월드에 고정 관절로 붙임(set_pose 무시) | effort 모드 물리 모델만 y 슬라이드 관절로(실제 주행은 현대 파트) |
| 놓은 뒤 task 모드 떨림 | 박스 질량이 빠지며 손목 불안정 — **원인 분석 중** | 놓으면 위치 제어 복귀 |
| 화면 동작이 느림 | 두 로봇 1 ms 물리로 RTF ≈ 0.3 | 결과엔 영향 없음, 기다리면 됨 |

남은 과제: T12(중력만)·T4(외력) 재측정, 정착 시간은 시뮬 시간(/clock) 기준 분석, 중력 모델 오차 2~5 mm 원인, 실물 전류 모드 검증.
