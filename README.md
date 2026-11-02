# AI Worker 두 대의 독립 양팔 제어

Ubuntu 24.04 / ROS 2 Jazzy 기준이다. Host의 Command Client가 ROS/Zenoh로
각 Robot Adapter에 EE 목표를 전달한다. 두 Robot 컨테이너는 같은 이미지에서
각각 Cyclo QP·외부 Controller Manager·동기식 표준 JTC·RemoteGazeboSystem을 실행한다.
Gazebo 컨테이너는 한 World의 두 고정 Base 모델과 관절 통신을 담당한다.
Gazebo 내부에는 Controller Manager가 없다. 임피던스·물체 운반·베이스 이동은 포함하지 않는다.

## 빌드

외부 저장소는 `src/ai_worker`, `src/cyclo_control`, `src/robotis_interfaces`,
`src/realsense-ros`에 둔다. 우리 코드는 `src/coop_carry_*` 두 패키지에 있다.
이전 workspace의 setup.bash를 불러오지 않은 새 Jazzy 터미널에서 실행한다.

```bash
cd ~/internship_ws
bash scripts/build_host.bash
docker compose -f docker/compose.yaml build
source scripts/environment.bash
```

Host는 명령·상태 Client에 필요한 패키지만 선택 빌드한다. Docker 이미지는
현재 clone한 Cyclo 소스도 빌드한다. 공식 bringup은 모델·World·Controller 설정을
재사용하며, 이번 시뮬레이션에서 사용하지 않는 실물·텔레오퍼레이션 패키지는 빌드하지 않는다.
Host와 Docker의 빌드 결과는 서로 공유하지 않는다.

## 실행

```bash
cd ~/internship_ws
source scripts/environment.bash
docker compose -f docker/compose.yaml up -d --no-build
ros2 run coop_carry_bringup robot_command --status
```

Compose가 Router를 하나 실행한다. domain 71, GZ partition `internship_dual`을 사용하며,
같은 포트 7447의 다른 Zenoh Router나 같은 domain의 이전 시뮬레이션을 함께 실행하지 않는다.
GUI를 사용하려면 먼저 실행을 종료한 뒤 아래처럼 시작한다.

```bash
xhost +si:localuser:root
COOP_GUI=true docker compose -f docker/compose.yaml up -d --no-build
```

## 명령과 검사

목표는 각 로봇의 **로컬 base_link 좌표계**다. offset은 현재 측정 자세를 기준으로 하며,
orientation을 생략하면 측정 방향을 유지한다. 이동 시간은 Cyclo에 전달하는 목표 시간이다.

```bash
# Robot 1 오른팔을 로컬 x 방향으로 1.5cm 이동
ros2 run coop_carry_bringup robot_command --robot robot1 --arm r --offset 0.015 0 0
# Robot 2 양팔에 목표 전달
ros2 run coop_carry_bringup robot_command --robot robot2 --arm both --offset 0.015 0 0
# 네 팔에 서로 다른 목표를 함께 전달; YAML에서 position 또는 offset과 orientation을 지정
ros2 run coop_carry_bringup robot_command --goals src/coop_carry_bringup/config/four_arm_goals.yaml
# 네 팔 개별, 로봇별 양팔, 로봇 간 교차, 네 팔 전체 동시 동작 검사
ros2 run coop_carry_bringup test_independent_arms --suite all
# 실제 컨테이너 PID namespace 및 로딩된 Hardware/Gazebo 플러그인 검사
python3 scripts/check_processes.py
# 모델·설정 검사
colcon test --base-paths src/coop_carry_bringup
colcon test-result --verbose
```

같은 팔에 여러 명령 Client를 동시에 사용하지 않는다. Controller 장애가 발생하면
원인을 해소한 뒤 해당 Robot 서비스를 재시작한다. World를 새로 시작할 때는 Robot도 함께 재시작한다.

```bash
docker compose -f docker/compose.yaml logs --tail=50 robot1 robot2 gazebo
docker compose -f docker/compose.yaml restart robot1
# 실행 종료
docker compose -f docker/compose.yaml down
# GUI용 접근을 허용했다면 종료 후 해제
xhost -si:localuser:root
```
