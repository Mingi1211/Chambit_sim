#!/usr/bin/env python3
"""정지 마찰(출발 문턱) 실측 — 관절 하나만 잠깐 전류 모드로 바꿔 Goal Current를 천천히 올림. 김민기 (11/17).

근거: MR 8.9.4 그림 8.13(b) — 정지 시 마찰은 [−b_static, b_static] 안의 값, 넘으면 출발
      BAM 2025 II.C 정지 영역(drive/backdrive): 정지 조건 |τ_m + τ_e| ≤ τ_f^m. 외부 토크 τ_e 고정, 모터 토크 τ_m을 바꿔
      정지 영역의 양 끝(+/− 방향 출발 순간)을 찾음 → τ_f^m = (τ₊ − τ₋)/2, −τ_e = (τ₊ + τ₋)/2
      BAM V.A 3)처럼 천천히 바꾸다 정지가 풀리는 순간을 기록. 토크 τ_m = c × Present Current (11/10 c)

방식 (공용 로봇 설정 파일은 건드리지 않음 — 실행 중에만 바꾸고 끝나면 원래 모드로 되돌림):
  ROBOTIS 하드웨어 인터페이스 서비스 set_dxl_data / get_dxl_data 사용 (dynamixel_hardware_interface.cpp)
  - 이 서비스로 Torque Enable 외 항목을 쓰면 인터페이스가 그 모터 토크를 먼저 끔 (dynamixel.cpp WriteItemBuf)
    → 쓸 때마다 Torque Enable 1을 뒤에 붙임 (Goal Current 쓰는 순간 수 ms 토크 꺼짐 △)
  - 순서: 원래 Operating Mode 읽기 → 0(전류) → Goal Current 0 → 토크 ON → 램프 → 0 → JTC 목표를 현재 자세로
          → Operating Mode 원래값 → 토크 ON.  예외·Ctrl+C에도 finally에서 복구
  - 그래도 이상하면 bringup 재시작 = xacro 설정으로 전부 초기화
안전: 램프는 위치 모드에서 버티던 전류에서 시작(잡는 자세 대비), 출발(|q − q₀| > --onset rad) 즉시 버티는 전류로, 토크 상한 --tau_max, 누적 이동 0.25 rad 넘으면 중단
기준 자세: 홈(팔 늘어뜨림) — 대부분 관절 중력 ≈ 0 → 토크 끄는 순간에도 안 떨어짐. j2는 한계(0)에서 떨어진 ±0.3

사용 (내 PC, kt_cmds.sh의 stic 명령):  --check 만 주면 서비스 찾기·모드 읽기만 (쓰기 없음)
  ssh robotis@<로봇> "docker exec -i ai_worker bash -ic 'python3 - --arm l --joint 6 --base 0 0 0 0 0 0 0'" < stiction.py | sed -n '/^t,/,$p' > l_stic_j6.csv
"""
import argparse
import sys
import time
from functools import partial

import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import DynamicJointState
from dynamixel_interfaces.srv import GetDataFromDxl, SetDataToDxl
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ap = argparse.ArgumentParser()
ap.add_argument('--arm', required=True, choices=['l', 'r'])
ap.add_argument('--joint', type=int, required=True)
ap.add_argument('--base', type=float, nargs=7, default=[0.0] * 7)
ap.add_argument('--trials', type=int, default=2)
ap.add_argument('--ramp_time', type=float, default=4.0, help='11/10 운동 마찰 f_c까지 올라가는 시간 [s]')
ap.add_argument('--tau_max_ratio', type=float, default=2.5, help='토크 상한 = 비율 × f_c(11/10)')
ap.add_argument('--onset', type=float, default=0.005, help='출발 판정 이동량 [rad]')
ap.add_argument('--check', action='store_true', help='서비스·모드 확인만 (쓰기 없음)')
a = ap.parse_args()
print = partial(print, file=sys.stderr)   # noqa: A001 — 안내는 stderr, 데이터만 stdout

k = a.joint - 1
JOINTS = [f'arm_{a.arm}_joint{i}' for i in range(1, 8)]
GRIP = f'gripper_{a.arm}_joint1'
DXL_ID = (list(range(1, 8)) if a.arm == 'r' else list(range(31, 38)))[k]
DXL = f'dxl{DXL_ID}'
NOM = [0.07692307692] * 3 + [0.04] * 3 + [1.0]               # ROBOTIS 모델 파일 Present/Goal Current 배율 [N·m/raw] (j7 raw)
C = {'l': [0.8099, 0.6729, 0.5772, 0.7537, 0.8187, 0.807, 0.0055],   # 11/10 kt_friction_table c_all
     'r': [0.7922, 0.5475, 0.7601, 0.7183, 0.8034, 0.8266, 0.0063]}
FC = {'l': [8.25, 6.036, 10.401, 4.713, 3.359, 3.201, 0.824],       # 11/10 운동 마찰 (박스 없음, 0.05 rad/s) [N·m]
      'r': [7.969, 7.247, 8.514, 5.959, 3.424, 3.095, 1.162]}
kj = C[a.arm][k] * NOM[k]                                     # 실제 [N·m / raw]
fc = FC[a.arm][k]
raw_max = int(a.tau_max_ratio * fc / kj)
rate = fc / a.ramp_time / kj                                  # [raw/s]

rclpy.init()
n = Node('mingi_stiction')
st = {'q': None, 'dq': None, 'I': None, 'all_q': {}}
rows = []
goal_raw = [0]
mark = [0]                                                    # 출발 순간 표시 (+1/−1) → CSV onset 열


def on_js(m):
    for nm, p, v in zip(m.name, m.position, m.velocity if len(m.velocity) == len(m.name) else [0.0] * len(m.name)):
        st['all_q'][nm] = p
        if nm == JOINTS[k]:
            st['q'], st['dq'] = p, v
    if st['q'] is not None and st['I'] is not None:
        rows.append(f"{time.time():.4f},{st['q']:.5f},{st['dq']:.5f},{st['I']:.4f},{goal_raw[0]},{mark[0]}")
        mark[0] = 0


def on_dyn(m):
    for nm, iv in zip(m.joint_names, m.interface_values):
        if nm == DXL and 'Present Current' in iv.interface_names:
            st['I'] = iv.values[iv.interface_names.index('Present Current')]


n.create_subscription(JointState, '/joint_states', on_js, 50)
n.create_subscription(DynamicJointState, '/dynamic_joint_states', on_dyn, 50)


def spin(sec):
    t = time.time() + sec
    while time.time() < t:
        rclpy.spin_once(n, timeout_sec=0.005)


def find(suffix):
    for nm, _ in n.get_service_names_and_types():
        if nm.endswith(suffix):
            return nm
    raise SystemExit(f'서비스 *{suffix} 없음 — `ros2 service list | grep dxl` 로 확인')


spin(2.0)
set_srv, get_srv = find('set_dxl_data'), find('get_dxl_data')
cs, cg = n.create_client(SetDataToDxl, set_srv), n.create_client(GetDataFromDxl, get_srv)
cs.wait_for_service(timeout_sec=3.0)
cg.wait_for_service(timeout_sec=3.0)


def call(cli, req):
    f = cli.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=3.0)
    if f.result() is None or not f.result().result:
        raise RuntimeError(f'서비스 실패: {req.item_name}')
    return f.result()


def get(item):
    return call(cg, GetDataFromDxl.Request(id=DXL_ID, item_name=item, timeout_sec=1.0)).item_data


def put(item, val, torque_on=True):
    call(cs, SetDataToDxl.Request(id=DXL_ID, item_name=item, item_data=int(val) & 0xFFFF if item == 'Goal Current' else int(val)))
    if torque_on:                                              # 인터페이스가 쓰기 전에 토크를 끄므로 다시 켬
        call(cs, SetDataToDxl.Request(id=DXL_ID, item_name='Torque Enable', item_data=1))


mode0 = get('Operating Mode')
print(f'{a.arm}팔 j{a.joint} = {DXL} (ID {DXL_ID}), 서비스 {set_srv}, 현재 Operating Mode {mode0}, '
      f'k_j {kj:.4f} N·m/raw, 상한 {raw_max} raw ({raw_max * kj:.1f} N·m), 램프 {rate:.1f} raw/s', flush=True)
if a.check:
    raise SystemExit(0)

jtc = ActionClient(n, FollowJointTrajectory, f'/arm_{a.arm}_controller/follow_joint_trajectory')
jtc.wait_for_server(timeout_sec=5.0)


def hold(q_arm, T=2.0):
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = JOINTS + [GRIP]
    pt = JointTrajectoryPoint()
    pt.positions = [float(v) for v in q_arm] + [float(st['all_q'].get(GRIP, 0.0))]
    pt.time_from_start.sec, pt.time_from_start.nanosec = int(T), int((T % 1) * 1e9)
    g.trajectory.points = [pt]
    f = jtc.send_goal_async(g)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
    r = f.result().get_result_async()
    rclpy.spin_until_future_complete(n, r, timeout_sec=T + 5.0)


print(f'기준 자세로 이동 {a.base}', flush=True)
hold(a.base, 4.0)
spin(1.0)
q_base = st['q']
i_hold = int(round(st['I'] / NOM[k]))                           # 위치 모드에서 지금 버티는 전류 [raw] — 잡는 자세는 중력이 커서 0 A로 시작하면 처짐
print(f'버티는 전류 {i_hold} raw = {i_hold * kj:+.2f} N·m (이 값에서 ± 램프)', flush=True)
res = {+1: [], -1: []}
try:
    put('Operating Mode', 0, torque_on=False)                  # 전류 모드 (토크 자동 OFF)
    goal_raw[0] = i_hold
    put('Goal Current', i_hold)                                  # 버티는 전류 + 토크 ON
    spin(1.0)
    print(f'전류 모드 진입. 버티는 전류에서 1 s 이동량 {st["q"] - q_base:+.4f} rad (≈ 0 이어야 함)', flush=True)
    for tr in range(a.trials):
        for s in (+1, -1):
            q0, t0 = st['q'], time.time()
            while True:
                goal_raw[0] = i_hold + int(s * rate * (time.time() - t0))
                if abs(goal_raw[0] - i_hold) > raw_max:
                    print(f'  시행{tr + 1} {"+" if s > 0 else "−"}: 상한 {raw_max * kj:.1f} N·m까지 안 움직임', flush=True)
                    break
                put('Goal Current', goal_raw[0])
                spin(0.03)
                if abs(st['q'] - q0) > a.onset:
                    tau = C[a.arm][k] * st['I']                # 측정 전류 기준 (명령값 아님)
                    res[s].append(tau)
                    mark[0] = s
                    print(f'  시행{tr + 1} {"+" if s > 0 else "−"}: 출발 토크 {tau:+.2f} N·m (명령 {goal_raw[0] * kj:+.2f})', flush=True)
                    break
                if abs(st['q'] - q_base) > 0.25:
                    raise RuntimeError('누적 이동 0.25 rad 초과 — 중단')
            goal_raw[0] = i_hold
            put('Goal Current', i_hold)
            spin(1.5)
finally:
    goal_raw[0] = i_hold if 'i_hold' in globals() else 0
    try:
        put('Goal Current', goal_raw[0])
        q_now = [st['all_q'].get(j, b) for j, b in zip(JOINTS, a.base)]
        hold(q_now, 1.0)                                       # 위치 모드 복귀 때 튀지 않게 현재 자세를 목표로
        put('Operating Mode', mode0, torque_on=False)
        call(cs, SetDataToDxl.Request(id=DXL_ID, item_name='Torque Enable', item_data=1))
        spin(0.5)
        print(f'복구: Operating Mode {get("Operating Mode")} (원래 {mode0}), 토크 {get("Torque Enable")}', flush=True)
        restored = get('Operating Mode') == mode0
    except Exception as e:                                     # noqa: BLE001
        restored = False
        print(f'!! 복구 실패 ({e}) — bringup을 재시작하세요', flush=True)

sys.stdout.write('t,q,dq,I,goal_raw,onset\n' + '\n'.join(rows) + '\n')
sys.stdout.flush()
# BAM II.C 정지 영역: 정지 조건 |τ_m + τ_e| ≤ τ_f^m → 경계 τ_m = −τ_e ± τ_f^m
#   ⇒ τ_f^m = (τ₊ − τ₋)/2 (정지 마찰 예산 = MR b_static),  −τ_e = (τ₊ + τ₋)/2 (그 자세 외부 토크 = 중력)
#   반복성: BAM V.B처럼 같은 시험 반복값을 그대로 보고 (합격 수치 기준은 논문에 없음)
if res[+1] and res[-1]:
    tp, tn = sum(res[+1]) / len(res[+1]), sum(res[-1]) / len(res[-1])
    print(f'결과 {a.arm}팔 j{a.joint}: τ_f^m = (τ₊ − τ₋)/2 = {(tp - tn) / 2:.2f} N·m, −τ_e = (τ₊ + τ₋)/2 = {(tp + tn) / 2:+.2f} N·m | '
          f'시행별 τ₊ {" ".join(f"{x:+.2f}" for x in res[+1])}, τ₋ {" ".join(f"{x:+.2f}" for x in res[-1])}', flush=True)
else:
    print(f'결과 {a.arm}팔 j{a.joint}: 출발 없는 방향 있음 (상한 {raw_max * kj:.1f} N·m)', flush=True)
raise SystemExit(0 if restored else 2)
