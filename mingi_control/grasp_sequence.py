#!/usr/bin/env python3
"""두 로봇 4팔 파지 → 임피던스 전환.

1 그리퍼 열기 → 2 접근 자세(그리퍼 90° 돌려 박스 쪽, 손잡이 앞) → 3 간격을 확인하며 x축 전진(손잡이가 손가락 사이로)
→ 4 닫기 + 접촉 감지(손가락이 목표 전에 멈추면 접촉) → 5 접촉 위치 + squeeze로 고정 → 6 토크(임피던스) 모드 전환
1~3은 현대 QP(MoveL) 위치 제어. 전제: 로봇마다 impedance_node가 mode:=position으로 실행 중.

사용: python3 grasp_sequence.py [--robots robot1 robot2]
"""
import argparse
import math
import os
import re
import subprocess
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from tf2_ros import Buffer, TransformListener
from robotis_interfaces.msg import MoveL
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

ap = argparse.ArgumentParser()
ap.add_argument('--robots', nargs='+', default=['robot1', 'robot2'])
ap.add_argument('--grasp-only', action='store_true',
                help='현재 팔 자세를 유지하고 그리퍼 닫기/접촉 확인만 실행')
ap.add_argument('--y', type=float, help='손잡이 y (base_link, 기본값: robots.yaml에서 계산)')
ap.add_argument('--z', type=float, help='손잡이 높이 (base_link, 기본값: robots.yaml에서 계산)')
ap.add_argument('--x_pre', type=float, help='접근 자세 EE x (base_link, 기본값: 손잡이 앞 4 cm)')
ap.add_argument('--x_grasp', type=float, help='파지 EE x (base_link, 기본값: 손잡이 중심 + tool offset)')
ap.add_argument('--box_config', default=str(Path(__file__).resolve().parents[1] / 'src/coop_carry_bringup/config/robots.yaml'),
                help='박스 및 손잡이 설정 YAML')
ap.add_argument('--approach_distance', type=float, default=0.04, help='손잡이 앞에서 시작할 접근 거리 [m]')
ap.add_argument('--tool_offset_x', type=float, default=0.0205,
                help='base_link +x 방향 EE 목표 오프셋 [m]')
ap.add_argument('--grasp_z_offset', type=float, default=0.023,
                help='FK에서 확인한 닫힘축 중심 오차 보정 [m], base_link +z')
ap.add_argument('--forward_step', type=float, default=0.02, help='전진 중간 확인 간격 [m]')
ap.add_argument('--z_up', type=float, default=0.06, help='회전할 때 손잡이보다 위로 띄우는 높이')
ap.add_argument('--squeeze', type=float, default=0.05, help='접촉 위치에서 더 닫는 양 [rad]')
a = ap.parse_args()
for name in ('forward_step', 'approach_distance'):
    value = getattr(a, name)
    if not np.isfinite(value) or value <= 0:
        ap.error(f'--{name}은 유한한 양수여야 합니다')
QUAT = (-0.5, -0.5, 0.5, 0.5)       # 손끝이 +x(박스 쪽), 손가락은 상하로 닫힘 — 두 팔 모두 IK 도달 확인
OPEN, CLOSED = 0.0, 1.1


def load_grasp_targets():
    """Gazebo 실측 박스 포즈와 시뮬레이션 형상으로 손잡이 중심을 계산."""
    config = yaml.safe_load(Path(a.box_config).read_text())
    box = config['box']
    robots = {robot['name']: robot for robot in config['robots']}

    def rotation(yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    env = dict(os.environ, GZ_PARTITION=os.environ.get('GZ_PARTITION', 'internship_dual'))
    pose = subprocess.run(['gz', 'model', '-m', 'box', '-p'], capture_output=True,
                          text=True, env=env, timeout=5.0, check=True).stdout
    pose_match = re.search(
        r'Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\n\s*'
        r'\[([-\d.e+ ]+)\]\s*\n\s*\[([-\d.e+ ]+)\]', pose)
    if not pose_match:
        raise RuntimeError(f'Gazebo box pose 형식을 읽지 못했습니다: {pose.strip()}')
    box_center_world = np.fromstring(pose_match.group(1), sep=' ')
    rpy = np.fromstring(pose_match.group(2), sep=' ')
    if box_center_world.size != 3 or rpy.size != 3:
        raise RuntimeError(f'Gazebo box 위치/방향이 잘못되었습니다: {pose_match.groups()}')
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    R_box = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    sx = float(box['size'][0])
    handle = box['handle']
    protrude = float(handle['protrude'])
    width_y = float(handle['y'])
    center_z = float(handle.get('z', 0.0))
    result, handle_centers = {}, {}
    for robot_name in a.robots:
        robot = robots[robot_name]
        R_robot = rotation(float(robot.get('yaw', 0.0)))
        robot_world = np.array([robot['x'], robot['y'], robot['z']], dtype=float)
        # 박스 local x 방향에서 이 로봇이 바라보는 끝면의 부호.
        robot_in_box = R_box.T @ (robot_world - box_center_world)
        face_sign = -1.0 if robot_in_box[0] < 0 else 1.0
        # 박스 좌표계의 y축은 로봇 yaw=pi에서 base_link y축과 반대가 된다.
        robot_y_in_box = R_box.T @ R_robot @ np.array([0.0, 1.0, 0.0])
        local_y_sign = 1.0 if robot_y_in_box[1] >= 0 else -1.0
        for side in ('r', 'l'):
            side_sign = -1.0 if side == 'r' else 1.0
            local_y = local_y_sign * side_sign * width_y
            handle_offset = np.array([face_sign * (sx / 2 + protrude / 2), local_y, center_z])
            handle_world = box_center_world + R_box @ handle_offset
            handle_center = R_robot.T @ (handle_world - robot_world)
            target = handle_center.copy()
            # SG2 end_effector_link는 손가락 접촉면보다 뒤쪽에 있다.
            # RH-P12 URDF의 그리퍼 조립 오프셋과 손가락 mesh 끝에서 계산한 +20.5 mm를 적용.
            target[0] += a.tool_offset_x
            target[2] += a.grasp_z_offset
            # 사용자 지정 좌표는 진단/수동 보정용이며, 기본은 형상에서 계산한다.
            if a.x_grasp is not None:
                target[0] = a.x_grasp
            if a.y is not None:
                target[1] = side_sign * a.y
            if a.z is not None:
                target[2] = a.z
            result[robot_name, side] = target
            handle_centers[robot_name, side] = handle_center
    return result, handle_centers


grasp_targets, handle_centers = load_grasp_targets()
approach_targets = {
    key: target + np.array([-a.approach_distance, 0.0, 0.0])
    for key, target in grasp_targets.items()
}
if a.x_pre is not None:
    for target in approach_targets.values():
        target[0] = a.x_pre
# 안전 hover는 손잡이 중심보다 z_up만큼 위로 고정한다. 닫힘축 보정은
# 접촉 높이에만 적용해야 안전 자세까지 불필요하게 높아져 IK가 막히지 않는다.
hover_targets = {key: target + np.array([0.0, 0.0, a.z_up - a.grasp_z_offset])
                 for key, target in approach_targets.items()}

rclpy.init()
n = Node('grasp_sequence', parameter_overrides=[Parameter('use_sim_time', value=True)])
state, sim_state, robot_models, adapter_ready = {}, {}, {}, {}
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, n)
pub_goal, pub_grip, pub_mode = {}, {}, {}
description_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)


def on_robot_description(msg, robot):
    if robot not in robot_models:
        robot_models[robot] = pin.buildModelFromXML(msg.data)


for r in a.robots:
    n.create_subscription(JointState, f'/{r}/joint_states',
                          lambda m, r=r: state.__setitem__(r, dict(zip(m.name, m.position))), 10)
    n.create_subscription(JointState, f'/sim/{r}/joint_states',
                          lambda m, r=r: sim_state.__setitem__(r, m), qos_profile_sensor_data)
    n.create_subscription(String, f'/{r}/robot_description',
                          lambda m, r=r: on_robot_description(m, r),
                          description_qos)
    n.create_subscription(DiagnosticArray, f'/{r}/status',
                          lambda m, r=r: adapter_ready.__setitem__(r, bool(m.status) and
                              m.status[0].level == DiagnosticStatus.OK and m.status[0].message == 'ready'), 10)
    pub_mode[r] = n.create_publisher(String, f'/{r}/impedance/mode', 10)
    for s in 'rl':
        pub_goal[r, s] = n.create_publisher(MoveL, f'/{r}/{s}_goal_move', 10)
        pub_grip[r, s] = n.create_publisher(Float64, f'/{r}/impedance/{s}/gripper', 10)


def spin(sec):
    t = time.time()
    while time.time() - t < sec:
        rclpy.spin_once(n, timeout_sec=0.01)


def ee(r, s):
    """현대 robot_state_publisher가 내보내는 TF에서 실제 손끝 자세를 읽는다."""
    transform = tf_buffer.lookup_transform(f'{r}/base_link', f'{r}/end_effector_{s}_link', Time()).transform
    position = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    orientation = np.array([transform.rotation.x, transform.rotation.y,
                             transform.rotation.z, transform.rotation.w])
    return position, orientation


def report_finger_alignment(r, s, stage):
    """Pinocchio FK of both distal finger links vs the handle center, all in base_link."""
    model = robot_models.get(r)
    msg = sim_state.get(r)
    if model is None or msg is None:
        say(f'  FK 진단 생략 {r}/{s}/{stage}: robot_description 또는 /sim joint_states 없음')
        return
    q = pin.neutral(model)
    qmap = {model.names[j]: model.joints[j].idx_q
            for j in range(1, model.njoints) if model.joints[j].nq == 1}
    for name, value in zip(msg.name, msg.position):
        joint = name.split('::')[-1]
        if joint in qmap:
            q[qmap[joint]] = value
    data = model.createData()
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    base = handle_centers[r, s]
    points = {}
    for finger in ('r2', 'l2'):
        frame_name = f'gripper_{s}_rh_p12_rn_{finger}'
        fid = model.getFrameId(frame_name)
        if fid >= len(model.frames):
            say(f'  FK 진단 링크 없음: {frame_name}')
            return
        points[finger] = data.oMf[fid].translation.copy()
    gap = points['l2'] - points['r2']
    gap_norm = float(np.linalg.norm(gap))
    axis = gap / max(gap_norm, 1e-9)
    midpoint = (points['l2'] + points['r2']) / 2
    delta = base - midpoint
    along = float(np.dot(delta, axis))
    across = delta - along * axis
    near = min(points, key=lambda key: np.linalg.norm(points[key] - base))
    say(f'  FK {r}/{s}/{stage} base_link m: r2={np.round(points["r2"], 4).tolist()} '
        f'l2={np.round(points["l2"], 4).tolist()} handle={np.round(base, 4).tolist()}')
    say(f'    손가락 간격={gap_norm*1000:.1f} mm, handle-mid={np.round(delta*1000,1)} mm '
        f'(닫힘축 {along*1000:+.1f} mm, 직교 {np.round(across*1000,1)} mm), 먼저 닿을 쪽 추정={near}')


def move(targets, T, tries=8, tol=0.01, angle_tol=0.05, orientations=None):
    """현대 QP에 고정 목표를 보내고 TF 오차를 확인한다. 재시도해도 목표는 바꾸지 않는다."""
    want = {k: v.copy() for k, v in targets.items()}
    want_q = {k: np.asarray((orientations or {}).get(k, QUAT), dtype=float) for k in want}
    for i in range(tries):
        send(want, T if i == 0 else max(5.0, T), want_q)
        poses = {k: ee(*k) for k in want}
        err = {k: want[k] - poses[k][0] for k in want}
        worst = max(np.linalg.norm(e) for e in err.values())
        angle = {k: 2 * math.acos(float(np.clip(abs(np.dot(want_q[k], poses[k][1])), 0.0, 1.0))) for k in want}
        worst_angle = max(angle.values())
        say(f'  위치 오차 최대 {worst*1000:.1f} mm, 방향 오차 최대 {np.degrees(worst_angle):.2f}° '
            + ' '.join(f'{k[0][-1]}{k[1]}:{np.round(e*1000,0)}mm/{np.degrees(angle[k]):.1f}°' for k, e in err.items()))
        if worst < tol and worst_angle < angle_tol:
            return True
    return False


def spacing_ok(tol=0.015):
    """각 로봇의 두 손이 목표 간격·평행도를 유지하는지 TF로 확인."""
    ok = True
    for r in a.robots:
        actual = ee(r, 'l')[0] - ee(r, 'r')[0]
        expected = grasp_targets[r, 'l'] - grasp_targets[r, 'r']
        error = actual - expected
        gap_error = np.linalg.norm(error)
        say(f'  {r} 양팔 간격 오차 {gap_error*1000:.1f} mm (x/y/z={np.round(error*1000, 1)})')
        ok = ok and gap_error <= tol
    return ok


def move_forward(T):
    """모든 팔을 동기화해 x축으로만 조금씩 전진, 매 구간에서 간격 확인."""
    distance = max(abs(grasp_targets[k][0] - approach_targets[k][0]) for k in pub_goal)
    count = max(1, int(np.ceil(distance / a.forward_step)))
    for fraction in np.linspace(0.0, 1.0, count + 1)[1:]:
        targets = {k: approach_targets[k] + fraction * (grasp_targets[k] - approach_targets[k]) for k in pub_goal}
        if not move(targets, T / count):
            say(f'  전진 중 위치 오차 초과 단계={fraction:.2f} — 다음 구간 중단')
            return False
        if not spacing_ok():
            say(f'  양팔 간격/평행도 허용 오차 초과 단계={fraction:.2f} — 다음 구간 중단')
            return False
    return True


def send(cmd, T, orientations=None):
    for (r, s), p in pub_goal.items():
        g = MoveL()
        g.pose.header.frame_id = 'base_link'
        g.pose.pose.position.x, g.pose.pose.position.y, g.pose.pose.position.z = cmd[r, s]
        q = (orientations or {}).get((r, s), QUAT)
        g.pose.pose.orientation.x, g.pose.pose.orientation.y, g.pose.pose.orientation.z, g.pose.pose.orientation.w = q
        g.time_from_start.sec = int(T)
        g.time_from_start.nanosec = int(round((T - int(T)) * 1_000_000_000))
        if g.time_from_start.nanosec == 1_000_000_000:
            g.time_from_start.sec += 1
            g.time_from_start.nanosec = 0
        p.publish(g)
    spin(T + 1.5)


def grip(values):
    for k, v in values.items():
        pub_grip[k].publish(Float64(data=v))


def say(msg):
    print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)


if a.grasp_only:
    spin(1.0)
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        ready = (all(r in state for r in a.robots)
                 and all(pub.get_subscription_count() > 0 for pub in pub_grip.values()))
        if ready:
            break
        spin(0.2)
    missing_grippers = [f'{r}/{s}' for (r, s), pub in pub_grip.items()
                        if pub.get_subscription_count() == 0]
    if missing_grippers:
        say(f'그리퍼 구독자 대기 시간 초과: {missing_grippers}. impedance_node를 해당 로봇 namespace로 실행하세요.')
        n.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)
    for r in a.robots:
        if pub_mode[r].get_subscription_count():
            pub_mode[r].publish(String(data='position'))
    spin(0.2)
    say('현재 팔 위치를 유지하며 네 그리퍼 닫기')
    grip({k: CLOSED for k in pub_grip})
    hist = {k: [] for k in pub_grip}
    contact = {}
    t0 = time.time()
    while len(contact) < len(pub_grip) and time.time() - t0 < 5.0:
        spin(0.05)
        for (r, s) in pub_grip:
            q = state.get(r, {}).get(f'gripper_{s}_joint1')
            if q is None or (r, s) in contact:
                continue
            hist[r, s].append((time.time(), q))
            old = [v for t, v in hist[r, s] if time.time() - t > 0.3]
            if old and abs(q - old[-1]) < 0.003 and time.time() - t0 > 0.5:
                contact[r, s] = q
                say(f'  {r}/{s}: 멈춤 q={q:.3f} rad → ' +
                    ('접촉(손잡이 잡음)' if q < CLOSED - 0.1 else '끝까지 닫힘 = 놓침'))
    miss = [k for k, q in contact.items() if q >= CLOSED - 0.1] + [k for k in pub_grip if k not in contact]
    if miss:
        say(f'파지 실패 {miss} — 토크 전환 안 함')
        n.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)
    grip({k: q + a.squeeze for k, q in contact.items()})
    spin(1.0)
    for r in a.robots:
        if pub_mode[r].get_subscription_count():
            pub_mode[r].publish(String(data='task'))
    spin(1.0)
    say('파지 완료: 현재 위치에서 임피던스 유지 중')
    n.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0)


spin(1.5)                                                   # 탐색 대기
wait_started = time.monotonic()
while True:
    ready = (len(state) == len(a.robots)
             and all(adapter_ready.get(r, False) for r in a.robots))
    if ready:
        try:
            for r in a.robots:
                for s in 'rl':
                    ee(r, s)
            break
        except Exception:
            pass  # TF frame tree is still arriving; keep waiting for the modern node graph.
    if time.monotonic() - wait_started > 60.0:
        raise RuntimeError(f'60초 안에 로봇 상태/TF/어댑터 준비가 완료되지 않았습니다. 상태: {list(state)}, 어댑터 준비: {adapter_ready}')
    spin(0.2)
spin(0.5)
graph = n.get_node_names_and_namespaces()
duplicate_controllers = {
    r: sum(name == 'impedance_controller' and namespace.rstrip('/') == f'/{r}'
           for name, namespace in graph)
    for r in a.robots
}
bad_controllers = {r: count for r, count in duplicate_controllers.items() if count != 1}
if bad_controllers:
    say(f'impedance_node는 로봇마다 정확히 하나여야 합니다: {bad_controllers}. '
        '터미널 clear는 프로세스를 종료하지 않습니다. 기존 노드를 Ctrl+C로 끝낸 뒤 robot당 하나만 실행하세요.')
    n.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1)
say('Gazebo 박스 실측 pose 기준으로 계산한 손잡이 중심 (각 base_link, m): ' + ' '.join(
    f'{r}/{s}={np.round(p, 4).tolist()}' for (r, s), p in grasp_targets.items()))
for r in a.robots:
    if pub_mode[r].get_subscription_count():
        pub_mode[r].publish(String(data='position'))
say('1 그리퍼 열기'); grip({k: OPEN for k in pub_grip}); spin(1.5)
current_orientations = {k: ee(*k)[1] for k in pub_goal}
say('2a 현재 손 방향을 유지하며 손잡이 위 안전 위치로 이동: ' + str({k: np.round(v, 3).tolist() for k, v in hover_targets.items()}))
current_positions = {k: ee(*k)[0] for k in pub_goal}
hover_midpoints = {k: (current_positions[k] + hover_targets[k]) / 2 for k in pub_goal}
if not move(hover_midpoints, 5, tries=12, orientations=current_orientations):
    say('중간 안전 waypoint 오차가 남아 이후 이동을 중단합니다'); raise SystemExit(1)
if not move(hover_targets, 5, tries=12, orientations=current_orientations):
    say('안전 접근 자세가 목표 오차 안에 들지 않아 하강을 중단합니다'); raise SystemExit(1)
say('2b 안전 높이에서 손잡이 파지 방향으로 회전')
grasp_orientations = {k: QUAT for k in pub_goal}
if not move(hover_targets, 5, tries=12, tol=0.03, angle_tol=0.10, orientations=grasp_orientations):
    say('안전 높이에서 파지 방향을 맞추지 못해 하강을 중단합니다'); raise SystemExit(1)
rotated_positions = {k: ee(*k)[0] for k in pub_goal}
approach_targets = {
    k: np.array([rotated_positions[k][0], rotated_positions[k][1], grasp_targets[k][2]])
    for k in pub_goal
}
say('회전 후 실제 TF x/y를 기준으로 하강 위치 재설정: ' + str(
    {k: np.round(v, 3).tolist() for k, v in approach_targets.items()}))
say('2c 손잡이 높이까지 수직 하강')
if not move(approach_targets, 3):
    say('손잡이 앞 정렬이 목표 오차 안에 들지 않아 전진을 중단합니다'); raise SystemExit(1)
if not spacing_ok():
    say('전진 전 양팔 간격/평행도 오차가 허용치 15 mm 초과 — 파지 중단'); raise SystemExit(1)
say(f'3 손잡이 중심까지 x 방향으로만 천천히 전진, 구간 간격 ≤{a.forward_step*1000:.0f} mm')
if not move_forward(4):
    say('정렬/전진 검증 실패 — 그리퍼를 닫지 않고 중단'); raise SystemExit(1)
for r, s in pub_goal:
    report_finger_alignment(r, s, '닫기 직전')
missing_grippers = [f'{r}/{s}' for (r, s), pub in pub_grip.items() if pub.get_subscription_count() == 0]
if missing_grippers:
    say(f'그리퍼 명령을 받을 현대 impedance_controller가 없습니다: {missing_grippers} — 해당 로봇의 impedance_node를 시작한 뒤 다시 실행하세요')
    raise SystemExit(1)
say('4 닫기 + 접촉 감지'); grip({k: CLOSED for k in pub_grip})
hist = {k: [] for k in pub_grip}
contact = {}
t0 = time.time()
while len(contact) < len(pub_grip) and time.time() - t0 < 5.0:
    spin(0.05)
    for (r, s) in pub_grip:
        q = state.get(r, {}).get(f'gripper_{s}_joint1')
        if q is None or (r, s) in contact:
            continue
        hist[r, s].append((time.time(), q))
        old = [v for t, v in hist[r, s] if time.time() - t > 0.3]
        if old and abs(q - old[-1]) < 0.003 and time.time() - t0 > 0.5:   # 0.3 s 동안 거의 안 움직임 = 멈춤
            contact[r, s] = q
            say(f'  {r}/{s}: 멈춤 q={q:.3f} rad → ' + ('접촉(손잡이 잡음)' if q < CLOSED - 0.1 else '끝까지 닫힘 = 놓침'))
miss = [k for k, q in contact.items() if q >= CLOSED - 0.1] + [k for k in pub_grip if k not in contact]
if miss:
    say(f'파지 실패 {miss} — 손잡이 위치와 --tool_offset_x를 확인하세요. 토크 전환 안 함')
else:
    for r, s in pub_grip:
        report_finger_alignment(r, s, f'접촉 q={contact[r, s]:.3f}')
    grip({k: q + a.squeeze for k, q in contact.items()})       # 접촉 위치 + 조금 더 → 쥐는 힘 제한
    spin(1.0)
    say('5 고정 완료 → 6 토크(임피던스) 모드 전환')
    for r in a.robots:
        pub_mode[r].publish(String(data='task'))
    spin(1.0)
    say('완료: 임피던스로 현재 자세 유지 중')
n.destroy_node()
rclpy.shutdown()
