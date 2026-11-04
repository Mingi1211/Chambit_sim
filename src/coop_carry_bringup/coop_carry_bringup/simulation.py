"""같은 공식 모델에서 Gazebo용 모델과 외부 제어용 모델을 생성한다."""

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from .model_factory import generate_model, read_robots, share
from .swerve_model import BASE_JOINTS, add_swerve


def write_xml(root, path):
    """작성 중인 파일을 읽지 않도록 임시 파일을 완성한 뒤 교체한다."""
    ET.indent(root)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(ET.tostring(root, encoding="unicode"))
    temporary.replace(path)


def _add_remote_hardware(control, name):
    """제어용 URDF에 Gazebo와 상태·명령을 교환할 하드웨어를 추가한다."""
    system = ET.SubElement(
        control,
        "ros2_control",
        {"name": f"{name}_remote_sim", "type": "system", "is_async": "false"},
    )
    hardware = ET.SubElement(system, "hardware")
    ET.SubElement(hardware, "plugin").text = "coop_carry_hardware/RemoteGazeboSystem"
    for key, value in {
        "state_topic": f"/sim/{name}/joint_states",
        "command_topic": f"/sim/{name}/joint_commands",
        "state_timeout": "0.5",
    }.items():
        ET.SubElement(hardware, "param", {"name": key}).text = value
    return system


def _add_gazebo_actuator(physics, name, command_count):
    """물리용 URDF에 관절 구동 플러그인을 추가한다."""
    gazebo = ET.SubElement(physics, "gazebo")
    actuator = ET.SubElement(
        gazebo,
        "plugin",
        {
            "filename": "libcoop_carry_gazebo_actuator.so",
            "name": "coop_carry_hardware::GazeboJointActuator",
        },
    )
    for key, value in {
        "command_topic": f"/sim/{name}/actuators",
        "command_timeout": "0.5",
        "position_gain": "10.0",
        "command_count": str(command_count),
    }.items():
        ET.SubElement(actuator, key).text = value
    return gazebo, actuator


def _joint_mapping(joints, commanded, initial_positions):
    """관절 이름·제한·명령 순서·mimic 관계를 하나의 목록으로 만든다."""
    indices = {joint.get("name"): index for index, joint in enumerate(joints)}
    mapping = []
    for joint in joints:
        joint_name = joint.get("name")
        limit = joint.find("limit")
        mimic = joint.find("mimic")
        item = dict(
            name=joint_name,
            initial=float(initial_positions.get(joint_name, 0.0)),
            lower=float(limit.get("lower", "-1e9")),
            upper=float(limit.get("upper", "1e9")),
            velocity=min(float(limit.get("velocity")), 6.5),
            command_index=(
                commanded.index(joint_name) if joint_name in commanded else -1
            ),
            mimic_index=indices[mimic.get("joint")] if mimic is not None else -1,
        )
        mapping.append(item)
    return mapping


def _add_joint_interfaces(system, actuator, mapping):
    """동일한 관절 정의로 Gazebo 구동과 ros2_control 인터페이스를 연결한다."""
    for item in mapping:
        definition = ET.SubElement(actuator, "joint")
        for key, value in item.items():
            ET.SubElement(definition, key).text = str(value)
        # Mimic feedback is produced by robot_state_publisher from its source joint.
        if item["mimic_index"] == -1:
            entry = ET.SubElement(system, "joint", {"name": item["name"]})
            if item["command_index"] >= 0:
                cmd = ET.SubElement(entry, "command_interface", {"name": "position"})
                for bound in ("min", "max"):
                    ET.SubElement(cmd, "param", {"name": bound}).text = str(
                        item["lower" if bound == "min" else "upper"]
                    )
            for interface in ("position", "velocity"):
                state = ET.SubElement(entry, "state_interface", {"name": interface})
                ET.SubElement(state, "param", {"name": "initial_value"}).text = str(
                    item["initial"] if interface == "position" else 0.0
                )


def _add_joint_state_publisher(gazebo, name):
    publisher = ET.SubElement(
        gazebo,
        "plugin",
        {
            "filename": "gz-sim-joint-state-publisher-system",
            "name": "gz::sim::systems::JointStatePublisher",
        },
    )
    ET.SubElement(publisher, "topic").text = f"/sim/{name}/joint_states"
    ET.SubElement(publisher, "update_rate").text = "100"


def _bridge_entries(name):
    """관절 상태는 Gazebo→ROS, 구동 명령은 ROS→Gazebo로 연결한다."""
    bridge = []
    for topic, ros_type, gz_type, direction in [
        (
            f"/sim/{name}/joint_states",
            "sensor_msgs/msg/JointState",
            "gz.msgs.Model",
            "GZ_TO_ROS",
        ),
        (
            f"/sim/{name}/actuators",
            "actuator_msgs/msg/Actuators",
            "gz.msgs.Actuators",
            "ROS_TO_GZ",
        ),
    ]:
        bridge.append(
            dict(
                ros_topic_name=topic,
                gz_topic_name=topic,
                ros_type_name=ros_type,
                gz_type_name=gz_type,
                direction=direction,
            )
        )
    return bridge


def _add_effort_path(physics, gazebo, actuator, robot):
    """[문서 밖] 김민기 하위 제어: effort 모드에서만 토크 경로를 추가한다. position이면 현대 기본 동작 그대로."""
    name = robot["name"]
    # 그리퍼 쥐는 힘 제한: Gazebo 위치 서보는 관절 effort 한계까지 밀어붙임(URDF 1000) → 손잡이에 파고들어
    # 접촉 계산 폭주(RTF 0.007, 11/3 확인). 물리 모델에서만 그리퍼 관절 effort 한계를 낮춤
    for joint in physics.findall("joint"):
        # 쥐는 토크는 구동 관절(joint1)에만. mimic 관절(2·3·4)은 실물에선 강체 링크라 한계를 낮추면 손잡이에 밀려 접힘(11/5 확인)
        if joint.get("name", "").startswith("gripper_") and joint.get("name", "").endswith("joint1") and joint.find("limit") is not None:
            joint.find("limit").set("effort", str(robot.get("gripper_effort", 5.0)))
    # 토크 명령 경로: /sim/<robot>/effort (Actuators.normalized = 관절 토크, NaN = 위치 서보 유지)
    for key, value in {
        "effort_topic": f"/sim/{name}/effort",
        "effort_timeout": "0.1",
        "effort_limit": "60.0",
    }.items():
        ET.SubElement(actuator, key).text = value
    # 상태 주기 = 토크 루프 주기. 문서 #8 감쇠 상한 D < 2Λ/T 때문에 T를 줄일 수 있게 설정값으로
    for plugin in gazebo.findall("plugin"):
        if plugin.get("name") == "gz::sim::systems::JointStatePublisher":
            plugin.find("update_rate").text = str(robot.get("state_rate", 100))
    # 손가락 충돌은 원본 메시 그대로 (11/5: 손끝을 경계 상자로 바꾸면 손잡이 접촉이 사라져 파지 실패 → 되돌림).
    # 쥐는 힘·마찰·미끄러짐은 접촉 역학으로 계산 (고정 구속 없음). 손끝 마찰계수 = 박스와 같은 μ
    for link in physics.findall("link"):
        if link.get("name", "").startswith("gripper_") and link.get("name", "").rsplit("_", 1)[-1] in ("r1", "r2", "l1", "l2"):
            reference = ET.SubElement(physics, "gazebo", {"reference": link.get("name")})
            ET.SubElement(reference, "mu1").text = str(robot.get("finger_mu", 1.0))
            ET.SubElement(reference, "mu2").text = str(robot.get("finger_mu", 1.0))
    return [
        dict(
            ros_topic_name=f"/sim/{name}/effort",
            gz_topic_name=f"/sim/{name}/effort",
            ros_type_name="actuator_msgs/msg/Actuators",
            gz_type_name="gz.msgs.Actuators",
            direction="ROS_TO_GZ",
        )
    ]


def handles(box, sx, surface):
    """양 끝면(로봇 쪽)에 수평 손잡이 4개 — 그리퍼(앞을 향함, 손가락 상하로 닫힘)가 집는다"""
    h = box.get('handle')
    if not h:
        return ''
    out, (p, w, t) = [], (h['protrude'], h['width'], h['thickness'])
    for i, (sx_sign, y) in enumerate([(s, y) for s in (-1, 1) for y in (-h['y'], h['y'])]):
        x = sx_sign * (sx / 2 + p / 2)
        geo = f"<pose>{x} {y} {h.get('z', 0.0)} 0 0 0</pose><geometry><box><size>{p} {w} {t}</size></box></geometry>"
        out.append(f"<collision name='h{i}'>{geo}{surface}</collision><visual name='hv{i}'>{geo}</visual>")
    return ''.join(out)


def configure_effort_world(world, robots_file):
    """[문서 밖] 김민기: 토크 제어용 월드 설정.
    ① 물리 간격 10 ms → 1 ms (10 ms에서는 토크 루프가 지연으로 진동 — 10/28 확인)
    ② ApplyLinkWrench: 시험용 외력 입력(실물의 무게추 걸기에 해당, 센서 아님)
    ③ 박스·받침대 (robots.yaml의 box 절)"""
    physics = world.find('physics')
    physics.find('max_step_size').text = '0.001'
    physics.find('real_time_update_rate').text = '1000'
    ET.SubElement(world, 'plugin', {'name': 'gz::sim::systems::ApplyLinkWrench',
                                    'filename': 'gz-sim-apply-link-wrench-system'})
    config = yaml.safe_load(Path(robots_file).read_text())
    box = config.get('box')
    if not box:
        return
    base = next(r for r in config['robots'] if r['name'] == box['robot'])
    yaw = float(base.get('yaw', 0.0))
    import math
    bx, by, bz = box['center_base']                      # 로봇 base_link 기준 박스 중심
    wx = base['x'] + math.cos(yaw) * bx - math.sin(yaw) * by
    wy = base['y'] + math.sin(yaw) * bx + math.cos(yaw) * by
    wz = base['z'] + bz                                 # 스워브는 robots.yaml z 0 (바닥에 선 base_link)
    sx, sy, sz = box['size']
    m, mu = float(box['mass']), float(box['mu'])
    ixx, iyy, izz = m * (sy**2 + sz**2) / 12, m * (sx**2 + sz**2) / 12, m * (sx**2 + sy**2) / 12
    surface = f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>"
    world.append(ET.fromstring(f"""<model name='box'><pose>{wx} {wy} {wz} 0 0 {yaw}</pose><link name='link'>
      <inertial><mass>{m}</mass><inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
      <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name='c'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>{surface}</collision>
      <visual name='v'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      <material><ambient>0.8 0.5 0.2 1</ambient><diffuse>0.8 0.5 0.2 1</diffuse></material></visual>
      {handles(box, sx, surface)}</link></model>"""))
    th = wz - sz / 2                                      # 받침대 윗면 = 박스 바닥
    tw = box.get('table_size', [sx, 0.6 * sy])
    place = box.get('place')                              # 내려놓을 곳: 같은 높이 받침대를 월드 (dx, dy)만큼 옆에
    if place:
        world.append(ET.fromstring(f"""<model name='table_place'><static>true</static>
      <pose>{wx + place.get('dx', 0.0)} {wy + place.get('dy', 0.0)} {th / 2} 0 0 {yaw}</pose><link name='link'>
      <collision name='c'><geometry><box><size>{tw[0]} {tw[1]} {th}</size></box></geometry>{surface}</collision>
      <visual name='v'><geometry><box><size>{tw[0]} {tw[1]} {th}</size></box></geometry>
      <material><ambient>0.3 0.6 0.3 1</ambient><diffuse>0.3 0.6 0.3 1</diffuse></material></visual>
      </link></model>"""))
    world.append(ET.fromstring(f"""<model name='table'><static>true</static>
      <pose>{wx} {wy} {th / 2} 0 0 {yaw}</pose><link name='link'>
      <collision name='c'><geometry><box><size>{tw[0]} {tw[1]} {th}</size></box></geometry>{surface}</collision>
      <visual name='v'><geometry><box><size>{tw[0]} {tw[1]} {th}</size></box></geometry></visual>
      </link></model>"""))


def generate_robot(robot, runtime):
    """한 로봇의 물리 모델·제어 모델·관절 매핑을 저장하고 브리지 설정을 반환한다."""
    name = robot["name"]
    base_mode = robot.get("base_mode", "fixed")
    original, controllers = generate_model(
        name, runtime, robot["initial_joint_positions"], base_mode
    )
    # generate_model에서 내부 제어기를 제거한 모델을 두 용도로 복제한다.
    physics = deepcopy(original)
    control = deepcopy(original)

    config = yaml.safe_load(controllers.read_text())
    commanded = []
    for controller in ("arm_r_controller", "arm_l_controller"):
        commanded.extend(config[f"/{name}/{controller}"]["ros__parameters"]["joints"])

    joints = [
        joint
        for joint in original.findall("joint")
        if joint.get("type") != "fixed"
        and not (base_mode == "swerve" and joint.get("name") in BASE_JOINTS)
    ]
    mapping = _joint_mapping(joints, commanded, robot["initial_joint_positions"])
    system = _add_remote_hardware(control, name)
    gazebo, actuator = _add_gazebo_actuator(physics, name, len(commanded))
    _add_joint_interfaces(system, actuator, mapping)
    _add_joint_state_publisher(gazebo, name)

    bridge = _bridge_entries(name)
    if robot.get("control_mode") == "effort":
        bridge.extend(_add_effort_path(physics, gazebo, actuator, robot))
    if base_mode == "swerve":
        bridge.extend(add_swerve(physics, control, name))
    directory = runtime / name
    write_xml(physics, directory / "physics.urdf")
    write_xml(control, directory / "control.urdf")
    (directory / "joint_map.yaml").write_text(
        yaml.safe_dump(
            {"commanded_joints": commanded, "joints": mapping}, sort_keys=False
        )
    )
    return bridge


def generate_simulation(runtime, robots_file):
    """두 로봇의 실행 파일과 공통 월드·브리지 설정을 생성한다."""
    runtime = Path(runtime)
    runtime.mkdir(parents=True, exist_ok=True)
    robots = read_robots(robots_file)
    if [r["name"] for r in robots] != ["robot1", "robot2"]:
        raise ValueError("Expected robot1 and robot2")
    bridge = []
    for robot in robots:
        bridge.extend(generate_robot(robot, runtime))
    (runtime / "bridge.yaml").write_text(yaml.safe_dump(bridge))
    # Camera/lidar are outside this task; retain original physics and contacts.
    world = ET.parse(share("ffw_bringup") / "worlds/default.sdf").getroot()
    for plugin in list(world.find("world").findall("plugin")):
        if plugin.get("name") == "gz::sim::systems::Sensors":
            world.find("world").remove(plugin)
    if any(robot["base_mode"] == "swerve" for robot in robots):
        # 바퀴 접촉과 조향 서보를 1ms 물리 간격으로 계산한다. 센서는 실행하지 않는다.
        physics = world.find("world/physics")
        physics.find("max_step_size").text = "0.001"
        physics.find("real_time_update_rate").text = "1000"
    if any(robot.get("control_mode") == "effort" for robot in robots):
        configure_effort_world(world.find("world"), robots_file)
    write_xml(world, runtime / "world.sdf")
    (runtime / "ready").write_text("robot1 robot2\n")
    return robots
