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
    write_xml(world, runtime / "world.sdf")
    (runtime / "ready").write_text("robot1 robot2\n")
    return robots
