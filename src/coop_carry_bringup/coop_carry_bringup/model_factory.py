"""공식 모델을 수정하지 않고 로봇별 모델·제어기·QP 설정을 생성한다."""

from copy import deepcopy
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

MODEL = "ffw_sg2_rev1_follower"
ACTIVE_CONTROLLERS = ("joint_state_broadcaster", "arm_r_controller", "arm_l_controller")


def share(package):
    """설치된 ROS 패키지의 리소스 디렉터리를 반환한다."""
    return Path(get_package_share_directory(package))


def read_robots(path):
    """두 로봇의 이름·배치를 검사하고 공통 초기 관절 자세를 연결한다."""
    config = yaml.safe_load(Path(path).read_text())
    robots = config["robots"]
    if len(robots) != 2 or len({r["name"] for r in robots}) != 2:
        raise ValueError("Exactly two uniquely named robots are required")
    for robot in robots:
        robot["initial_joint_positions"] = config.get("initial_joint_positions", {})
        robot["base_mode"] = config.get("base_mode", "fixed")
        if robot["base_mode"] not in ("fixed", "swerve"):
            raise ValueError("base_mode must be fixed or swerve")
        # [문서 밖] 김민기: position(현대 기본) | effort(토크 경로 추가, 임피던스 제어용)
        robot["control_mode"] = config.get("control_mode", "position")
        robot["state_rate"] = int(config.get("state_rate", 100))
        robot["gripper_effort"] = float(config.get("gripper_effort", 5.0))
        if not re.fullmatch(r"[a-zA-Z][a-zA-Z0-9_]*", robot["name"]):
            raise ValueError(f"Invalid robot namespace: {robot['name']}")
        for key in ("x", "y", "z", "yaw"):
            if not math.isfinite(float(robot[key])):
                raise ValueError(f"Invalid initial pose: {robot}")
    return robots


def read_controller_sections(path):
    """공식 YAML의 반복된 /** 아래 설정을 누락 없이 합친다."""
    # ROBOTIS repeats the top-level /** mapping. safe_load would discard all
    # but its final occurrence. Read the YAML syntax tree and merge sections.
    root = yaml.compose(Path(path).read_text())
    sections = {}
    for scope, node in root.value:
        if scope.value != "/**":
            raise ValueError(f"Unexpected upstream controller scope: {scope.value}")
        for key, value in node.value:
            if key.value in sections:
                raise ValueError(f"Duplicate controller section: {key.value}")
            sections[key.value] = yaml.safe_load(yaml.serialize(value))
    # Upstream uses the broadcaster defaults and only declares its type.
    sections.setdefault("joint_state_broadcaster", {"ros__parameters": {}})
    for name in ("controller_manager", *ACTIVE_CONTROLLERS):
        if name not in sections:
            raise ValueError(f"Missing upstream controller section: {name}")
    return sections


def _write_controller_config(name, directory, base_mode="fixed"):
    """관절 상태 발행기와 양팔 JTC만 로봇별 이름으로 설정한다."""
    sections = read_controller_sections(
        share("ffw_bringup")
        / "config"
        / MODEL
        / "ffw_sg2_follower_ai_hardware_controller.yaml"
    )
    active_controllers = ACTIVE_CONTROLLERS
    if base_mode == "swerve":
        active_controllers += ("swerve_drive_controller",)
    manager = deepcopy(sections["controller_manager"])
    manager["ros__parameters"]["use_sim_time"] = True
    for controller in ("arm_r_controller", "arm_l_controller"):
        controller_params = manager["ros__parameters"][controller]
        controller_params["type"] = (
            "joint_trajectory_controller/JointTrajectoryController"
        )
    manager["ros__parameters"] = {
        key: value
        for key, value in manager["ros__parameters"].items()
        if not isinstance(value, dict) or key in active_controllers
    }
    config = {f"/{name}/controller_manager": manager}
    for controller, section in sections.items():
        if controller not in active_controllers:
            continue
        section = deepcopy(section)
        params = section["ros__parameters"]
        params["use_sim_time"] = True
        params["is_async"] = False
        if controller == "joint_state_broadcaster":
            params["use_local_topics"] = False  # /robotN/joint_states
        if controller == "swerve_drive_controller":
            # 저속 패턴이 기본 deadband(0.1m/s)에 잘리지 않도록 명시한다.
            params.update(
                cmd_vel_topic=f"/{name}/cmd_vel",
                cmd_vel_timeout=0.3,
                base_frame_id=f"{name}/base_link",
                odom_frame_id=f"{name}/odom",
                # 시뮬레이션의 실제 차체 TF는 base_state_tf가 발행한다.
                enable_odom_tf=False,
                odom_source="feedback",
                enabled_open_loop=False,
                enabled_speed_limits=False,
                linear_vel_deadband=0.0001,
                angular_vel_deadband=0.0001,
                steering_angular_velocity_limit=2.0,
                steering_alignment_angle_error_threshold=0.1,
                steering_alignment_start_angle_error_threshold=0.02,
                enable_visualization=False,
                is_async=False,
            )
        config[f"/{name}/{controller}"] = section
    controllers = directory / "controllers.yaml"
    controllers.write_text(yaml.safe_dump(config, sort_keys=False))
    return controllers


def _load_simulation_model():
    """공식 Xacro를 펼치고 시뮬레이션 하드웨어 구성인지 확인한다."""
    xacro_path = (
        share("ffw_description") / "urdf" / MODEL / "ffw_sg2_follower.urdf.xacro"
    )
    description = xacro.process_file(
        str(xacro_path),
        mappings={"model": MODEL, "use_sim": "true", "prefix": ""},
    )
    root = ET.fromstring(description.toxml())
    # The sensor system has no simulated hardware. Avoid passing an empty
    # hardware definition to ros2_control's resource parser.
    for control in list(root.findall("ros2_control")):
        plugin = control.findtext("hardware/plugin")
        if plugin is None:
            root.remove(control)
        elif plugin != "gz_ros2_control/GazeboSimSystem":
            raise ValueError(f"Refusing non-Gazebo hardware plugin: {plugin}")
        else:
            # GazeboSimSystem reads/writes Gazebo's entity-component state in
            # the simulation update, rather than an independent hardware loop.
            control.set("is_async", "false")
    return root


def _apply_initial_positions(root, initial_positions):
    """초기 팔 각도가 Gazebo 모델의 관절 범위 안에 있는지 검사한다."""
    for joint_name, position in (initial_positions or {}).items():
        joint = root.find(f"joint[@name='{joint_name}']")
        state = root.find(
            f"ros2_control/joint[@name='{joint_name}']/state_interface[@name='position']"
        )
        if (
            joint is None
            or state is None
            or not joint_name.startswith(("arm_r_joint", "arm_l_joint"))
        ):
            raise ValueError(
                f"Initial position must name a simulated arm joint: {joint_name}"
            )
        limit = joint.find("limit")
        if (
            not float(limit.get("lower"))
            <= float(position)
            <= float(limit.get("upper"))
        ):
            raise ValueError(
                f"Initial position outside Gazebo joint limit: {joint_name}"
            )
        ET.SubElement(state, "param", {"name": "initial_value"}).text = str(
            float(position)
        )


def _remove_internal_control(root):
    """제어기는 로봇 컨테이너에 있으므로 Gazebo 내부 제어 구성을 제거한다."""
    for control in list(root.findall("ros2_control")):
        root.remove(control)
    for group in root.findall("gazebo"):
        for plugin in list(group.findall("plugin")):
            if plugin.get("name") == "gz_ros2_control::GazeboSimROS2ControlPlugin":
                group.remove(plugin)


def _namespace_sensors(root, name):
    """ROS namespace와 별개인 Gazebo 센서 토픽·프레임도 구분한다."""
    for sensor in root.findall("./gazebo/sensor"):
        topic = sensor.find("topic")
        if topic is not None:
            topic.text = f"/{name}/{topic.text.lstrip('/')}"
        frame = sensor.find("gz_frame_id")
        if frame is not None:
            frame.text = f"{name}/{frame.text}"


def _fix_base_to_world(root):
    """로봇 바닥을 물리적으로 고정한다. 월드 TF를 발행하는 코드는 아니다."""
    ET.SubElement(root, "link", {"name": "world"})
    joint = ET.SubElement(root, "joint", {"name": "base_world_fixed", "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": "world"})
    ET.SubElement(joint, "child", {"link": "base_link"})


def generate_model(name, runtime_root, initial_positions=None, base_mode="fixed"):
    """모델 XML과 controllers.yaml 경로를 반환한다. URDF 저장은 simulation이 한다."""
    directory = Path(runtime_root) / name
    directory.mkdir(parents=True, exist_ok=True)
    controllers = _write_controller_config(name, directory, base_mode)

    root = _load_simulation_model()
    _apply_initial_positions(root, initial_positions)
    _remove_internal_control(root)
    _namespace_sensors(root, name)
    if base_mode == "fixed":
        _fix_base_to_world(root)
    return root, controllers


def qp_parameters(name, config_path):
    """공통 QP 설정에 로봇별 토픽과 Cyclo 모델 경로를 적용한다."""
    models = share("cyclo_motion_controller_models") / "models" / "ai_worker"
    params = yaml.safe_load(Path(config_path).read_text())
    # Explicit values override every absolute default declared by Cyclo.
    topics = {
        "joint_states_topic": "joint_states",
        "right_movel_topic": "r_goal_move",
        "left_movel_topic": "l_goal_move",
        "virtual_object_movel_topic": "virtual_object_goal_move",
        "grasp_capture_topic": "capture_grasp",
        "right_traj_topic": "arm_r_controller/joint_trajectory",
        "left_traj_topic": "arm_l_controller/joint_trajectory",
        "right_raw_traj_topic": "arm_r_controller/raw_joint_trajectory",
        "left_raw_traj_topic": "arm_l_controller/raw_joint_trajectory",
        "lift_topic": "lift_controller/joint_trajectory",
        "r_gripper_pose_topic": "r_gripper_pose",
        "l_gripper_pose_topic": "l_gripper_pose",
    }
    params.update({key: f"/{name}/{value}" for key, value in topics.items()})
    params.update(
        right_movel_topic=f"/{name}/qp/r_goal_move",
        left_movel_topic=f"/{name}/qp/l_goal_move",
        use_sim_time=True,
        urdf_path=str(models / "ffw_sg2_follower.urdf"),
        srdf_path=str(models / "ffw_sg2_follower_default.srdf"),
    )
    for key in ("urdf_path", "srdf_path"):
        if not Path(params[key]).is_file():
            raise FileNotFoundError(params[key])
    if params["lift_vel_bound"] != 0.0:
        raise ValueError(
            "this simulation requires lift_vel_bound: 0.0 (no lift controller)"
        )
    return params
