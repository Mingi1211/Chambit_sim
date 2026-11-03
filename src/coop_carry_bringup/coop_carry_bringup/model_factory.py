"""Adapt expanded upstream models; never edit the source Xacro or controller YAML."""

from copy import deepcopy
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

MODEL = 'ffw_sg2_rev1_follower'
ACTIVE_CONTROLLERS = ('joint_state_broadcaster', 'arm_r_controller', 'arm_l_controller')


def share(package):
    return Path(get_package_share_directory(package))


def read_robots(path):
    config = yaml.safe_load(Path(path).read_text())
    robots = config['robots']
    if len(robots) != 2 or len({r['name'] for r in robots}) != 2:
        raise ValueError('Exactly two uniquely named robots are required')
    for robot in robots:
        robot['initial_joint_positions'] = config.get('initial_joint_positions', {})
        # [문서 밖] 김민기: position(현대 기본) | effort(토크 경로 추가, 임피던스 제어용)
        robot['control_mode'] = config.get('control_mode', 'position')
        robot['state_rate'] = int(config.get('state_rate', 100))
        if not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', robot['name']):
            raise ValueError(f"Invalid robot namespace: {robot['name']}")
        for key in ('x', 'y', 'z', 'yaw'):
            if not math.isfinite(float(robot[key])):
                raise ValueError(f'Invalid initial pose: {robot}')
    return robots


def read_controller_sections(path):
    # ROBOTIS repeats the top-level /** mapping. safe_load would discard all
    # but its final occurrence. Read the YAML syntax tree and merge sections.
    root = yaml.compose(Path(path).read_text())
    sections = {}
    for scope, node in root.value:
        if scope.value != '/**':
            raise ValueError(f'Unexpected upstream controller scope: {scope.value}')
        for key, value in node.value:
            if key.value in sections:
                raise ValueError(f'Duplicate controller section: {key.value}')
            sections[key.value] = yaml.safe_load(yaml.serialize(value))
    # Upstream uses the broadcaster defaults and only declares its type.
    sections.setdefault('joint_state_broadcaster', {'ros__parameters': {}})
    for name in ('controller_manager', *ACTIVE_CONTROLLERS):
        if name not in sections:
            raise ValueError(f'Missing upstream controller section: {name}')
    return sections


def generate_model(name, runtime_root, initial_positions=None):
    directory = Path(runtime_root) / name
    directory.mkdir(parents=True, exist_ok=True)
    sections = read_controller_sections(
        share('ffw_bringup') / 'config' / MODEL /
        'ffw_sg2_follower_ai_hardware_controller.yaml')
    manager = deepcopy(sections['controller_manager'])
    manager['ros__parameters']['use_sim_time'] = True
    for controller in ('arm_r_controller', 'arm_l_controller'):
        manager['ros__parameters'][controller]['type'] = (
            'joint_trajectory_controller/JointTrajectoryController')
    manager['ros__parameters'] = {k: v for k, v in manager['ros__parameters'].items()
                                  if not isinstance(v, dict) or k in ACTIVE_CONTROLLERS}
    config = {f'/{name}/controller_manager': manager}
    for controller, section in sections.items():
        if controller not in ACTIVE_CONTROLLERS:
            continue
        section = deepcopy(section)
        params = section['ros__parameters']
        params['use_sim_time'] = True
        params['is_async'] = False
        if controller == 'joint_state_broadcaster':
            params['use_local_topics'] = False  # /robotN/joint_states
        config[f'/{name}/{controller}'] = section
    controllers = directory / 'controllers.yaml'
    controllers.write_text(yaml.safe_dump(config, sort_keys=False))

    root = ET.fromstring(xacro.process_file(
        str(share('ffw_description') / 'urdf' / MODEL / 'ffw_sg2_follower.urdf.xacro'),
        mappings={'model': MODEL, 'use_sim': 'true', 'prefix': ''}).toxml())
    # The sensor system has no simulated hardware. Avoid passing an empty
    # hardware definition to ros2_control's resource parser.
    for control in list(root.findall('ros2_control')):
        plugin = control.findtext('hardware/plugin')
        if plugin is None:
            root.remove(control)
        elif plugin != 'gz_ros2_control/GazeboSimSystem':
            raise ValueError(f'Refusing non-Gazebo hardware plugin: {plugin}')
        else:
            # GazeboSimSystem reads/writes Gazebo's entity-component state in
            # the simulation update, rather than an independent hardware loop.
            control.set('is_async', 'false')
    for joint_name, position in (initial_positions or {}).items():
        joint = root.find(f"joint[@name='{joint_name}']")
        state = root.find(f"ros2_control/joint[@name='{joint_name}']/state_interface[@name='position']")
        if joint is None or state is None or not joint_name.startswith(('arm_r_joint', 'arm_l_joint')):
            raise ValueError(f'Initial position must name a simulated arm joint: {joint_name}')
        limit = joint.find('limit')
        if not float(limit.get('lower')) <= float(position) <= float(limit.get('upper')):
            raise ValueError(f'Initial position outside Gazebo joint limit: {joint_name}')
        ET.SubElement(state, 'param', {'name': 'initial_value'}).text = str(float(position))
    # Both models have one external actuation owner. No Gazebo Controller Manager.
    for control in list(root.findall('ros2_control')):
        root.remove(control)
    for group in root.findall('gazebo'):
        for plugin in list(group.findall('plugin')):
            if plugin.get('name') == 'gz_ros2_control::GazeboSimROS2ControlPlugin':
                group.remove(plugin)
    # Sensor topic names are Gazebo transport names, separate from ROS namespace.
    for sensor in root.findall('./gazebo/sensor'):
        topic = sensor.find('topic')
        if topic is not None:
            topic.text = f"/{name}/{topic.text.lstrip('/')}"
        frame = sensor.find('gz_frame_id')
        if frame is not None:
            frame.text = f'{name}/{frame.text}'
    # Physically anchor the base for this simulation, retaining articulated arms. This
    # is a Gazebo world joint, not an odometry estimate or a world TF publisher.
    ET.SubElement(root, 'link', {'name': 'world'})
    joint = ET.SubElement(root, 'joint', {'name': 'base_world_fixed', 'type': 'fixed'})
    ET.SubElement(joint, 'parent', {'link': 'world'})
    ET.SubElement(joint, 'child', {'link': 'base_link'})
    return root, controllers


def qp_parameters(name, config_path):
    models = share('cyclo_motion_controller_models') / 'models' / 'ai_worker'
    params = yaml.safe_load(Path(config_path).read_text())
    # Explicit values override every absolute default declared by Cyclo.
    topics = {
        'joint_states_topic': 'joint_states',
        'right_movel_topic': 'r_goal_move', 'left_movel_topic': 'l_goal_move',
        'virtual_object_movel_topic': 'virtual_object_goal_move',
        'grasp_capture_topic': 'capture_grasp',
        'right_traj_topic': 'arm_r_controller/joint_trajectory',
        'left_traj_topic': 'arm_l_controller/joint_trajectory',
        'right_raw_traj_topic': 'arm_r_controller/raw_joint_trajectory',
        'left_raw_traj_topic': 'arm_l_controller/raw_joint_trajectory',
        'lift_topic': 'lift_controller/joint_trajectory',
        'r_gripper_pose_topic': 'r_gripper_pose',
        'l_gripper_pose_topic': 'l_gripper_pose',
    }
    params.update({key: f'/{name}/{value}' for key, value in topics.items()})
    params.update(right_movel_topic=f'/{name}/qp/r_goal_move',
                  left_movel_topic=f'/{name}/qp/l_goal_move',
                  use_sim_time=True,
                  urdf_path=str(models / 'ffw_sg2_follower.urdf'),
                  srdf_path=str(models / 'ffw_sg2_follower_default.srdf'))
    for key in ('urdf_path', 'srdf_path'):
        if not Path(params[key]).is_file():
            raise FileNotFoundError(params[key])
    if params['lift_vel_bound'] != 0.0:
        raise ValueError('this simulation requires lift_vel_bound: 0.0 (no lift controller)')
    return params
