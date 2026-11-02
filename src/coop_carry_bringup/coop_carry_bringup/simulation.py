"""Generate both external simulation interfaces from upstream AI Worker models."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from .model_factory import generate_model, read_robots, share


def write_xml(root, path):
    ET.indent(root)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(ET.tostring(root, encoding='unicode'))
    temporary.replace(path)


def generate_robot(robot, runtime):
    name = robot['name']
    original, controllers = generate_model(name, runtime, robot['initial_joint_positions'])
    physics, control = deepcopy(original), deepcopy(original)
    for root in (physics, control):
        for item in list(root.findall('ros2_control')):
            root.remove(item)
        for group in list(root.findall('gazebo')):
            for plugin in list(group.findall('plugin')):
                if plugin.get('name') == 'gz_ros2_control::GazeboSimROS2ControlPlugin':
                    group.remove(plugin)

    joints = [j for j in original.findall('joint') if j.get('type') != 'fixed']
    commanded = []
    config = yaml.safe_load(controllers.read_text())
    for controller in ('arm_r_controller', 'arm_l_controller'):
        commanded.extend(config[f'/{name}/{controller}']['ros__parameters']['joints'])
    indices = {j.get('name'): i for i, j in enumerate(joints)}
    mapping = []
    system = ET.SubElement(control, 'ros2_control', {
        'name': f'{name}_remote_sim', 'type': 'system', 'is_async': 'false'})
    hardware = ET.SubElement(system, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'coop_carry_hardware/RemoteGazeboSystem'
    for key, value in {'state_topic': f'/sim/{name}/joint_states',
                       'command_topic': f'/sim/{name}/joint_commands', 'state_timeout': '0.5'}.items():
        ET.SubElement(hardware, 'param', {'name': key}).text = value
    gazebo = ET.SubElement(physics, 'gazebo')
    actuator = ET.SubElement(gazebo, 'plugin', {
        'filename': 'libcoop_carry_gazebo_actuator.so',
        'name': 'coop_carry_hardware::GazeboJointActuator'})
    for key, value in {'command_topic': f'/sim/{name}/actuators', 'command_timeout': '0.5',
                       'position_gain': '10.0', 'command_count': str(len(commanded))}.items():
        ET.SubElement(actuator, key).text = value
    for joint in joints:
        joint_name = joint.get('name')
        limit = joint.find('limit')
        mimic = joint.find('mimic')
        item = dict(name=joint_name, initial=float(robot['initial_joint_positions'].get(joint_name, 0.0)),
                    lower=float(limit.get('lower', '-1e9')), upper=float(limit.get('upper', '1e9')),
                    velocity=min(float(limit.get('velocity')), 6.5),
                    command_index=commanded.index(joint_name) if joint_name in commanded else -1,
                    mimic_index=indices[mimic.get('joint')] if mimic is not None else -1)
        mapping.append(item)
        definition = ET.SubElement(actuator, 'joint')
        for key, value in item.items():
            ET.SubElement(definition, key).text = str(value)
        # Mimic feedback is produced by robot_state_publisher from its source joint.
        if mimic is None:
            entry = ET.SubElement(system, 'joint', {'name': joint_name})
            if joint_name in commanded:
                cmd = ET.SubElement(entry, 'command_interface', {'name': 'position'})
                for bound in ('min', 'max'):
                    ET.SubElement(cmd, 'param', {'name': bound}).text = str(
                        item['lower' if bound == 'min' else 'upper'])
            for interface in ('position', 'velocity'):
                state = ET.SubElement(entry, 'state_interface', {'name': interface})
                ET.SubElement(state, 'param', {'name': 'initial_value'}).text = str(
                    item['initial'] if interface == 'position' else 0.0)
    publisher = ET.SubElement(gazebo, 'plugin', {
        'filename': 'gz-sim-joint-state-publisher-system',
        'name': 'gz::sim::systems::JointStatePublisher'})
    ET.SubElement(publisher, 'topic').text = f'/sim/{name}/joint_states'
    ET.SubElement(publisher, 'update_rate').text = '100'
    physics_path = runtime / name / 'physics.urdf'
    control_path = runtime / name / 'control.urdf'
    write_xml(physics, physics_path)
    write_xml(control, control_path)
    mapping_path = runtime / name / 'joint_map.yaml'
    mapping_path.write_text(yaml.safe_dump({'commanded_joints': commanded, 'joints': mapping}, sort_keys=False))
    bridge = []
    for topic, ros_type, gz_type, direction in [
        (f'/sim/{name}/joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model', 'GZ_TO_ROS'),
        (f'/sim/{name}/actuators', 'actuator_msgs/msg/Actuators', 'gz.msgs.Actuators', 'ROS_TO_GZ')]:
        bridge.append(dict(ros_topic_name=topic, gz_topic_name=topic, ros_type_name=ros_type,
                           gz_type_name=gz_type, direction=direction))
    return bridge


def generate_simulation(runtime, robots_file):
    runtime = Path(runtime)
    runtime.mkdir(parents=True, exist_ok=True)
    robots = read_robots(robots_file)
    if [r['name'] for r in robots] != ['robot1', 'robot2']:
        raise ValueError('Expected robot1 and robot2')
    bridge = []
    for robot in robots:
        bridge.extend(generate_robot(robot, runtime))
    (runtime / 'bridge.yaml').write_text(yaml.safe_dump(bridge))
    # Camera/lidar are outside this task; retain original physics and contacts.
    world = ET.parse(share('ffw_bringup') / 'worlds/default.sdf').getroot()
    for plugin in list(world.find('world').findall('plugin')):
        if plugin.get('name') == 'gz::sim::systems::Sensors':
            world.find('world').remove(plugin)
    write_xml(world, runtime / 'world.sdf')
    (runtime / 'ready').write_text('robot1 robot2\n')
    return robots
