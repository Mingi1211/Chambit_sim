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
    if robot.get('control_mode') == 'effort':
        # [문서 밖] 그리퍼 쥐는 힘 제한: Gazebo 위치 서보는 관절 effort 한계까지 밀어붙임(URDF 1000) → 손잡이에 파고들어
        # 접촉 계산 폭주(RTF 0.007, 11/3 확인). 물리 모델에서만 그리퍼 관절 effort 한계를 낮춤
        for j in physics.findall('joint'):
            if j.get('name', '').startswith('gripper_') and j.find('limit') is not None:
                j.find('limit').set('effort', str(robot.get('gripper_effort', 5.0)))
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
    effort = robot.get('control_mode', 'position') == 'effort'
    if effort:
        # [문서 밖] 토크 명령 경로: /sim/<robot>/effort (Actuators.normalized = 관절 토크, NaN = 위치 서보 유지)
        for key, value in {'effort_topic': f'/sim/{name}/effort', 'effort_timeout': '0.1',
                           'effort_limit': '60.0'}.items():
            ET.SubElement(actuator, key).text = value
    if effort and robot.get('base_motion'):
        # [문서 밖] 베이스 이동 대역(실제 스워브 주행은 현대 파트): 물리 모델에서만 base_world_fixed → y 슬라이드 관절.
        # 제어 모델(control.urdf)·Pinocchio는 그대로(팔은 모두 base_link 기준이라 순수 평행이동은 영향 없음)
        bj = next(j for j in physics.findall('joint') if j.get('name') == 'base_world_fixed')
        bj.set('type', 'prismatic')
        ET.SubElement(bj, 'axis', {'xyz': '0 1 0'})
        ET.SubElement(bj, 'limit', {'lower': '-5', 'upper': '5', 'effort': '100000', 'velocity': '1.0'})
        base_def = ET.SubElement(actuator, 'joint')
        for key, value in dict(name='base_world_fixed', initial=0.0, lower=-5.0, upper=5.0, velocity=1.0,
                               command_index=-1, mimic_index=-1, base_index=0).items():
            ET.SubElement(base_def, key).text = str(value)
        ET.SubElement(actuator, 'base_topic').text = f'/sim/{name}/base'
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
    # [문서 밖] effort 모드: 상태 주기 = 토크 루프 주기. 문서 #8 감쇠 상한 D < 2Λ/T 때문에 T를 줄일 수 있게 설정값으로
    ET.SubElement(publisher, 'update_rate').text = str(robot.get('state_rate', 100) if effort else 100)
    physics_path = runtime / name / 'physics.urdf'
    control_path = runtime / name / 'control.urdf'
    write_xml(physics, physics_path)
    write_xml(control, control_path)
    mapping_path = runtime / name / 'joint_map.yaml'
    mapping_path.write_text(yaml.safe_dump({'commanded_joints': commanded, 'joints': mapping}, sort_keys=False))
    bridge = []
    for topic, ros_type, gz_type, direction in [
        (f'/sim/{name}/joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model', 'GZ_TO_ROS'),
        (f'/sim/{name}/actuators', 'actuator_msgs/msg/Actuators', 'gz.msgs.Actuators', 'ROS_TO_GZ')] + (
        [(f'/sim/{name}/effort', 'actuator_msgs/msg/Actuators', 'gz.msgs.Actuators', 'ROS_TO_GZ')] if effort else []) + (
        [(f'/sim/{name}/base', 'actuator_msgs/msg/Actuators', 'gz.msgs.Actuators', 'ROS_TO_GZ')]
        if effort and robot.get('base_motion') else []):
        bridge.append(dict(ros_topic_name=topic, gz_topic_name=topic, ros_type_name=ros_type,
                           gz_type_name=gz_type, direction=direction))
    return bridge


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
    wz = base['z'] + bz
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
    if any(r.get('control_mode') == 'effort' for r in robots):
        configure_effort_world(world.find('world'), robots_file)
    write_xml(world, runtime / 'world.sdf')
    (runtime / 'ready').write_text('robot1 robot2\n')
    return robots
