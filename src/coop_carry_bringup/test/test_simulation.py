"""Actuation ownership and endpoint isolation, using real upstream Xacro assets."""
import xml.etree.ElementTree as ET
import yaml
from coop_carry_bringup.model_factory import share, qp_parameters
from coop_carry_bringup.simulation import generate_simulation


def test_two_external_models_have_no_internal_managers(tmp_path):
    generate_simulation(tmp_path,share('coop_carry_bringup')/'config/robots.yaml')
    for name in ('robot1','robot2'):
        physics=ET.parse(tmp_path/name/'physics.urdf').getroot()
        control=ET.parse(tmp_path/name/'control.urdf').getroot()
        assert not physics.findall('ros2_control')
        assert not physics.findall("./gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
        assert physics.find("joint[@name='base_world_fixed']").get('type')=='fixed'
        assert control.findtext('ros2_control/hardware/plugin')=='coop_carry_hardware/RemoteGazeboSystem'
        assert control.findtext("ros2_control/hardware/param[@name='state_topic']")==f'/sim/{name}/joint_states'
        mapping=yaml.safe_load((tmp_path/name/'joint_map.yaml').read_text())
        assert len(mapping['commanded_joints'])==16
        for side in ('r','l'):
            assert control.findtext(f"ros2_control/joint[@name='arm_{side}_joint4']/state_interface[@name='position']/param")== '-1.3'
        configs=yaml.safe_load((tmp_path/name/'controllers.yaml').read_text())
        for side in ('r','l'):
            assert configs[f'/{name}/controller_manager']['ros__parameters'][f'arm_{side}_controller']['type']=='joint_trajectory_controller/JointTrajectoryController'
            assert configs[f'/{name}/arm_{side}_controller']['ros__parameters']['is_async'] is False
    topics={v['ros_topic_name'] for v in yaml.safe_load((tmp_path/'bridge.yaml').read_text())}
    assert topics=={f'/sim/{r}/{topic}' for r in ('robot1','robot2') for topic in ('joint_states','actuators')}


def test_qp_topics_are_separate_and_adapter_is_the_public_endpoint():
    for robot in ('robot1','robot2'):
        params=qp_parameters(robot,share('coop_carry_bringup')/'config/qp.yaml')
        assert all(v.startswith(f'/{robot}/') for k,v in params.items() if k.endswith('_topic'))
        assert params['right_movel_topic']==f'/{robot}/qp/r_goal_move'
        assert params['left_movel_topic']==f'/{robot}/qp/l_goal_move'
        assert params['joint_states_topic']==f'/{robot}/joint_states'
