"""Actuation ownership and endpoint isolation, using real upstream Xacro assets."""

import xml.etree.ElementTree as ET
import yaml
import pytest
from coop_carry_bringup.model_factory import share, qp_parameters
from coop_carry_bringup.simulation import generate_simulation


@pytest.mark.parametrize("base_mode", ["fixed", "swerve"])
def test_two_external_models_have_no_internal_managers(tmp_path, base_mode):
    settings = yaml.safe_load(
        (share("coop_carry_bringup") / "config/robots.yaml").read_text()
    )
    settings["base_mode"] = base_mode
    robots_file = tmp_path / "robots.yaml"
    robots_file.write_text(yaml.safe_dump(settings))
    generate_simulation(tmp_path, robots_file)
    for name in ("robot1", "robot2"):
        physics = ET.parse(tmp_path / name / "physics.urdf").getroot()
        control = ET.parse(tmp_path / name / "control.urdf").getroot()
        assert not physics.findall("ros2_control")
        assert not physics.findall(
            "./gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']"
        )
        anchor = physics.find("joint[@name='base_world_fixed']")
        if base_mode == "fixed":
            assert anchor.get("type") == "fixed"
        else:
            assert anchor is None
            assert control.find("joint[@name='base_world_fixed']") is None
            from coop_carry_bringup.swerve_model import BASE_JOINTS

            arm_system = control.find(f"ros2_control[@name='{name}_remote_sim']")
            base_system = control.find(f"ros2_control[@name='{name}_remote_swerve']")
            assert (
                base_system.findtext("hardware/plugin")
                == "coop_carry_hardware/RemoteSwerveSystem"
            )
            assert {joint.get("name") for joint in base_system.findall("joint")} == set(
                BASE_JOINTS
            )
            assert not set(BASE_JOINTS) & {
                joint.get("name") for joint in arm_system.findall("joint")
            }
            for index, joint_name in enumerate(BASE_JOINTS):
                joint = base_system.find(f"joint[@name='{joint_name}']")
                assert joint.find("command_interface").get("name") == (
                    "position" if index < 3 else "velocity"
                )
        assert (
            control.findtext("ros2_control/hardware/plugin")
            == "coop_carry_hardware/RemoteGazeboSystem"
        )
        assert (
            control.findtext("ros2_control/hardware/param[@name='state_topic']")
            == f"/sim/{name}/joint_states"
        )
        mapping = yaml.safe_load((tmp_path / name / "joint_map.yaml").read_text())
        assert len(mapping["commanded_joints"]) == 16
        for side in ("r", "l"):
            assert (
                control.findtext(
                    f"ros2_control/joint[@name='arm_{side}_joint4']/state_interface[@name='position']/param"
                )
                == "0.0"
            )
        configs = yaml.safe_load((tmp_path / name / "controllers.yaml").read_text())
        for side in ("r", "l"):
            assert (
                configs[f"/{name}/controller_manager"]["ros__parameters"][
                    f"arm_{side}_controller"
                ]["type"]
                == "joint_trajectory_controller/JointTrajectoryController"
            )
            assert (
                configs[f"/{name}/arm_{side}_controller"]["ros__parameters"]["is_async"]
                is False
            )
    topics = {
        v["ros_topic_name"]
        for v in yaml.safe_load((tmp_path / "bridge.yaml").read_text())
    }
    expected = {
        f"/sim/{r}/{topic}"
        for r in ("robot1", "robot2")
        for topic in ("joint_states", "actuators")
    }
    if base_mode == "swerve":
        expected |= {
            f"/sim/{robot}/{topic}"
            for robot in ("robot1", "robot2")
            for topic in ("swerve_actuators", "base_odometry")
        }
    assert topics == expected


def test_qp_topics_are_separate_and_adapter_is_the_public_endpoint():
    for robot in ("robot1", "robot2"):
        params = qp_parameters(robot, share("coop_carry_bringup") / "config/qp.yaml")
        assert all(
            v.startswith(f"/{robot}/")
            for k, v in params.items()
            if k.endswith("_topic")
        )
        assert params["right_movel_topic"] == f"/{robot}/qp/r_goal_move"
        assert params["left_movel_topic"] == f"/{robot}/qp/l_goal_move"
        assert params["joint_states_topic"] == f"/{robot}/joint_states"
