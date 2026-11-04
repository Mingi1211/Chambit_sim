"""팔과 소유 관절을 공유하지 않는 스워브 하드웨어·Gazebo 연결을 생성한다."""

import xml.etree.ElementTree as ET

STEERING_JOINTS = tuple(f"{side}_wheel_steer" for side in ("left", "right", "rear"))
DRIVE_JOINTS = tuple(f"{side}_wheel_drive" for side in ("left", "right", "rear"))
BASE_JOINTS = STEERING_JOINTS + DRIVE_JOINTS


def add_swerve(physics, control, name):
    """조향은 position, 구동은 velocity 인터페이스와 전용 Gazebo 플러그인을 쓴다."""
    system = ET.SubElement(
        control,
        "ros2_control",
        {
            "name": f"{name}_remote_swerve",
            "type": "system",
            "is_async": "false",
        },
    )
    hardware = ET.SubElement(system, "hardware")
    ET.SubElement(hardware, "plugin").text = "coop_carry_hardware/RemoteSwerveSystem"
    for key, value in {
        "state_topic": f"/sim/{name}/joint_states",
        "command_topic": f"/sim/{name}/swerve_actuators",
        "state_timeout": "0.5",
    }.items():
        ET.SubElement(hardware, "param", {"name": key}).text = value

    gazebo = ET.SubElement(physics, "gazebo")
    plugin = ET.SubElement(
        gazebo,
        "plugin",
        {
            "filename": "libcoop_carry_gazebo_swerve.so",
            "name": "coop_carry_hardware::GazeboSwerveActuator",
        },
    )
    for key, value in {
        "robot": name,
        "command_topic": f"/sim/{name}/swerve_actuators",
        "feedback_topic": f"/sim/{name}/base_odometry",
        "command_timeout": "0.3",
        "steering_gain": "15.0",
    }.items():
        ET.SubElement(plugin, key).text = value

    for index, joint_name in enumerate(BASE_JOINTS):
        interface = "position" if index < 3 else "velocity"
        joint = ET.SubElement(system, "joint", {"name": joint_name})
        ET.SubElement(joint, "command_interface", {"name": interface})
        for state_interface in ("position", "velocity"):
            state = ET.SubElement(joint, "state_interface", {"name": state_interface})
            ET.SubElement(state, "param", {"name": "initial_value"}).text = "0.0"
        ET.SubElement(plugin, "steering_joint" if index < 3 else "drive_joint").text = (
            joint_name
        )

    return [
        dict(
            ros_topic_name=f"/sim/{name}/swerve_actuators",
            gz_topic_name=f"/sim/{name}/swerve_actuators",
            ros_type_name="actuator_msgs/msg/Actuators",
            gz_type_name="gz.msgs.Actuators",
            direction="ROS_TO_GZ",
        ),
        dict(
            ros_topic_name=f"/sim/{name}/base_odometry",
            gz_topic_name=f"/sim/{name}/base_odometry",
            ros_type_name="nav_msgs/msg/Odometry",
            gz_type_name="gz.msgs.Odometry",
            direction="GZ_TO_ROS",
        ),
    ]
