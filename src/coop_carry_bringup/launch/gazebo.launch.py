"""하나의 월드에 두 로봇을 생성한다. 제어기는 각 로봇 컨테이너에서 실행한다."""

import os
from pathlib import Path
import shutil
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    EmitEvent,
)
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from coop_carry_bringup.model_factory import share
from coop_carry_bringup.simulation import generate_simulation


def setup(context):
    """모델 생성 → 서버·브리지 실행 → 두 로봇 생성 순으로 실행 구성을 만든다."""
    runtime = Path(LaunchConfiguration("runtime_dir").perform(context))
    robots = generate_simulation(
        runtime, share("coop_carry_bringup") / "config/robots.yaml"
    )
    prefix = Path(get_package_prefix("coop_carry_hardware"))
    actions = [
        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            os.pathsep.join(
                filter(
                    None,
                    [
                        str(prefix / "lib"),
                        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                    ],
                )
            ),
        ),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.pathsep.join(
                filter(
                    None,
                    [
                        str(share("ffw_description").parent.resolve()),
                        str(share("realsense2_description").parent.resolve()),
                        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                    ],
                )
            ),
        ),
        ExecuteProcess(
            cmd=[
                str(prefix / "lib/coop_carry_hardware/coop_gazebo_server"),
                str(runtime / "world.sdf"),
            ],
            output="screen",
            on_exit=[EmitEvent(event=Shutdown(reason="Gazebo server exited"))],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="joint_bridge",
            parameters=[{"config_file": str(runtime / "bridge.yaml")}],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
    ]
    for robot in robots:
        name = robot["name"]
        actions.extend(
            [
                Node(
                    package="coop_carry_bringup",
                    executable="sim_command_adapter",
                    namespace=name,
                    parameters=[
                        {
                            "robot": name,
                            "joint_map": str(runtime / name / "joint_map.yaml"),
                            "use_sim_time": True,
                        }
                    ],
                    output="screen",
                ),
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    namespace=name,
                    arguments=[
                        "-world",
                        "empty",
                        "-file",
                        str(runtime / name / "physics.urdf"),
                        "-name",
                        name,
                        "-allow_renaming",
                        "false",
                        "-x",
                        str(robot["x"]),
                        "-y",
                        str(robot["y"]),
                        "-z",
                        str(robot["z"]),
                        "-Y",
                        str(robot["yaw"]),
                    ],
                    output="screen",
                ),
            ]
        )
    if LaunchConfiguration("gui").perform(context).lower() == "true":
        actions.append(
            ExecuteProcess(
                cmd=[shutil.which("gz"), "sim", "-g"],
                output="screen",
                on_exit=[EmitEvent(event=Shutdown(reason="Gazebo GUI closed"))],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("runtime_dir", default_value="/tmp/internship_sim"),
            DeclareLaunchArgument("gui", default_value="false"),
            OpaqueFunction(function=setup),
        ]
    )
