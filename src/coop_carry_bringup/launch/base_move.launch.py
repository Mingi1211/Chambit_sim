"""베이스 시퀀스 노드를 시작한다. launch만 실행해도 주행을 시작하지는 않는다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from coop_carry_bringup.model_factory import share


def setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    return [
        Node(
            package="coop_carry_bringup",
            executable="base_move_node",
            name="carry_base_move",
            output="screen",
            parameters=[
                str(share("coop_carry_bringup") / "config/base_move.yaml"),
                {"mode": mode},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="swerve"),
            OpaqueFunction(function=setup),
        ]
    )
