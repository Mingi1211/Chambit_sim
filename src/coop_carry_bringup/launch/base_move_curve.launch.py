"""기존 승인·시작 인터페이스를 사용하는 곡선 실험. launch만으로는 주행하지 않는다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from coop_carry_bringup.model_factory import share


def setup(context):
    config = share("coop_carry_bringup") / "config"
    # [김민기 11/24] params:=<yaml> 로 추가 파라미터(정렬 판정·폐루프 추종·side_offset 등)를 덮어씀. 기본(빈 값)은 기존과 같음
    extra = LaunchConfiguration("params").perform(context)
    return [
        Node(
            package="coop_carry_bringup",
            executable="base_move_curve_node",
            # 직선과 같은 이름·서비스·실행 잠금으로 중복 제어를 차단한다.
            name="carry_base_move",
            output="screen",
            parameters=[
                str(config / "base_move.yaml"),
                str(config / "base_move_curve.yaml"),
                {"mode": LaunchConfiguration("mode").perform(context)},
            ] + ([extra] if extra else []),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="swerve"),
            DeclareLaunchArgument("params", default_value=""),
            OpaqueFunction(function=setup),
        ]
    )
