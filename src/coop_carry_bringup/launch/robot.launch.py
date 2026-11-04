"""로봇 한 대의 하드웨어 → 제어기 → QP·Adapter를 순서대로 실행한다."""

import signal
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.events.process import SignalProcess
from launch.events.matchers import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from coop_carry_bringup.model_factory import qp_parameters, share
from coop_carry_bringup.swerve_model import BASE_JOINTS


def _after_success(target, following):
    """앞 단계가 정상 종료된 경우에만 다음 실행 단계를 시작한다."""

    def on_exit(event, context):
        if event.returncode == 0:
            return following
        return [EmitEvent(event=Shutdown(reason="Robot initialization failed"))]

    return RegisterEventHandler(OnProcessExit(target_action=target, on_exit=on_exit))


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    robot = arg("robot")
    if robot not in ("robot1", "robot2"):
        raise ValueError("robot must be robot1 or robot2")
    runtime_directory = Path(arg("runtime_dir")) / robot
    description = (runtime_directory / "control.urdf").read_text()
    has_swerve = f"{robot}_remote_swerve" in description
    hardware_names = [f"{robot}_remote_sim"]
    if has_swerve:
        hardware_names.append(f"{robot}_remote_swerve")
    manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=robot,
        parameters=[
            str(runtime_directory / "controllers.yaml"),
            {
                "use_sim_time": True,
                "hardware_components_initial_state": {"inactive": hardware_names},
            },
        ],
        output="screen",
        on_exit=[EmitEvent(event=Shutdown(reason="Controller Manager exited"))],
    )
    activate = Node(
        package="coop_carry_bringup",
        executable="activate_remote_hardware",
        namespace=robot,
        parameters=[{"robot": robot}],
        output="screen",
    )
    activate_base = Node(
        package="coop_carry_bringup",
        executable="activate_remote_hardware",
        namespace=robot,
        name="activate_swerve_hardware",
        parameters=[
            {
                "robot": robot,
                "component": "remote_swerve",
                "required_joints": list(BASE_JOINTS),
            }
        ],
        output="screen",
    )
    controllers = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot,
        arguments=(["swerve_drive_controller"] if has_swerve else [])
        + [
            "joint_state_broadcaster",
            "arm_r_controller",
            "arm_l_controller",
            "-c",
            f"/{robot}/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )
    qp = Node(
        package="cyclo_motion_controller_ros",
        executable="ai_worker_bimanual_movel_controller_node",
        name="bimanual_qp",
        namespace=robot,
        parameters=[
            qp_parameters(robot, share("coop_carry_bringup") / "config/qp.yaml")
        ],
        output="screen",
        on_exit=[EmitEvent(event=Shutdown(reason="Cyclo QP exited"))],
    )
    adapter = Node(
        package="coop_carry_bringup",
        executable="robot_adapter",
        namespace=robot,
        parameters=[{"use_sim_time": True}],
        output="screen",
        on_exit=[EmitEvent(event=Shutdown(reason="Robot Adapter exited"))],
    )

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot,
            parameters=[
                {
                    "robot_description": description,
                    "use_sim_time": True,
                    "frame_prefix": f"{robot}/",
                }
            ],
        ),
        manager,
        # 관절 상태 확인과 하드웨어 활성화가 끝난 뒤 제어기를 시작한다.
        _after_success(activate, [activate_base] if has_swerve else [controllers]),
        _after_success(controllers, [qp, adapter]),
        # OSQP can consume SIGINT during a solve. rclcpp handles SIGTERM normally.
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    EmitEvent(
                        event=SignalProcess(
                            signal_number=signal.SIGTERM,
                            process_matcher=matches_action(qp),
                        )
                    )
                ]
            )
        ),
        activate,
    ]
    if has_swerve:
        actions.extend(
            [
                _after_success(activate_base, [controllers]),
                Node(
                    package="coop_carry_bringup",
                    executable="base_state_tf",
                    namespace=robot,
                    parameters=[{"use_sim_time": True}],
                    output="screen",
                ),
            ]
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="robot1"),
            DeclareLaunchArgument("runtime_dir", default_value="/tmp/internship_sim"),
            OpaqueFunction(function=setup),
        ]
    )
