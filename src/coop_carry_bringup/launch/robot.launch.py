"""Identical per-robot controller/QP/adapter runtime, selected by namespace."""
import signal
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.events.process import SignalProcess
from launch.events.matchers import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from coop_carry_bringup.model_factory import qp_parameters, share


def setup(context):
    def arg(name): return LaunchConfiguration(name).perform(context)
    robot = arg('robot')
    if robot not in ('robot1', 'robot2'): raise ValueError('robot must be robot1 or robot2')
    root = Path(arg('runtime_dir')) / robot
    description = (root / 'control.urdf').read_text()
    manager = Node(package='controller_manager', executable='ros2_control_node', namespace=robot,
        parameters=[str(root / 'controllers.yaml'), {'use_sim_time': True,
                    'hardware_components_initial_state': {'inactive': [f'{robot}_remote_sim']}}], output='screen',
        on_exit=[EmitEvent(event=Shutdown(reason='Controller Manager exited'))])
    activate = Node(package='coop_carry_bringup', executable='activate_remote_hardware', namespace=robot,
                    parameters=[{'robot': robot}], output='screen')
    controllers = Node(package='controller_manager', executable='spawner', namespace=robot,
        arguments=['joint_state_broadcaster', 'arm_r_controller', 'arm_l_controller',
                   '-c', f'/{robot}/controller_manager', '--controller-manager-timeout', '120'], output='screen')
    qp = Node(package='cyclo_motion_controller_ros', executable='ai_worker_bimanual_movel_controller_node',
        name='bimanual_qp', namespace=robot,
        parameters=[qp_parameters(robot, share('coop_carry_bringup') / 'config/qp.yaml')], output='screen',
        on_exit=[EmitEvent(event=Shutdown(reason='Cyclo QP exited'))])
    adapter = Node(package='coop_carry_bringup', executable='robot_adapter', namespace=robot,
                   parameters=[{'use_sim_time': True}], output='screen',
                   on_exit=[EmitEvent(event=Shutdown(reason='Robot Adapter exited'))])
    def after(target, following):
        def done(event, context):
            return following if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Robot initialization failed'))]
        return RegisterEventHandler(OnProcessExit(target_action=target, on_exit=done))
    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=robot,
             parameters=[{'robot_description': description, 'use_sim_time': True, 'frame_prefix': f'{robot}/'}]),
        manager, after(activate, [controllers]),
        after(controllers, [qp, adapter]),
        # OSQP can consume SIGINT during a solve. rclcpp handles SIGTERM normally.
        RegisterEventHandler(OnShutdown(on_shutdown=[EmitEvent(event=SignalProcess(
            signal_number=signal.SIGTERM, process_matcher=matches_action(qp)))])),
        activate,
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='robot1'),
        DeclareLaunchArgument('runtime_dir', default_value='/tmp/internship_sim'), OpaqueFunction(function=setup)])
