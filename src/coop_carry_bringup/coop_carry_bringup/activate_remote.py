"""Activate hardware only after increasing, complete Gazebo feedback is present."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SetHardwareComponentState
from lifecycle_msgs.msg import State


def main():
    rclpy.init()
    node = Node('activate_remote_hardware')
    robot = node.declare_parameter('robot', 'robot1').value
    if robot not in ('robot1', 'robot2'): raise ValueError('Invalid robot')
    latest, previous, advancing = [0.0], [-1], [0]
    required = {f'arm_{s}_joint{i}' for s in ('r', 'l') for i in range(1, 8)}
    required |= {'gripper_r_joint1', 'gripper_l_joint1', 'lift_joint', 'head_joint1', 'head_joint2'}

    def state(msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if (required.issubset(msg.name) and len(msg.name) == len(msg.position) == len(msg.velocity)
                and all(math.isfinite(v) for v in msg.position + msg.velocity)
                and stamp > previous[0]):
            latest[0], previous[0] = time.monotonic(), stamp
            advancing[0] += 1

    sub = node.create_subscription(JointState, f'/sim/{robot}/joint_states', state, qos_profile_sensor_data)
    client = node.create_client(SetHardwareComponentState, f'/{robot}/controller_manager/set_hardware_component_state')
    deadline = time.monotonic() + 120
    result = 1
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if advancing[0] >= 5 and time.monotonic() - latest[0] < 0.25 and client.service_is_ready():
            request = SetHardwareComponentState.Request()
            request.name = f'{robot}_remote_sim'
            request.target_state.id = State.PRIMARY_STATE_ACTIVE
            request.target_state.label = 'active'
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if future.done() and future.result().ok and future.result().state.id == State.PRIMARY_STATE_ACTIVE:
                node.get_logger().info('Fresh feedback confirmed; remote hardware ACTIVE')
                result = 0
                break
            # Activation can precede discovery by the hardware's own subscription.
            # Retry only after another batch of increasing source measurements.
            advancing[0] = 0
    if result:
        node.get_logger().error('Remote hardware activation failed or feedback did not become ready')
    node.destroy_node()
    rclpy.shutdown()
    return result
