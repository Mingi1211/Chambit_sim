"""정상적인 Gazebo 관절 상태가 연속 수신된 뒤 원격 하드웨어를 활성화한다."""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SetHardwareComponentState
from lifecycle_msgs.msg import State


class FeedbackMonitor:
    """필요한 관절이 모두 포함된, 시간이 증가하는 측정만 센다."""

    REQUIRED_JOINTS = {
        f"arm_{side}_joint{index}" for side in ("r", "l") for index in range(1, 8)
    } | {
        "gripper_r_joint1",
        "gripper_l_joint1",
        "lift_joint",
        "head_joint1",
        "head_joint2",
    }

    def __init__(self, required_joints=None):
        self.required_joints = (
            set(required_joints)
            if required_joints is not None
            else self.REQUIRED_JOINTS
        )
        self.last_received = 0.0
        self.previous_stamp = -1
        self.advancing_samples = 0

    def on_state(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if (
            self.required_joints.issubset(msg.name)
            and len(msg.name) == len(msg.position) == len(msg.velocity)
            and all(math.isfinite(v) for v in msg.position + msg.velocity)
            and stamp > self.previous_stamp
        ):
            self.last_received = time.monotonic()
            self.previous_stamp = stamp
            self.advancing_samples += 1

    def ready(self):
        return (
            self.advancing_samples >= 5 and time.monotonic() - self.last_received < 0.25
        )


def main():
    rclpy.init()
    node = Node("activate_remote_hardware")
    robot = node.declare_parameter("robot", "robot1").value
    if robot not in ("robot1", "robot2"):
        raise ValueError("Invalid robot")

    component = node.declare_parameter("component", "remote_sim").value
    required = node.declare_parameter(
        "required_joints", sorted(FeedbackMonitor.REQUIRED_JOINTS)
    ).value
    feedback = FeedbackMonitor(required)

    node.create_subscription(
        JointState,
        f"/sim/{robot}/joint_states",
        feedback.on_state,
        qos_profile_sensor_data,
    )
    client = node.create_client(
        SetHardwareComponentState,
        f"/{robot}/controller_manager/set_hardware_component_state",
    )
    deadline = time.monotonic() + 120
    result = 1
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if feedback.ready() and client.service_is_ready():
            request = SetHardwareComponentState.Request()
            request.name = f"{robot}_{component}"
            request.target_state.id = State.PRIMARY_STATE_ACTIVE
            request.target_state.label = "active"
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if (
                future.done()
                and future.result().ok
                and future.result().state.id == State.PRIMARY_STATE_ACTIVE
            ):
                node.get_logger().info(
                    "Fresh feedback confirmed; remote hardware ACTIVE"
                )
                result = 0
                break
            # Activation can precede discovery by the hardware's own subscription.
            # Retry only after another batch of increasing source measurements.
            feedback.advancing_samples = 0
    if result:
        node.get_logger().error(
            "Remote hardware activation failed or feedback did not become ready"
        )
    node.destroy_node()
    rclpy.shutdown()
    return result
