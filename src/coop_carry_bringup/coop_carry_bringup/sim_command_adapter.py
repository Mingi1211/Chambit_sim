"""관절 이름이 있는 명령을 Gazebo 구동 플러그인의 배열 순서로 변환한다."""

import math
import signal
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from actuator_msgs.msg import Actuators


class Adapter(Node):
    """목표 계산 없이 관절 이름·값을 검사하고 명령 형식만 변환한다."""

    def __init__(self):
        super().__init__("sim_command_adapter")
        robot = self.declare_parameter("robot", "robot1").value
        if robot not in ("robot1", "robot2"):
            raise ValueError("Invalid robot")
        path = self.declare_parameter("joint_map", "").value
        self.commanded_joints = yaml.safe_load(Path(path).read_text())[
            "commanded_joints"
        ]
        self.command_publisher = self.create_publisher(
            Actuators, f"/sim/{robot}/actuators", 1
        )
        self.command_subscription = self.create_subscription(
            JointState, f"/sim/{robot}/joint_commands", self.command, 1
        )

    def command(self, msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        positions_by_name = dict(zip(msg.name, msg.position))
        if set(positions_by_name) != set(self.commanded_joints) or not all(
            math.isfinite(value) for value in positions_by_name.values()
        ):
            return
        output = Actuators()
        output.header = msg.header
        output.position = [positions_by_name[name] for name in self.commanded_joints]
        self.command_publisher.publish(output)


def main():
    # Keep ROS context cleanup in this thread; native signal shutdown can race
    # with finally when both the terminal and launch deliver SIGINT.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = Adapter()
    try:
        while not stop_requested and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
