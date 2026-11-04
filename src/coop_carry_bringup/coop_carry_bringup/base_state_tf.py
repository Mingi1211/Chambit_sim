"""Gazebo 실제 차체 pose를 TF에 반영한다. 주행 명령이나 위치 보정은 하지 않는다."""

import math
import signal
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class BaseStateTF(Node):
    def __init__(self):
        super().__init__("base_state_tf")
        self.robot = self.get_namespace().strip("/")
        self.broadcaster = TransformBroadcaster(self)
        self.last_stamp = -1
        self.subscription = self.create_subscription(
            Odometry,
            f"/sim/{self.robot}/base_odometry",
            self.on_state,
            qos_profile_sensor_data,
        )

    def on_state(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        pose = msg.pose.pose
        values = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        if (
            stamp <= self.last_stamp
            or msg.header.frame_id != "world"
            or not all(map(math.isfinite, values))
        ):
            return
        self.last_stamp = stamp
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = f"{self.robot}/base_link"
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = BaseStateTF()
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
