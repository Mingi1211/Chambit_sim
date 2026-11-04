"""시퀀스와 구동 연결을 분리한다. mock 모드는 실제 구동 토픽을 만들지 않는다."""

from dataclasses import dataclass
import math
import time

from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .swerve_model import BASE_JOINTS, DRIVE_JOINTS

ROBOTS = ("robot1", "robot2")


def yaw_of(quaternion):
    return math.atan2(
        2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1 - 2 * (quaternion.y**2 + quaternion.z**2),
    )


def world_to_body(vx, vy, yaw):
    return (
        math.cos(yaw) * vx + math.sin(yaw) * vy,
        -math.sin(yaw) * vx + math.cos(yaw) * vy,
    )


@dataclass
class BaseFeedback:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    linear_speed: float = 0.0
    angular_speed: float = 0.0
    wheel_speed: float = 0.0
    received: float = 0.0
    joint_received: float = 0.0
    stamp: float = -1.0
    joint_stamp: float = -1.0


class MockBaseInterface:
    """가상 피드백으로 시퀀스를 검증한다. Gazebo/실물 명령을 발행하지 않는다."""

    def __init__(self, node, settings):
        self.states = {robot: BaseFeedback() for robot in ROBOTS}
        self.velocities = {robot: (0.0, 0.0) for robot in ROBOTS}
        self.settings = settings

    def update(self, dt):
        now = time.monotonic()
        for robot, state in self.states.items():
            vx, vy = self.velocities[robot]
            state.x += vx * dt
            state.y += vy * dt
            state.linear_speed = math.hypot(vx, vy)
            state.wheel_speed = state.linear_speed / 0.0865
            state.received = state.joint_received = now

    def ready(self):
        return True, "mock feedback"

    def command(self, vx, vy, initial_yaws):
        for robot in ROBOTS:
            self.velocities[robot] = (vx, vy)

    def stop(self):
        self.command(0.0, 0.0, {})


class SwerveBaseInterface:
    """바퀴 odom으로 명령을 보정하지 않고, 실제 차체·바퀴 피드백으로 정지만 확인한다."""

    def __init__(self, node, settings):
        self.node = node
        self.settings = settings
        self.states = {robot: BaseFeedback() for robot in ROBOTS}
        self.publishers = {
            robot: node.create_publisher(Twist, f"/{robot}/cmd_vel", 1)
            for robot in ROBOTS
        }
        self.subscriptions = []
        self.queries = {}
        self.pending = {}
        self.controllers = {}
        self.hardware = {}
        self.query_times = {}
        self.last_query = 0.0
        for robot in ROBOTS:
            self.subscriptions.extend(
                [
                    node.create_subscription(
                        Odometry,
                        f"/sim/{robot}/base_odometry",
                        lambda msg, robot=robot: self.on_pose(robot, msg),
                        qos_profile_sensor_data,
                    ),
                    node.create_subscription(
                        JointState,
                        f"/sim/{robot}/joint_states",
                        lambda msg, robot=robot: self.on_joints(robot, msg),
                        qos_profile_sensor_data,
                    ),
                ]
            )
            for kind, service_type, suffix in (
                ("controllers", ListControllers, "list_controllers"),
                ("hardware", ListHardwareComponents, "list_hardware_components"),
            ):
                self.queries[robot, kind] = (
                    node.create_client(
                        service_type, f"/{robot}/controller_manager/{suffix}"
                    ),
                    service_type,
                )

    def on_pose(self, robot, msg):
        state = self.states[robot]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        pose, twist = msg.pose.pose, msg.twist.twist
        values = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        ]
        source_age = self.node.get_clock().now().nanoseconds / 1e9 - stamp
        if (
            stamp <= state.stamp
            or not all(map(math.isfinite, values))
            or not -0.1 <= source_age <= self.settings.feedback_timeout
            or msg.header.frame_id != "world"
            or msg.child_frame_id != f"{robot}/base_link"
            or abs(sum(value**2 for value in values[3:7]) - 1) > 0.01
        ):
            return
        state.x, state.y, state.yaw = (
            pose.position.x,
            pose.position.y,
            yaw_of(pose.orientation),
        )
        state.linear_speed = math.sqrt(sum(value**2 for value in values[7:10]))
        state.angular_speed = math.sqrt(sum(value**2 for value in values[10:13]))
        state.stamp, state.received = stamp, time.monotonic()

    def on_joints(self, robot, msg):
        state = self.states[robot]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.node.get_clock().now().nanoseconds / 1e9 - stamp
        if (
            stamp <= state.joint_stamp
            or not -0.1 <= age <= self.settings.feedback_timeout
            or len(msg.name) != len(msg.position)
            or len(msg.name) != len(msg.velocity)
            or len(set(msg.name)) != len(msg.name)
            or not set(BASE_JOINTS).issubset(msg.name)
        ):
            return
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        if not all(
            math.isfinite(positions[name]) and math.isfinite(velocities[name])
            for name in BASE_JOINTS
        ):
            return
        state.wheel_speed = max(abs(velocities[name]) for name in DRIVE_JOINTS)
        state.joint_stamp, state.joint_received = stamp, time.monotonic()

    def update(self, dt):
        now = time.monotonic()
        for key, (future, sent) in list(self.pending.items()):
            if future.done():
                try:
                    result = future.result()
                    if key[1] == "controllers":
                        self.controllers[key[0]] = {
                            entry.name: entry for entry in result.controller
                        }
                    else:
                        self.hardware[key[0]] = {
                            entry.name: entry for entry in result.component
                        }
                    self.query_times[key] = now
                except Exception:
                    pass
                del self.pending[key]
            elif now - sent > 1.0:
                future.cancel()
                del self.pending[key]
        if now - self.last_query < 0.25:
            return
        self.last_query = now
        for key, (client, service_type) in self.queries.items():
            if key not in self.pending and client.service_is_ready():
                self.pending[key] = (client.call_async(service_type.Request()), now)

    def ready(self):
        now = time.monotonic()
        for robot, state in self.states.items():
            if (
                max(now - state.received, now - state.joint_received)
                > self.settings.feedback_timeout
            ):
                return False, f"{robot}: missing/stale actual chassis or wheel feedback"
            if any(
                now - self.query_times.get((robot, kind), 0.0) > 1.5
                for kind in ("controllers", "hardware")
            ):
                return False, f"{robot}: Controller Manager query unavailable"
            controller = self.controllers.get(robot, {}).get("swerve_drive_controller")
            hardware = self.hardware.get(robot, {}).get(f"{robot}_remote_swerve")
            if (
                controller is None
                or controller.state != "active"
                or controller.is_async
                or hardware is None
                or hardware.state.label != "active"
                or hardware.plugin_name != "coop_carry_hardware/RemoteSwerveSystem"
            ):
                return False, f"{robot}: swerve controller/hardware not active"
            publisher = self.publishers[robot]
            owners = self.node.get_publishers_info_by_topic(f"/{robot}/cmd_vel")
            if (
                len(owners) != 1
                or owners[0].node_name != self.node.get_name()
                or publisher.get_subscription_count() != 1
            ):
                return (
                    False,
                    f"{robot}: cmd_vel must have one command owner and one controller",
                )
        return True, "swerve feedback and controllers ready"

    def command(self, vx, vy, initial_yaws):
        for robot, publisher in self.publishers.items():
            message = Twist()
            message.linear.x, message.linear.y = world_to_body(
                vx, vy, initial_yaws[robot]
            )
            publisher.publish(message)

    def stop(self):
        for publisher in self.publishers.values():
            publisher.publish(Twist())
