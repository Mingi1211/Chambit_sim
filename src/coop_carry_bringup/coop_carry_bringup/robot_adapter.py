"""공개 목표를 검사해 QP로 전달하고 로봇의 제어 준비 상태를 보고한다."""

from copy import deepcopy
import math
import signal
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from robotis_interfaces.msg import MoveL
from sensor_msgs.msg import JointState


class RobotAdapter(Node):
    """로봇 namespace 안에서만 명령과 상태를 처리한다."""

    def __init__(self):
        super().__init__("robot_adapter")
        self.robot = self.get_namespace().strip("/")
        if self.robot not in ("robot1", "robot2"):
            raise ValueError("Robot namespace required")
        self.joints_seen = 0.0
        self.qp_seen = 0.0
        self.controller_states = {}
        self.hardware_states = {}
        self.query_time = {"controllers": 0.0, "hardware": 0.0}
        self.pending = {}
        self.state_clients = {
            "controllers": (
                self.create_client(
                    ListControllers, "controller_manager/list_controllers"
                ),
                ListControllers,
            ),
            "hardware": (
                self.create_client(
                    ListHardwareComponents,
                    "controller_manager/list_hardware_components",
                ),
                ListHardwareComponents,
            ),
        }
        self.status_pub = self.create_publisher(DiagnosticArray, "status", 10)
        self.goal_pubs = {
            side: self.create_publisher(MoveL, f"qp/{side}_goal_move", 10)
            for side in ("r", "l")
        }
        self.subs = [
            self.create_subscription(JointState, "joint_states", self.on_joints, 10),
            self.create_subscription(PoseStamped, "r_gripper_pose", self.on_qp, 10),
        ]
        for side in ("r", "l"):
            self.subs.append(
                self.create_subscription(
                    MoveL,
                    f"{side}_goal_move",
                    lambda msg, s=side: self.goal(msg, s),
                    10,
                )
            )
        # Report lost simulation/ROS feedback even when /clock has stopped.
        self.timer = self.create_timer(
            0.25, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def on_joints(self, msg):
        if msg.name and len(msg.name) == len(msg.position):
            self.joints_seen = time.monotonic()

    def on_qp(self, msg):
        self.qp_seen = time.monotonic()

    def ready(self):
        """최신 피드백, 활성 제어기·하드웨어, QP 입력 연결을 모두 확인한다."""
        now = time.monotonic()
        return (
            now - self.joints_seen < 0.5
            and now - self.qp_seen < 0.5
            and all(now - queried_at < 1.5 for queried_at in self.query_time.values())
            and all(
                self.controller_states.get(name) == "active"
                for name in (
                    "joint_state_broadcaster",
                    "arm_r_controller",
                    "arm_l_controller",
                )
            )
            and self.hardware_states.get(f"{self.robot}_remote_sim") == "active"
            and all(
                publisher.get_subscription_count() == 1
                for publisher in self.goal_pubs.values()
            )
        )

    def goal(self, msg, side):
        """유효한 로봇 좌표계 목표만 내부 QP 토픽으로 전달한다."""
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
        quaternion_norm_squared = sum(value * value for value in values[3:])
        frame = msg.pose.header.frame_id
        duration = msg.time_from_start
        if (
            not all(math.isfinite(v) for v in values)
            or abs(quaternion_norm_squared - 1.0) > 0.01
            or frame not in ("base_link", f"{self.robot}/base_link")
            or duration.sec < 0
            or not 0 <= duration.nanosec < 1000000000
        ):
            self.get_logger().error(
                f"Rejected invalid {side} MoveL (use robot-local base_link and unit quaternion)"
            )
            return
        if not self.ready():
            self.get_logger().error(f"Rejected {side} MoveL: robot not ready")
            return
        forwarded = deepcopy(msg)
        forwarded.pose.header.frame_id = "base_link"
        self.goal_pubs[side].publish(forwarded)

    def _update_state_queries(self, now):
        """종류별 요청을 하나씩 유지하고 응답이 없으면 취소 후 다시 조회한다."""
        for key, (client, service_type) in self.state_clients.items():
            if key in self.pending:
                future, sent = self.pending[key]
                if future.done():
                    try:
                        result = future.result()
                        if key == "controllers":
                            self.controller_states = {
                                controller.name: controller.state
                                for controller in result.controller
                            }
                        else:
                            self.hardware_states = {
                                component.name: component.state.label
                                for component in result.component
                            }
                        self.query_time[key] = now
                    except Exception as exc:
                        self.get_logger().warning(f"{key} status query failed: {exc}")
                    del self.pending[key]
                elif now - sent > 1.0:
                    future.cancel()
                    del self.pending[key]
            if key not in self.pending and client.service_is_ready():
                self.pending[key] = (client.call_async(service_type.Request()), now)

    def _publish_status(self, now):
        item = DiagnosticStatus(
            name=f"{self.robot}/runtime",
            hardware_id=self.robot,
            level=DiagnosticStatus.OK if self.ready() else DiagnosticStatus.WARN,
            message="ready" if self.ready() else "waiting or disconnected",
        )
        values = {
            "joint_state_age_sec": now - self.joints_seen,
            "qp_age_sec": now - self.qp_seen,
            **self.controller_states,
            **self.hardware_states,
        }
        item.values = [
            KeyValue(key=key, value=str(value)) for key, value in values.items()
        ]
        status = DiagnosticArray()
        status.header.stamp = self.get_clock().now().to_msg()
        status.status = [item]
        self.status_pub.publish(status)

    def poll(self):
        """시뮬레이션 시간이 멈춰도 실제 시간 기준으로 연결 상태를 갱신한다."""
        now = time.monotonic()
        self._update_state_queries(now)
        self._publish_status(now)


def main():
    # 종료 신호는 요청만 남기고, ROS 정리는 아래 finally에서 한 번 수행한다.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = RobotAdapter()
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
