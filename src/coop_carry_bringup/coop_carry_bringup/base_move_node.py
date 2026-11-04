"""들어올림 확인 후 사용자가 시작하는 두 로봇의 방향·시간 기반 스워브 주행."""

from dataclasses import asdict, fields
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .base_interface import MockBaseInterface, SwerveBaseInterface
from .base_sequence import BaseSequence, MoveSettings
from .command_client import require_simulation

STATE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class BaseMoveNode(Node):
    def __init__(self):
        super().__init__("carry_base_move")
        self.mode = self.declare_parameter("mode", "swerve").value
        if self.mode not in ("mock", "swerve"):
            raise ValueError("mode must be mock or swerve")
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", value=self.mode == "swerve")]
        )
        defaults = MoveSettings()
        self.settings = MoveSettings(
            **{
                item.name: self.declare_parameter(
                    item.name, getattr(defaults, item.name)
                ).value
                for item in fields(defaults)
            }
        )
        self.sequence = BaseSequence(self.settings)
        self.interface = (
            MockBaseInterface if self.mode == "mock" else SwerveBaseInterface
        )(self, self.settings)
        self.done_publisher = self.create_publisher(
            Bool, "/carry/base_move_done", STATE_QOS
        )
        self.status_publisher = self.create_publisher(
            String, "/carry/base_move_status", STATE_QOS
        )
        # 일반 수동 발행과 retained 상태 발행을 모두 수신한다. 중복된 동일 bool은 무해하다.
        self.lift_subscriptions = [
            self.create_subscription(Bool, "/carry/lift_done", self.on_lift, STATE_QOS),
            self.create_subscription(Bool, "/carry/lift_done", self.on_lift, 10),
        ]
        self.service_servers = [
            self.create_service(Trigger, "/carry/start_base_move", self.start),
            self.create_service(Trigger, "/carry/reset_base_move", self.reset),
            self.create_service(Trigger, "/carry/stop_base_move", self.abort),
            self.create_service(Trigger, "/carry/get_base_move_status", self.status),
        ]
        self.initial_positions = {}
        self.initial_yaws = {}
        self.last_tick = self.created_at = time.monotonic()
        self.last_status = 0.0
        self.reported_state = None
        self.timer = self.create_timer(
            0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.report()
        self.get_logger().info(
            f"[{self.mode.upper()}] Ready to monitor lift; movement requires /carry/start_base_move"
        )

    def sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_lift(self, msg):
        self.sequence.on_lift(msg.data)
        if self.sequence.state == "FAULT":
            self.interface.stop()
        self.report()

    def ready(self):
        if time.monotonic() - self.created_at < 1.0:
            return False, "Waiting for graph discovery"
        instances = [
            name
            for name, namespace in self.get_node_names_and_namespaces()
            if name == self.get_name()
        ]
        if len(instances) != 1:
            return False, "Duplicate carry_base_move node detected"
        return self.interface.ready()

    def stopped(self):
        return all(
            state.linear_speed <= self.settings.stop_linear
            and state.angular_speed <= self.settings.stop_angular
            and state.wheel_speed <= self.settings.stop_wheel
            for state in self.interface.states.values()
        )

    def moved(self):
        norm = math.hypot(self.settings.direction_x, self.settings.direction_y)
        sign = math.copysign(1.0, self.settings.distance)
        minimum = min(self.settings.min_travel, abs(self.settings.distance) * 0.1)
        for robot, state in self.interface.states.items():
            x0, y0 = self.initial_positions[robot]
            progress = (
                (
                    (state.x - x0) * self.settings.direction_x
                    + (state.y - y0) * self.settings.direction_y
                )
                * sign
                / norm
            )
            if progress < minimum:
                return False
        return True

    def start(self, request, response):
        ready, reason = self.ready()
        if self.sequence.state != "ARMED":
            response.success = False
            response.message = f"Start rejected: {self.sequence.state}; a NEW lift confirmation is required"
        elif not ready or not self.stopped():
            response.success = False
            response.message = (
                reason if not ready else "Both bases must be stopped before start"
            )
        else:
            self.initial_positions = {
                robot: (state.x, state.y)
                for robot, state in self.interface.states.items()
            }
            self.initial_yaws = {
                robot: state.yaw for robot, state in self.interface.states.items()
            }
            response.success, response.message = self.sequence.start(
                self.sim_time(), time.monotonic()
            )
        self.report()
        return response

    def reset(self, request, response):
        response.success, response.message = self.sequence.reset()
        if response.success:
            self.initial_positions.clear()
            self.initial_yaws.clear()
        self.report()
        return response

    def abort(self, request, response):
        self.interface.stop()
        self.sequence.fail("Stopped by user; reset and NEW lift confirmation required")
        response.success, response.message = True, self.sequence.reason
        self.report()
        return response

    def snapshot(self):
        ready, reason = self.ready()
        positions = {}
        for robot, state in self.interface.states.items():
            values = asdict(state)
            if robot in self.initial_positions:
                x0, y0 = self.initial_positions[robot]
                values.update(delta_x=state.x - x0, delta_y=state.y - y0)
            positions[robot] = values
        return dict(
            mode=self.mode,
            state=self.sequence.state,
            reason=self.sequence.reason,
            feedback_ready=ready,
            feedback_reason=reason,
            lift_done=self.sequence.lift_level,
            done=self.sequence.state == "DONE",
            elapsed=(
                max(0.0, self.sim_time() - self.sequence.started_sim)
                if self.initial_positions
                else 0.0
            ),
            robots=positions,
        )

    def status(self, request, response):
        response.success = True
        response.message = json.dumps(self.snapshot(), ensure_ascii=False)
        return response

    def report(self):
        changed = self.reported_state != self.sequence.state
        if changed:
            self.done_publisher.publish(Bool(data=self.sequence.state == "DONE"))
            prefix = f"[{self.mode.upper()}] {self.sequence.state}: "
            if self.sequence.state == "DONE":
                self.get_logger().info(
                    prefix
                    + "베이스 이동 시퀀스 완료 — 두 로봇 정지 확인. 내려놓기는 수동 실행."
                )
            elif self.sequence.state == "FAULT":
                self.get_logger().error(prefix + self.sequence.reason)
            else:
                self.get_logger().info(prefix + self.sequence.reason)
            self.reported_state = self.sequence.state
        now = time.monotonic()
        if changed or now - self.last_status >= 0.25:
            self.status_publisher.publish(
                String(data=json.dumps(self.snapshot(), ensure_ascii=False))
            )
            self.last_status = now

    def tick(self):
        now = time.monotonic()
        try:
            self.interface.update(min(now - self.last_tick, 0.1))
            self.last_tick = now
            ready, reason = self.ready()
            self.sequence.tick(
                self.sim_time(),
                now,
                ready,
                self.stopped(),
                self.moved() if self.initial_positions else False,
            )
            if self.sequence.state == "MOVING":
                speed = self.sequence.speed(self.sim_time())
                norm = math.hypot(self.settings.direction_x, self.settings.direction_y)
                self.interface.command(
                    speed * self.settings.direction_x / norm,
                    speed * self.settings.direction_y / norm,
                    self.initial_yaws,
                )
            else:
                self.interface.stop()
            if self.sequence.state == "FAULT" and not ready:
                self.sequence.reason = reason
        except Exception as error:
            self.sequence.fail(f"Base runtime error: {error}")
            self.interface.stop()
        self.report()


def main():
    require_simulation()
    # 같은 PC의 별도 터미널 중복 실행은 노드 생성 전 차단한다. 다른 호스트는 ROS graph로 감시한다.
    lock_path = Path(f"/tmp/coop_carry_base_{os.getuid()}_71.lock")
    with lock_path.open("w") as instance_lock:
        try:
            fcntl.flock(instance_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("Another base_move_node is already running")
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        stopping = False

        def request_stop(signum, frame):
            nonlocal stopping
            stopping = True

        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        node = None
        try:
            node = BaseMoveNode()
            while rclpy.ok() and not stopping:
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            if node is not None:
                node.interface.stop()
                node.done_publisher.publish(Bool(data=False))
                node.destroy_node()
            rclpy.try_shutdown()
