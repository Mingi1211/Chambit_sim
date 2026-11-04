"""Host-only ROS client: state monitoring and robot-local EE commands, no QP."""

import argparse
from copy import deepcopy
import json
import math
import os
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from robotis_interfaces.msg import MoveL
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from tf2_ros import Buffer, TransformListener


class RobotClient(Node):
    """상태·TF를 받아 손끝 목표를 만든다. QP 계산은 로봇 컨테이너가 담당한다."""

    def __init__(self, robots=("robot1", "robot2")):
        super().__init__(
            "robot_command_client",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
        )
        self.robots = robots
        self.states = {}
        self.status = {}
        self.received = {}
        self.commands = {}
        self.histories = {robot: [] for robot in robots}
        self.command_counts = {}
        self.raw = {}
        self.raw_stamp = {}
        self.pubs = {}
        self.subs = []
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        for robot in robots:
            self.subs.append(
                self.create_subscription(
                    JointState,
                    f"/{robot}/joint_states",
                    lambda msg, r=robot: self.on_state(msg, r),
                    10,
                )
            )
            self.subs.append(
                self.create_subscription(
                    DiagnosticArray,
                    f"/{robot}/status",
                    lambda msg, r=robot: self.on_status(msg, r),
                    10,
                )
            )
            self.subs.append(
                self.create_subscription(
                    JointState,
                    f"/sim/{robot}/joint_states",
                    lambda msg, r=robot: self.on_raw(msg, r),
                    rclpy.qos.qos_profile_sensor_data,
                )
            )
            for side in ("r", "l"):
                self.pubs[robot, side] = self.create_publisher(
                    MoveL, f"/{robot}/{side}_goal_move", 10
                )
                self.subs.append(
                    self.create_subscription(
                        JointTrajectory,
                        f"/{robot}/arm_{side}_controller/joint_trajectory",
                        lambda msg, k=(robot, side): self.on_command(msg, k),
                        10,
                    )
                )

    def on_state(self, msg, robot):
        self.states[robot] = dict(zip(msg.name, msg.position))
        self.histories[robot].append(self.states[robot])
        # Bound monitoring memory during long interactive sessions.
        if len(self.histories[robot]) > 2000:
            del self.histories[robot][:1000]
        self.received[robot] = time.monotonic()

    def on_status(self, msg, robot):
        if msg.status:
            self.status[robot] = (time.monotonic(), msg.status[0])

    def on_raw(self, msg, robot):
        self.raw[robot] = dict(zip(msg.name, msg.position))
        self.raw_stamp[robot] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def on_command(self, msg, key):
        if msg.points:
            self.commands[key] = dict(zip(msg.joint_names, msg.points[0].positions))
            self.command_counts[key] = self.command_counts.get(key, 0) + 1

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(
                self, timeout_sec=max(0.0, min(0.02, deadline - time.monotonic()))
            )

    def pose(self, robot, side):
        """해당 로봇의 base_link 기준으로 측정된 손끝 위치·방향을 반환한다."""
        return self.buffer.lookup_transform(
            f"{robot}/base_link", f"{robot}/end_effector_{side}_link", Time()
        ).transform

    def wait_ready(self, timeout=120):
        """최신 상태·목표 입력 연결·양팔 TF가 모두 준비될 때까지 기다린다."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin_for(0.1)
            now = time.monotonic()
            if not all(
                r in self.states
                and r in self.status
                and now - self.received[r] < 0.5
                and now - self.status[r][0] < 1.0
                and self.status[r][1].level == DiagnosticStatus.OK
                and all(
                    self.pubs[r, s].get_subscription_count() == 1 for s in ("r", "l")
                )
                for r in self.robots
            ):
                continue
            try:
                for r in self.robots:
                    for s in ("r", "l"):
                        self.pose(r, s)
            except Exception:
                continue
            return
        raise RuntimeError(
            "Robot adapters, live feedback, TF or controllers did not become ready"
        )

    def make_goal(
        self,
        robot,
        side,
        position=None,
        offset=(0.015, 0.0, 0.0),
        orientation=None,
        duration=2.0,
    ):
        """절대 좌표 또는 현재 위치에 대한 offset을 MoveL 목표로 변환한다."""
        measured = self.pose(robot, side)
        goal = MoveL()
        goal.pose.header.frame_id = "base_link"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        for index, axis in enumerate(("x", "y", "z")):
            if position is not None:
                target = position[index]
            else:
                target = getattr(measured.translation, axis) + offset[index]
            setattr(goal.pose.pose.position, axis, target)

        goal.pose.pose.orientation = deepcopy(measured.rotation)
        if orientation is not None:
            for axis, value in zip(("x", "y", "z", "w"), orientation):
                setattr(goal.pose.pose.orientation, axis, value)

        goal.time_from_start.sec = int(duration)
        goal.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        return goal


def require_simulation():
    if (
        os.environ.get("ROS_DOMAIN_ID") != "71"
        or os.environ.get("RMW_IMPLEMENTATION") != "rmw_zenoh_cpp"
    ):
        raise RuntimeError("Use ROS_DOMAIN_ID=71 and RMW_IMPLEMENTATION=rmw_zenoh_cpp")


def load_goals(path):
    """YAML 목표의 로봇·팔 중복, 좌표 형식, 방향, 이동 시간을 검사한다."""
    entries = yaml.safe_load(Path(path).read_text())["goals"]
    if not isinstance(entries, list) or not entries:
        raise ValueError("Nonempty goals list required")
    seen = set()
    for goal in entries:
        if set(goal) - {
            "robot",
            "arm",
            "position",
            "offset",
            "orientation",
            "duration",
        }:
            raise ValueError("Unknown goal field")
        key = (goal["robot"], goal["arm"])
        if (
            key[0] not in ("robot1", "robot2")
            or key[1] not in ("r", "l")
            or key in seen
        ):
            raise ValueError("Each goal must identify a unique robot and arm")
        seen.add(key)
        if ("position" in goal) == ("offset" in goal):
            raise ValueError("Specify position or offset per goal")
        for field, size in [("position", 3), ("offset", 3), ("orientation", 4)]:
            if field in goal:
                if len(goal[field]) != size or not all(
                    math.isfinite(float(v)) for v in goal[field]
                ):
                    raise ValueError(f"Invalid {field}")
                goal[field] = [float(v) for v in goal[field]]
        if (
            "orientation" in goal
            and abs(sum(v * v for v in goal["orientation"]) - 1) > 0.01
        ):
            raise ValueError("Orientation must be a unit quaternion")
        goal["duration"] = float(goal.get("duration", 2.0))
        if not math.isfinite(goal["duration"]) or goal["duration"] <= 0:
            raise ValueError("Positive duration required")
    return entries


def _parse_arguments():
    """명령행 옵션과 선택적인 YAML 계획을 검사한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=["robot1", "robot2"], default="robot1")
    parser.add_argument("--arm", choices=["r", "l", "both"], default="r")
    parser.add_argument(
        "--goals", help="YAML list of distinct robot/arm goals to send together"
    )
    parser.add_argument(
        "--status",
        action="store_true",
        help="Print both robots without sending motion goals",
    )
    targets = parser.add_mutually_exclusive_group()
    targets.add_argument("--position", nargs=3, type=float, metavar=("X", "Y", "Z"))
    targets.add_argument(
        "--offset",
        nargs=3,
        type=float,
        default=[0.015, 0.0, 0.0],
        metavar=("DX", "DY", "DZ"),
    )
    parser.add_argument(
        "--orientation", nargs=4, type=float, metavar=("QX", "QY", "QZ", "QW")
    )
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()
    values = [
        args.duration,
        *args.offset,
        *(args.position or []),
        *(args.orientation or []),
    ]
    if not all(math.isfinite(v) for v in values) or args.duration <= 0:
        parser.error("Finite values and positive duration required")
    if args.orientation and abs(sum(v * v for v in args.orientation) - 1) > 0.01:
        parser.error("Orientation must be a unit quaternion")
    plan = None
    if args.goals:
        if args.status:
            parser.error("--goals and --status cannot be combined")
        try:
            plan = load_goals(args.goals)
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            parser.error(str(exc))
    return args, plan


def _print_status(node):
    """3초 동안 수집한 두 로봇의 상태를 한 번 출력한다."""
    node.spin_for(3.0)
    now = time.monotonic()
    result = {}
    for robot in node.robots:
        live = robot in node.status and now - node.status[robot][0] < 1.0
        result[robot] = {
            "status": node.status[robot][1].message if live else "disconnected",
            "joint_state_age_sec": (
                now - node.received[robot] if robot in node.received else None
            ),
            "controllers_and_hardware": (
                {value.key: value.value for value in node.status[robot][1].values}
                if live
                else {}
            ),
            "joints": node.states.get(robot, {}),
        }
    print(json.dumps(result, indent=2))


def _send_plan(node, plan):
    """모든 목표를 먼저 계산한 뒤 차례로 전송해 전송 시차를 줄인다."""
    node.wait_ready()
    prepared = []
    for entry in plan:
        goal = node.make_goal(
            entry["robot"],
            entry["arm"],
            position=entry.get("position"),
            offset=entry.get("offset", (0, 0, 0)),
            orientation=entry.get("orientation"),
            duration=entry["duration"],
        )
        prepared.append((entry["robot"], entry["arm"], goal))

    for robot, side, goal in prepared:
        node.pubs[robot, side].publish(goal)
        print(f"{robot}/{side}: MoveL goal sent in base_link", flush=True)
    node.spin_for(0.5)


def _send_single_command(node, args):
    """선택한 로봇의 한 팔 또는 양팔에 명령행 목표를 보낸다."""
    node.wait_ready()
    sides = ("r", "l") if args.arm == "both" else (args.arm,)
    for side in sides:
        goal = node.make_goal(
            args.robot,
            side,
            args.position,
            args.offset,
            args.orientation,
            args.duration,
        )
        node.pubs[args.robot, side].publish(goal)
        print(f"{args.robot}/{side}: MoveL goal sent in base_link", flush=True)
    node.spin_for(0.5)


def main():
    args, plan = _parse_arguments()
    require_simulation()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    robots = tuple(dict.fromkeys(g["robot"] for g in plan)) if plan else (args.robot,)
    node = RobotClient() if args.status else RobotClient(robots)
    try:
        if args.status:
            _print_status(node)
        elif plan:
            _send_plan(node, plan)
        else:
            _send_single_command(node, args)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
