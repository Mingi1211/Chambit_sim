"""네 팔의 목표 도달, 비선택 팔 간섭, Gazebo 피드백 연결을 검증한다."""

import argparse
from copy import deepcopy
import json
import math
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from .command_client import RobotClient, require_simulation


ARMS = [("robot1", "r"), ("robot1", "l"), ("robot2", "r"), ("robot2", "l")]


def joint_delta(before, after, side):
    return max(
        abs(after[joint_name] - before[joint_name])
        for joint_name in before
        if joint_name.startswith(f"arm_{side}_joint")
    )


def position_error(a, b):
    return math.sqrt(
        sum((getattr(a, axis) - getattr(b, axis)) ** 2 for axis in ("x", "y", "z"))
    )


def angle_error(a, b):
    dot = sum(getattr(a, axis) * getattr(b, axis) for axis in ("x", "y", "z", "w"))
    return 2 * math.acos(min(1.0, abs(dot)))


def stationary(node):
    """두 번 연속으로 2초간 관절 변화가 작아질 때까지 기다린다."""
    quiet = 0
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        for robot in node.robots:
            node.histories[robot].clear()
        node.spin_for(2)
        drift = []
        for robot, side in ARMS:
            samples = node.histories[robot]
            if len(samples) < 2:
                raise RuntimeError(f"{robot}: measured joint stream stopped")
            drift.extend(
                max(entry[joint_name] for entry in samples)
                - min(entry[joint_name] for entry in samples)
                for joint_name in samples[0]
                if joint_name.startswith(f"arm_{side}_joint")
            )
        quiet = quiet + 1 if max(drift) < 0.0001 else 0
        if quiet >= 2:
            return max(drift)
    raise RuntimeError("Joints did not settle before next test")


def check_graph(node):
    for robot in node.robots:
        for topic, expected in [
            (f"/{robot}/joint_states", "joint_state_broadcaster"),
            (f"/sim/{robot}/joint_commands", f"{robot}_remote_sim"),
        ]:
            publishers = node.get_publishers_info_by_topic(topic)
            if len(publishers) != 1 or publishers[0].node_name != expected:
                raise RuntimeError(f"{topic}: wrong publisher ownership: {publishers}")
        for service_type, suffix, field in [
            (ListControllers, "list_controllers", "controller"),
            (ListHardwareComponents, "list_hardware_components", "component"),
        ]:
            client = node.create_client(
                service_type, f"/{robot}/controller_manager/{suffix}"
            )
            future = client.call_async(service_type.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done() or future.result() is None:
                raise RuntimeError("Manager query failed")
            entries = getattr(future.result(), field)
            if field == "controller":
                controllers = {entry.name: entry for entry in entries}
                for name in (
                    "arm_r_controller",
                    "arm_l_controller",
                    "joint_state_broadcaster",
                ):
                    controller = controllers[name]
                    if controller.state != "active" or controller.is_async:
                        raise RuntimeError(
                            f"{robot}/{name}: active synchronous controller required"
                        )
                if any("Serialized" in entry.type for entry in entries):
                    raise RuntimeError("Unexpected legacy mutex controller")
            else:
                hardware = {entry.name: entry for entry in entries}
                component = hardware[f"{robot}_remote_sim"]
                if (
                    component.state.label != "active"
                    or component.plugin_name != "coop_carry_hardware/RemoteGazeboSystem"
                ):
                    raise RuntimeError("Remote hardware ownership mismatch")
            node.destroy_client(client)
    names = {
        ns.rstrip("/") + "/" + name for name, ns in node.get_node_names_and_namespaces()
    }
    if any("gz_ros_control" in entry for entry in names):
        raise RuntimeError("Unexpected Gazebo internal Controller Manager")
    print(
        "PASS: independent ROS endpoints, external hardware, active synchronous stock JTCs",
        flush=True,
    )


def _observe_motion(node, goals, before, before_commands, observe):
    """목표를 전송하고 비선택 팔의 측정값·명령 변화 최대치를 수집한다."""
    idle = {key: 0.0 for key in ARMS if key not in goals}
    idle_commands = {key: 0.0 for key in idle}
    start = node.get_clock().now().nanoseconds
    send_times = []
    for key, goal in goals.items():
        node.pubs[key].publish(goal)
        send_times.append(time.monotonic())
    deadline = time.monotonic() + 60
    elapsed = 0.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        for robot, side in idle:
            idle[robot, side] = max(
                idle[robot, side], joint_delta(before[robot], node.states[robot], side)
            )
            if (robot, side) in before_commands and (robot, side) in node.commands:
                idle_commands[robot, side] = max(
                    idle_commands[robot, side],
                    joint_delta(
                        before_commands[robot, side], node.commands[robot, side], side
                    ),
                )
        elapsed = (node.get_clock().now().nanoseconds - start) / 1e9
        if elapsed >= observe:
            break
    return idle, idle_commands, send_times, elapsed


def _active_arm_results(node, label, goals, before):
    """선택한 팔이 실제로 움직여 위치·방향 오차 기준을 만족했는지 확인한다."""
    results = []
    for (robot, side), goal in goals.items():
        measured = node.pose(robot, side)
        error = position_error(measured.translation, goal.pose.pose.position)
        angular = angle_error(measured.rotation, goal.pose.pose.orientation)
        movement = joint_delta(before[robot], node.states[robot], side)
        if not (error < 0.003 and angular < 0.03 and movement > 0.001):
            raise RuntimeError(
                f"{label} {robot}/{side}: position={error}, orientation={angular}, movement={movement}"
            )
        results.append(
            {
                "robot": robot,
                "arm": side,
                "position_error_mm": round(error * 1000, 4),
                "orientation_error_rad": angular,
                "joint_motion_rad": movement,
                "goal": [
                    goal.pose.pose.position.x,
                    goal.pose.pose.position.y,
                    goal.pose.pose.position.z,
                ],
            }
        )
    return results


def _feedback_mismatch(node):
    """Gazebo 원본 상태와 외부 Controller Manager가 발행한 상태를 비교한다."""
    node.spin_for(0.2)
    mismatch = {
        robot: max(
            abs(node.raw[robot][joint_name] - node.states[robot][joint_name])
            for joint_name in node.states[robot]
        )
        for robot in node.robots
    }
    if max(mismatch.values()) > 0.002:
        raise RuntimeError(f"Manager/raw feedback mismatch: {mismatch}")
    return mismatch


def trial(node, label, chosen, observe):
    """정지 확인 → 목표 생성·전송 → 움직임·간섭·피드백 검사 순으로 실행한다."""
    baseline = stationary(node)
    before = deepcopy(node.states)
    before_commands = deepcopy(node.commands)
    stamps = dict(node.raw_stamp)
    offsets = {
        ("robot1", "r"): (0.015, 0, 0),
        ("robot1", "l"): (0.012, 0, 0.003),
        ("robot2", "r"): (0.014, 0, -0.002),
        ("robot2", "l"): (0.011, 0, 0.004),
    }
    goals = {key: node.make_goal(*key, offset=offsets[key]) for key in chosen}
    idle, idle_commands, send_times, elapsed = _observe_motion(
        node, goals, before, before_commands, observe
    )
    results = _active_arm_results(node, label, goals, before)
    if (
        max(idle.values(), default=0) > 0.001
        or max(idle_commands.values(), default=0) > 0.001
    ):
        raise RuntimeError(
            f"{label}: interference: measured={idle}, commands={idle_commands}"
        )
    if elapsed < observe or not all(
        node.raw_stamp.get(robot, 0) > stamps.get(robot, 0) for robot in node.robots
    ):
        raise RuntimeError("Simulation time or raw measured feedback stopped")
    mismatch = _feedback_mismatch(node)
    print(
        json.dumps(
            {
                "test": label,
                "result": "PASS",
                "active": results,
                "baseline_drift_rad": baseline,
                "inactive_arm_max_drift_rad": max(idle.values(), default=0),
                "inactive_arm_max_command_change_rad": max(
                    idle_commands.values(), default=0
                ),
                "raw_vs_manager_error_rad": mismatch,
                "send_window_ms": 1000 * (max(send_times) - min(send_times)),
                "elapsed_sim_sec": elapsed,
            }
        ),
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--suite", choices=["individual", "simultaneous", "all"], default="all"
    )
    parser.add_argument("--observation-time", type=float, default=6.0)
    args = parser.parse_args()
    if not math.isfinite(args.observation_time) or args.observation_time <= 0:
        parser.error("Positive observation time required")
    require_simulation()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RobotClient()
    try:
        node.wait_ready()
        check_graph(node)
        if args.suite in ("individual", "all"):
            for key in ARMS:
                trial(
                    node,
                    "single_" + key[0] + "_" + key[1],
                    [key],
                    args.observation_time,
                )
        if args.suite in ("simultaneous", "all"):
            for label, chosen in [
                ("robot1_both", ARMS[:2]),
                ("robot2_both", ARMS[2:]),
                ("cross_robot", [ARMS[0], ARMS[3]]),
                ("four_arms", ARMS),
            ]:
                trial(node, label, chosen, args.observation_time)
        print("PASS: requested motion suite completed", flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
