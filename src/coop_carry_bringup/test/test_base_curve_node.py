"""ROS를 시작하지 않고 실제 노드 메서드와 가상 피드백으로 연동을 검증한다."""

import io
import math
from types import SimpleNamespace

import pytest

from coop_carry_bringup import base_move_node, base_move_curve_node
from coop_carry_bringup.base_curve import CurveProfile
from coop_carry_bringup.base_interface import (
    ROBOTS,
    BaseFeedback,
    MockBaseInterface,
    SwerveBaseInterface,
    world_to_body,
)
from coop_carry_bringup.base_move_node import BaseMoveNode
from coop_carry_bringup.base_move_curve_node import BaseMoveCurveNode
from coop_carry_bringup.base_sequence import BaseSequence, MoveSettings, profile_speed


def test_straight_command_still_sends_the_original_common_velocity():
    commands = []
    settings = MoveSettings(direction_x=3.0, direction_y=4.0)
    sequence = BaseSequence(settings)
    sequence.on_lift(True)
    sequence.start(1.0, 1.0)
    yaws = {"robot1": 0.0, "robot2": math.pi}
    node = SimpleNamespace(
        settings=settings,
        sequence=sequence,
        initial_yaws=yaws,
        sim_time=lambda: 3.5,
        interface=SimpleNamespace(command=lambda *args: commands.append(args)),
    )
    BaseMoveNode.command_motion(node)
    speed = profile_speed(2.5, 10.0, 2.0)
    assert commands == [(speed * 3.0 / 5.0, speed * 4.0 / 5.0, yaws)]


def test_curve_commands_are_robot_specific_and_transformed_by_initial_yaw():
    messages = {robot: [] for robot in ROBOTS}
    interface = SimpleNamespace(
        publishers={
            robot: SimpleNamespace(publish=messages[robot].append) for robot in ROBOTS
        }
    )
    interface.command_each = lambda *args: SwerveBaseInterface.command_each(
        interface, *args
    )
    profile = CurveProfile(MoveSettings())
    yaws = {"robot1": 0.0, "robot2": math.pi}
    node = SimpleNamespace(
        profile=profile,
        interface=interface,
        initial_yaws=yaws,
        sim_time=lambda: 3.5,
        sequence=SimpleNamespace(started_sim=1.0),
    )
    BaseMoveCurveNode.command_motion(node)
    for robot in ROBOTS:
        world_velocity = profile.velocity(2.5, curved=robot == "robot1")
        message = messages[robot][-1]
        assert (message.linear.x, message.linear.y) == pytest.approx(
            world_to_body(*world_velocity, yaws[robot])
        )
        assert message.angular.z == 0.0
        assert math.hypot(message.linear.x, message.linear.y) <= 0.4
    assert messages["robot1"][-1].linear.x < 0.0
    assert messages["robot2"][-1].linear.x == pytest.approx(0.0, abs=1e-15)


def simulated_node(monkeypatch):
    clock = SimpleNamespace(value=1.0)
    monkeypatch.setattr(base_move_node.time, "monotonic", lambda: clock.value)
    settings = MoveSettings()
    interface = MockBaseInterface(None, settings)
    node = SimpleNamespace(
        settings=settings,
        profile=CurveProfile(settings),
        interface=interface,
        sequence=BaseSequence(settings),
        initial_yaws={"robot1": 0.0, "robot2": math.pi},
        initial_positions={robot: (0.0, 0.0) for robot in ROBOTS},
        aligning=None,
        last_tick=clock.value,
        sim_time=lambda: clock.value,
        ready=lambda: (True, "mock ready"),
        report=lambda: None,
    )
    node.stopped = lambda: BaseMoveNode.stopped(node)
    node.moved = lambda: BaseMoveNode.moved(node)
    node.command_motion = lambda: BaseMoveCurveNode.command_motion(node)
    return node, clock


def test_inherited_tick_requires_start_and_completes_both_mock_paths(monkeypatch):
    node, clock = simulated_node(monkeypatch)
    BaseMoveNode.tick(node)
    assert node.sequence.state == "WAIT_LIFT"
    node.sequence.on_lift(True)
    BaseMoveNode.tick(node)
    assert node.sequence.state == "ARMED"
    assert all(
        velocity == (0.0, 0.0) for velocity in node.interface.velocities.values()
    )
    assert node.sequence.start(clock.value, clock.value)[0]
    peak_side = 0.0
    for index in range(1, 1201):
        clock.value = 1.0 + index * 0.01
        BaseMoveNode.tick(node)
        peak_side = max(peak_side, -node.interface.states["robot1"].x)
    assert node.sequence.state == "DONE"
    assert peak_side == pytest.approx(0.03, abs=1e-5)
    for state in node.interface.states.values():
        assert (state.x, state.y) == pytest.approx((0.0, 2.0), abs=1e-5)
        assert state.linear_speed == 0.0


@pytest.mark.parametrize("failure", ["feedback", "lift", "stop", "clock"])
def test_inherited_safety_stops_both_curve_commands(monkeypatch, failure):
    node, clock = simulated_node(monkeypatch)
    node.sequence.on_lift(True)
    node.sequence.start(clock.value, clock.value)
    clock.value += 2.5
    BaseMoveNode.tick(node)
    assert node.interface.velocities["robot1"][0] < 0.0
    if failure == "feedback":
        node.ready = lambda: (False, "feedback lost")
    elif failure == "lift":
        BaseMoveNode.on_lift(node, SimpleNamespace(data=False))
    elif failure == "stop":
        response = SimpleNamespace()
        BaseMoveNode.abort(node, None, response)
        assert response.success
    else:
        frozen = clock.value
        node.sim_time = lambda: frozen
        clock.value += 3.0
    BaseMoveNode.tick(node)
    assert node.sequence.state == "FAULT"
    assert all(
        velocity == (0.0, 0.0) for velocity in node.interface.velocities.values()
    )


def test_curve_status_reports_measured_final_error_and_yaw_drift(monkeypatch):
    monkeypatch.setattr(BaseMoveNode, "__init__", lambda self: None)
    node = BaseMoveCurveNode()
    node.settings = MoveSettings()
    node.profile = CurveProfile(node.settings)
    node.sequence = BaseSequence(node.settings)
    node.mode = "mock"
    node.ready = lambda: (True, "mock ready")
    node.sim_time = lambda: 10.0
    node.initial_positions = {"robot1": (-0.55, 0.0), "robot2": (0.55, 0.0)}
    node.initial_yaws = {"robot1": 0.0, "robot2": math.pi}
    node.interface = SimpleNamespace(
        states={
            "robot1": BaseFeedback(x=-0.56, y=1.98, yaw=0.01),
            "robot2": BaseFeedback(x=0.55, y=2.0, yaw=math.pi),
        }
    )
    status = node.snapshot()
    robot1 = status["robots"]["robot1"]
    assert (robot1["target_x"], robot1["target_y"]) == pytest.approx((-0.55, 2.0))
    assert robot1["target_error"] == pytest.approx(math.hypot(0.01, 0.02))
    assert robot1["side_displacement"] == pytest.approx(0.01)
    assert robot1["forward_progress"] == pytest.approx(1.98)
    assert robot1["yaw_error"] == pytest.approx(0.01)
    assert status["robots"]["robot2"]["target_error"] == 0.0


def test_both_entry_points_use_the_same_guarded_runner(monkeypatch):
    factories = []
    monkeypatch.setattr(base_move_node, "run_base_node", factories.append)
    monkeypatch.setattr(base_move_curve_node, "run_base_node", factories.append)
    base_move_node.main()
    base_move_curve_node.main()
    assert factories == [BaseMoveNode, BaseMoveCurveNode]


def test_shared_lock_rejects_duplicate_before_creating_ros_node(monkeypatch):
    class LockFile(io.StringIO):
        def fileno(self):
            return 42

    monkeypatch.setattr(base_move_node, "require_simulation", lambda: None)
    monkeypatch.setattr(base_move_node.Path, "open", lambda *args: LockFile())

    def occupied(*args):
        raise BlockingIOError()

    monkeypatch.setattr(base_move_node.fcntl, "flock", occupied)
    monkeypatch.setattr(
        base_move_node.rclpy, "init", lambda **kwargs: pytest.fail("ROS init")
    )
    with pytest.raises(SystemExit, match="already running"):
        base_move_node.run_base_node(lambda: pytest.fail("node creation"))
