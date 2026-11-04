"""수동 단계 분리·완료 판정·고장 처리를 Gazebo 없이 검증한다."""

import math
from types import SimpleNamespace
import pytest

from coop_carry_bringup.base_sequence import BaseSequence, MoveSettings, profile_speed
from coop_carry_bringup.base_interface import MockBaseInterface, world_to_body


def armed_sequence(**settings):
    sequence = BaseSequence(MoveSettings(**settings))
    sequence.on_lift(True)
    return sequence


def test_lift_does_not_start_motion_and_start_requires_lift():
    sequence = BaseSequence(MoveSettings())
    assert not sequence.start(1.0, 1.0)[0]
    sequence.on_lift(True)
    sequence.tick(1.0, 1.0, True, True, False)
    assert sequence.state == "ARMED"
    assert sequence.speed(1.0) == 0
    assert sequence.start(1.0, 1.0)[0]
    assert not sequence.start(2.0, 2.0)[0]


def test_time_alone_is_not_completion():
    sequence = armed_sequence()
    sequence.start(1, 1)
    sequence.tick(31, 31, True, False, True)
    assert sequence.state == "STOPPING"
    assert sequence.speed(31) == 0
    sequence.tick(32, 32, True, True, True)
    assert sequence.state == "STOPPING"
    sequence.tick(32.6, 32.6, True, True, True)
    assert sequence.state == "DONE"
    assert not sequence.start(33, 33)[0]


def test_settle_must_be_continuous():
    sequence = armed_sequence()
    sequence.start(1, 1)
    for t, stopped in ((31, True), (31.3, False), (31.6, True), (31.9, True)):
        sequence.tick(t, t, True, stopped, True)
    assert sequence.state == "STOPPING"
    sequence.tick(32.2, 32.2, True, True, True)
    assert sequence.state == "DONE"


def test_stationary_robot_is_not_reported_as_successful_travel():
    sequence = armed_sequence()
    sequence.start(1, 1)
    sequence.tick(31, 31, True, True, False)
    sequence.tick(32, 32, True, True, False)
    assert sequence.state == "FAULT"


@pytest.mark.parametrize(
    "sim_time,wall_time,feedback,reason",
    [
        (2, 2, False, "Feedback"),
        (0, 2, True, "backwards"),
        (1, 4, True, "clock stopped"),
        (2, 200, True, "wall-clock"),
    ],
)
def test_faults_stop_commands(sim_time, wall_time, feedback, reason):
    sequence = armed_sequence()
    sequence.start(1, 1)
    sequence.tick(sim_time, wall_time, feedback, False, True)
    assert sequence.state == "FAULT"
    assert reason.lower() in sequence.reason.lower()
    assert sequence.speed(10) == 0


def test_stop_timeout():
    sequence = armed_sequence()
    sequence.start(1, 1)
    sequence.tick(31, 31, True, False, True)
    sequence.tick(42, 42, True, False, True)
    assert sequence.state == "FAULT"
    assert "stop timeout" in sequence.reason


def test_reset_cannot_reuse_retained_true():
    sequence = armed_sequence()
    sequence.start(1, 1)
    assert not sequence.reset()[0]
    sequence.fail("user stop")
    assert sequence.reset()[0]
    sequence.on_lift(True)
    assert sequence.state == "WAIT_LIFT"
    sequence.on_lift(False)
    sequence.on_lift(True)
    assert sequence.state == "ARMED"


def test_withdrawn_lift_stops_active_sequence():
    sequence = armed_sequence()
    sequence.start(1, 1)
    sequence.on_lift(False)
    assert sequence.state == "FAULT"


def test_quintic_integral_and_boundaries():
    assert profile_speed(0, 30, 2) == 0
    assert profile_speed(30, 30, 2) == 0
    assert profile_speed(15, 30, 2) == pytest.approx(0.125)
    dt = 30 / 10000
    distance = sum(
        profile_speed((index + 0.5) * dt, 30, 2) * dt for index in range(10000)
    )
    assert distance == pytest.approx(2.0, abs=1e-6)
    assert profile_speed(15, 30, -2) == pytest.approx(-0.125)


@pytest.mark.parametrize(
    "settings",
    [
        {"duration": 0.0},
        {"distance": float("nan")},
        {"direction_y": 0.0},
        {"distance": 20.0},
        {"feedback_timeout": -1.0},
    ],
)
def test_invalid_settings(settings):
    with pytest.raises(ValueError):
        MoveSettings(**settings)


def test_facing_robots_get_opposite_body_velocities():
    assert world_to_body(0.0, 0.1, 0.0) == pytest.approx((0.0, 0.1))
    assert world_to_body(0.0, 0.1, math.pi) == pytest.approx((0.0, -0.1))


def test_mock_never_creates_physical_publishers():
    interface = MockBaseInterface(SimpleNamespace(), MoveSettings())
    interface.command(0.0, 0.1, {})
    interface.update(1.0)
    assert all(state.y == pytest.approx(0.1) for state in interface.states.values())
    interface.stop()
    interface.update(1.0)
    assert all(state.linear_speed == 0 for state in interface.states.values())
