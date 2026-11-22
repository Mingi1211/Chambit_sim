"""곡선 위치·미분·적분·속도 제한을 ROS 없이 검증한다."""

import math

import pytest

from coop_carry_bringup.base_curve import CurveProfile
from coop_carry_bringup.base_sequence import MoveSettings, profile_speed


def dot(vector, direction):
    return sum(value * axis for value, axis in zip(vector, direction))


@pytest.mark.parametrize("direction", [(0, 1), (1, 0), (3, 4), (-2, 1)])
@pytest.mark.parametrize("distance", [2.0, -2.0])
@pytest.mark.parametrize("offset", [0.03, -0.03, 0.0])
def test_endpoints_and_side_extremum(direction, distance, offset):
    profile = CurveProfile(
        MoveSettings(
            direction_x=direction[0], direction_y=direction[1], distance=distance
        ),
        offset,
    )
    for curved in (True, False):
        for time in (-1.0, 0.0):
            assert profile.position(time, curved) == (0.0, 0.0)
            assert profile.velocity(time, curved) == (0.0, 0.0)
        target = tuple(distance * axis for axis in profile.forward)
        for time in (10.0, 11.0):
            assert profile.position(time, curved) == pytest.approx(target)
            assert profile.velocity(time, curved) == (0.0, 0.0)
        middle = profile.position(5.0, curved)
        assert dot(middle, profile.forward) == pytest.approx(distance / 2.0)
        assert dot(middle, profile.side) == pytest.approx(offset if curved else 0.0)


def test_analytic_velocity_matches_position_derivative_and_is_continuous():
    profile = CurveProfile(MoveSettings(direction_x=3.0, direction_y=-4.0))
    dt = 1e-5
    for curved in (True, False):
        for time in (0.0, 0.001, 1.0, 2.5, 5.0, 7.5, 9.999, 10.0):
            before = profile.position(time - dt, curved)
            after = profile.position(time + dt, curved)
            derivative = tuple((b - a) / (2 * dt) for a, b in zip(before, after))
            assert profile.velocity(time, curved) == pytest.approx(derivative, abs=1e-8)
        for boundary in (0.0, 5.0, 10.0):
            assert profile.velocity(boundary - dt, curved) == pytest.approx(
                profile.velocity(boundary + dt, curved), abs=1e-5
            )


@pytest.mark.parametrize("direction", [(0.0, 1.0), (3.0, 4.0)])
def test_integrals_match_targets_and_both_robots_share_forward_motion(direction):
    settings = MoveSettings(direction_x=direction[0], direction_y=direction[1])
    profile = CurveProfile(settings)
    integrated = {curved: [0.0, 0.0] for curved in (True, False)}
    count = 10000
    dt = settings.duration / count
    max_side = 0.0
    for index in range(count):
        time = (index + 0.5) * dt
        max_side = max(max_side, dot(profile.position(time), profile.side))
        for curved in (True, False):
            velocity = profile.velocity(time, curved)
            assert dot(velocity, profile.forward) == pytest.approx(
                profile_speed(time, settings.duration, settings.distance), abs=1e-12
            )
            assert math.hypot(*velocity) <= profile.speed_bound <= settings.max_speed
            if not curved:
                assert dot(velocity, profile.side) == pytest.approx(0.0, abs=1e-12)
            for axis in range(2):
                integrated[curved][axis] += velocity[axis] * dt
    assert max_side == pytest.approx(profile.side_offset, abs=1e-8)
    for curved in (True, False):
        assert integrated[curved] == pytest.approx(
            profile.position(settings.duration), abs=1e-8
        )


def test_side_velocity_changes_sign_once_and_forward_is_unchanged():
    profile = CurveProfile(MoveSettings())
    for time in (1.0, 2.5, 4.9):
        assert dot(profile.velocity(time), profile.side) > 0.0
        assert dot(profile.velocity(10.0 - time), profile.side) < 0.0
    assert dot(profile.velocity(5.0), profile.side) == pytest.approx(0.0, abs=1e-15)
    assert profile.position(5.0) == pytest.approx((-0.03, 1.0))


@pytest.mark.parametrize("offset", [float("nan"), float("inf"), -float("inf"), 0.5])
def test_invalid_or_excessive_curve_settings_are_rejected(offset):
    with pytest.raises(ValueError):
        CurveProfile(MoveSettings(), offset)


def test_larger_offset_can_be_used_with_longer_duration():
    profile = CurveProfile(MoveSettings(duration=20.0), side_offset=0.5)
    assert profile.speed_bound < profile.settings.max_speed


def test_thirty_centimeter_offset_is_valid_at_two_meters_in_ten_seconds():
    profile = CurveProfile(MoveSettings(), side_offset=0.30)
    # 이전 전역 상한은 0.414552였지만 서로 다른 시점의 최대값을 합친 결과였다.
    assert profile.speed_bound < 0.4
    assert profile.speed_bound > 0.381
    for index in range(10001):
        assert math.hypot(*profile.velocity(index / 1000.0)) <= profile.speed_bound


@pytest.mark.parametrize("offset", [0.0, 0.03, 0.30, -0.30, 0.5, 1.0])
def test_interval_speed_bound_covers_the_whole_profile(offset):
    profile = CurveProfile(MoveSettings(duration=30.0), side_offset=offset)
    measured_peak = max(
        math.hypot(*profile.velocity(index * 30.0 / 20000)) for index in range(20001)
    )
    assert measured_peak <= profile.speed_bound
    assert profile.speed_bound - measured_peak < 0.002
