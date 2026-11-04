"""ROS와 독립적인 수동 운반 시퀀스. 거리 오차 보정·다음 단계 자동 실행은 하지 않는다."""

from dataclasses import dataclass
import math


def profile_speed(elapsed, duration, distance):
    """5차 위치 시간 스케일링의 미분. 적분한 명목 이동량은 distance다."""
    tau = min(1.0, max(0.0, elapsed / duration))
    return distance / duration * 30.0 * tau**2 * (1.0 - tau) ** 2


@dataclass(frozen=True)
class MoveSettings:
    duration: float = 10.0
    distance: float = 2.0
    direction_x: float = 0.0
    direction_y: float = 1.0
    max_speed: float = 0.4
    feedback_timeout: float = 0.5
    settle_time: float = 0.5
    stop_timeout: float = 10.0
    wall_timeout: float = 180.0
    clock_timeout: float = 2.0
    stop_linear: float = 0.01
    stop_angular: float = 0.02
    stop_wheel: float = 0.1
    min_travel: float = 0.01

    def __post_init__(self):
        if not all(math.isfinite(value) for value in vars(self).values()):
            raise ValueError("Base settings must be finite")
        positive = (
            self.duration,
            self.max_speed,
            self.feedback_timeout,
            self.settle_time,
            self.stop_timeout,
            self.wall_timeout,
            self.clock_timeout,
            self.stop_linear,
            self.stop_angular,
            self.stop_wheel,
        )
        if min(positive) <= 0 or self.distance == 0 or self.min_travel < 0:
            raise ValueError("Positive times/limits and nonzero distance required")
        if math.hypot(self.direction_x, self.direction_y) < 1e-9:
            raise ValueError("A nonzero XY direction is required")
        if 1.875 * abs(self.distance) / self.duration > self.max_speed:
            raise ValueError("Quintic peak speed exceeds max_speed; increase duration")


class BaseSequence:
    """lift 신호와 시작 요청을 분리하고, 한 lift 허가를 한 번만 사용한다."""

    ACTIVE = ("MOVING", "STOPPING")

    def __init__(self, settings):
        self.settings = settings
        self.state = "WAIT_LIFT"
        self.reason = "Waiting for lift_done=True and a manual start request"
        self.lift_level = False
        self.require_low = False
        self.started_sim = self.started_wall = 0.0
        self.last_sim = self.last_clock_wall = 0.0
        self.stopping_wall = None
        self.still_since = None

    def on_lift(self, value):
        self.lift_level = value
        if not value:
            self.require_low = False
            if self.state in self.ACTIVE:
                self.fail("lift_done became False during movement")
            elif self.state == "ARMED":
                self.state = "WAIT_LIFT"
                self.reason = "Lift permission withdrawn"
        elif self.state == "WAIT_LIFT" and not self.require_low:
            self.state = "ARMED"
            self.reason = "Lift confirmed; waiting for MANUAL start"

    def start(self, sim_time, wall_time):
        if self.state != "ARMED" or not self.lift_level:
            return False, f"Start rejected: state={self.state}; confirm lift first"
        self.started_sim = self.last_sim = sim_time
        self.started_wall = self.last_clock_wall = wall_time
        self.stopping_wall = self.still_since = None
        self.state = "MOVING"
        self.reason = "Manual start accepted"
        return True, self.reason

    def reset(self):
        if self.state in self.ACTIVE:
            return False, "Reset rejected during movement; stop first"
        self.state = "WAIT_LIFT"
        # 이전 retained True를 다음 회차의 허가로 재사용하지 않는다.
        self.require_low = self.lift_level
        self.reason = (
            "Reset; publish lift_done=False, then a NEW True for the next cycle"
        )
        return True, self.reason

    def fail(self, reason):
        self.state = "FAULT"
        self.reason = reason

    def tick(self, sim_time, wall_time, feedback_ok, stopped, moved):
        if self.state not in self.ACTIVE:
            return
        if not feedback_ok:
            self.fail("Feedback/controller/command ownership lost")
            return
        if sim_time < self.last_sim - 1e-6:
            self.fail("Simulation clock moved backwards")
            return
        if sim_time > self.last_sim:
            self.last_sim = sim_time
            self.last_clock_wall = wall_time
        if wall_time - self.last_clock_wall > self.settings.clock_timeout:
            self.fail("Simulation clock stopped")
            return
        if wall_time - self.started_wall > self.settings.wall_timeout:
            self.fail("Movement wall-clock timeout")
            return
        if (
            self.state == "MOVING"
            and sim_time - self.started_sim >= self.settings.duration
        ):
            self.state = "STOPPING"
            self.reason = "Pattern finished; checking BOTH robots are stopped"
            self.stopping_wall = wall_time
        if self.state == "STOPPING":
            if wall_time - self.stopping_wall > self.settings.stop_timeout:
                self.fail("Measured stop timeout")
                return
            if stopped:
                if self.still_since is None:
                    self.still_since = wall_time
                if wall_time - self.still_since >= self.settings.settle_time:
                    if not moved:
                        self.fail("No measured base translation on one or both robots")
                    else:
                        self.state = "DONE"
                        self.reason = "BOTH robots stopped; nominal pattern complete (not exact-distance arrival)"
            else:
                self.still_since = None

    def speed(self, sim_time):
        if self.state != "MOVING":
            return 0.0
        return profile_speed(
            sim_time - self.started_sim, self.settings.duration, self.settings.distance
        )
