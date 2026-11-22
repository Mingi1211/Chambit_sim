"""ROS 없이 계산하는 측면 이탈·복귀 궤적. 모든 위치와 속도는 월드 XY 기준이다."""

from dataclasses import dataclass
from functools import cached_property
import math

from .base_sequence import MoveSettings, profile_speed


@dataclass(frozen=True)
class CurveProfile:
    settings: MoveSettings
    side_offset: float = 0.03

    def __post_init__(self):
        if not math.isfinite(self.side_offset):
            raise ValueError("side_offset must be finite")
        # 구간별 합성 속도 상한으로 검사한다. 속도를 잘라 궤적을 바꾸지 않는다.
        if self.speed_bound > self.settings.max_speed:
            raise ValueError(
                f"Curve speed bound {self.speed_bound:.6g} exceeds max_speed "
                f"{self.settings.max_speed:g}; increase duration or reduce side_offset"
            )

    @property
    def forward(self):
        norm = math.hypot(self.settings.direction_x, self.settings.direction_y)
        return self.settings.direction_x / norm, self.settings.direction_y / norm

    @property
    def side(self):
        """진행 방향 벡터의 왼쪽 수직 방향. +Y 전진이면 양의 오프셋은 -X다."""
        ux, uy = self.forward
        return -uy, ux

    @cached_property
    def speed_bound(self):
        """구간 내 s_dot·|sin(2πs)|의 각각의 최대값으로 전체 속도를 상계한다.

        점 샘플의 최대값이 아니라 각 구간 전체를 포함하는 상한이다.
        전진·측면 속도의 최대 시점이 다른 점을 반영해 과도한 거부를 줄인다.
        """
        upper = 0.0
        intervals = 256
        for index in range(intervals):
            lo, hi = index / intervals, (index + 1) / intervals
            s_lo = self.progress(lo * self.settings.duration)
            s_hi = self.progress(hi * self.settings.duration)
            sine_max = max(
                abs(math.sin(2.0 * math.pi * s_lo)),
                abs(math.sin(2.0 * math.pi * s_hi)),
            )
            # s는 단조 증가하며 |sin(2πs)|의 내부 최댓값은 s=1/4, 3/4다.
            if any(s_lo <= peak <= s_hi for peak in (0.25, 0.75)):
                sine_max = 1.0
            # s_dot은 tau=1/2까지 증가하고 이후 감소한다.
            tau_peak = min(hi, max(lo, 0.5))
            ds_max = profile_speed(
                tau_peak * self.settings.duration, self.settings.duration, 1.0
            )
            upper = max(
                upper,
                ds_max
                * math.hypot(
                    self.settings.distance, math.pi * self.side_offset * sine_max
                ),
            )
        return upper

    def progress(self, elapsed):
        tau = min(1.0, max(0.0, elapsed / self.settings.duration))
        return tau**3 * (10.0 - 15.0 * tau + 6.0 * tau**2)

    def position(self, elapsed, curved=True):
        """패턴 시작점 기준 명목 변위. 실제 베이스 위치를 보정하지 않는다."""
        s = self.progress(elapsed)
        lateral = (
            self.side_offset * math.sin(math.pi * s) ** 2
            if curved and 0.0 < elapsed < self.settings.duration
            else 0.0
        )
        ux, uy = self.forward
        nx, ny = self.side
        return (
            self.settings.distance * s * ux + lateral * nx,
            self.settings.distance * s * uy + lateral * ny,
        )

    def velocity(self, elapsed, curved=True):
        """d[L*s]/dt와 d[A*sin²(pi*s)]/dt를 합성한다."""
        s = self.progress(elapsed)
        forward_speed = profile_speed(
            elapsed, self.settings.duration, self.settings.distance
        )
        lateral_speed = (
            self.side_offset
            * math.pi
            * math.sin(2.0 * math.pi * s)
            * profile_speed(elapsed, self.settings.duration, 1.0)
            if curved and 0.0 < elapsed < self.settings.duration
            else 0.0
        )
        ux, uy = self.forward
        nx, ny = self.side
        return (
            forward_speed * ux + lateral_speed * nx,
            forward_speed * uy + lateral_speed * ny,
        )
