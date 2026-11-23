"""Robot 1만 측면으로 이탈했다 복귀한다. 상태·승인·안전 감시는 직선 노드를 상속한다."""

import math

from .base_curve import CurveProfile
from .base_interface import ROBOTS
from .base_move_node import BaseMoveNode, run_base_node


class BaseMoveCurveNode(BaseMoveNode):
    def configure_trajectory(self):
        side_offset = self.declare_parameter("side_offset", 0.03).value
        self.profile = CurveProfile(self.settings, side_offset)

    def curved(self, robot):
        return robot == "robot1"

    def command_motion(self):
        if getattr(self, "start_floor", False) or getattr(self, "track_feedback", False):
            return self.tracked_motion()
        elapsed = self.sim_time() - self.sequence.started_sim
        self.interface.command_each(
            {
                robot: self.profile.velocity(elapsed, curved=robot == "robot1")
                for robot in ROBOTS
            },
            self.initial_yaws,
        )

    def snapshot(self):
        status = super().snapshot()
        status.update(
            trajectory="curve",
            side_offset=self.profile.side_offset,
            side_direction_world=self.profile.side,
            command_speed_bound=self.profile.speed_bound,
        )
        ux, uy = self.profile.forward
        nx, ny = self.profile.side
        dx, dy = self.profile.position(self.settings.duration)
        for robot, values in status["robots"].items():
            if robot not in self.initial_positions:
                continue
            x0, y0 = self.initial_positions[robot]
            target_x, target_y = x0 + dx, y0 + dy
            error_x, error_y = target_x - values["x"], target_y - values["y"]
            yaw_delta = values["yaw"] - self.initial_yaws[robot]
            values.update(
                target_x=target_x,
                target_y=target_y,
                target_error_x=error_x,
                target_error_y=error_y,
                target_error=math.hypot(error_x, error_y),
                forward_progress=values["delta_x"] * ux + values["delta_y"] * uy,
                side_displacement=values["delta_x"] * nx + values["delta_y"] * ny,
                yaw_error=math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)),
            )
        return status


def main():
    run_base_node(BaseMoveCurveNode)
