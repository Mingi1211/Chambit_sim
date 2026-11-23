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

from .base_curve import CurveProfile
from .base_interface import MockBaseInterface, SwerveBaseInterface, world_to_body
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
        self.configure_trajectory()
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
        # [김민기 11/5] 바퀴 정렬 단계: 스워브 제어기는 조향이 이동 방향에 정렬될 때까지 구동을 막는데(정지 시 오차 0.01 rad),
        # 10초 속도 패턴은 시작 명령과 동시에 흐르므로 정렬 시간만큼 명령 거리가 사라짐(새 시뮬 첫 주행 2 m → 0.65~0.72 m 실측).
        # → 작은 속도로 방향만 주고, 두 로봇 차체가 실제로 움직이기 시작(= 두 로봇 모두 정렬 통과)한 순간 공통 10초 타이머 시작
        self.align_speed = self.declare_parameter("align_speed", 0.005).value      # [m/s] 정렬 중 방향 지령
        self.align_detect = self.declare_parameter("align_detect", 0.003).value   # [m/s] 차체 실측 속도 > 이 값 = 구동 시작
        self.align_timeout = self.declare_parameter("align_timeout", 15.0).value  # [s, 시뮬 시간]
        self.aligning = None
        # [김민기 11/24] 아래는 모두 기본 꺼짐 = 기존 동작 그대로. 박스 파지 곡선 주행에서 출발 지연(정렬 오탐) 수정용
        #  align_mode 'steering': 차체 속도 대신 두 로봇 조향 관절 3개가 모두 이동 방향과 align_tol 이내로 align_hold 유지되면 정렬 완료
        #     (박스를 쥐면 팔 반력으로 차체 속도가 align_detect를 넘어 바퀴가 돌기 전에 '정렬 완료' → 패턴 시작 후 구동이 막힘, 11/23 로그)
        #     align_tol = 스워브 제어기 steering_alignment_start_angle_error_threshold (model_factory.py 0.02 rad)
        #  start_floor: 패턴 초반 지령 속도가 align_speed보다 작으면 align_speed로 (5차 프로파일 시작 속도 0 → 제어기가 정지 상태로 돌아가 재정렬하는 것 방지)
        #  track_feedback: MR (11.17) ẋ = ẋ_d(t) + K_p (x_d(t) − x(t))를 경로 진행 방향 좌표에 적용 — 피드포워드 속도 크기에 진행 방향 위치 오차 비례항 (x = Gazebo 차체 자세)
        #  lag_limit/lag_time: 한 로봇이 지령 위치에서 lag_limit 넘게 lag_time 이상 뒤처지면 상태에 tracking_invalid 기록 (비교에서 제외)
        self.align_mode = self.declare_parameter("align_mode", "chassis").value
        self.align_tol = self.declare_parameter("align_tol", 0.02).value          # [rad]
        self.align_hold = self.declare_parameter("align_hold", 0.2).value         # [s, 시뮬 시간]
        self.start_floor = self.declare_parameter("start_floor", False).value
        self.track_feedback = self.declare_parameter("track_feedback", False).value
        self.track_kp = self.declare_parameter("track_kp", 1.0).value             # [1/s]
        self.lag_limit = self.declare_parameter("lag_limit", 0.05).value          # [m]
        self.lag_time = self.declare_parameter("lag_time", 1.0).value             # [s, 시뮬 시간]
        if not hasattr(self, "profile"):
            self.profile = CurveProfile(self.settings, 0.0)                       # 직선 = 옆 이탈 0인 같은 프로파일
        self.aligned_since = None
        self.align_error = float("nan")
        self.lag_since = {}
        self.tracking_invalid = ""
        self.max_lag = 0.0
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

    def configure_trajectory(self):
        """주행별 추가 파라미터를 검증한다. 직선 주행은 MoveSettings만 사용한다."""

    def curved(self, robot):
        """이 로봇이 옆 이탈을 하는가 (직선 주행은 모두 아님)"""
        return False

    def tracked_motion(self):
        """[김민기 11/24] start_floor / track_feedback이 켜졌을 때의 로봇별 월드 속도 지령"""
        elapsed = self.sim_time() - self.sequence.started_sim
        norm = math.hypot(self.settings.direction_x, self.settings.direction_y)
        sign = math.copysign(1.0, self.settings.distance)
        forward = (sign * self.settings.direction_x / norm, sign * self.settings.direction_y / norm)
        out = {}
        for robot, state in self.interface.states.items():
            vx, vy = self.profile.velocity(elapsed, curved=self.curved(robot))
            if self.start_floor and elapsed < 0.5 * self.settings.duration and math.hypot(vx, vy) < self.align_speed:
                n = math.hypot(vx, vy)
                ux, uy = (vx / n, vy / n) if n > 1e-9 else forward
                vx, vy = self.align_speed * ux, self.align_speed * uy
            dx, dy = self.profile.position(elapsed, curved=self.curved(robot))
            x0, y0 = self.initial_positions[robot]
            ex, ey = x0 + dx - state.x, y0 + dy - state.y
            if self.track_feedback:
                # MR (11.17)을 경로 진행 방향 좌표에: v = (|ẋ_d| + K_p e_t) t̂,  t̂ = 피드포워드 방향, e_t = 위치 오차의 t̂ 성분.
                # 지령 방향은 피드포워드 그대로 → 조향이 다시 정렬할 일이 없음 (조향 속도 ≈ 0.27 rad/s로 느려, 옆 성분 보정이
                # 저속에서 방향을 크게 틀면 스워브 제어기가 구동을 막음, 11/24 t1 런). 뒤로 가지 않게 크기 ≥ 0
                n = math.hypot(vx, vy)
                tx, ty = (vx / n, vy / n) if n > 1e-9 else forward
                speed = min(max(0.0, n + self.track_kp * (ex * tx + ey * ty)), self.settings.max_speed)
                vx, vy = speed * tx, speed * ty
            lag = math.hypot(ex, ey)
            self.max_lag = max(self.max_lag, lag)
            if lag > self.lag_limit:
                self.lag_since.setdefault(robot, elapsed)
                if elapsed - self.lag_since[robot] >= self.lag_time and not self.tracking_invalid:
                    self.tracking_invalid = f"{robot} lag {lag:.3f} m > {self.lag_limit} m for {self.lag_time} s at t={elapsed:.2f} s"
                    self.get_logger().warn(f"[SWERVE] 추종 무효: {self.tracking_invalid}")
            else:
                self.lag_since.pop(robot, None)
            out[robot] = (vx, vy)
        self.interface.command_each(out, self.initial_yaws)

    def command_motion(self):
        """기존 직선 속도를 두 로봇에 동일하게 전달한다."""
        if getattr(self, "start_floor", False) or getattr(self, "track_feedback", False):
            return self.tracked_motion()
        speed = self.sequence.speed(self.sim_time())
        norm = math.hypot(self.settings.direction_x, self.settings.direction_y)
        self.interface.command(
            speed * self.settings.direction_x / norm,
            speed * self.settings.direction_y / norm,
            self.initial_yaws,
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
            if self.sequence.state != "ARMED" or not self.sequence.lift_level:
                response.success, response.message = False, "Start rejected: confirm lift first"
            else:
                self.aligning = self.sim_time()
                response.success = True
                response.message = "Manual start accepted; aligning wheels before the 10 s pattern"
                self.get_logger().info("[SWERVE] ALIGNING: 두 로봇 바퀴 정렬 후 공통 타이머 시작")
        self.report()
        return response

    def align_tick(self):
        """정렬 단계: 방향만 작은 속도로 지령. 두 로봇 모두 차체가 움직이면 속도 패턴 시작"""
        norm = math.hypot(self.settings.direction_x, self.settings.direction_y)
        sign = math.copysign(1.0, self.settings.distance)
        self.interface.command(
            sign * self.align_speed * self.settings.direction_x / norm,
            sign * self.align_speed * self.settings.direction_y / norm,
            self.initial_yaws,
        )
        # 이동 방향 성분 속도로 판정 (정렬 중 차체가 반대쪽으로 수 mm/s 미끄러지는 것은 구동 아님, 11/5 실측)
        ux, uy = sign * self.settings.direction_x / norm, sign * self.settings.direction_y / norm
        engaged = all(st.vx * ux + st.vy * uy > self.align_detect for st in self.interface.states.values())
        elapsed = self.sim_time() - self.aligning
        if getattr(self, "align_mode", "chassis") == "steering":                     # [김민기 11/24] 조향 관절 기준
            err = 0.0
            for robot, st in self.interface.states.items():
                bx, by = world_to_body(ux, uy, self.initial_yaws[robot])
                target = math.atan2(by, bx)
                if len(st.steer) != 3:
                    err = float("inf"); break
                # 바퀴는 뒤로도 구동하므로 π 차이는 같은 정렬 (오차를 [−π/2, π/2)로)
                err = max(err, max(abs((q - target + math.pi / 2) % math.pi - math.pi / 2) for q in st.steer))
            self.align_error = err
            now = self.sim_time()
            if err < self.align_tol:
                self.aligned_since = now if self.aligned_since is None else self.aligned_since
            else:
                self.aligned_since = None
            engaged = self.aligned_since is not None and now - self.aligned_since >= self.align_hold
        if engaged or elapsed > self.align_timeout:
            self.get_logger().info(
                f"[SWERVE] {'정렬 완료' if engaged else '정렬 시간 초과'} ({elapsed:.2f} s, mode={getattr(self, 'align_mode', 'chassis')}, "
                f"조향 오차 {getattr(self, 'align_error', float('nan')):.4f} rad) → 10초 패턴 시작"
            )
            self.aligning = None
            self.aligned_since = None
            self.lag_since, self.tracking_invalid, self.max_lag = {}, "", 0.0
            if getattr(self, "align_mode", "chassis") == "steering":
                # 패턴 기준점 = 정렬이 끝난 순간 위치 (정렬 중 팔 반력으로 차체가 수 mm 밀린 것을 지령 오차로 보지 않음)
                self.initial_positions = {r: (st.x, st.y) for r, st in self.interface.states.items()}
            self.sequence.start(self.sim_time(), time.monotonic())

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
            align_mode=getattr(self, "align_mode", "chassis"),
            track_feedback=getattr(self, "track_feedback", False),
            tracking_invalid=getattr(self, "tracking_invalid", ""),
            max_lag=getattr(self, "max_lag", 0.0),
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
            if self.aligning is not None and not ready:
                self.aligning = None
                self.sequence.fail(reason)
            self.sequence.tick(
                self.sim_time(),
                now,
                ready,
                self.stopped(),
                self.moved() if self.initial_positions else False,
            )
            if self.aligning is not None and self.sequence.state == "ARMED":
                self.align_tick()
            elif self.sequence.state == "MOVING":
                self.command_motion()
            else:
                self.interface.stop()
            if self.sequence.state == "FAULT" and not ready:
                self.sequence.reason = reason
        except Exception as error:
            self.sequence.fail(f"Base runtime error: {error}")
            self.interface.stop()
        self.report()


def run_base_node(node_type):
    """직선·곡선이 같은 실행 잠금, ROS graph 검사, 종료 시 정지를 사용한다."""
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
            node = node_type()
            while rclpy.ok() and not stopping:
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            if node is not None:
                node.interface.stop()
                node.done_publisher.publish(Bool(data=False))
                node.destroy_node()
            rclpy.try_shutdown()


def main():
    run_base_node(BaseMoveNode)
