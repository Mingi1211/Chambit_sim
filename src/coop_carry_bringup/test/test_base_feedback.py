"""잘못된 피드백을 정상적인 정지 측정으로 취급하지 않는지 검증한다."""

from types import SimpleNamespace
import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from coop_carry_bringup.base_interface import BaseFeedback, SwerveBaseInterface
from coop_carry_bringup import base_interface
from coop_carry_bringup.base_move_node import BaseMoveNode
from coop_carry_bringup.base_sequence import BaseSequence, MoveSettings
from coop_carry_bringup.swerve_model import BASE_JOINTS


def receiver():
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    return SimpleNamespace(
        states={"robot1": BaseFeedback()},
        settings=MoveSettings(),
        node=SimpleNamespace(get_clock=lambda: clock),
    )


@pytest.mark.parametrize(
    "defect", ["old", "future", "nan", "frame", "quaternion", "child"]
)
def test_bad_chassis_feedback_is_not_fresh(defect):
    interface = receiver()
    message = Odometry()
    message.header.stamp.sec = 10
    message.header.frame_id = "world"
    message.child_frame_id = "robot1/base_link"
    message.pose.pose.orientation.w = 1.0
    if defect == "old":
        message.header.stamp.sec = 8
    if defect == "future":
        message.header.stamp.sec = 11
    if defect == "nan":
        message.twist.twist.linear.x = float("nan")
    if defect == "frame":
        message.header.frame_id = "odom"
    if defect == "child":
        message.child_frame_id = "robot2/base_link"
    if defect == "quaternion":
        message.pose.pose.orientation.w = 0.0
    SwerveBaseInterface.on_pose(interface, "robot1", message)
    assert interface.states["robot1"].received == 0.0


def test_chassis_speed_uses_measured_body_and_rejects_repeated_stamp():
    interface = receiver()
    message = Odometry()
    message.header.stamp.sec = 10
    message.header.frame_id = "world"
    message.child_frame_id = "robot1/base_link"
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x = 0.03
    message.twist.twist.linear.y = 0.04
    SwerveBaseInterface.on_pose(interface, "robot1", message)
    assert interface.states["robot1"].linear_speed == pytest.approx(0.05)
    received = interface.states["robot1"].received
    message.twist.twist.linear.x = 0.0
    SwerveBaseInterface.on_pose(interface, "robot1", message)
    assert interface.states["robot1"].received == received
    assert interface.states["robot1"].linear_speed == pytest.approx(0.05)


def test_feedback_loss_during_alignment_stops_without_starting_pattern():
    actions = []
    sequence = BaseSequence(MoveSettings())
    sequence.on_lift(True)
    node = SimpleNamespace(
        interface=SimpleNamespace(
            update=lambda dt: None, stop=lambda: actions.append("stop")
        ),
        sequence=sequence,
        last_tick=0.0,
        aligning=0.0,
        ready=lambda: (False, "feedback reception timeout"),
        sim_time=lambda: 10.0,
        stopped=lambda: True,
        initial_positions={},
        align_tick=lambda: actions.append("align"),
        report=lambda: None,
    )
    BaseMoveNode.tick(node)
    assert node.aligning is None
    assert sequence.state == "FAULT"
    assert sequence.reason == "feedback reception timeout"
    assert actions == ["stop"]


@pytest.mark.parametrize(
    "defect", ["missing_velocity", "missing_joint", "duplicate", "nan"]
)
def test_bad_wheel_feedback_cannot_appear_stopped(defect):
    interface = receiver()
    message = JointState()
    message.header.stamp.sec = 10
    message.name = list(BASE_JOINTS)
    message.position = [0.0] * 6
    message.velocity = [0.0] * 6
    if defect == "missing_velocity":
        message.velocity = []
    if defect == "missing_joint":
        message.name[-1] = "unknown"
    if defect == "duplicate":
        message.name[-1] = message.name[0]
    if defect == "nan":
        message.velocity[0] = float("nan")
    SwerveBaseInterface.on_joints(interface, "robot1", message)
    assert interface.states["robot1"].joint_received == 0.0


def ready_interface(monkeypatch):
    monkeypatch.setattr(base_interface.time, "monotonic", lambda: 100.0)
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    interface = SimpleNamespace(
        settings=MoveSettings(),
        states={
            robot: BaseFeedback(
                received=99.3, joint_received=99.9, stamp=9.98, joint_stamp=10.0
            )
            for robot in ("robot1", "robot2")
        },
        node=SimpleNamespace(
            get_clock=lambda: clock,
            get_name=lambda: "carry_base_move",
            get_publishers_info_by_topic=lambda topic: [
                SimpleNamespace(node_name="carry_base_move")
            ],
        ),
        query_times={
            (robot, kind): 99.9
            for robot in ("robot1", "robot2")
            for kind in ("controllers", "hardware")
        },
        controllers={
            robot: {
                "swerve_drive_controller": SimpleNamespace(
                    state="active", is_async=False
                )
            }
            for robot in ("robot1", "robot2")
        },
        hardware={
            robot: {
                f"{robot}_remote_swerve": SimpleNamespace(
                    state=SimpleNamespace(label="active"),
                    plugin_name="coop_carry_hardware/RemoteSwerveSystem",
                )
            }
            for robot in ("robot1", "robot2")
        },
        publishers={
            robot: SimpleNamespace(get_subscription_count=lambda: 1)
            for robot in ("robot1", "robot2")
        },
    )
    return interface


def test_slow_odometry_wall_interval_is_not_a_disconnection(monkeypatch):
    interface = ready_interface(monkeypatch)
    # 0.7초 전에 수신했지만 시뮬레이션에서는 0.02초밖에 지나지 않은 데이터.
    assert SwerveBaseInterface.ready(interface)[0]


@pytest.mark.parametrize("field", ["received", "joint_received"])
def test_real_reception_loss_still_rejects_start(monkeypatch, field):
    interface = ready_interface(monkeypatch)
    setattr(interface.states["robot1"], field, 97.9)
    ready, reason = SwerveBaseInterface.ready(interface)
    assert not ready
    assert "wall clock" in reason


@pytest.mark.parametrize(
    "field,value", [("stamp", 9.4), ("joint_stamp", 9.4), ("stamp", 10.2)]
)
def test_recent_arrival_does_not_hide_bad_source_time(monkeypatch, field, value):
    interface = ready_interface(monkeypatch)
    setattr(interface.states["robot1"], field, value)
    ready, reason = SwerveBaseInterface.ready(interface)
    assert not ready
    assert "simulation clock" in reason
