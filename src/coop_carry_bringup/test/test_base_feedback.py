"""잘못된 피드백을 정상적인 정지 측정으로 취급하지 않는지 검증한다."""

from types import SimpleNamespace
import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from coop_carry_bringup.base_interface import BaseFeedback, SwerveBaseInterface
from coop_carry_bringup.base_sequence import MoveSettings
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
