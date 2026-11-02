"""Convert named hardware commands into the generated Gazebo actuator order."""
import math
import signal
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from actuator_msgs.msg import Actuators


class Adapter(Node):
    def __init__(self):
        super().__init__('sim_command_adapter')
        robot = self.declare_parameter('robot', 'robot1').value
        if robot not in ('robot1', 'robot2'): raise ValueError('Invalid robot')
        path = self.declare_parameter('joint_map', '').value
        self.names = yaml.safe_load(open(path))['commanded_joints']
        self.pub = self.create_publisher(Actuators, f'/sim/{robot}/actuators', 1)
        self.sub = self.create_subscription(JointState, f'/sim/{robot}/joint_commands', self.command, 1)

    def command(self, msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        data = dict(zip(msg.name, msg.position))
        if set(data) != set(self.names) or not all(math.isfinite(v) for v in data.values()):
            return
        output = Actuators()
        output.header = msg.header
        output.position = [data[name] for name in self.names]
        self.pub.publish(output)


def main():
    # Keep ROS context cleanup in this thread; native signal shutdown can race
    # with finally when both the terminal and launch deliver SIGINT.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = Adapter()
    try:
        while not stop_requested and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
