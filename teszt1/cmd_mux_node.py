import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .qos import ControlQoS


class CmdMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_mux')
        self.declare_parameters('', [
            ('autonomous_topic', '/control/cmd_vel_safe'),
            ('teleop_topic', '/teleop/cmd_vel'),
            ('output_topic', '/roboworks/cmd_vel'),
            ('priority', 'autonomous'),
        ])
        self.priority = str(self.get_parameter('priority').value)
        self.pub = self.create_publisher(Twist, self.get_parameter('output_topic').value, ControlQoS)
        self.autonomous = Twist()
        self.teleop = Twist()
        self.create_subscription(Twist, self.get_parameter('autonomous_topic').value, self.on_auto, ControlQoS)
        self.create_subscription(Twist, self.get_parameter('teleop_topic').value, self.on_teleop, ControlQoS)
        self.timer = self.create_timer(1.0/60.0, self.step)


    def on_auto(self, msg: Twist):
        self.autonomous = msg
    def on_teleop(self, msg: Twist):
        self.teleop = msg
    def step(self):
        out = self.autonomous if self.priority == 'autonomous' else self.teleop
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdMuxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()