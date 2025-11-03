import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from .qos import ControlQoS

class VizNode(Node):
    def __init__(self):
        super().__init__('viz_node')
        self.declare_parameters('', [
            ('target_angle_topic', '/planning/target_angle'),
            ('lookahead_topic', '/planning/lookahead'),
            ('marker_topic', '/viz/lookahead_marker'),
        ])
        self.theta = 0.0
        self.L = 1.0
        self.pub = self.create_publisher(Marker, self.get_parameter('marker_topic').value, ControlQoS)
        self.create_subscription(Float32, self.get_parameter('target_angle_topic').value, self.on_theta, ControlQoS)
        self.create_subscription(Float32, self.get_parameter('lookahead_topic').value, self.on_L, ControlQoS)
        self.timer = self.create_timer(1.0/20.0, self.step)

    def on_theta(self, m: Float32):
        self.theta = float(m.data)

    def on_L(self, m: Float32):
        self.L = float(m.data)

    def step(self):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.ns = 'lookahead'
        m.id = 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 0.1
        m.color.a = 1.0
        m.color.r = 1.0
        m.pose.position.x = self.L * math.cos(self.theta)
        m.pose.position.y = self.L * math.sin(self.theta)
        m.pose.position.z = 0.0
        self.pub.publish(m)

def main():
    rclpy.init()
    node = VizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
