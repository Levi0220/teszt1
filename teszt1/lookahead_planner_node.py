import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from .qos import ControlQoS
from .utils import lpf


class LookaheadPlannerNode(Node):
    def __init__(self):
        super().__init__('lookahead_planner')
        self.declare_parameters('', [
            ('gap_theta_topic', '/perception/gap_center'),
            ('target_angle_topic', '/planning/target_angle'),
            ('lookahead_topic', '/planning/lookahead'),
            ('speed_feedback_topic', '/control/speed_raw'),
            ('L_min', 0.6),
            ('kL', 0.3),
            ('alpha_v_est', 0.2),
        ])
        self.L_min = float(self.get_parameter('L_min').value)
        self.kL = float(self.get_parameter('kL').value)
        self.alpha_v = float(self.get_parameter('alpha_v_est').value)
        self.v_est = 0.0
        self.pub_target = self.create_publisher(Float32, self.get_parameter('target_angle_topic').value, ControlQoS)
        self.pub_L = self.create_publisher(Float32, self.get_parameter('lookahead_topic').value, ControlQoS)
        self.create_subscription(Float32, self.get_parameter('gap_theta_topic').value, self.on_theta, ControlQoS)
        self.create_subscription(Float32, self.get_parameter('speed_feedback_topic').value, self.on_v, ControlQoS)


    def on_v(self, msg: Float32):
        self.v_est = lpf(self.v_est, float(msg.data), self.alpha_v)


    def on_theta(self, msg: Float32):
        theta = float(msg.data)
        L = self.L_min + self.kL * max(0.0, self.v_est)
        self.pub_L.publish(Float32(data=float(L)))
        self.pub_target.publish(Float32(data=theta))


def main():
    rclpy.init()
    node = LookaheadPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()