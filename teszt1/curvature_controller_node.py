import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from .qos import ControlQoS
from .utils import clamp, lpf, RateLimiter


class CurvatureControllerNode(Node):
    def __init__(self):
        super().__init__('curvature_controller')
        self.declare_parameters('', [
            ('target_angle_topic', '/planning/target_angle'),
            ('lookahead_topic', '/planning/lookahead'),
            ('curvature_topic', '/control/curvature'),
            ('omega_raw_topic', '/control/omega_raw'),
            ('v_ref_topic', '/control/speed_raw'),
            ('omega_max', 2.5),
            ('alpha_omega', 0.2),
            ('domega_max', 2.0),
            ('control_rate_hz', 60.0),
        ])
        self.omega_max = float(self.get_parameter('omega_max').value)
        self.alpha = float(self.get_parameter('alpha_omega').value)
        self.rate = float(self.get_parameter('control_rate_hz').value)
        self.rl = RateLimiter(dv_max=float(self.get_parameter('domega_max').value) / max(1e-3, self.rate))
        self.theta = 0.0
        self.L = 1.0
        self.v_ref = 0.0
        self.pub_kappa = self.create_publisher(Float32, self.get_parameter('curvature_topic').value, ControlQoS)
        self.pub_omega = self.create_publisher(Float32, self.get_parameter('omega_raw_topic').value, ControlQoS)
        self.create_subscription(Float32, self.get_parameter('target_angle_topic').value, lambda m: setattr(self, 'theta', float(m.data)), ControlQoS)
        self.create_subscription(Float32, self.get_parameter('lookahead_topic').value, lambda m: setattr(self, 'L', max(0.1, float(m.data))), ControlQoS)
        self.create_subscription(Float32, self.get_parameter('v_ref_topic').value, lambda m: setattr(self, 'v_ref', max(0.0, float(m.data))), ControlQoS)
        self.timer = self.create_timer(1.0 / self.rate, self.step)
        self.omega_prev = 0.0


    def step(self):
        y = self.L * math.sin(self.theta)
        kappa = 2.0 * y / max(1e-3, self.L * self.L)
        omega = self.v_ref * kappa
        omega = clamp(omega, -self.omega_max, self.omega_max)
        omega = lpf(self.omega_prev, omega, self.alpha)
        omega = self.rl.step(omega)
        self.omega_prev = omega
        self.pub_kappa.publish(Float32(data=float(kappa)))
        self.pub_omega.publish(Float32(data=float(omega)))


def main():
    rclpy.init()
    node = CurvatureControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()