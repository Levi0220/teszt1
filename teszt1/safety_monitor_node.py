import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from .qos import SensorQoS, ControlQoS
from .utils import lpf

class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        self.declare_parameters('', [
            ('scan_topic', '/perception/scan_filtered'),
            ('omega_raw_topic', '/control/omega_raw'),
            ('speed_raw_topic', '/control/speed_raw'),
            ('cmd_vel_topic', '/control/cmd_vel_safe'),
            ('r_stop', 0.25),
            ('watchdog_ms', 150),
            ('alpha_final', 0.2),
            ('rate_hz', 60.0),
        ])
        self.r_stop = float(self.get_parameter('r_stop').value)
        self.alpha_final = float(self.get_parameter('alpha_final').value)
        self.watchdog_ms = int(self.get_parameter('watchdog_ms').value)
        self.rate = float(self.get_parameter('rate_hz').value)

        self.v = 0.0
        self.w = 0.0
        self.v_f = 0.0
        self.w_f = 0.0
        self.min_range = float('inf')
        self.last_scan_time = self.get_clock().now()

        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.on_scan, SensorQoS)
        self.create_subscription(Float32, self.get_parameter('omega_raw_topic').value, lambda m: setattr(self, 'w', float(m.data)), ControlQoS)
        self.create_subscription(Float32, self.get_parameter('speed_raw_topic').value, lambda m: setattr(self, 'v', float(m.data)), ControlQoS)
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, ControlQoS)
        self.timer = self.create_timer(1.0 / self.rate, self.step)

    def on_scan(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=np.float32)
        self.min_range = float(np.min(np.where(r > 0.0, r, np.inf)))
        self.last_scan_time = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        staleness_ms = (now - self.last_scan_time).nanoseconds * 1e-6

        v_cmd = self.v
        w_cmd = self.w
        if staleness_ms > self.watchdog_ms or self.min_range < self.r_stop:
            v_cmd = 0.0
            w_cmd = 0.0

        self.v_f = lpf(self.v_f, v_cmd, self.alpha_final)
        self.w_f = lpf(self.w_f, w_cmd, self.alpha_final)

        t = Twist()
        t.linear.x = float(self.v_f)
        t.angular.z = float(self.w_f)
        self.pub.publish(t)

def main():
    rclpy.init()
    node = SafetyMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
