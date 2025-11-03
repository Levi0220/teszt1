import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from .qos import SensorQoS, ControlQoS
from .utils import clamp, RateLimiter

class SpeedManagerNode(Node):
    def __init__(self):
        super().__init__('speed_manager')
        self.declare_parameters('', [
            ('scan_topic', '/perception/scan_filtered'),
            ('curvature_topic', '/control/curvature'),
            ('speed_topic', '/control/speed_raw'),
            ('v_max', 3.0),
            ('c_kappa', 0.9),
            ('ttc_threshold', 0.8),
            ('dv_max', 0.5),
            ('rate_hz', 60.0),
        ])
        self.v_max = float(self.get_parameter('v_max').value)
        self.c_kappa = float(self.get_parameter('c_kappa').value)
        self.ttc_thr = float(self.get_parameter('ttc_threshold').value)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.rl = RateLimiter(dv_max=float(self.get_parameter('dv_max').value) / max(1e-3, self.rate))

        self.kappa = 0.0
        self.angles = None
        self.ranges = None

        self.sub_scan = self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.on_scan, SensorQoS)
        self.sub_k = self.create_subscription(Float32, self.get_parameter('curvature_topic').value, self.on_kappa, ControlQoS)
        self.pub_v = self.create_publisher(Float32, self.get_parameter('speed_topic').value, ControlQoS)
        self.timer = self.create_timer(1.0 / self.rate, self.step)

    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if self.angles is None:
            self.angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        self.ranges = np.asarray(msg.ranges, dtype=np.float32)

    def on_kappa(self, m: Float32):
        self.kappa = float(m.data)

    def step(self):
        if self.ranges is None:
            return

        v_curve = self.v_max / (1.0 + self.c_kappa * abs(self.kappa))
        v_cmd = v_curve

        if self.angles is not None:
            front_mask = (np.abs(self.angles) <= np.deg2rad(45.0))
            r = self.ranges[front_mask]
            c = np.cos(self.angles[front_mask])
            proj = np.maximum(1e-2, c)
            if r.size > 0:
                dmin = float(np.min(r / proj))
                if dmin / max(1e-3, v_cmd) < self.ttc_thr:
                    v_cmd = min(v_cmd, dmin / max(1e-3, self.ttc_thr))

        v_cmd = clamp(v_cmd, 0.0, self.v_max)
        v_cmd = self.rl.step(v_cmd)
        self.pub_v.publish(Float32(data=float(v_cmd)))

def main():
    import rclpy
    rclpy.init()
    node = SpeedManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()