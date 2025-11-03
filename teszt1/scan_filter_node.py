import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from .qos import SensorQoS
from .utils import medfilt1d_same

class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self.declare_parameters('', [
            ('fov_deg', 240.0),
            ('decim', 2),
            ('median_win', 5),
            ('deadzone_r', 0.15),
            ('safety_bubble_bins', 3),
            ('range_max_clip', 30.0),
            ('input_scan_topic', '/roboworks/scan'),
            ('output_scan_topic', '/perception/scan_filtered'),
        ])
        self.fov_deg = float(self.get_parameter('fov_deg').value)
        self.decim = int(self.get_parameter('decim').value)
        self.median_win = int(self.get_parameter('median_win').value)
        self.deadzone_r = float(self.get_parameter('deadzone_r').value)
        self.safety_bubble_bins = int(self.get_parameter('safety_bubble_bins').value)
        self.range_max_clip = float(self.get_parameter('range_max_clip').value)
        in_topic = self.get_parameter('input_scan_topic').value
        out_topic = self.get_parameter('output_scan_topic').value

        self._angles = None
        self._decim_idx = None

        self.sub = self.create_subscription(LaserScan, in_topic, self.on_scan, SensorQoS)
        self.pub = self.create_publisher(LaserScan, out_topic, SensorQoS)

    def _prepare_static(self, msg: LaserScan):
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        half = np.deg2rad(self.fov_deg / 2.0)
        fov_mask = (angles >= -half) & (angles <= half)
        decim = max(1, self.decim)
        decim_idx = np.flatnonzero(fov_mask)[::decim]
        if decim_idx.size == 0:
            decim_idx = np.arange(0, n, decim)
        self._angles = angles
        self._decim_idx = decim_idx

    def on_scan(self, msg: LaserScan):
        if self._angles is None:
            self._prepare_static(msg)
        r = np.asarray(msg.ranges, dtype=np.float32)
        r[~np.isfinite(r)] = msg.range_max
        r = np.clip(r, 0.0, min(self.range_max_clip, msg.range_max))
        r = r[self._decim_idx]
        r[r < self.deadzone_r] = 0.0
        if self.median_win > 1:
            r = medfilt1d_same(r, self.median_win).astype(np.float32)

        valid = np.where(r > 0.0, r, np.inf)
        i_min = int(np.argmin(valid))
        a = max(0, i_min - self.safety_bubble_bins)
        b = min(r.size, i_min + self.safety_bubble_bins + 1)
        r[a:b] = 0.0

        out = LaserScan()
        out.header = msg.header
        angs = self._angles[self._decim_idx]
        out.angle_min = float(angs[0])
        out.angle_increment = float(angs[1] - angs[0]) if angs.size > 1 else msg.angle_increment * self.decim
        out.angle_max = out.angle_min + out.angle_increment * (r.size - 1)
        out.time_increment = msg.time_increment * self.decim
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = min(self.range_max_clip, msg.range_max)
        out.ranges = r.tolist()
        out.intensities = []
        self.pub.publish(out)

def main():
    rclpy.init()
    node = ScanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
