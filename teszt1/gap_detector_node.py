import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from .qos import SensorQoS, ControlQoS
from .utils import longest_true_run


class GapDetectorNode(Node):
    def __init__(self):
        super().__init__('gap_detector')
        self.declare_parameters('', [
            ('scan_topic', '/perception/scan_filtered'),
            ('gap_theta_topic', '/perception/gap_center'),
            ('marker_topic', '/perception/gap_marker'),
            ('gap_threshold', 1.8),
            ('min_gap_bins', 5),
        ])
        self.gap_th = float(self.get_parameter('gap_threshold').value)
        self.min_bins = int(self.get_parameter('min_gap_bins').value)
        self.pub_theta = self.create_publisher(Float32, self.get_parameter('gap_theta_topic').value, ControlQoS)
        self.pub_marker = self.create_publisher(Marker, self.get_parameter('marker_topic').value, ControlQoS)
        self.sub = self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.on_scan, SensorQoS)


    def on_scan(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=np.float32)
        free = r > self.gap_th
        start, length = longest_true_run(free)
        if length < self.min_bins:
            i = int(np.argmax(r))
            theta = msg.angle_min + i * msg.angle_increment
            start = i
            end = i
        else:
            i = start + length // 2
            theta = msg.angle_min + i * msg.angle_increment
            end = start + length - 1
        out = Float32(data=float(theta))
        self.pub_theta.publish(out)
        # marker (line strip from start to end at gap_th)
        m = Marker()
        m.header = msg.header
        m.ns = 'gap'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.03
        m.color.a = 1.0
        m.color.g = 1.0
        from geometry_msgs.msg import Point
        for idx in (start, end):
            th = msg.angle_min + idx * msg.angle_increment
            rr = max(self.gap_th, 0.01)
            p = Point()
            p.x = rr * float(np.cos(th))
            p.y = rr * float(np.sin(th))
            p.z = 0.0
            m.points.append(p)
            self.pub_marker.publish(m)


def main():
    rclpy.init()
    node = GapDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()