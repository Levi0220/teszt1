from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('teszt1')
    params = os.path.join(pkg, 'params', 'teszt1.yaml')
    return LaunchDescription([
        Node(package='teszt1', executable='scan_filter', name='scan_filter', parameters=[params]),
        Node(package='teszt1', executable='gap_detector', name='gap_detector', parameters=[params]),
        Node(package='teszt1', executable='lookahead_planner', name='lookahead_planner', parameters=[params]),
        Node(package='teszt1', executable='curvature_controller', name='curvature_controller', parameters=[params]),
        Node(package='teszt1', executable='speed_manager', name='speed_manager', parameters=[params]),
        Node(package='teszt1', executable='safety_monitor', name='safety_monitor', parameters=[params]),
        Node(package='teszt1', executable='cmd_mux', name='cmd_mux', parameters=[params]),
        Node(package='teszt1', executable='viz', name='viz', parameters=[params]),
    ])