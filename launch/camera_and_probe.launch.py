from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('adaptive_object_grasping')) / 'config' / 'hardware.yaml'
    return LaunchDescription([
        Node(
            package='adaptive_object_grasping',
            executable='rgbd_shm_bridge_robot_env.sh',
            name='rgbd_shm_bridge',
            parameters=[str(config)],
            output='screen',
        ),
        Node(
            package='adaptive_object_grasping',
            executable='hardware_probe_node.py',
            name='adaptive_grasp_hardware_probe',
            parameters=[str(config)],
            output='screen',
        ),
    ])
