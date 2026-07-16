from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('adaptive_object_grasping'))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'camera_and_probe.launch.py'))
        ),
        Node(
            package='adaptive_object_grasping',
            executable='yolo_tracker_venv.sh',
            name='yolo_object_tracker',
            parameters=[str(share / 'config' / 'perception.yaml')],
            output='screen',
        ),
    ])
