from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('adaptive_object_grasping'))
    config = share / 'config'
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz with the grasp visualization displays.',
        ),
        Node(
            package='adaptive_object_grasping',
            executable='grasp_visualizer_node.py',
            name='adaptive_grasp_visualizer',
            parameters=[str(config / 'visualization.yaml')],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='adaptive_grasp_rviz',
            arguments=['-d', str(config / 'grasp_visualization.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
