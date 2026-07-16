from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('adaptive_object_grasping'))
    config = share / 'config'
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'perception.launch.py'))
        ),
        Node(
            package='adaptive_object_grasping',
            executable='pbvs_servo_node.py',
            name='pbvs_target_servo',
            parameters=[str(config / 'pbvs.yaml')],
            output='screen',
        ),
        Node(
            package='adaptive_object_grasping',
            executable='graspnet_server_venv.sh',
            name='graspnet_server',
            parameters=[str(config / 'graspnet.yaml')],
            output='screen',
        ),
        Node(
            package='adaptive_object_grasping',
            executable='motion_executor_robot_env.sh',
            name='adaptive_grasp_motion_executor',
            parameters=[str(config / 'motion.yaml')],
            output='screen',
        ),
        Node(
            package='adaptive_object_grasping',
            executable='grasp_coordinator_node.py',
            name='adaptive_grasp_coordinator',
            parameters=[str(config / 'coordinator.yaml')],
            output='screen',
        ),
    ])
