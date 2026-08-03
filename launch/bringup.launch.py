from pathlib import Path
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('adaptive_object_grasping'))
    config = share / 'config'
    moveit_config = config / 'moveit'
    robot_description = (moveit_config / 'robot_v2_2.urdf').read_text().replace(
        '../meshes/robot_v2_2/',
        'package://adaptive_object_grasping/meshes/robot_v2_2/',
    )
    target_robot_description = re.sub(
        r'rgba="[^"]+"',
        'rgba="0.0 0.95 1.0 1.0"',
        robot_description,
    )
    robot_description_semantic = (moveit_config / 'autolife_s2.srdf').read_text()
    robot_description_kinematics = yaml.safe_load((moveit_config / 'kinematics.yaml').read_text())
    moveit_planning = {
        'planning_pipelines': {'pipeline_names': ['ompl']},
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
            ],
            'start_state_max_bounds_error': 0.1,
        },
    }
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
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='adaptive_grasp_robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='adaptive_grasp_target_base_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'Link_Zero_Point',
                '--child-frame-id', 'grasp_target/Link_Ground_Vehicle_X',
            ],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='adaptive_grasp_target_robot_state_publisher',
            parameters=[{
                'robot_description': target_robot_description,
                'frame_prefix': 'grasp_target/',
                'publish_frequency': 15.0,
            }],
            remappings=[
                ('joint_states', '/adaptive_grasp/target_joint_states'),
                ('robot_description', '/adaptive_grasp/target_robot_description'),
            ],
            output='screen',
        ),
        Node(
            package='adaptive_object_grasping',
            executable='motion_executor_robot_env.sh',
            name='adaptive_grasp_motion_executor',
            parameters=[
                str(config / 'motion.yaml'),
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                {'robot_description_kinematics': robot_description_kinematics},
                moveit_planning,
            ],
            output='screen',
        ),
        Node(
            package='adaptive_object_grasping',
            executable='grasp_coordinator_node.py',
            name='adaptive_grasp_coordinator',
            parameters=[str(config / 'coordinator.yaml')],
            output='screen',
        ),
        Node(
            package='adaptive_object_grasping',
            executable='grasp_visualizer_node.py',
            name='adaptive_grasp_visualizer',
            parameters=[str(config / 'visualization.yaml')],
            output='screen',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with the grasp visualization displays.',
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
