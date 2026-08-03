#!/usr/bin/env python3
"""Qwen-RobotNav / RoboStral Navigate ROS 2 node.

Subscribes to the head camera RGB topic, accepts navigation instructions via
a service, and publishes waypoint goal poses.

Supports two backends, selected via the ``model_type`` parameter:
  - ``qwen_robotnav`` (default):  Qwen3-VL + MLP head → 8 waypoints
  - ``robostral``:                Mistral RoboStral Navigate → VIEW/MOVE/DONE
"""

import os
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from adaptive_object_grasping_nodes.navigation_core import (
    NavigationCommand,
    ObservationConfig,
    QwenRobotNavAdapter,
    RobostralNavigateAdapter,
    TaskMode,
    displacement_to_pose_stamped,
    pixel_to_world_displacement,
    waypoints_to_command,
)
from adaptive_object_grasping_nodes.perception_core import (
    CameraIntrinsics,
    image_buffer_to_array,
)
from adaptive_object_grasping_nodes.robot_geometry_core import quaternion_matrix


class NavigateServerNode(Node):
    """ROS 2 node wrapping a navigation model backend."""

    def __init__(self):
        super().__init__('robostral_navigate_server')

        # ---- Parameters ----
        self.declare_parameter('model_type', 'qwen_robotnav')
        self.declare_parameter('color_topic', '/head_camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/head_camera/color/camera_info')
        self.declare_parameter('base_frame', 'Link_Zero_Point')
        self.declare_parameter('model_path', '')
        self.declare_parameter('model_size', '4B')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('default_task_mode', 'vln')
        self.declare_parameter('obs_token_budget', 3584)
        self.declare_parameter('obs_temporal_decay', 2.0)
        self.declare_parameter('obs_frame_sample_mode', 'random')
        self.declare_parameter('obs_camera_weights', {'Front View': 1.0})
        self.declare_parameter('api_url', '')
        self.declare_parameter('api_key', '')
        self.declare_parameter('vl_model', 'qwen3-vl-8b-instruct')
        self.declare_parameter('llm_model', 'qwen3.5-35b-a3b')
        self.declare_parameter('maximum_steps', 80)
        self.declare_parameter('task_timeout', 120.0)
        self.declare_parameter('minimum_step_period', 0.1)
        self.declare_parameter('camera_ground_distance_m', 1.25)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('publish_target_pose', True)
        self.declare_parameter('goal_frame', 'grasp_nav_target')

        self._lock = threading.Lock()
        self._color = None
        self._intrinsics = None
        self._step_count = 0

        model_type = str(self.get_parameter('model_type').value)

        # ---- Model backend ----
        if model_type == 'qwen_robotnav':
            task_mode_str = str(self.get_parameter('default_task_mode').value)
            try:
                default_mode = TaskMode(task_mode_str)
            except ValueError:
                self.get_logger().warn(
                    f'unknown task_mode={task_mode_str}, falling back to vln'
                )
                default_mode = TaskMode.VLN
            api_url = str(self.get_parameter('api_url').value) or os.getenv(
                'GPUSTACK_API_URL', ''
            )
            api_key = str(self.get_parameter('api_key').value) or os.getenv(
                'GPUSTACK_API_KEY', ''
            )
            self._model = QwenRobotNavAdapter(
                model_path=str(self.get_parameter('model_path').value),
                model_size=str(self.get_parameter('model_size').value),
                device=str(self.get_parameter('device').value),
                image_size=(
                    int(self.get_parameter('image_width').value),
                    int(self.get_parameter('image_height').value),
                ),
                default_task_mode=default_mode,
                api_url=api_url,
                api_key=api_key,
            )
            self._default_obs_config = ObservationConfig(
                token_budget=int(self.get_parameter('obs_token_budget').value),
                temporal_decay=float(self.get_parameter('obs_temporal_decay').value),
                frame_sample_mode=str(
                    self.get_parameter('obs_frame_sample_mode').value
                ),
                camera_weights={
                    k: float(v) for k, v in
                    self.get_parameter('obs_camera_weights').value.items()
                } if self.get_parameter('obs_camera_weights').value else {"Front View": 1.0},
            )
        elif model_type == 'robostral':
            self._model = RobostralNavigateAdapter(
                model_path=str(self.get_parameter('model_path').value),
                device=str(self.get_parameter('device').value),
                image_size=(
                    int(self.get_parameter('image_width').value),
                    int(self.get_parameter('image_height').value),
                ),
            )
            self._default_obs_config = None
        else:
            raise ValueError(f'unknown model_type: {model_type!r}')

        # ---- Camera subscriptions ----
        self.create_subscription(
            Image,
            str(self.get_parameter('color_topic').value),
            self._on_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_info,
            qos_profile_sensor_data,
        )

        # ---- Output publishers ----
        self._goal_pub = self.create_publisher(PoseStamped, 'navigate_goal_pose', 10)
        self._status_pub = self.create_publisher(String, 'navigate_status', 10)

        # ---- Services ----
        self.create_service(Trigger, 'navigate_start', self._on_start)
        self.create_service(Trigger, 'navigate_stop', self._on_stop)

        if model_type == 'qwen_robotnav' and api_url:
            path_info = '(GPUStack API backend)'
        elif str(self.get_parameter('model_path').value):
            path_info = '(local model backend)'
        else:
            path_info = '(placeholder mode – no API URL or model_path configured)'
        self.get_logger().info(
            f'Navigate server ready: backend={model_type} {path_info}'
        )

    # ------------------------------------------------------------------
    # Camera callbacks
    # ------------------------------------------------------------------

    def _on_color(self, message):
        try:
            array = image_buffer_to_array(message)
            with self._lock:
                self._color = array
        except Exception as exc:
            self.get_logger().warning(f'color frame rejected: {exc}')

    def _on_info(self, message):
        intrinsics = CameraIntrinsics(
            float(message.k[0]), float(message.k[4]),
            float(message.k[2]), float(message.k[5]),
        )
        with self._lock:
            self._intrinsics = intrinsics

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _on_start(self, _request, response):
        response.success = True
        response.message = 'navigation start acknowledged (placeholder)'
        return response

    def _on_stop(self, _request, response):
        self._model.reset()
        self._step_count = 0
        response.success = True
        response.message = 'navigation reset'
        return response

    # ------------------------------------------------------------------
    # Inference step
    # ------------------------------------------------------------------

    def step(self, instruction: str, task_mode_str: str = "") -> NavigationCommand:
        """Run one inference step and return a unified NavigationCommand."""
        with self._lock:
            color = None if self._color is None else self._color.copy()
            intrinsics = self._intrinsics
        if color is None:
            raise RuntimeError('no RGB frame available for navigation inference')
        self._step_count += 1

        if isinstance(self._model, QwenRobotNavAdapter):
            task_mode = TaskMode(task_mode_str) if task_mode_str else self._model._default_task_mode
            waypoints = self._model.predict(
                color, instruction,
                task_mode=task_mode,
                obs_config=self._default_obs_config,
            )
            command = waypoints_to_command(waypoints)
        else:
            command = self._model.predict(color, instruction)

        if bool(self.get_parameter('publish_target_pose').value):
            self._publish_goal(command, intrinsics)
        return command

    def _publish_goal(self, command, intrinsics):
        if command.image_space:
            if intrinsics is None:
                self.get_logger().warning('no camera intrinsics; cannot project goal')
                return
            camera_to_base = quaternion_matrix([0.0, 0.0, 0.0, 1.0])
            try:
                dx, dy = pixel_to_world_displacement(
                    command.u, command.v,
                    intrinsics.fx, intrinsics.fy,
                    intrinsics.cx, intrinsics.cy,
                    float(self.get_parameter('camera_ground_distance_m').value),
                    camera_to_base,
                )
            except Exception as exc:
                self.get_logger().warning(f'pixel projection failed: {exc}')
                return
        else:
            dx, dy = command.dx_m, command.dy_m

        goal_frame = str(self.get_parameter('goal_frame').value)
        pose = displacement_to_pose_stamped(dx, dy, command.yaw_rad, goal_frame)
        pose.header.stamp = self.get_clock().now().to_msg()
        self._goal_pub.publish(pose)
        self._status_pub.publish(String(data=(
            f'model={self.get_parameter("model_type").value} '
            f'step={self._step_count} done={command.done} '
            f'dx={dx:.3f} dy={dy:.3f} yaw={command.yaw_rad:.3f}'
            f'{" image_space" if command.image_space else ""}'
        )))


def main(args=None):
    rclpy.init(args=args)
    node = NavigateServerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
