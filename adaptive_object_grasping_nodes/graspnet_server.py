import importlib
import importlib.metadata
import os
import sys
import threading
import types

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from adaptive_object_grasping.msg import GraspCandidate, GraspCandidateArray
from adaptive_object_grasping.srv import EstimateGrasps
from adaptive_object_grasping_nodes.graspnet_core import (
    apply_grasp_depth_offset,
    is_horizontal_grasp,
    matrix_to_quaternion,
    parse_grasp_array,
    sample_point_cloud,
    target_point_cloud,
    transform_grasp_pose,
)
from adaptive_object_grasping_nodes.perception_core import CameraIntrinsics, image_buffer_to_array


def load_grasp_group_type():
    """Load GraspGroup without importing graspnetAPI's optional evaluation stack."""
    package_name = 'graspnetAPI'
    if package_name not in sys.modules:
        package_root = importlib.metadata.distribution(package_name).locate_file(package_name)
        package = types.ModuleType(package_name)
        package.__path__ = [str(package_root)]
        package.__package__ = package_name
        sys.modules[package_name] = package
    grasp_group_type = importlib.import_module(f'{package_name}.grasp').GraspGroup
    setattr(sys.modules[package_name], 'GraspGroup', grasp_group_type)
    return grasp_group_type


class OfficialGraspNetBackend:
    """Thin adapter around a separately installed official graspnet-baseline checkout."""

    def __init__(self, root, checkpoint, device, number_points, number_views):
        if not os.path.isdir(root):
            raise FileNotFoundError(f'GraspNet root does not exist: {root}')
        if not os.path.isfile(checkpoint):
            raise FileNotFoundError(f'GraspNet checkpoint does not exist: {checkpoint}')
        for child in ('models', 'dataset', 'utils'):
            path = os.path.join(root, child)
            if path not in sys.path:
                sys.path.insert(0, path)
        import torch
        from graspnet import GraspNet, pred_decode

        self._torch = torch
        self._pred_decode = pred_decode
        self._grasp_group_type = load_grasp_group_type()
        self._device = torch.device(device)
        self._number_points = int(number_points)
        self._net = GraspNet(
            input_feature_dim=0,
            num_view=int(number_views),
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        ).to(self._device)
        checkpoint_data = torch.load(checkpoint, map_location=self._device, weights_only=False)
        self._net.load_state_dict(checkpoint_data['model_state_dict'])
        self._net.eval()

    @property
    def number_points(self):
        return self._number_points

    def infer(self, sampled_points, scene_points, collision_threshold, voxel_size, maximum):
        tensor = self._torch.from_numpy(sampled_points[np.newaxis].astype(np.float32)).to(
            self._device
        )
        end_points = {'point_clouds': tensor}
        with self._torch.no_grad():
            predictions = self._pred_decode(self._net(end_points))
        group = self._grasp_group_type(predictions[0].detach().cpu().numpy())
        if collision_threshold > 0.0:
            from collision_detector import ModelFreeCollisionDetector

            detector = ModelFreeCollisionDetector(scene_points, voxel_size=float(voxel_size))
            collisions = detector.detect(
                group,
                approach_dist=0.05,
                collision_thresh=float(collision_threshold),
            )
            group = group[~collisions]
        group.nms()
        group.sort_by_score()
        return np.asarray(group.grasp_group_array[: int(maximum)])


class GraspNetServer(Node):
    def __init__(self):
        super().__init__('graspnet_server')
        self.declare_parameter('color_topic', '/head_camera/color/image_raw')
        self.declare_parameter('depth_topic', '/head_camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/head_camera/color/camera_info')
        self.declare_parameter('selected_mask_topic', '/selected_object_mask')
        self.declare_parameter('use_selected_mask', False)
        self.declare_parameter('base_frame', 'Link_Zero_Point')
        self.declare_parameter('graspnet_root', 'third_party/graspnet-baseline')
        self.declare_parameter('checkpoint_path', 'models/graspnet/checkpoint-rs.tar')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('number_points', 20000)
        self.declare_parameter('number_views', 300)
        self.declare_parameter('minimum_target_points', 300)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('target_depth_band', 0.08)
        self.declare_parameter('collision_threshold', 0.01)
        self.declare_parameter('auto_arm_deadband_y', 0.04)
        self.declare_parameter('prevent_cross_body_grasps', True)
        self.declare_parameter('collision_voxel_size', 0.01)
        self.declare_parameter('minimum_grasp_score', 0.25)
        self.declare_parameter('maximum_gripper_width', 0.10)
        self.declare_parameter('maximum_candidates', 20)
        self.declare_parameter('maximum_raw_candidates', 100)
        self.declare_parameter('inference_attempts', 3)
        self.declare_parameter('pregrasp_offset', 0.12)
        self.declare_parameter('approach_axis', 0)
        self.declare_parameter('horizontal_grasp_filter_enabled', True)
        self.declare_parameter('closing_axis', 1)
        self.declare_parameter('vertical_axis', 2)
        self.declare_parameter('maximum_approach_tilt_degrees', 15.0)
        self.declare_parameter('maximum_closing_tilt_degrees', 15.0)
        self.declare_parameter('apply_grasp_depth', True)
        self.declare_parameter('grasp_depth_scale', 1.0)
        self.declare_parameter('maximum_grasp_depth_offset', 0.05)
        self.declare_parameter('tf_timeout', 0.2)

        self._data_lock = threading.Lock()
        self._inference_lock = threading.Lock()
        self._color = None
        self._depth = None
        self._intrinsics = None
        self._selected_mask = None
        self._backend = None
        self._backend_error = ''
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._candidates_pub = self.create_publisher(GraspCandidateArray, 'grasp_candidates', 10)
        self.create_subscription(
            Image, str(self.get_parameter('color_topic').value), self._on_color, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value), self._on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('selected_mask_topic').value),
            self._on_selected_mask,
            qos_profile_sensor_data,
        )
        self.create_service(EstimateGrasps, 'estimate_grasps', self._estimate)
        self.get_logger().info('GraspNet service ready; model loads on first request')

    def _on_color(self, message):
        try:
            array = image_buffer_to_array(message)
            with self._data_lock:
                self._color = array
        except Exception as exc:
            self.get_logger().warning(f'color frame rejected: {exc}')

    def _on_depth(self, message):
        try:
            array = image_buffer_to_array(message)
            with self._data_lock:
                self._depth = array
        except Exception as exc:
            self.get_logger().warning(f'depth frame rejected: {exc}')

    def _on_info(self, message):
        intrinsics = CameraIntrinsics(message.k[0], message.k[4], message.k[2], message.k[5])
        with self._data_lock:
            self._intrinsics = intrinsics

    def _on_selected_mask(self, message):
        try:
            mask = image_buffer_to_array(message) > 0
            with self._data_lock:
                self._selected_mask = mask
        except Exception as exc:
            self.get_logger().warning(f'selected mask rejected: {exc}')

    def _load_backend(self):
        if self._backend is not None:
            return self._backend
        self._backend = OfficialGraspNetBackend(
            str(self.get_parameter('graspnet_root').value),
            str(self.get_parameter('checkpoint_path').value),
            str(self.get_parameter('device').value),
            int(self.get_parameter('number_points').value),
            int(self.get_parameter('number_views').value),
        )
        return self._backend

    def _estimate(self, request, response):
        if not request.target.depth_valid:
            response.message = 'target has no valid aligned depth'
            return response
        with self._data_lock:
            color = None if self._color is None else self._color.copy()
            depth = None if self._depth is None else self._depth.copy()
            intrinsics = self._intrinsics
            selected_mask = (
                None
                if not bool(self.get_parameter('use_selected_mask').value) or self._selected_mask is None
                else self._selected_mask.copy()
            )
        if color is None or depth is None or intrinsics is None:
            response.message = 'RGB-D frame or camera intrinsics unavailable'
            return response
        box = [
            request.target.bbox_x,
            request.target.bbox_y,
            request.target.bbox_x + request.target.bbox_width,
            request.target.bbox_y + request.target.bbox_height,
        ]
        points, colors = target_point_cloud(
            depth,
            color,
            box,
            intrinsics,
            target_depth=request.target.median_depth,
            mask=selected_mask,
            depth_scale=float(self.get_parameter('depth_scale').value),
            depth_band=float(self.get_parameter('target_depth_band').value),
        )
        minimum = int(self.get_parameter('minimum_target_points').value)
        if len(points) < minimum:
            response.message = f'target point cloud has {len(points)} points; need {minimum}'
            return response
        scene_points, _ = target_point_cloud(
            depth,
            color,
            [0, 0, depth.shape[1], depth.shape[0]],
            intrinsics,
            target_depth=0.0,
            depth_scale=float(self.get_parameter('depth_scale').value),
        )
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter('base_frame').value),
                request.target.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=float(self.get_parameter('tf_timeout').value)),
            )
            with self._inference_lock:
                backend = self._load_backend()
                rows = np.empty((0, 17), dtype=np.float32)
                for _ in range(max(1, int(self.get_parameter('inference_attempts').value))):
                    sampled_points, _ = sample_point_cloud(points, colors, backend.number_points)
                    rows = backend.infer(
                        sampled_points,
                        scene_points,
                        float(self.get_parameter('collision_threshold').value),
                        float(self.get_parameter('collision_voxel_size').value),
                        int(self.get_parameter('maximum_raw_candidates').value),
                    )
                    if len(rows):
                        break
            candidates = self._make_candidates(rows, request, transform)
        except TransformException as exc:
            response.message = f'camera-to-base TF unavailable: {exc}'
            return response
        except Exception as exc:
            response.message = f'GraspNet inference unavailable: {exc}'
            self.get_logger().error(response.message)
            return response
        response.candidates = candidates
        response.success = bool(candidates)
        response.message = (
            f'generated {len(candidates)} grasp candidates from {len(rows)} raw rows '
            f'(minimum_score={float(self.get_parameter("minimum_grasp_score").value):.3f}, '
            f'maximum_width={float(self.get_parameter("maximum_gripper_width").value):.3f}m, '
            f'collision_threshold={float(self.get_parameter("collision_threshold").value):.3f}, '
            f'horizontal_filter='
            f'{bool(self.get_parameter("horizontal_grasp_filter_enabled").value)})'
        )
        self._publish_candidates(request.target, candidates)
        return response

    def _make_candidates(self, rows, request, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        tf_translation = [translation.x, translation.y, translation.z]
        tf_quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        minimum_score = float(self.get_parameter('minimum_grasp_score').value)
        maximum_width = float(self.get_parameter('maximum_gripper_width').value)
        maximum_candidates = int(self.get_parameter('maximum_candidates').value)
        axis = int(self.get_parameter('approach_axis').value)
        horizontal_filter = bool(
            self.get_parameter('horizontal_grasp_filter_enabled').value
        )
        closing_axis = int(self.get_parameter('closing_axis').value)
        vertical_axis = int(self.get_parameter('vertical_axis').value)
        maximum_approach_tilt = float(
            self.get_parameter('maximum_approach_tilt_degrees').value
        )
        maximum_closing_tilt = float(
            self.get_parameter('maximum_closing_tilt_degrees').value
        )
        offset = float(self.get_parameter('pregrasp_offset').value)
        apply_depth = bool(self.get_parameter('apply_grasp_depth').value)
        depth_scale = float(self.get_parameter('grasp_depth_scale').value)
        maximum_depth_offset = float(
            self.get_parameter('maximum_grasp_depth_offset').value
        )
        candidates = []
        for row in rows:
            grasp = parse_grasp_array(row)
            if grasp['score'] < minimum_score or grasp['width'] > maximum_width:
                continue
            position, grasp_rotation = transform_grasp_pose(
                grasp['translation'], grasp['rotation'], tf_translation, tf_quaternion
            )
            if horizontal_filter:
                accepted, _, _ = is_horizontal_grasp(
                    grasp_rotation,
                    approach_axis=axis,
                    closing_axis=closing_axis,
                    vertical_axis=vertical_axis,
                    maximum_approach_tilt_degrees=maximum_approach_tilt,
                    maximum_closing_tilt_degrees=maximum_closing_tilt,
                )
                if not accepted:
                    continue
            approach = grasp_rotation[:, axis]
            grasp_position = position
            if apply_depth:
                grasp_position, _ = apply_grasp_depth_offset(
                    position,
                    grasp_rotation,
                    axis,
                    grasp['depth'],
                    depth_scale,
                    maximum_depth_offset,
                )
            pregrasp = grasp_position - approach * offset
            arm = self._select_arm(request.preferred_arm, grasp_position[1])
            message = GraspCandidate()
            message.header = request.target.header
            message.header.frame_id = str(self.get_parameter('base_frame').value)
            message.track_id = request.target.track_id
            message.label = request.target.label
            message.arm = arm
            message.score = grasp['score']
            message.required_width = grasp['width']
            message.grasp_depth = grasp['depth']
            message.grasp_height = grasp['height']
            quaternion = matrix_to_quaternion(grasp_rotation)
            self._fill_pose(message.grasp_pose, grasp_position, quaternion)
            self._fill_pose(message.pregrasp_pose, pregrasp, quaternion)
            candidates.append(message)
            if len(candidates) >= maximum_candidates:
                break
        return candidates

    def _select_arm(self, requested_arm, base_y):
        arm = requested_arm if requested_arm in ('left', 'right') else 'auto'
        deadband = abs(float(self.get_parameter('auto_arm_deadband_y').value))
        prevent_cross_body = bool(self.get_parameter('prevent_cross_body_grasps').value)
        object_side = 'left' if base_y > deadband else 'right' if base_y < -deadband else 'auto'
        if object_side == 'auto':
            return arm if arm in ('left', 'right') else 'left'
        if prevent_cross_body and arm in ('left', 'right') and arm != object_side:
            self.get_logger().warning(
                f'corrected requested arm {arm} to {object_side}: target base_y={base_y:.3f}m'
            )
            return object_side
        return arm if arm in ('left', 'right') else object_side

    def _fill_pose(self, message, position, quaternion):
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter('base_frame').value)
        message.pose.position.x, message.pose.position.y, message.pose.position.z = [
            float(value) for value in position
        ]
        (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ) = [float(value) for value in quaternion]

    def _publish_candidates(self, target, candidates):
        message = GraspCandidateArray()
        message.header = target.header
        message.header.frame_id = str(self.get_parameter('base_frame').value)
        message.track_id = target.track_id
        message.candidates = candidates
        self._candidates_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = GraspNetServer()
    executor = MultiThreadedExecutor(num_threads=3)
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
