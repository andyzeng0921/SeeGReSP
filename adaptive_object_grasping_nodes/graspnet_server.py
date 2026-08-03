import copy
import importlib
import importlib.metadata
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
)
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from adaptive_object_grasping.msg import (
    GraspCandidate,
    GraspCandidateArray,
)
from adaptive_object_grasping.srv import EstimateGrasps
from adaptive_object_grasping_nodes.dual_arm_core import select_arm
from adaptive_object_grasping_nodes.graspnet_core import (
    apply_grasp_depth_offset,
    collision_scene_cloud,
    horizontal_grasp_orientation_variants,
    is_horizontal_grasp,
    matrix_to_quaternion,
    parse_grasp_array,
    sample_point_cloud,
    target_point_cloud,
    transform_grasp_pose,
    wrist_aligned_side_grasp_rows,
)
from adaptive_object_grasping_nodes.pbvs_core import quaternion_matrix
from adaptive_object_grasping_nodes.observation_store_core import (
    load_observation,
    resolve_store_directory,
)
from adaptive_object_grasping_nodes.perception_core import CameraIntrinsics
from adaptive_object_grasping_nodes.perception_sync_core import (
    freshness_error,
    stamp_nanoseconds,
)


def load_grasp_group_type():
    """Load GraspGroup without importing graspnetAPI's optional evaluation stack."""
    package_name = 'graspnetAPI'
    if package_name not in sys.modules:
        package_root = importlib.metadata.distribution(package_name).locate_file(package_name)
        placeholder = importlib.import_module('types').ModuleType(package_name)
        placeholder.__path__ = [str(package_root)]
        placeholder.__package__ = package_name
        sys.modules[package_name] = placeholder
    return importlib.import_module(f'{package_name}.grasp').GraspGroup


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

    def warmup(self):
        points = self._torch.zeros(
            (1, self._number_points, 3),
            dtype=self._torch.float32,
            device=self._device,
        )
        with self._torch.no_grad():
            predictions = self._pred_decode(
                self._net({'point_clouds': points})
            )
        # Force asynchronous CUDA work to finish before readiness is reported.
        if predictions:
            float(predictions[0].sum().item())

    def infer(
        self,
        sampled_points,
        scene_points,
        collision_threshold,
        voxel_size,
        maximum,
        approach_distance,
        wrist_aligned_context=None,
    ):
        started = time.perf_counter()
        tensor = self._torch.from_numpy(sampled_points[np.newaxis].astype(np.float32)).to(
            self._device
        )
        end_points = {'point_clouds': tensor}
        with self._torch.no_grad():
            predictions = self._pred_decode(self._net(end_points))
        network_finished = time.perf_counter()
        group = self._grasp_group_type(predictions[0].detach().cpu().numpy())
        # Bound collision-filter work before the O(grasp x scene-points) pass.
        group.nms()
        group.sort_by_score()
        group = group[: max(1, int(maximum))]
        wrist_aligned_count = 0
        if wrist_aligned_context:
            aligned_rows = wrist_aligned_side_grasp_rows(
                group.grasp_group_array,
                wrist_aligned_context['tcp_position'],
                wrist_aligned_context['tcp_rotation'],
                wrist_aligned_context['vertical_direction'],
                approach_axis=wrist_aligned_context['approach_axis'],
                closing_axis=wrist_aligned_context['closing_axis'],
                maximum_approach_tilt_degrees=wrist_aligned_context[
                    'maximum_approach_tilt_degrees'
                ],
                grasp_center=wrist_aligned_context['grasp_center'],
            )
            wrist_aligned_count = len(aligned_rows)
            if wrist_aligned_count:
                combined = np.vstack((
                    aligned_rows,
                    group.grasp_group_array,
                ))
                group = self._grasp_group_type(combined)
        preparation_finished = time.perf_counter()
        if collision_threshold > 0.0:
            from collision_detector import ModelFreeCollisionDetector

            detector = ModelFreeCollisionDetector(scene_points, voxel_size=float(voxel_size))
            collisions = detector.detect(
                group,
                approach_dist=max(0.0, float(approach_distance)),
                collision_thresh=float(collision_threshold),
            )
            group = group[~collisions]
        collision_finished = time.perf_counter()
        if not wrist_aligned_count:
            group.nms()
            group.sort_by_score()
        self.last_timings = {
            'network_seconds': network_finished - started,
            'preparation_seconds': preparation_finished - network_finished,
            'collision_seconds': collision_finished - preparation_finished,
            'finalize_seconds': time.perf_counter() - collision_finished,
            'wrist_aligned_rows': wrist_aligned_count,
        }
        return np.asarray(group.grasp_group_array[: int(maximum)])


class GraspNetServer(Node):
    def __init__(self):
        super().__init__('graspnet_server')
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
        self.declare_parameter('collision_scene_margin', 0.20)
        self.declare_parameter('auto_arm_deadband_y', 0.04)
        self.declare_parameter('prevent_cross_body_grasps', True)
        self.declare_parameter('collision_voxel_size', 0.01)
        self.declare_parameter('minimum_grasp_score', 0.25)
        self.declare_parameter('maximum_gripper_width', 0.10)
        self.declare_parameter('maximum_candidates', 20)
        self.declare_parameter('maximum_raw_candidates', 100)
        self.declare_parameter('inference_attempts', 1)
        self.declare_parameter('pregrasp_offset', 0.12)
        self.declare_parameter('approach_axis', 0)
        self.declare_parameter('horizontal_grasp_filter_enabled', True)
        self.declare_parameter('closing_axis', 1)
        self.declare_parameter('vertical_axis', 2)
        self.declare_parameter('maximum_approach_tilt_degrees', 15.0)
        self.declare_parameter('maximum_closing_tilt_degrees', 15.0)
        self.declare_parameter('bottle_side_grasp_enabled', True)
        self.declare_parameter(
            'bottle_side_grasp_labels', ['bottle']
        )
        self.declare_parameter(
            'bottle_side_roll_degrees', [90.0, -90.0, 180.0]
        )
        self.declare_parameter('bottle_wrist_aligned_enabled', True)
        self.declare_parameter(
            'bottle_wrist_aligned_use_target_center', True
        )
        self.declare_parameter('left_tcp_frame', 'left_grasp_tcp')
        self.declare_parameter('right_tcp_frame', 'right_grasp_tcp')
        self.declare_parameter('apply_grasp_depth', False)
        self.declare_parameter('grasp_depth_scale', 1.0)
        self.declare_parameter('maximum_grasp_depth_offset', 0.05)
        self.declare_parameter('tf_timeout', 0.2)
        self.declare_parameter('maximum_target_age_seconds', 0.50)
        self.declare_parameter(
            'maximum_post_inference_source_age_seconds',
            1.50,
        )
        self.declare_parameter('maximum_future_skew_seconds', 0.02)
        self.declare_parameter('preload_backend', True)
        self.declare_parameter(
            'observation_store_directory',
            'runtime/selected_observations',
        )
        self.declare_parameter(
            'maximum_observation_payload_bytes',
            64 * 1024 * 1024,
        )

        self._inference_lock = threading.Lock()
        self._backend = None
        self._backend_error = ''
        self._backend_ready = threading.Event()
        self._observation_store = resolve_store_directory(
            os.environ.get('ADAPTIVE_GRASP_PACKAGE_ROOT', ''),
            str(
                self.get_parameter('observation_store_directory').value
            ),
        )
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._candidates_pub = self.create_publisher(GraspCandidateArray, 'grasp_candidates', 10)
        self.create_service(
            EstimateGrasps,
            'estimate_grasps',
            self._estimate,
            callback_group=self._service_group,
        )
        if bool(self.get_parameter('preload_backend').value):
            threading.Thread(
                target=self._preload_backend,
                name='graspnet-preload',
                daemon=True,
            ).start()
            load_detail = 'backend preloading in background'
        else:
            load_detail = 'backend loads on first request'
        self.get_logger().info(
            'GraspNet service ready; RGB-D observations arrive atomically '
            'inside EstimateGrasps requests; '
            + load_detail
        )

    def _decode_observation(self, message, target):
        outer_stamp_ns = stamp_nanoseconds(message.header.stamp)
        target_stamp_ns = stamp_nanoseconds(target.header.stamp)
        if int(message.track_id) != int(target.track_id):
            raise ValueError(
                f'observation track {message.track_id} does not match '
                f'target track {target.track_id}'
            )
        if outer_stamp_ns != target_stamp_ns:
            raise ValueError(
                f'observation/target timestamps differ: '
                f'{outer_stamp_ns} != {target_stamp_ns}'
            )
        if message.header.frame_id != target.header.frame_id:
            raise ValueError(
                f'observation frame {message.header.frame_id} does not match '
                f'target frame {target.header.frame_id}'
            )
        stored = load_observation(
            self._observation_store,
            token=message.storage_token,
            expected_size=message.payload_size,
            expected_sha256=message.sha256,
            maximum_payload_bytes=int(
                self.get_parameter(
                    'maximum_observation_payload_bytes'
                ).value
            ),
            consume=True,
        )
        if stored['track_id'] != int(target.track_id):
            raise ValueError(
                f'stored observation track {stored["track_id"]} does not '
                f'match target track {target.track_id}'
            )
        if stored['stamp_ns'] != target_stamp_ns:
            raise ValueError(
                f'stored observation timestamp {stored["stamp_ns"]} does not '
                f'match target timestamp {target_stamp_ns}'
            )
        if stored['frame_id'] != target.header.frame_id:
            raise ValueError(
                f'stored observation frame {stored["frame_id"]} does not '
                f'match target frame {target.header.frame_id}'
            )
        values = stored['intrinsics']
        return {
            'color': stored['color'],
            'depth': stored['depth'],
            'intrinsics': CameraIntrinsics(*[float(value) for value in values]),
            'mask': stored['mask'],
        }

    def _load_backend(self):
        if self._backend is not None:
            return self._backend
        try:
            self._backend = OfficialGraspNetBackend(
                str(self.get_parameter('graspnet_root').value),
                str(self.get_parameter('checkpoint_path').value),
                str(self.get_parameter('device').value),
                int(self.get_parameter('number_points').value),
                int(self.get_parameter('number_views').value),
            )
            self._backend_error = ''
        except Exception as exc:
            self._backend_error = str(exc)
            raise
        return self._backend

    def _preload_backend(self):
        try:
            with self._inference_lock:
                backend = self._load_backend()
                backend.warmup()
            self.get_logger().info('GraspNet backend preload complete')
        except Exception as exc:
            self.get_logger().error(f'GraspNet backend preload failed: {exc}')
        finally:
            self._backend_ready.set()

    def _estimate(self, request, response):
        estimate_started = time.perf_counter()
        if not request.target.depth_valid:
            response.message = 'target has no valid aligned depth'
            return response
        preload_enabled = bool(self.get_parameter('preload_backend').value)
        if preload_enabled and not self._backend_ready.is_set():
            response.message = (
                'GraspNet backend is still preloading; retry after the '
                'graspnet_server reports preload complete'
            )
            return response
        if preload_enabled and self._backend_error:
            response.message = (
                'GraspNet backend preload failed: ' + self._backend_error
            )
            return response
        target_stamp_ns = stamp_nanoseconds(request.target.header.stamp)
        maximum_target_age = float(
            self.get_parameter('maximum_target_age_seconds').value
        )
        maximum_future_skew = float(
            self.get_parameter('maximum_future_skew_seconds').value
        )
        age_error = freshness_error(
            self.get_clock().now().nanoseconds,
            target_stamp_ns,
            maximum_target_age,
            maximum_future_seconds=maximum_future_skew,
        )
        if age_error:
            response.message = f'selected target is not fresh: {age_error}'
            return response
        try:
            observation = self._decode_observation(
                request.observation, request.target
            )
        except ValueError as exc:
            response.message = 'invalid selected RGB-D observation: ' + str(exc)
            return response
        decode_finished = time.perf_counter()
        snapshot_age_error = freshness_error(
            self.get_clock().now().nanoseconds,
            target_stamp_ns,
            maximum_target_age,
            maximum_future_seconds=maximum_future_skew,
        )
        if snapshot_age_error:
            response.message = (
                'target became stale while decoding its exact observation: '
                + snapshot_age_error
            )
            return response
        color = np.asarray(observation['color']).copy()
        depth = np.asarray(observation['depth']).copy()
        intrinsics = observation['intrinsics']
        selected_mask = np.asarray(observation['mask']).copy()
        if selected_mask.shape != depth.shape[:2]:
            response.message = (
                f'selected mask shape {selected_mask.shape} does not match '
                f'depth shape {depth.shape[:2]}'
            )
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
        raw_scene_point_count = len(scene_points)
        scene_points = collision_scene_cloud(
            scene_points,
            points,
            margin=float(
                self.get_parameter('collision_scene_margin').value
            ),
            voxel_size=float(
                self.get_parameter('collision_voxel_size').value
            ),
        )
        cloud_finished = time.perf_counter()
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter('base_frame').value),
                request.target.header.frame_id,
                rclpy.time.Time.from_msg(request.target.header.stamp),
                timeout=Duration(seconds=float(self.get_parameter('tf_timeout').value)),
            )
            wrist_aligned_context = self._wrist_aligned_context(
                request, transform
            )
            with self._inference_lock:
                backend = self._load_backend()
                rows = np.empty((0, 17), dtype=np.float32)
                attempts_used = 0
                for _ in range(max(1, int(self.get_parameter('inference_attempts').value))):
                    attempts_used += 1
                    sampled_points, _ = sample_point_cloud(points, colors, backend.number_points)
                    rows = backend.infer(
                        sampled_points,
                        scene_points,
                        float(self.get_parameter('collision_threshold').value),
                        float(self.get_parameter('collision_voxel_size').value),
                        int(self.get_parameter('maximum_raw_candidates').value),
                        float(self.get_parameter('pregrasp_offset').value),
                        wrist_aligned_context=wrist_aligned_context,
                    )
                    if len(rows):
                        break
            inference_finished = time.perf_counter()
            timings = getattr(backend, 'last_timings', {})
            self.get_logger().info(
                'GraspNet timing: '
                f'decode={decode_finished - estimate_started:.3f}s, '
                f'clouds={cloud_finished - decode_finished:.3f}s, '
                f'network={timings.get("network_seconds", float("nan")):.3f}s, '
                f'prepare={timings.get("preparation_seconds", float("nan")):.3f}s, '
                f'collision={timings.get("collision_seconds", float("nan")):.3f}s, '
                f'finalize={timings.get("finalize_seconds", float("nan")):.3f}s, '
                f'total={inference_finished - estimate_started:.3f}s, '
                f'attempts={attempts_used}, '
                f'target_points={len(points)}, '
                f'scene_points={raw_scene_point_count}->{len(scene_points)}'
                f', wrist_aligned_rows='
                f'{timings.get("wrist_aligned_rows", 0)}'
            )
            post_inference_age_error = freshness_error(
                self.get_clock().now().nanoseconds,
                target_stamp_ns,
                float(
                    self.get_parameter(
                        'maximum_post_inference_source_age_seconds'
                    ).value
                ),
                maximum_future_seconds=maximum_future_skew,
            )
            if post_inference_age_error:
                response.message = (
                    'target became stale during GraspNet inference: '
                    f'{post_inference_age_error}; retry with a fresh frame'
                )
                return response
            candidates, filter_stats = self._make_candidates(
                rows, request, transform
            )
        except TransformException as exc:
            response.message = f'camera-to-base TF unavailable: {exc}'
            return response
        except Exception as exc:
            response.message = f'GraspNet inference unavailable: {exc}'
            self.get_logger().error(response.message)
            return response
        response.candidates = candidates
        response.success = bool(candidates)
        candidate_scores = [float(item.score) for item in candidates]
        score_range = (
            f'[{min(candidate_scores):.3f},{max(candidate_scores):.3f}]'
            if candidate_scores else '[]'
        )
        response.message = (
            f'generated {len(candidates)} grasp candidates from {len(rows)} raw rows '
            f'(minimum_score={float(self.get_parameter("minimum_grasp_score").value):.3f}, '
            f'maximum_width={float(self.get_parameter("maximum_gripper_width").value):.3f}m, '
            f'collision_threshold={float(self.get_parameter("collision_threshold").value):.3f}, '
            f'horizontal_filter='
            f'{bool(self.get_parameter("horizontal_grasp_filter_enabled").value)}, '
            f'rejected_score={filter_stats["score"]}, '
            f'rejected_width={filter_stats["width"]}, '
            f'rejected_orientation={filter_stats["orientation"]}, '
            f'bottle_rows_recovered={filter_stats["bottle_rows_recovered"]}, '
            f'bottle_equivalents={filter_stats["bottle_equivalents"]}, '
            f'score_range={score_range})'
        )
        self.get_logger().info(response.message)
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
        bottle_side_enabled = bool(
            self.get_parameter('bottle_side_grasp_enabled').value
        )
        bottle_labels = {
            str(value).strip().casefold()
            for value in self.get_parameter('bottle_side_grasp_labels').value
            if str(value).strip()
        }
        bottle_roll_degrees = [
            float(value)
            for value in self.get_parameter('bottle_side_roll_degrees').value
        ]
        is_bottle = request.target.label.strip().casefold() in bottle_labels
        offset = float(self.get_parameter('pregrasp_offset').value)
        apply_depth = bool(self.get_parameter('apply_grasp_depth').value)
        depth_scale = float(self.get_parameter('grasp_depth_scale').value)
        maximum_depth_offset = float(
            self.get_parameter('maximum_grasp_depth_offset').value
        )
        base_header = copy.deepcopy(request.target.header)
        base_header.frame_id = str(self.get_parameter('base_frame').value)
        candidates = []
        filter_stats = {
            'score': 0,
            'width': 0,
            'orientation': 0,
            'bottle_rows_recovered': 0,
            'bottle_equivalents': 0,
        }
        for row in rows:
            grasp = parse_grasp_array(row)
            if grasp['score'] < minimum_score:
                filter_stats['score'] += 1
                continue
            if grasp['width'] > maximum_width:
                filter_stats['width'] += 1
                continue
            position, grasp_rotation = transform_grasp_pose(
                grasp['translation'], grasp['rotation'], tf_translation, tf_quaternion
            )
            orientation_variants = [{
                'rotation': grasp_rotation,
                'roll_degrees': 0.0,
            }]
            if horizontal_filter:
                accepted, _, _ = is_horizontal_grasp(
                    grasp_rotation,
                    approach_axis=axis,
                    closing_axis=closing_axis,
                    vertical_axis=vertical_axis,
                    maximum_approach_tilt_degrees=maximum_approach_tilt,
                    maximum_closing_tilt_degrees=maximum_closing_tilt,
                )
                if accepted and bottle_side_enabled and is_bottle:
                    orientation_variants = horizontal_grasp_orientation_variants(
                        grasp_rotation,
                        approach_axis=axis,
                        closing_axis=closing_axis,
                        vertical_axis=vertical_axis,
                        maximum_approach_tilt_degrees=maximum_approach_tilt,
                        maximum_closing_tilt_degrees=maximum_closing_tilt,
                        roll_degrees=(0.0, *bottle_roll_degrees),
                    )
                    filter_stats['bottle_equivalents'] += max(
                        0, len(orientation_variants) - 1
                    )
                elif accepted:
                    orientation_variants = horizontal_grasp_orientation_variants(
                        grasp_rotation,
                        approach_axis=axis,
                        closing_axis=closing_axis,
                        vertical_axis=vertical_axis,
                        maximum_approach_tilt_degrees=maximum_approach_tilt,
                        maximum_closing_tilt_degrees=maximum_closing_tilt,
                        roll_degrees=(0.0,),
                    )
                elif bottle_side_enabled and is_bottle:
                    orientation_variants = horizontal_grasp_orientation_variants(
                        grasp_rotation,
                        approach_axis=axis,
                        closing_axis=closing_axis,
                        vertical_axis=vertical_axis,
                        maximum_approach_tilt_degrees=maximum_approach_tilt,
                        maximum_closing_tilt_degrees=maximum_closing_tilt,
                        roll_degrees=bottle_roll_degrees,
                    )
                    if orientation_variants:
                        filter_stats['bottle_rows_recovered'] += 1
                        filter_stats['bottle_equivalents'] += len(
                            orientation_variants
                        )
                if not orientation_variants:
                    filter_stats['orientation'] += 1
                    continue
            for variant in orientation_variants:
                candidate_rotation = variant['rotation']
                approach = candidate_rotation[:, axis]
                grasp_position = position
                if apply_depth:
                    grasp_position, _ = apply_grasp_depth_offset(
                        position,
                        candidate_rotation,
                        axis,
                        grasp['depth'],
                        depth_scale,
                        maximum_depth_offset,
                    )
                pregrasp = grasp_position - approach * offset
                arm = self._select_arm(request.preferred_arm, grasp_position[1])
                message = GraspCandidate()
                message.header = copy.deepcopy(base_header)
                message.track_id = request.target.track_id
                message.label = request.target.label
                message.arm = arm
                message.score = grasp['score']
                message.required_width = grasp['width']
                message.grasp_depth = grasp['depth']
                message.grasp_height = grasp['height']
                quaternion = matrix_to_quaternion(candidate_rotation)
                self._fill_pose(
                    message.grasp_pose,
                    grasp_position,
                    quaternion,
                    base_header,
                )
                self._fill_pose(
                    message.pregrasp_pose,
                    pregrasp,
                    quaternion,
                    base_header,
                )
                candidates.append(message)
                if len(candidates) >= maximum_candidates:
                    break
            if len(candidates) >= maximum_candidates:
                break
        return candidates, filter_stats

    def _wrist_aligned_context(self, request, camera_to_base):
        if not bool(
            self.get_parameter('bottle_wrist_aligned_enabled').value
        ):
            return None
        bottle_labels = {
            str(value).strip().casefold()
            for value in self.get_parameter('bottle_side_grasp_labels').value
            if str(value).strip()
        }
        if request.target.label.strip().casefold() not in bottle_labels:
            return None
        translation = camera_to_base.transform.translation
        quaternion = camera_to_base.transform.rotation
        camera_to_base_translation = np.array([
            translation.x, translation.y, translation.z
        ], dtype=np.float64)
        camera_to_base_rotation = quaternion_matrix([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        ])
        target_base = (
            camera_to_base_rotation
            @ np.array([
                request.target.position_camera.x,
                request.target.position_camera.y,
                request.target.position_camera.z,
            ], dtype=np.float64)
            + camera_to_base_translation
        )
        arm = self._select_arm(request.preferred_arm, target_base[1])
        tcp_frame = str(
            self.get_parameter(
                'left_tcp_frame' if arm == 'left' else 'right_tcp_frame'
            ).value
        )
        camera_from_tcp = self._tf_buffer.lookup_transform(
            request.target.header.frame_id,
            tcp_frame,
            rclpy.time.Time.from_msg(request.target.header.stamp),
            timeout=Duration(
                seconds=float(self.get_parameter('tf_timeout').value)
            ),
        )
        tcp_translation = camera_from_tcp.transform.translation
        tcp_quaternion = camera_from_tcp.transform.rotation
        vertical_camera = camera_to_base_rotation.T @ np.array(
            [0.0, 0.0, 1.0], dtype=np.float64
        )
        return {
            'tcp_position': np.array([
                tcp_translation.x, tcp_translation.y, tcp_translation.z
            ], dtype=np.float64),
            'tcp_rotation': quaternion_matrix([
                tcp_quaternion.x,
                tcp_quaternion.y,
                tcp_quaternion.z,
                tcp_quaternion.w,
            ]),
            'vertical_direction': vertical_camera,
            'grasp_center': (
                np.array([
                    request.target.position_camera.x,
                    request.target.position_camera.y,
                    request.target.position_camera.z,
                ], dtype=np.float64)
                if bool(
                    self.get_parameter(
                        'bottle_wrist_aligned_use_target_center'
                    ).value
                )
                else None
            ),
            'approach_axis': int(
                self.get_parameter('approach_axis').value
            ),
            'closing_axis': int(
                self.get_parameter('closing_axis').value
            ),
            'maximum_approach_tilt_degrees': float(
                self.get_parameter(
                    'maximum_approach_tilt_degrees'
                ).value
            ),
        }

    def _select_arm(self, requested_arm, base_y):
        deadband = abs(float(self.get_parameter('auto_arm_deadband_y').value))
        prevent_cross_body = bool(self.get_parameter('prevent_cross_body_grasps').value)
        decision = select_arm(
            requested_arm,
            base_y,
            deadband_y=deadband,
            prevent_cross_body=prevent_cross_body,
        )
        if decision.corrected_cross_body_request:
            self.get_logger().warning(
                f'corrected requested arm {requested_arm} to {decision.arm}: '
                f'target base_y={base_y:.3f}m'
            )
        return decision.arm

    @staticmethod
    def _fill_pose(message, position, quaternion, header):
        message.header = copy.deepcopy(header)
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
        message.header = copy.deepcopy(target.header)
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
