#!/usr/bin/env python3
"""Capture one exact YOLO/RGB-D frame and relate the target to both robot arms.

This tool is read-only.  It does not publish any motion topic or action goal.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from collections import OrderedDict
from pathlib import Path
import shutil
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from adaptive_object_grasping.msg import TrackedObjectArray
from adaptive_object_grasping_nodes.pbvs_core import transform_point
from adaptive_object_grasping_nodes.perception_core import image_buffer_to_array
from adaptive_object_grasping_nodes.perception_sync_core import stamp_nanoseconds
from adaptive_object_grasping_nodes.scene_understanding_core import (
    choose_scene_target,
    classify_depth_evidence,
    detection_quality,
    project_camera_point,
    relative_geometry,
    robust_depth_at_pixel,
)


ARM_FRAMES = {
    'left': [
        ('shoulder', 'Link_Left_Shoulder_Inner_to_Shoulder_Outer'),
        ('elbow', 'Link_Left_Elbow_to_Forearm'),
        ('wrist', 'Link_Left_Wrist_Lower_to_Gripper'),
        ('tcp', 'left_grasp_tcp'),
    ],
    'right': [
        ('shoulder', 'Link_Right_Shoulder_Inner_to_Shoulder_Outer'),
        ('elbow', 'Link_Right_Elbow_to_Forearm'),
        ('wrist', 'Link_Right_Wrist_Lower_to_Gripper'),
        ('tcp', 'right_grasp_tcp'),
    ],
}


def _vector3(value):
    return [float(value.x), float(value.y), float(value.z)]


def _quaternion(value):
    return [float(value.x), float(value.y), float(value.z), float(value.w)]


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


class ExactSceneObserver(Node):
    def __init__(self, args):
        super().__init__('exact_scene_observer')
        self.args = args
        self.result = None
        self.error = None
        self.seen_objects = []
        self._colors = OrderedDict()
        self._depths = OrderedDict()
        self._intrinsics = OrderedDict()
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            Image,
            args.color_topic,
            self._on_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            args.depth_topic,
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            args.camera_info_topic,
            self._on_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TrackedObjectArray,
            args.objects_topic,
            self._on_objects,
            10,
        )

    @staticmethod
    def _trim(cache, maximum=90):
        while len(cache) > maximum:
            cache.popitem(last=False)

    def _on_color(self, message):
        try:
            image = np.asarray(image_buffer_to_array(message)).copy()
            self._colors[stamp_nanoseconds(message.header.stamp)] = (
                image,
                str(message.encoding).lower(),
                copy.deepcopy(message.header),
            )
            self._trim(self._colors)
        except Exception as exc:
            self.get_logger().warning(f'color frame rejected: {exc}')

    def _on_depth(self, message):
        try:
            depth = np.asarray(image_buffer_to_array(message)).copy()
            encoding = str(message.encoding).lower()
            scale = 1.0 if encoding in ('32fc1', '64fc1') else self.args.depth_scale
            self._depths[stamp_nanoseconds(message.header.stamp)] = (
                depth,
                scale,
                copy.deepcopy(message.header),
            )
            self._trim(self._depths)
        except Exception as exc:
            self.get_logger().warning(f'depth frame rejected: {exc}')

    def _on_info(self, message):
        try:
            values = [float(message.k[0]), float(message.k[4]), float(message.k[2]), float(message.k[5])]
            if not all(math.isfinite(value) for value in values) or values[0] <= 0 or values[1] <= 0:
                raise ValueError('invalid focal lengths')
            self._intrinsics[stamp_nanoseconds(message.header.stamp)] = (
                values,
                int(message.width),
                int(message.height),
                copy.deepcopy(message.header),
            )
            self._trim(self._intrinsics)
        except Exception as exc:
            self.get_logger().warning(f'camera info rejected: {exc}')

    @staticmethod
    def _record(item):
        return {
            'track_id': int(item.track_id),
            'label': str(item.label),
            'confidence': float(item.confidence),
            'bbox_x': int(item.bbox_x),
            'bbox_y': int(item.bbox_y),
            'bbox_width': int(item.bbox_width),
            'bbox_height': int(item.bbox_height),
            'depth_valid': bool(item.depth_valid),
            'median_depth': float(item.median_depth),
            'position_camera_m': _vector3(item.position_camera),
        }

    def _on_objects(self, message):
        if self.result is not None or self.error is not None:
            return
        records = [self._record(item) for item in message.objects]
        self.seen_objects = [
            {'track_id': item['track_id'], 'label': item['label'], 'confidence': item['confidence']}
            for item in records
        ]
        selected = choose_scene_target(records, self.args.label, self.args.track_id)
        if selected is None:
            return
        stamp_ns = stamp_nanoseconds(message.header.stamp)
        if stamp_ns <= 0:
            self.error = 'tracked object timestamp is zero'
            return
        snapshot = (
            self._colors.get(stamp_ns),
            self._depths.get(stamp_ns),
            self._intrinsics.get(stamp_ns),
        )
        if any(item is None for item in snapshot):
            return
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age < -self.args.maximum_future_skew or age > self.args.maximum_age:
            return
        try:
            result = self._build_report(
                message,
                selected,
                records,
                snapshot,
                age,
            )
            report_path = Path(result['capture']['sample_directory']) / 'scene.json'
            result['capture']['scene_json_path'] = str(report_path)
            report_path.write_text(
                json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + '\n',
                encoding='utf-8',
            )
            report_path.chmod(0o600)
            result['capture']['scene_json_sha256'] = _sha256(report_path)
            self.result = result
        except Exception as exc:
            self.get_logger().warning(f'exact scene sample rejected: {exc}')

    def _lookup(self, target_frame, source_frame, stamp):
        return self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time.from_msg(stamp),
            timeout=Duration(seconds=self.args.tf_timeout),
        )

    @staticmethod
    def _transform_camera_point(point, transform):
        return transform_point(
            point,
            _vector3(transform.transform.translation),
            _quaternion(transform.transform.rotation),
        )

    @staticmethod
    def _frame_origin(transform):
        return np.asarray(_vector3(transform.transform.translation), dtype=np.float64)

    def _arm_report(self, arm, stamp, camera_frame, intrinsics, width, height, depth, depth_scale):
        keypoints = {}
        missing = []
        for name, frame in ARM_FRAMES[arm]:
            try:
                camera_tf = self._lookup(camera_frame, frame, stamp)
                base_tf = self._lookup(self.args.base_frame, frame, stamp)
                point_camera = self._frame_origin(camera_tf)
                point_base = self._frame_origin(base_tf)
                projection = project_camera_point(point_camera, intrinsics, width, height)
                observed = None
                evidence = {'state': 'outside_image', 'depth_error_m': None}
                if projection['inside_image']:
                    observed = robust_depth_at_pixel(
                        depth,
                        projection['pixel'],
                        depth_scale=depth_scale,
                        radius=self.args.arm_depth_radius,
                        minimum_depth=self.args.minimum_depth,
                        maximum_depth=self.args.maximum_depth,
                    )
                    evidence = classify_depth_evidence(
                        point_camera[2],
                        observed,
                        tolerance=self.args.arm_depth_tolerance,
                    )
                keypoints[name] = {
                    'frame': frame,
                    'position_base_m': point_base.tolist(),
                    'position_camera_m': point_camera.tolist(),
                    'pixel': projection['pixel'],
                    'projected_inside_image': projection['inside_image'],
                    'predicted_depth_m': float(point_camera[2]),
                    'observed_depth_m': observed,
                    'depth_evidence': evidence,
                }
            except TransformException as exc:
                missing.append({'frame': frame, 'error': str(exc)})
        inside = sum(int(item['projected_inside_image']) for item in keypoints.values())
        supported = sum(
            int(item['depth_evidence']['state'] == 'depth_consistent')
            for item in keypoints.values()
        )
        occluded = sum(
            int(item['depth_evidence']['state'] == 'occluded_by_nearer_surface')
            for item in keypoints.values()
        )
        if inside == 0:
            assessment = 'outside_field_of_view'
        elif supported > 0:
            assessment = 'depth_supported'
        elif occluded >= max(1, inside // 2):
            assessment = 'projected_but_occluded'
        else:
            assessment = 'projected_only_not_depth_confirmed'
        return {
            'keypoints': keypoints,
            'missing_transforms': missing,
            'projected_inside_count': inside,
            'depth_supported_count': supported,
            'assessment': assessment,
        }

    def _build_report(self, message, selected, records, snapshot, age):
        (color, encoding, color_header), (depth, depth_scale, depth_header), info = snapshot
        intrinsics, width, height, info_header = info
        frame_id = str(message.header.frame_id)
        if not frame_id or any(
            str(header.frame_id) != frame_id
            for header in (color_header, depth_header, info_header)
        ):
            raise ValueError('RGB, depth, intrinsics and tracked object frames differ')
        if color.shape[:2] != depth.shape[:2] or color.shape[1] != width or color.shape[0] != height:
            raise ValueError('RGB, depth and intrinsics dimensions differ')
        if encoding == 'rgb8':
            color_bgr = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
        elif encoding == 'rgba8':
            color_bgr = cv2.cvtColor(color, cv2.COLOR_RGBA2BGR)
        elif encoding == 'bgra8':
            color_bgr = cv2.cvtColor(color, cv2.COLOR_BGRA2BGR)
        elif color.ndim == 3 and color.shape[2] >= 3:
            color_bgr = np.ascontiguousarray(color[:, :, :3])
        else:
            raise ValueError(f'unsupported color encoding {encoding!r}')

        base_from_camera = self._lookup(self.args.base_frame, frame_id, message.header.stamp)
        target_base = self._transform_camera_point(selected['position_camera_m'], base_from_camera)
        target_projection = project_camera_point(
            selected['position_camera_m'], intrinsics, width, height
        )
        arms = {
            arm: self._arm_report(
                arm,
                message.header.stamp,
                frame_id,
                intrinsics,
                width,
                height,
                depth,
                depth_scale,
            )
            for arm in ('left', 'right')
        }
        distances = {}
        for arm, arm_report in arms.items():
            tcp = arm_report['keypoints'].get('tcp')
            if tcp is not None:
                geometry = relative_geometry(
                    target_base,
                    tcp['position_base_m'],
                    standoff_distance=self.args.standoff_distance,
                )
                arm_report['target_geometry'] = geometry
                distances[arm] = geometry['distance_m']
        nearest_arm = min(distances, key=distances.get) if distances else None
        selected['position_base_m'] = target_base.tolist()
        selected['quality'] = detection_quality(selected, width, height)
        selected['projected_pixel_from_3d'] = target_projection['pixel']
        sample = self._write_sample(
            message,
            color_bgr,
            depth,
            selected,
            arms,
            intrinsics,
            depth_scale,
        )
        return {
            'success': True,
            'schema_version': 1,
            'capture': {
                'source': 'AutoLife RGB-D shared memory via exact-ID ROS bridge',
                'stamp_ns': stamp_nanoseconds(message.header.stamp),
                'age_seconds_at_capture': float(age),
                'frame_id': frame_id,
                'base_frame': self.args.base_frame,
                'base_from_camera': {
                    'translation_m': _vector3(base_from_camera.transform.translation),
                    'quaternion_xyzw': _quaternion(base_from_camera.transform.rotation),
                },
                'image_width': width,
                'image_height': height,
                'intrinsics_fx_fy_cx_cy': intrinsics,
                **sample,
            },
            'target': selected,
            'detected_objects': records,
            'arms': arms,
            'geometric_nearest_arm': nearest_arm,
            'visibility_reposition_needed': bool(
                all(item['assessment'] == 'outside_field_of_view' for item in arms.values())
            ),
            'safety': {
                'hardware_motion_sent': False,
                'arm_reset_executed': False,
                'standoff_is_geometry_only': True,
                'next_motion_step_requires_graspnet_moveit_and_site_acceptance': True,
            },
        }

    def _write_sample(self, message, color, depth, target, arms, intrinsics, depth_scale):
        root = Path(self.args.output_directory).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True, mode=0o700)
        stamp_ns = stamp_nanoseconds(message.header.stamp)
        name = f'{stamp_ns}-{target["label"]}-{target["track_id"]}'
        final = root / name
        temporary = root / f'.{name}.tmp-{os.getpid()}'
        if temporary.exists():
            shutil.rmtree(temporary)
        temporary.mkdir(mode=0o700)
        raw_path = temporary / 'rgb.png'
        depth_path = temporary / 'aligned_depth.png'
        annotated_path = temporary / 'annotated.jpg'
        if not cv2.imwrite(str(raw_path), color):
            raise RuntimeError('failed to save RGB sample')
        depth_to_write = depth
        if np.issubdtype(depth.dtype, np.floating):
            depth_to_write = np.clip(depth / depth_scale, 0, 65535).astype(np.uint16)
        if not cv2.imwrite(str(depth_path), depth_to_write):
            raise RuntimeError('failed to save aligned depth sample')
        annotated = self._annotate(color, target, arms, intrinsics)
        if not cv2.imwrite(str(annotated_path), annotated, [cv2.IMWRITE_JPEG_QUALITY, 94]):
            raise RuntimeError('failed to save annotated sample')
        if final.exists():
            raise RuntimeError(f'sample path already exists: {final}')
        temporary.rename(final)
        for path in final.iterdir():
            path.chmod(0o600)
        return {
            'sample_directory': str(final),
            'rgb_path': str(final / raw_path.name),
            'rgb_sha256': _sha256(final / raw_path.name),
            'depth_path': str(final / depth_path.name),
            'depth_sha256': _sha256(final / depth_path.name),
            'annotated_path': str(final / annotated_path.name),
            'annotated_sha256': _sha256(final / annotated_path.name),
            'depth_scale': float(depth_scale),
        }

    @staticmethod
    def _annotate(color, target, arms, intrinsics):
        image = color.copy()
        x1, y1 = int(target['bbox_x']), int(target['bbox_y'])
        x2 = x1 + int(target['bbox_width'])
        y2 = y1 + int(target['bbox_height'])
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 3)
        text = (
            f"[{target['track_id']}] {target['label']} "
            f"conf={target['confidence']:.2f} depth={target['median_depth']:.3f}m"
        )
        cv2.putText(image, text, (max(5, x1), max(24, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2, cv2.LINE_AA)
        if target.get('projected_pixel_from_3d'):
            u, v = [int(round(value)) for value in target['projected_pixel_from_3d']]
            cv2.drawMarker(image, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 20, 3)
        colors = {'left': (255, 80, 80), 'right': (80, 80, 255)}
        for arm, report in arms.items():
            previous = None
            for name in ('shoulder', 'elbow', 'wrist', 'tcp'):
                item = report['keypoints'].get(name)
                if item is None or not item['projected_inside_image']:
                    previous = None
                    continue
                point = tuple(int(round(value)) for value in item['pixel'])
                state = item['depth_evidence']['state']
                color_value = (40, 220, 40) if state == 'depth_consistent' else colors[arm]
                if previous is not None:
                    cv2.line(image, previous, point, colors[arm], 3, cv2.LINE_AA)
                cv2.circle(image, point, 7 if name == 'tcp' else 5, color_value, -1, cv2.LINE_AA)
                cv2.putText(image, f'{arm[0].upper()}-{name}', (point[0] + 7, point[1] - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.43, color_value, 1, cv2.LINE_AA)
                previous = point
        cv2.putText(image, 'Projected arm TF: green=depth supported', (12, image.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 2, cv2.LINE_AA)
        return image


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', default='bottle')
    parser.add_argument('--track-id', type=int, default=-1)
    parser.add_argument('--timeout', type=float, default=20.0)
    parser.add_argument('--output-directory', required=True)
    parser.add_argument('--base-frame', default='Link_Zero_Point')
    parser.add_argument('--color-topic', default='/head_camera/color/image_raw')
    parser.add_argument('--depth-topic', default='/head_camera/aligned_depth_to_color/image_raw')
    parser.add_argument('--camera-info-topic', default='/head_camera/color/camera_info')
    parser.add_argument('--objects-topic', default='tracked_objects')
    parser.add_argument('--depth-scale', type=float, default=0.001)
    parser.add_argument('--minimum-depth', type=float, default=0.15)
    parser.add_argument('--maximum-depth', type=float, default=3.0)
    parser.add_argument('--maximum-age', type=float, default=0.75)
    parser.add_argument('--maximum-future-skew', type=float, default=0.02)
    parser.add_argument('--tf-timeout', type=float, default=0.25)
    parser.add_argument('--arm-depth-radius', type=int, default=5)
    parser.add_argument('--arm-depth-tolerance', type=float, default=0.10)
    parser.add_argument('--standoff-distance', type=float, default=0.25)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ExactSceneObserver(args)
    deadline = time.monotonic() + max(1.0, args.timeout)
    try:
        while rclpy.ok() and node.result is None and node.error is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.result is not None:
            print(json.dumps(node.result, ensure_ascii=False, indent=2, sort_keys=True))
            return 0
        report = {
            'success': False,
            'error': node.error or f'no fresh exact-frame target {args.label!r} before timeout',
            'seen_objects': node.seen_objects,
            'hardware_motion_sent': False,
        }
        print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
