import copy
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String

from adaptive_object_grasping.msg import (
    SelectedObjectObservation,
    TrackedObject,
    TrackedObjectArray,
)
from adaptive_object_grasping.srv import ListObjects, SelectObject
from adaptive_object_grasping_nodes.perception_core import (
    CameraIntrinsics,
    deproject_pixel,
    image_buffer_to_array,
    mask_to_image_message,
    object_depth_and_pixel,
    object_menu_lines,
)
from adaptive_object_grasping_nodes.observation_store_core import (
    resolve_store_directory,
    write_observation,
)
from adaptive_object_grasping_nodes.perception_sync_core import (
    ExactRgbdCache,
    freshness_error,
    stamp_nanoseconds,
)


class YoloTracker(Node):
    """High-rate YOLO segmentation and persistent multi-object tracking."""

    def __init__(self):
        super().__init__('yolo_object_tracker')
        self.declare_parameter('color_topic', '/head_camera/color/image_raw')
        self.declare_parameter('depth_topic', '/head_camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/head_camera/color/camera_info')
        self.declare_parameter('model', 'models/yolo/yolo11n-seg.pt')
        self.declare_parameter('tracker', 'bytetrack.yaml')
        self.declare_parameter('device', 'auto')
        self.declare_parameter('image_size', 640)
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.55)
        self.declare_parameter('maximum_inference_rate', 30.0)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('minimum_depth', 0.15)
        self.declare_parameter('maximum_depth', 2.5)
        self.declare_parameter('allowed_classes', ['*'])
        self.declare_parameter('selected_mask_topic', '/selected_object_mask')
        self.declare_parameter('rgbd_cache_size', 16)
        self.declare_parameter('maximum_observation_age_seconds', 0.50)
        self.declare_parameter('maximum_selection_age_seconds', 0.35)
        self.declare_parameter('maximum_future_skew_seconds', 0.02)
        self.declare_parameter(
            'observation_store_directory',
            'runtime/selected_observations',
        )
        self.declare_parameter('maximum_observation_files', 32)

        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._rgbd_cache = ExactRgbdCache(
            int(self.get_parameter('rgbd_cache_size').value)
        )
        self._objects = []
        self._object_masks = {}
        self._observation_color = None
        self._observation_depth = None
        self._observation_intrinsics = None
        self._objects_stamp_ns = 0
        self._selected_id = None
        self._model = None
        self._model_error = ''
        self._resolved_device = None
        self._observation_store = resolve_store_directory(
            os.environ.get('ADAPTIVE_GRASP_PACKAGE_ROOT', ''),
            str(
                self.get_parameter('observation_store_directory').value
            ),
        )

        self._objects_pub = self.create_publisher(TrackedObjectArray, 'tracked_objects', 10)
        self._selected_pub = self.create_publisher(TrackedObject, 'selected_object', 10)
        self._selected_mask_pub = self.create_publisher(
            Image, str(self.get_parameter('selected_mask_topic').value), 10
        )
        self._menu_pub = self.create_publisher(String, 'object_menu', 10)
        self.create_subscription(
            Image,
            str(self.get_parameter('color_topic').value),
            self._on_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_info,
            qos_profile_sensor_data,
        )
        self.create_service(ListObjects, 'list_grasp_objects', self._list_objects)
        self.create_service(SelectObject, 'select_grasp_object', self._select_object)
        self.create_timer(1.0, self._publish_menu)
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f'YOLO tracker starting: model={self.get_parameter("model").value}, '
            f'device={self.get_parameter("device").value}'
        )

    def _resolve_device(self):
        requested = str(self.get_parameter('device').value).strip().lower()
        if requested not in ('', 'auto'):
            return requested
        if self._resolved_device is not None:
            return self._resolved_device
        try:
            import torch

            self._resolved_device = '0' if torch.cuda.is_available() else 'cpu'
        except Exception:
            self._resolved_device = 'cpu'
        self.get_logger().info(f'YOLO device resolved to {self._resolved_device}')
        return self._resolved_device

    def _on_color(self, message):
        try:
            image = image_buffer_to_array(message)
            self._cache_rgbd_component('color', message.header, image)
        except Exception as exc:
            self.get_logger().warning(f'color image rejected: {exc}')

    def _on_depth(self, message):
        try:
            depth = image_buffer_to_array(message)
            self._cache_rgbd_component('depth', message.header, depth)
        except Exception as exc:
            self.get_logger().warning(f'depth image rejected: {exc}')

    def _on_info(self, message):
        try:
            intrinsics = CameraIntrinsics(
                float(message.k[0]), float(message.k[4]), float(message.k[2]), float(message.k[5])
            )
            self._cache_rgbd_component('intrinsics', message.header, intrinsics)
        except Exception as exc:
            self.get_logger().warning(f'camera info rejected: {exc}')

    def _cache_rgbd_component(self, stream, header, value):
        with self._lock:
            snapshot = self._rgbd_cache.add(
                stream,
                stamp_nanoseconds(header.stamp),
                header.frame_id,
                value,
            )
        if snapshot is not None:
            self._wake.set()

    def _load_model(self):
        if self._model is not None:
            return True
        try:
            from ultralytics import YOLO

            model = YOLO(str(self.get_parameter('model').value))
            task = str(getattr(model, 'task', '')).strip().lower()
            if task != 'segment':
                raise RuntimeError(
                    f'configured YOLO model task is {task or "unknown"}, '
                    'but instance segmentation is required'
                )
            self._model = model
            self._model_error = ''
            self.get_logger().info('YOLO segmentation model loaded')
            return True
        except Exception as exc:
            error = str(exc)
            if error != self._model_error:
                self.get_logger().error(
                    f'YOLO unavailable: {error}. Run tools/install_ai_environment.sh first.'
                )
                self._model_error = error
            return False

    def _run_worker(self):
        previous_stamp_ns = None
        maximum_rate = max(1.0, float(self.get_parameter('maximum_inference_rate').value))
        minimum_period = 1.0 / maximum_rate
        while not self._stop.is_set():
            self._wake.wait(timeout=0.5)
            self._wake.clear()
            started = time.monotonic()
            if not self._load_model():
                self._stop.wait(2.0)
                continue
            # Loading and warming a model can take seconds. Reacquire the
            # newest complete frame only after the model is ready.
            with self._lock:
                snapshot = self._rgbd_cache.latest(
                    after_stamp_ns=previous_stamp_ns
                )
            if snapshot is None:
                continue
            previous_stamp_ns = snapshot.stamp_ns
            color = np.asarray(snapshot.color).copy()
            depth = np.asarray(snapshot.depth).copy()
            intrinsics = snapshot.intrinsics
            header = self._header(snapshot.stamp_ns, snapshot.frame_id)
            try:
                result = self._infer(color)
                objects, masks_by_id = self._messages_from_result(
                    result,
                    header,
                    depth,
                    intrinsics,
                    color.shape[:2],
                )
                age_error = self._freshness_error(snapshot.stamp_ns)
                if age_error:
                    self.get_logger().warning(
                        'discarding YOLO result because its RGB-D frame became '
                        f'stale during inference: {age_error}'
                    )
                    objects, masks_by_id = [], {}
                self._publish_objects(
                    header,
                    objects,
                    masks_by_id,
                    color,
                    depth,
                    intrinsics,
                )
            except Exception as exc:
                self.get_logger().error(f'YOLO inference failed: {exc}')
            remaining = minimum_period - (time.monotonic() - started)
            if remaining > 0.0:
                self._stop.wait(remaining)

    def _infer(self, color):
        results = self._model.track(
            source=color,
            persist=True,
            tracker=str(self.get_parameter('tracker').value),
            conf=float(self.get_parameter('confidence_threshold').value),
            iou=float(self.get_parameter('iou_threshold').value),
            imgsz=int(self.get_parameter('image_size').value),
            device=self._resolve_device(),
            retina_masks=True,
            verbose=False,
        )
        return results[0]

    def _messages_from_result(self, result, header, depth, intrinsics, image_shape):
        if result.boxes is None or len(result.boxes) == 0:
            return [], {}
        boxes = result.boxes.xyxy.detach().cpu().numpy()
        classes = result.boxes.cls.detach().cpu().numpy().astype(int)
        scores = result.boxes.conf.detach().cpu().numpy()
        ids = result.boxes.id
        track_ids = (
            ids.detach().cpu().numpy().astype(int)
            if ids is not None
            else np.arange(len(boxes), dtype=int) + 100000
        )
        if result.masks is None or result.masks.data is None:
            raise ValueError(
                'segmentation model returned boxes without instance masks'
            )
        masks = result.masks.data.detach().cpu().numpy()
        if masks.ndim != 3 or tuple(masks.shape[1:]) != tuple(image_shape):
            raise ValueError(
                f'instance masks have shape {masks.shape}; expected '
                f'(N, {image_shape[0]}, {image_shape[1]}). '
                'retina_masks=True must preserve original image coordinates'
            )
        if len(masks) != len(boxes):
            raise ValueError(
                f'instance mask count {len(masks)} does not match '
                f'box count {len(boxes)}'
            )
        allowed = {str(value).lower() for value in self.get_parameter('allowed_classes').value}
        if '*' in allowed:
            allowed.clear()
        names = result.names
        messages = []
        masks_by_id = {}
        for index, (box, class_id, score, track_id) in enumerate(
            zip(boxes, classes, scores, track_ids)
        ):
            label = str(names[int(class_id)])
            if allowed and label.lower() not in allowed:
                continue
            message = TrackedObject()
            message.header = header
            message.track_id = int(track_id)
            message.label = label
            message.confidence = float(score)
            x1, y1, x2, y2 = box
            message.bbox_x = int(round(x1))
            message.bbox_y = int(round(y1))
            message.bbox_width = max(1, int(round(x2 - x1)))
            message.bbox_height = max(1, int(round(y2 - y1)))
            mask = np.asarray(masks[index]) > 0.5
            masks_by_id[int(track_id)] = mask.copy()
            if depth is not None and intrinsics is not None:
                sample = object_depth_and_pixel(
                    depth,
                    box,
                    mask,
                    depth_scale=float(self.get_parameter('depth_scale').value),
                    minimum_depth=float(self.get_parameter('minimum_depth').value),
                    maximum_depth=float(self.get_parameter('maximum_depth').value),
                )
                if sample is not None:
                    distance, u, v = sample
                    point = deproject_pixel(u, v, distance, intrinsics)
                    message.depth_valid = True
                    message.median_depth = distance
                    message.position_camera.x = float(point[0])
                    message.position_camera.y = float(point[1])
                    message.position_camera.z = float(point[2])
            messages.append(message)
        return messages, masks_by_id

    def _publish_objects(
        self,
        header,
        objects,
        masks_by_id,
        color,
        depth,
        intrinsics,
    ):
        array = TrackedObjectArray()
        array.header = copy.deepcopy(header)
        array.objects = objects
        with self._lock:
            self._objects = objects
            self._object_masks = masks_by_id
            self._observation_color = color
            self._observation_depth = depth
            self._observation_intrinsics = intrinsics
            self._objects_stamp_ns = stamp_nanoseconds(header.stamp)
            selected = next(
                (item for item in objects if item.track_id == self._selected_id), None
            )
            selected = None if selected is None else copy.deepcopy(selected)
            selected_mask = (
                None
                if selected is None
                else masks_by_id.get(selected.track_id)
            )
            selected_mask = (
                None if selected_mask is None else selected_mask.copy()
            )
        self._objects_pub.publish(array)
        if selected is not None:
            self._selected_pub.publish(selected)
            if selected_mask is not None:
                self._selected_mask_pub.publish(
                    mask_to_image_message(
                        selected_mask, copy.deepcopy(selected.header)
                    )
                )

    def _list_objects(self, _request, response):
        with self._lock:
            stamp_ns = self._objects_stamp_ns
            objects = list(self._objects)
        response.objects = [] if self._freshness_error(stamp_ns) else objects
        response.display_text = '\n'.join(object_menu_lines(response.objects))
        return response

    def _select_object(self, request, response):
        preferred_arm = request.preferred_arm.strip().lower() or 'auto'
        if preferred_arm not in ('auto', 'left', 'right'):
            response.message = 'preferred_arm must be auto, left or right'
            return response
        label = request.label.strip().lower()
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            age_error = freshness_error(
                now_ns,
                self._objects_stamp_ns,
                float(
                    self.get_parameter(
                        'maximum_selection_age_seconds'
                    ).value
                ),
                maximum_future_seconds=float(
                    self.get_parameter(
                        'maximum_future_skew_seconds'
                    ).value
                ),
            )
            if age_error:
                response.message = (
                    'tracked object frame is stale; wait for a fresh RGB-D '
                    f'frame ({age_error})'
                )
                return response
            candidates = list(self._objects)
            selected = next(
                (
                    item
                    for item in candidates
                    if request.track_id >= 0
                    and item.track_id == request.track_id
                ),
                None,
            )
            if selected is None and label:
                matching = [
                    item for item in candidates if label in item.label.lower()
                ]
                if matching:
                    selected = max(
                        matching, key=lambda item: item.confidence
                    )
            if selected is None:
                response.message = 'requested object is not in the current frame'
                return response
            selected_mask = self._object_masks.get(selected.track_id)
            if selected_mask is None:
                response.message = (
                    'requested object has no instance segmentation mask'
                )
                return response
            self._selected_id = selected.track_id
            selected = copy.deepcopy(selected)
            selected_mask = selected_mask.copy()
            observation = (
                self._observation_color,
                self._observation_depth,
                self._observation_intrinsics,
            )
            if any(value is None for value in observation):
                response.message = 'selected object has no atomic RGB-D observation'
                return response
        response.message = f'selected [{selected.track_id}] {selected.label} ({preferred_arm})'
        response.selected = selected
        if request.include_observation:
            try:
                response.observation = self._make_selected_observation(
                    selected.track_id,
                    selected_mask,
                    selected.header,
                    *observation,
                )
            except Exception as exc:
                response.message = (
                    'failed to persist the selected RGB-D observation: '
                    + str(exc)
                )
                self.get_logger().error(response.message)
                return response
        response.accepted = True
        self._selected_pub.publish(selected)
        self._selected_mask_pub.publish(
            mask_to_image_message(selected_mask, copy.deepcopy(selected.header))
        )
        return response

    def _make_selected_observation(
        self,
        track_id,
        mask,
        header,
        color,
        depth,
        intrinsics,
    ):
        record = SelectedObjectObservation()
        record.header = copy.deepcopy(header)
        record.track_id = int(track_id)
        token, payload_size, digest = write_observation(
            self._observation_store,
            color=color,
            depth=depth,
            mask=mask,
            intrinsics=(
                intrinsics.fx,
                intrinsics.fy,
                intrinsics.cx,
                intrinsics.cy,
            ),
            track_id=track_id,
            stamp_ns=stamp_nanoseconds(header.stamp),
            frame_id=header.frame_id,
            maximum_files=int(
                self.get_parameter('maximum_observation_files').value
            ),
        )
        record.storage_token = token
        record.payload_size = int(payload_size)
        record.sha256 = digest
        return record

    @staticmethod
    def _header(stamp_ns, frame_id):
        header = Header()
        header.stamp.sec = int(stamp_ns) // 1_000_000_000
        header.stamp.nanosec = int(stamp_ns) % 1_000_000_000
        header.frame_id = str(frame_id)
        return header

    def _publish_menu(self):
        with self._lock:
            stamp_ns = self._objects_stamp_ns
            objects = list(self._objects)
        if self._freshness_error(stamp_ns):
            objects = []
        self._menu_pub.publish(String(data='\n'.join(object_menu_lines(objects))))

    def _freshness_error(self, stamp_ns, *, now_ns=None):
        if now_ns is None:
            now_ns = self.get_clock().now().nanoseconds
        return freshness_error(
            now_ns,
            stamp_ns,
            float(
                self.get_parameter(
                    'maximum_observation_age_seconds'
                ).value
            ),
            maximum_future_seconds=float(
                self.get_parameter('maximum_future_skew_seconds').value
            ),
        )

    def destroy_node(self):
        self._stop.set()
        self._wake.set()
        if self._worker.is_alive():
            self._worker.join(timeout=3.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
