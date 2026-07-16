import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from adaptive_object_grasping.msg import TrackedObject, TrackedObjectArray
from adaptive_object_grasping.srv import ListObjects, SelectObject
from adaptive_object_grasping_nodes.perception_core import (
    CameraIntrinsics,
    deproject_pixel,
    image_buffer_to_array,
    object_depth_and_pixel,
    object_menu_lines,
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
        self.declare_parameter('device', '0')
        self.declare_parameter('image_size', 640)
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.55)
        self.declare_parameter('maximum_inference_rate', 30.0)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('minimum_depth', 0.15)
        self.declare_parameter('maximum_depth', 2.5)
        self.declare_parameter('allowed_classes', ['*'])

        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._color = None
        self._color_header = None
        self._color_sequence = 0
        self._depth = None
        self._intrinsics = None
        self._objects = []
        self._selected_id = None
        self._model = None
        self._model_error = ''

        self._objects_pub = self.create_publisher(TrackedObjectArray, 'tracked_objects', 10)
        self._selected_pub = self.create_publisher(TrackedObject, 'selected_object', 10)
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

    def _on_color(self, message):
        try:
            image = image_buffer_to_array(message)
        except Exception as exc:
            self.get_logger().warning(f'color image rejected: {exc}')
            return
        with self._lock:
            self._color = image
            self._color_header = message.header
            self._color_sequence += 1
        self._wake.set()

    def _on_depth(self, message):
        try:
            depth = image_buffer_to_array(message)
        except Exception as exc:
            self.get_logger().warning(f'depth image rejected: {exc}')
            return
        with self._lock:
            self._depth = depth

    def _on_info(self, message):
        with self._lock:
            self._intrinsics = CameraIntrinsics(
                float(message.k[0]), float(message.k[4]), float(message.k[2]), float(message.k[5])
            )

    def _load_model(self):
        if self._model is not None:
            return True
        try:
            from ultralytics import YOLO

            self._model = YOLO(str(self.get_parameter('model').value))
            self._model_error = ''
            self.get_logger().info('YOLO model loaded')
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
        previous_sequence = -1
        maximum_rate = max(1.0, float(self.get_parameter('maximum_inference_rate').value))
        minimum_period = 1.0 / maximum_rate
        while not self._stop.is_set():
            self._wake.wait(timeout=0.5)
            self._wake.clear()
            with self._lock:
                sequence = self._color_sequence
                color = None if self._color is None else self._color.copy()
                header = self._color_header
                depth = None if self._depth is None else self._depth.copy()
                intrinsics = self._intrinsics
            if color is None or header is None or sequence == previous_sequence:
                continue
            started = time.monotonic()
            if not self._load_model():
                self._stop.wait(2.0)
                continue
            previous_sequence = sequence
            try:
                result = self._infer(color)
                objects = self._messages_from_result(result, header, depth, intrinsics)
                self._publish_objects(header, objects)
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
            device=str(self.get_parameter('device').value),
            verbose=False,
        )
        return results[0]

    def _messages_from_result(self, result, header, depth, intrinsics):
        if result.boxes is None or len(result.boxes) == 0:
            return []
        boxes = result.boxes.xyxy.detach().cpu().numpy()
        classes = result.boxes.cls.detach().cpu().numpy().astype(int)
        scores = result.boxes.conf.detach().cpu().numpy()
        ids = result.boxes.id
        track_ids = (
            ids.detach().cpu().numpy().astype(int)
            if ids is not None
            else np.arange(len(boxes), dtype=int) + 100000
        )
        masks = None
        if result.masks is not None and result.masks.data is not None:
            masks = result.masks.data.detach().cpu().numpy()
        allowed = {str(value).lower() for value in self.get_parameter('allowed_classes').value}
        if '*' in allowed:
            allowed.clear()
        names = result.names
        messages = []
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
            mask = None if masks is None or index >= len(masks) else masks[index]
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
        return messages

    def _publish_objects(self, header, objects):
        array = TrackedObjectArray()
        array.header = header
        array.objects = objects
        with self._lock:
            self._objects = objects
            selected = next(
                (item for item in objects if item.track_id == self._selected_id), None
            )
        self._objects_pub.publish(array)
        if selected is not None:
            self._selected_pub.publish(selected)

    def _list_objects(self, _request, response):
        with self._lock:
            response.objects = list(self._objects)
        response.display_text = '\n'.join(object_menu_lines(response.objects))
        return response

    def _select_object(self, request, response):
        preferred_arm = request.preferred_arm.strip().lower() or 'auto'
        if preferred_arm not in ('auto', 'left', 'right'):
            response.message = 'preferred_arm must be auto, left or right'
            return response
        label = request.label.strip().lower()
        with self._lock:
            candidates = list(self._objects)
        selected = next(
            (item for item in candidates if request.track_id >= 0 and item.track_id == request.track_id),
            None,
        )
        if selected is None and label:
            matching = [item for item in candidates if label in item.label.lower()]
            if matching:
                selected = max(matching, key=lambda item: item.confidence)
        if selected is None:
            response.message = 'requested object is not in the current frame'
            return response
        with self._lock:
            self._selected_id = selected.track_id
        response.accepted = True
        response.message = f'selected [{selected.track_id}] {selected.label} ({preferred_arm})'
        response.selected = selected
        self._selected_pub.publish(selected)
        return response

    def _publish_menu(self):
        with self._lock:
            objects = list(self._objects)
        self._menu_pub.publish(String(data='\n'.join(object_menu_lines(objects))))

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
