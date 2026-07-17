import math
import threading

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from adaptive_object_grasping.msg import (
    GraspCandidateArray,
    TrackedObject,
    TrackedObjectArray,
)
from adaptive_object_grasping_nodes.perception_core import image_buffer_to_array


try:
    import cv2
except Exception:  # pragma: no cover - OpenCV is optional on robot images.
    cv2 = None


class GraspVisualizer(Node):
    """Publish lightweight image overlays and RViz markers for the grasp pipeline."""

    def __init__(self):
        super().__init__('adaptive_grasp_visualizer')
        self.declare_parameter('color_topic', '/head_camera/color/image_raw')
        self.declare_parameter('objects_topic', 'tracked_objects')
        self.declare_parameter('selected_topic', 'selected_object')
        self.declare_parameter('candidates_topic', 'grasp_candidates')
        self.declare_parameter('status_topic', 'motion_execution_status')
        self.declare_parameter('overlay_topic', '/adaptive_grasp/visualization_image')
        self.declare_parameter('marker_topic', '/adaptive_grasp/grasp_markers')
        self.declare_parameter('draw_all_objects', True)
        self.declare_parameter('maximum_markers', 20)
        self.declare_parameter('marker_lifetime', 1.5)
        self.declare_parameter('gripper_width_scale', 1.0)
        self.declare_parameter('axis_length', 0.08)
        self.declare_parameter('line_width', 0.01)

        self._lock = threading.Lock()
        self._objects = []
        self._selected = None
        self._candidates = []
        self._candidate_header = None
        self._status = ''

        self._overlay_pub = self.create_publisher(
            Image, str(self.get_parameter('overlay_topic').value), 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 10
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('color_topic').value),
            self._on_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TrackedObjectArray,
            str(self.get_parameter('objects_topic').value),
            self._on_objects,
            10,
        )
        self.create_subscription(
            TrackedObject,
            str(self.get_parameter('selected_topic').value),
            self._on_selected,
            10,
        )
        self.create_subscription(
            GraspCandidateArray,
            str(self.get_parameter('candidates_topic').value),
            self._on_candidates,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('status_topic').value),
            self._on_status,
            10,
        )
        self.create_timer(0.5, self._publish_markers)
        self.get_logger().info(
            'visualizer ready: image=%s markers=%s'
            % (
                self.get_parameter('overlay_topic').value,
                self.get_parameter('marker_topic').value,
            )
        )

    def _on_objects(self, message):
        with self._lock:
            self._objects = list(message.objects)

    def _on_selected(self, message):
        with self._lock:
            self._selected = message

    def _on_candidates(self, message):
        with self._lock:
            self._candidate_header = message.header
            self._candidates = list(message.candidates)
        self._publish_markers()

    def _on_status(self, message):
        with self._lock:
            self._status = message.data

    def _on_color(self, message):
        try:
            image = image_buffer_to_array(message)
        except Exception as exc:
            self.get_logger().warning(f'overlay skipped: {exc}')
            return
        with self._lock:
            objects = list(self._objects)
            selected = self._selected
            candidates = list(self._candidates)
            status = self._status
        overlay = self._draw_overlay(image, objects, selected, candidates, status)
        self._overlay_pub.publish(self._image_message(message, overlay))

    def _draw_overlay(self, image, objects, selected, candidates, status):
        output = image.copy()
        selected_id = selected.track_id if selected is not None else None
        draw_all = bool(self.get_parameter('draw_all_objects').value)
        for item in objects:
            if not draw_all and item.track_id != selected_id:
                continue
            is_selected = item.track_id == selected_id
            color = (40, 220, 80) if is_selected else (255, 190, 40)
            x1 = max(0, int(item.bbox_x))
            y1 = max(0, int(item.bbox_y))
            x2 = min(output.shape[1] - 1, x1 + max(1, int(item.bbox_width)))
            y2 = min(output.shape[0] - 1, y1 + max(1, int(item.bbox_height)))
            self._rectangle(output, x1, y1, x2, y2, color, thickness=3 if is_selected else 2)
            depth = f'{item.median_depth:.2f}m' if item.depth_valid else 'no depth'
            self._text(output, f'[{item.track_id}] {item.label} {depth}', x1, max(16, y1 - 6), color)
        if candidates:
            best = max(candidates, key=lambda item: item.score)
            self._text(
                output,
                f'GraspNet: {len(candidates)} candidates, best #{best.track_id} {best.arm} {best.score:.2f}',
                12,
                28,
                (60, 240, 255),
            )
        if status:
            self._text(output, f'status: {status[:96]}', 12, output.shape[0] - 16, (230, 230, 230))
        return output

    def _rectangle(self, image, x1, y1, x2, y2, color, thickness=2):
        if cv2 is not None:
            cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
            return
        image[y1 : min(image.shape[0], y1 + thickness), x1:x2] = color
        image[max(0, y2 - thickness) : y2, x1:x2] = color
        image[y1:y2, x1 : min(image.shape[1], x1 + thickness)] = color
        image[y1:y2, max(0, x2 - thickness) : x2] = color

    def _text(self, image, text, x, y, color):
        if cv2 is None:
            return
        cv2.putText(
            image,
            text,
            (int(x), int(y)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    def _image_message(self, source, image):
        message = Image()
        message.header = source.header
        message.height = int(image.shape[0])
        message.width = int(image.shape[1])
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = int(image.shape[1] * 3)
        message.data = np.ascontiguousarray(image[:, :, :3]).tobytes()
        return message

    def _publish_markers(self):
        with self._lock:
            candidates = list(self._candidates)
            header = self._candidate_header
            selected = self._selected
        markers = MarkerArray()
        markers.markers.append(self._delete_all_marker())
        if header is None:
            self._marker_pub.publish(markers)
            return
        maximum = int(self.get_parameter('maximum_markers').value)
        for index, candidate in enumerate(candidates[:maximum]):
            markers.markers.extend(self._candidate_markers(candidate, index))
        if selected is not None and selected.depth_valid:
            markers.markers.append(self._selected_object_marker(selected, len(markers.markers)))
        self._marker_pub.publish(markers)

    def _delete_all_marker(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    def _candidate_markers(self, candidate, index):
        lifetime = self._duration()
        color = self._arm_color(candidate.arm, 0.85)
        best = index == 0
        width = max(0.015, float(candidate.required_width)) * float(
            self.get_parameter('gripper_width_scale').value
        )
        depth = max(0.04, float(candidate.grasp_depth))
        pose = candidate.grasp_pose.pose
        pregrasp = candidate.pregrasp_pose.pose
        ns = 'best_grasp' if best else 'grasp_candidate'
        markers = [
            self._sphere(
                candidate.grasp_pose.header,
                ns,
                index * 10,
                pose.position,
                0.035 if best else 0.025,
                color,
                lifetime,
            ),
            self._line(
                candidate.grasp_pose.header,
                ns,
                index * 10 + 1,
                [pregrasp.position, pose.position],
                ColorRGBA(r=0.95, g=0.95, b=0.95, a=0.75),
                lifetime,
            ),
            self._gripper(
                candidate.grasp_pose.header,
                ns,
                index * 10 + 2,
                pose.position,
                width,
                depth,
                color,
                lifetime,
            ),
            self._text_marker(
                candidate.grasp_pose.header,
                ns,
                index * 10 + 3,
                pose.position,
                f'{index}: {candidate.arm} {candidate.score:.2f} {candidate.required_width * 100:.1f}cm',
                color,
                lifetime,
            ),
        ]
        return markers

    def _selected_object_marker(self, selected, marker_id):
        marker = Marker()
        marker.header = selected.header
        marker.ns = 'selected_object'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = selected.position_camera
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.06
        marker.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.8)
        marker.lifetime = self._duration()
        return marker

    def _sphere(self, header, ns, marker_id, point, size, color, lifetime):
        marker = Marker()
        marker.header = header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = float(size)
        marker.color = color
        marker.lifetime = lifetime
        return marker

    def _line(self, header, ns, marker_id, points, color, lifetime):
        marker = Marker()
        marker.header = header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = points
        marker.scale.x = float(self.get_parameter('line_width').value)
        marker.color = color
        marker.lifetime = lifetime
        return marker

    def _gripper(self, header, ns, marker_id, center, width, depth, color, lifetime):
        half = width * 0.5
        jaw = min(0.055, max(0.025, depth))
        points = [
            Point(x=center.x, y=center.y - half, z=center.z),
            Point(x=center.x + jaw, y=center.y - half, z=center.z),
            Point(x=center.x, y=center.y + half, z=center.z),
            Point(x=center.x + jaw, y=center.y + half, z=center.z),
        ]
        return self._line(header, ns, marker_id, points, color, lifetime)

    def _text_marker(self, header, ns, marker_id, point, text, color, lifetime):
        marker = Marker()
        marker.header = header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z + 0.08
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.045
        marker.color = color
        marker.text = text
        marker.lifetime = lifetime
        return marker

    def _arm_color(self, arm, alpha):
        if arm == 'right':
            return ColorRGBA(r=1.0, g=0.25, b=0.15, a=alpha)
        if arm == 'left':
            return ColorRGBA(r=0.15, g=0.45, b=1.0, a=alpha)
        return ColorRGBA(r=0.9, g=0.9, b=0.2, a=alpha)

    def _duration(self):
        seconds = max(0.1, float(self.get_parameter('marker_lifetime').value))
        whole = math.floor(seconds)
        return Duration(sec=int(whole), nanosec=int((seconds - whole) * 1e9))


def main(args=None):
    rclpy.init(args=args)
    node = GraspVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
