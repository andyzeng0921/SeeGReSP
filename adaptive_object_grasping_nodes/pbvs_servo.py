import json
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from adaptive_object_grasping.msg import TrackedObject
from adaptive_object_grasping_nodes.motion_core import parse_eef_feedback
from adaptive_object_grasping_nodes.pbvs_core import (
    StabilityWindow,
    limited_low_pass,
    transform_point,
)


class PbvsServo(Node):
    """Generate bounded EEF pose targets that follow the selected 3D object."""

    def __init__(self):
        super().__init__('pbvs_target_servo')
        self.declare_parameter('base_frame', 'Link_Zero_Point')
        self.declare_parameter('eef_pose_topic', '/topic_arm_current_robot_eef_pose_0_283')
        self.declare_parameter('required_stable_duration', 1.5)
        self.declare_parameter('stable_motion_threshold', 0.012)
        self.declare_parameter('maximum_follow_distance', 0.25)
        self.declare_parameter('maximum_target_age', 0.5)
        self.declare_parameter('maximum_servo_speed', 0.12)
        self.declare_parameter('filter_alpha', 0.35)
        self.declare_parameter('tf_timeout', 0.1)

        self._lock = threading.Lock()
        self._current_eef = {'left': None, 'right': None}
        self._preferred_arm = 'auto'
        self._active_arm = ''
        self._track_id = None
        self._reference_offset = None
        self._held_orientation = None
        self._last_output = None
        self._last_output_time = None
        self._last_target_time = None
        self._target_is_lost = False
        self._last_status = {}
        self._stability = self._new_stability_window()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pose_pub = self.create_publisher(PoseStamped, 'pbvs_target_pose', 10)
        self._stable_pub = self.create_publisher(Bool, 'target_stable', 10)
        self._status_pub = self.create_publisher(String, 'target_stability', 10)
        self.create_subscription(TrackedObject, 'selected_object', self._on_target, 10)
        self.create_subscription(String, 'selected_arm', self._on_arm, 10)
        self.create_subscription(
            String,
            str(self.get_parameter('eef_pose_topic').value),
            self._on_eef_pose,
            10,
        )
        self.create_service(Trigger, 'reset_pbvs_tracking', self._reset_service)
        self.create_timer(0.2, self._watchdog)

    def _new_stability_window(self):
        return StabilityWindow(
            self.get_parameter('required_stable_duration').value,
            self.get_parameter('stable_motion_threshold').value,
            self.get_parameter('maximum_follow_distance').value,
        )

    def _on_arm(self, message):
        value = message.data.strip().lower()
        if value in ('auto', 'left', 'right'):
            with self._lock:
                self._preferred_arm = value

    def _on_eef_pose(self, message):
        try:
            feedback = parse_eef_feedback(message.data)
            parsed = {
                arm: (
                    np.asarray(pose['position'], dtype=np.float64),
                    np.asarray(pose['orientation'], dtype=np.float64),
                )
                for arm, pose in feedback.items()
            }
            with self._lock:
                self._current_eef.update(parsed)
        except Exception as exc:
            self.get_logger().warning(f'invalid EEF feedback JSON: {exc}')

    def _on_target(self, target):
        if not target.depth_valid:
            self._publish_status('blocked', 'selected target has no valid depth')
            return
        try:
            object_base = self._object_in_base(target)
        except TransformException as exc:
            self._publish_status('blocked', f'camera-to-base TF unavailable: {exc}')
            return
        now = time.monotonic()
        with self._lock:
            if self._track_id != target.track_id:
                self._reset_locked(target.track_id)
            arm = self._choose_arm(object_base)
            current = self._current_eef.get(arm)
            if current is None:
                self._publish_status_locked('blocked', f'{arm} EEF feedback unavailable')
                return
            current_position, current_orientation = current
            if self._reference_offset is None:
                self._active_arm = arm
                self._reference_offset = current_position - object_base
                self._held_orientation = current_orientation.copy()
            result = self._stability.update(now, object_base)
            self._last_target_time = now
            self._target_is_lost = False
            if not self._stability.within_follow_limit(object_base):
                self._publish_status_locked(
                    'blocked',
                    'target exceeded maximum follow distance',
                    result,
                )
                return
            if result.stable:
                self._publish_status_locked('stable', 'target is stable; grasp estimation allowed', result)
                return
            raw_target = object_base + self._reference_offset
            delta_time = 0.05 if self._last_output_time is None else max(0.001, now - self._last_output_time)
            maximum_step = float(self.get_parameter('maximum_servo_speed').value) * delta_time
            self._last_output = limited_low_pass(
                self._last_output,
                raw_target,
                float(self.get_parameter('filter_alpha').value),
                maximum_step,
            )
            self._last_output_time = now
            pose = self._pose_message(self._last_output, self._held_orientation)
            self._publish_status_locked('tracking', f'following [{target.track_id}] {target.label}', result)
        self._pose_pub.publish(pose)

    def _object_in_base(self, target):
        base_frame = str(self.get_parameter('base_frame').value)
        transform = self._tf_buffer.lookup_transform(
            base_frame,
            target.header.frame_id,
            rclpy.time.Time(),
            timeout=Duration(seconds=float(self.get_parameter('tf_timeout').value)),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return transform_point(
            [target.position_camera.x, target.position_camera.y, target.position_camera.z],
            [translation.x, translation.y, translation.z],
            [rotation.x, rotation.y, rotation.z, rotation.w],
        )

    def _choose_arm(self, object_base):
        if self._preferred_arm in ('left', 'right'):
            return self._preferred_arm
        return 'left' if object_base[1] >= 0.0 else 'right'

    def _pose_message(self, position, orientation):
        message = PoseStamped()
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
        ) = [float(value) for value in orientation]
        return message

    def _watchdog(self):
        with self._lock:
            last = self._last_target_time
        if last is not None and not self._target_is_lost and time.monotonic() - last > float(
            self.get_parameter('maximum_target_age').value
        ):
            with self._lock:
                self._target_is_lost = True
                self._stability.invalidate_stability()
            self._publish_status('lost', 'selected target tracking timed out')

    def _reset_service(self, _request, response):
        with self._lock:
            self._reset_locked(None)
        response.success = True
        response.message = 'PBVS tracking state reset'
        return response

    def _reset_locked(self, track_id):
        self._track_id = track_id
        self._active_arm = ''
        self._reference_offset = None
        self._held_orientation = None
        self._last_output = None
        self._last_output_time = None
        self._last_target_time = None
        self._target_is_lost = False
        self._stability.reset()

    def _publish_status(self, state, detail, result=None):
        with self._lock:
            self._publish_status_locked(state, detail, result)

    def _publish_status_locked(self, state, detail, result=None):
        status = {
            'state': state,
            'detail': detail,
            'track_id': self._track_id,
            'arm': self._active_arm,
            'stable': state == 'stable',
            'stable_elapsed': 0.0 if result is None else result.stable_elapsed,
            'window_motion': 0.0 if result is None else result.window_motion,
            'total_displacement': 0.0 if result is None else result.total_displacement,
        }
        if status == self._last_status:
            return
        self._last_status = status
        self._stable_pub.publish(Bool(data=status['stable']))
        self._status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = PbvsServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
