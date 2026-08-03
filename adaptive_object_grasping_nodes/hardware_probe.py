import json
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from adaptive_object_grasping_nodes.hardware_health_core import (
    DEFAULT_REQUIRED_HEARTBEAT_MASK,
    joint_status_blockers,
    robot_state_blockers,
    robot_state_summary,
)
from adaptive_object_grasping_nodes.motion_core import parse_eef_feedback


class HardwareProbe(Node):
    """Refuse execution until fresh feedback is both present and healthy."""

    def __init__(self):
        super().__init__('adaptive_grasp_hardware_probe')
        self.declare_parameter('camera_info_topic', '/head_camera/color/camera_info')
        self.declare_parameter(
            'joint_state_topic',
            '/topic_arm_whole_body_and_gripper_current_joints_status_0_306',
        )
        self.declare_parameter(
            'eef_pose_topic', '/topic_arm_current_robot_eef_pose_0_306'
        )
        self.declare_parameter('robot_state_topic', '/robot_state_topic_0_306')
        self.declare_parameter('base_frame', 'Link_Zero_Point')
        self.declare_parameter('camera_frame', 'rgbd_head_color_optical_frame')
        self.declare_parameter('left_tcp_frame', 'left_grasp_tcp')
        self.declare_parameter('right_tcp_frame', 'right_grasp_tcp')
        self.declare_parameter('maximum_data_age', 1.0)
        self.declare_parameter('status_publish_period', 0.5)
        self.declare_parameter(
            'required_heartbeat_mask', DEFAULT_REQUIRED_HEARTBEAT_MASK
        )

        self._lock = threading.Lock()
        self._required_heartbeat_mask = int(
            self.get_parameter('required_heartbeat_mask').value
        )
        self._last_seen = {}
        self._camera_info = None
        self._joint_status_blockers = ['joint status has not been received']
        self._eef_pose_blockers = ['EEF pose has not been received']
        self._robot_state_blockers = ['robot protection state has not been received']
        self._robot_state_summary = {}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._status_pub = self.create_publisher(String, 'hardware_check_status', 10)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('joint_state_topic').value),
            self._joint_state_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('eef_pose_topic').value),
            self._eef_pose_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('robot_state_topic').value),
            self._robot_state_callback,
            10,
        )
        self.create_service(Trigger, 'check_grasp_hardware', self._check_service)
        self.create_timer(
            max(0.1, float(self.get_parameter('status_publish_period').value)),
            self._publish_status,
        )

    def _camera_info_callback(self, message):
        with self._lock:
            self._camera_info = message
        self._mark('camera_info')

    def _joint_state_callback(self, message):
        blockers = joint_status_blockers(message.data)
        with self._lock:
            self._joint_status_blockers = blockers
        if not blockers:
            self._mark('joint_state')

    def _eef_pose_callback(self, message):
        try:
            parse_eef_feedback(message.data)
            blockers = []
        except Exception as exc:
            blockers = [f'invalid dual-arm EEF feedback: {exc}']
        with self._lock:
            self._eef_pose_blockers = blockers
        if not blockers:
            self._mark('eef_pose')

    def _robot_state_callback(self, message):
        blockers = robot_state_blockers(
            message.data,
            required_heartbeat_mask=self._required_heartbeat_mask,
        )
        try:
            summary = robot_state_summary(message.data)
        except Exception:
            summary = {}
        with self._lock:
            self._robot_state_blockers = blockers
            self._robot_state_summary = summary
        if not blockers:
            self._mark('robot_state')

    def _mark(self, key):
        with self._lock:
            self._last_seen[key] = time.monotonic()

    def _check_service(self, _request, response):
        report = self.build_report()
        response.success = report['ready']
        response.message = json.dumps(report, ensure_ascii=False)
        return response

    def _publish_status(self):
        self._status_pub.publish(String(data=json.dumps(self.build_report())))

    def build_report(self):
        now = time.monotonic()
        max_age = float(self.get_parameter('maximum_data_age').value)
        with self._lock:
            last_seen = dict(self._last_seen)
            camera_info = self._camera_info
            joint_blockers = list(self._joint_status_blockers)
            eef_pose_blockers = list(self._eef_pose_blockers)
            robot_state_blockers_current = list(self._robot_state_blockers)
            robot_state_summary_current = dict(self._robot_state_summary)
        checks = {}
        for key in ('camera_info', 'joint_state', 'eef_pose', 'robot_state'):
            stamp = last_seen.get(key)
            age = None if stamp is None else now - stamp
            checks[key] = {
                'ok': age is not None and 0.0 <= age <= max_age,
                'age_seconds': None if age is None else round(age, 3),
            }
        checks['joint_state_content'] = {
            'ok': not joint_blockers,
            'blockers': joint_blockers,
        }
        checks['eef_pose_content'] = {
            'ok': not eef_pose_blockers,
            'blockers': eef_pose_blockers,
        }
        checks['robot_protection'] = {
            'ok': not robot_state_blockers_current,
            'blockers': robot_state_blockers_current,
            'state': robot_state_summary_current,
        }
        checks['camera_intrinsics'] = {
            'ok': bool(camera_info and camera_info.k[0] > 0.0 and camera_info.k[4] > 0.0),
            'fx': 0.0 if camera_info is None else camera_info.k[0],
            'fy': 0.0 if camera_info is None else camera_info.k[4],
        }
        base = str(self.get_parameter('base_frame').value)
        for name, parameter in (
            ('camera_tf', 'camera_frame'),
            ('left_tcp_tf', 'left_tcp_frame'),
            ('right_tcp_tf', 'right_tcp_frame'),
        ):
            target = str(self.get_parameter(parameter).value)
            ok = self._tf_buffer.can_transform(
                base, target, rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
            checks[name] = {'ok': bool(ok), 'from': target, 'to': base}
        ready = all(item['ok'] for item in checks.values())
        return {
            'ready': ready,
            'probe_ready': ready,
            # This probe is only one input to MotionExecutor. It must never
            # claim that dry-run, site acceptance or operator gates unlocked.
            'hardware_execution_locked': True,
            'hardware_execution_lock_note': (
                'probe readiness alone never unlocks hardware; MotionExecutor '
                'also enforces dry-run, site acceptance and operator gates'
            ),
            'checks': checks,
        }


def main(args=None):
    rclpy.init(args=args)
    node = HardwareProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
