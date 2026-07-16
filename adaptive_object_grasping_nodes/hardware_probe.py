import json
import threading

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


class HardwareProbe(Node):
    """Refuse hardware execution until camera, feedback topics and TF are present."""

    def __init__(self):
        super().__init__('adaptive_grasp_hardware_probe')
        self.declare_parameter('camera_info_topic', '/head_camera/color/camera_info')
        self.declare_parameter(
            'joint_state_topic',
            '/topic_arm_whole_body_and_gripper_current_joints_status_0_283',
        )
        self.declare_parameter(
            'eef_pose_topic', '/topic_arm_current_robot_eef_pose_0_283'
        )
        self.declare_parameter('base_frame', 'Link_Zero_Point')
        self.declare_parameter('camera_frame', 'rgbd_head_color_optical_frame')
        self.declare_parameter('left_tcp_frame', 'Link_Left_Wrist_Lower_to_Gripper')
        self.declare_parameter('right_tcp_frame', 'Link_Right_Wrist_Lower_to_Gripper')
        self.declare_parameter('maximum_data_age', 1.0)

        self._lock = threading.Lock()
        self._last_seen = {}
        self._camera_info = None
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
            lambda _: self._mark('joint_state'),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('eef_pose_topic').value),
            lambda _: self._mark('eef_pose'),
            10,
        )
        self.create_service(Trigger, 'check_grasp_hardware', self._check_service)
        self.create_timer(5.0, self._publish_status)

    def _camera_info_callback(self, message):
        with self._lock:
            self._camera_info = message
        self._mark('camera_info')

    def _mark(self, key):
        with self._lock:
            self._last_seen[key] = self.get_clock().now()

    def _check_service(self, _request, response):
        report = self.build_report()
        response.success = report['ready']
        response.message = json.dumps(report, ensure_ascii=False)
        return response

    def _publish_status(self):
        self._status_pub.publish(String(data=json.dumps(self.build_report())))

    def build_report(self):
        now = self.get_clock().now()
        max_age = float(self.get_parameter('maximum_data_age').value)
        with self._lock:
            last_seen = dict(self._last_seen)
            camera_info = self._camera_info
        checks = {}
        for key in ('camera_info', 'joint_state', 'eef_pose'):
            stamp = last_seen.get(key)
            age = None if stamp is None else (now - stamp).nanoseconds / 1e9
            checks[key] = {
                'ok': age is not None and age <= max_age,
                'age_seconds': None if age is None else round(age, 3),
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
        return {
            'ready': all(item['ok'] for item in checks.values()),
            'hardware_execution_locked': not all(item['ok'] for item in checks.values()),
            'checks': checks,
        }


def main(args=None):
    rclpy.init(args=args)
    node = HardwareProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
