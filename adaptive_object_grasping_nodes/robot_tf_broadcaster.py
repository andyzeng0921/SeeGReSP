import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from adaptive_object_grasping_nodes.motion_core import parse_eef_feedback
from adaptive_object_grasping_nodes.robot_geometry_core import (
    UrdfForwardKinematics,
    matrix_pose,
    parse_robot_joint_feedback,
)


class RobotTfBroadcaster(Node):
    def __init__(self):
        super().__init__('adaptive_grasp_robot_tf')
        self.declare_parameter(
            'robot_description_path',
            '/home/ubuntu/miniconda3/envs/robot_env/lib/python3.12/site-packages/'
            'autolife_robot_sdk/descriptions/autolife_s1/urdfs/robot_v2_2.urdf',
        )
        self.declare_parameter('joint_state_topic', '/topic_arm_whole_body_and_gripper_current_joints_status_0_283')
        self.declare_parameter('eef_pose_topic', '/topic_arm_current_robot_eef_pose_0_283')
        self.declare_parameter('base_frame', 'Link_Zero_Point')
        self.declare_parameter('camera_mount_frame', 'Link_Camera_Head_Forehead')
        self.declare_parameter('camera_frame', 'rgbd_head_color_optical_frame')
        self.declare_parameter('left_eef_frame', 'Link_Left_Wrist_Lower_to_Gripper')
        self.declare_parameter('right_eef_frame', 'Link_Right_Wrist_Lower_to_Gripper')
        self.declare_parameter('left_tcp_frame', 'left_grasp_tcp')
        self.declare_parameter('right_tcp_frame', 'right_grasp_tcp')
        self.declare_parameter('camera_mount_to_optical_translation', [0.0, 0.0, 0.0])
        self.declare_parameter('camera_mount_to_optical_quaternion', [0.0, 0.0, 0.7071067812, 0.7071067812])
        self.declare_parameter('left_tcp_translation', [0.20187, -0.00014, -0.03035])
        self.declare_parameter('right_tcp_translation', [0.20188, 0.0, -0.03035])
        self.declare_parameter('left_tcp_quaternion', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('right_tcp_quaternion', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('head_offsets_deg', [0.0, -1.0, 4.0])

        self._names = {
            'leg_waist': ['Joint_Ankle', 'Joint_Knee', 'Joint_Waist_Pitch', 'Joint_Waist_Yaw'],
            'left_arm': ['Joint_Left_Shoulder_Inner', 'Joint_Left_Shoulder_Outer', 'Joint_Left_UpperArm',
                         'Joint_Left_Elbow', 'Joint_Left_Forearm', 'Joint_Left_Wrist_Upper', 'Joint_Left_Wrist_Lower'],
            'right_arm': ['Joint_Right_Shoulder_Inner', 'Joint_Right_Shoulder_Outer', 'Joint_Right_UpperArm',
                          'Joint_Right_Elbow', 'Joint_Right_Forearm', 'Joint_Right_Wrist_Upper', 'Joint_Right_Wrist_Lower'],
            'neck': ['Joint_Neck_Roll', 'Joint_Neck_Pitch', 'Joint_Neck_Yaw'],
        }
        self._fk = UrdfForwardKinematics(str(self.get_parameter('robot_description_path').value))
        self._dynamic = TransformBroadcaster(self)
        self._static = StaticTransformBroadcaster(self)
        self._publish_static_transforms()
        self.create_subscription(String, str(self.get_parameter('joint_state_topic').value), self._on_joints, 10)
        self.create_subscription(String, str(self.get_parameter('eef_pose_topic').value), self._on_eef, 10)
        self.get_logger().info('dynamic camera, wrist and grasp TCP TF broadcaster ready')

    def _publish_static_transforms(self):
        transforms = [
            self._message(
                str(self.get_parameter('camera_mount_frame').value),
                str(self.get_parameter('camera_frame').value),
                self.get_parameter('camera_mount_to_optical_translation').value,
                self.get_parameter('camera_mount_to_optical_quaternion').value,
            ),
            self._message(
                str(self.get_parameter('left_eef_frame').value),
                str(self.get_parameter('left_tcp_frame').value),
                self.get_parameter('left_tcp_translation').value,
                self.get_parameter('left_tcp_quaternion').value,
            ),
            self._message(
                str(self.get_parameter('right_eef_frame').value),
                str(self.get_parameter('right_tcp_frame').value),
                self.get_parameter('right_tcp_translation').value,
                self.get_parameter('right_tcp_quaternion').value,
            ),
        ]
        self._static.sendTransform(transforms)

    def _on_joints(self, message):
        try:
            positions_deg = parse_robot_joint_feedback(message.data, self._names)
            offsets = self.get_parameter('head_offsets_deg').value
            for name, offset in zip(self._names['neck'], offsets):
                positions_deg[name] += float(offset)
            positions_rad = {name: math.radians(value) for name, value in positions_deg.items()}
            transform = self._fk.transform(
                str(self.get_parameter('base_frame').value),
                str(self.get_parameter('camera_mount_frame').value),
                positions_rad,
            )
            position, orientation = matrix_pose(transform)
            self._dynamic.sendTransform(self._message(
                str(self.get_parameter('base_frame').value),
                str(self.get_parameter('camera_mount_frame').value),
                position,
                orientation,
            ))
        except Exception as exc:
            self.get_logger().warning(f'camera TF update rejected: {exc}')

    def _on_eef(self, message):
        try:
            poses = parse_eef_feedback(message.data)
            transforms = []
            for arm in ('left', 'right'):
                pose = poses[arm]
                transforms.append(self._message(
                    str(self.get_parameter('base_frame').value),
                    str(self.get_parameter(f'{arm}_eef_frame').value),
                    pose['position'],
                    pose['orientation'],
                ))
            self._dynamic.sendTransform(transforms)
        except Exception as exc:
            self.get_logger().warning(f'wrist TF update rejected: {exc}')

    def _message(self, parent, child, position, orientation):
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x = float(position[0])
        message.transform.translation.y = float(position[1])
        message.transform.translation.z = float(position[2])
        message.transform.rotation.x = float(orientation[0])
        message.transform.rotation.y = float(orientation[1])
        message.transform.rotation.z = float(orientation[2])
        message.transform.rotation.w = float(orientation[3])
        return message


def main(args=None):
    rclpy.init(args=args)
    node = RobotTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
