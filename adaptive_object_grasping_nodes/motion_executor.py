import copy
import json
import threading
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from autolife_robot_srvs.srv import SetString
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from adaptive_object_grasping.srv import ExecuteCandidate
from adaptive_object_grasping_nodes.motion_core import (
    compose_vendor_rrt_target,
    parse_eef_feedback,
    pose_error,
    validate_pose,
    validate_vendor_trajectory,
    vendor_pose_payload,
    width_to_gripper_position,
)
from adaptive_object_grasping_nodes.robot_geometry_core import tcp_target_to_eef
from adaptive_object_grasping_nodes.robot_geometry_core import parse_robot_joint_feedback


class MotionExecutor(Node):
    def __init__(self):
        super().__init__('adaptive_grasp_motion_executor')
        self.declare_parameter('planning_backend', 'vendor_rrt')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('allow_hardware_execution', False)
        self.declare_parameter('require_hardware_ready', True)
        self.declare_parameter('base_frame', 'Link_Zero_Point')
        self.declare_parameter('topic_suffix', '0_283')
        self.declare_parameter('workspace_minimum', [0.10, -0.75, 0.20])
        self.declare_parameter('workspace_maximum', [0.85, 0.75, 1.45])
        self.declare_parameter('motion_timeout', 12.0)
        self.declare_parameter('position_tolerance', 0.025)
        self.declare_parameter('orientation_tolerance', 0.15)
        self.declare_parameter('maximum_gripper_width', 0.10)
        self.declare_parameter('gripper_open_position', 10.0)
        self.declare_parameter('gripper_closed_position', 330.0)
        self.declare_parameter('gripper_wait', 1.0)
        self.declare_parameter('lift_distance', 0.10)
        self.declare_parameter('left_tcp_translation', [0.20187, -0.00014, -0.03035])
        self.declare_parameter('right_tcp_translation', [0.20188, 0.0, -0.03035])
        self.declare_parameter('left_tcp_quaternion', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('right_tcp_quaternion', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('moveit_left_group', 'Left_Arm')
        self.declare_parameter('moveit_right_group', 'Right_Arm')
        self.declare_parameter('moveit_left_tip', 'Link_Left_Wrist_Lower_to_Gripper')
        self.declare_parameter('moveit_right_tip', 'Link_Right_Wrist_Lower_to_Gripper')
        self.declare_parameter('moveit_planning_time', 5.0)
        self.declare_parameter('moveit_planning_attempts', 5)
        self.declare_parameter('moveit_max_velocity_scaling', 0.25)
        self.declare_parameter('moveit_max_acceleration_scaling', 0.25)
        self.declare_parameter('enable_pbvs_hardware_follow', False)
        self.declare_parameter('pbvs_follow_backend', 'moveit_servo')
        self.declare_parameter('moveit_servo_pose_topic', '/servo_node/pose_target_cmds')
        self.declare_parameter('pbvs_command_rate', 10.0)
        self.declare_parameter('vendor_service_timeout', 15.0)
        self.declare_parameter('vendor_rrt_max_step_size', 0.06)
        self.declare_parameter('vendor_trajectory_duration', 4.0)
        self.declare_parameter('vendor_excluded_joints', ['Joint_Ankle', 'Joint_Knee'])
        self.declare_parameter('inactive_arm_position_tolerance', 0.06)
        self.declare_parameter('inactive_arm_orientation_tolerance', 0.30)
        self.declare_parameter('auto_prepare_hardware', True)
        self.declare_parameter('hardware_prepare_timeout', 18.0)
        self.declare_parameter('hardware_prepare_joint_tolerance_deg', 3.0)

        suffix = str(self.get_parameter('topic_suffix').value)
        self._condition = threading.Condition()
        self._current_poses = {}
        self._current_joints = None
        self._hardware_ready = False
        self._hardware_status_time = 0.0
        self._selected_arm = 'left'
        self._last_pbvs_command = 0.0
        self._moveit = None
        self._moveit_components = {}
        self._last_joint_feedback_time = 0.0
        self._joint_target_error_deg = None
        self._joint_names = {
            'leg_waist': ['Joint_Ankle', 'Joint_Knee', 'Joint_Waist_Pitch', 'Joint_Waist_Yaw'],
            'left_arm': ['Joint_Left_Shoulder_Inner', 'Joint_Left_Shoulder_Outer', 'Joint_Left_UpperArm',
                         'Joint_Left_Elbow', 'Joint_Left_Forearm', 'Joint_Left_Wrist_Upper', 'Joint_Left_Wrist_Lower'],
            'right_arm': ['Joint_Right_Shoulder_Inner', 'Joint_Right_Shoulder_Outer', 'Joint_Right_UpperArm',
                          'Joint_Right_Elbow', 'Joint_Right_Forearm', 'Joint_Right_Wrist_Upper', 'Joint_Right_Wrist_Lower'],
            'neck': ['Joint_Neck_Roll', 'Joint_Neck_Pitch', 'Joint_Neck_Yaw'],
        }
        callback_group = ReentrantCallbackGroup()

        self._vendor_pose_pub = self.create_publisher(
            String, f'/topic_arm_move_eef_pose_in_robot_frame_{suffix}', 10
        )
        self._gripper_pub = self.create_publisher(
            String, f'/topic_arm_gripper_target_joints_position_{suffix}', 10
        )
        self._vendor_trajectory_pub = self.create_publisher(
            String, f'/topic_arm_move_joints_trajectory_{suffix}', 10
        )
        self._ik_client = self.create_client(
            SetString, f'/service_arm_robot_inverse_kinematics_{suffix}'
        )
        self._fk_client = self.create_client(
            SetString, f'/service_arm_robot_forward_kinematics_{suffix}'
        )
        self._rrt_client = self.create_client(
            SetString, f'/service_arm_whole_body_joint_motion_planning_{suffix}'
        )
        self._servo_pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('moveit_servo_pose_topic').value), 10
        )
        self._status_pub = self.create_publisher(String, 'motion_execution_status', 10)
        self._joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._control_reset_pub = self.create_publisher(
            String, f'/control_reset_{suffix}', 10
        )
        self._joint_enable_pub = self.create_publisher(
            String, f'/topic_arm_joints_set_enable_state_{suffix}', 10
        )
        self.create_subscription(
            String,
            f'/topic_arm_current_robot_eef_pose_{suffix}',
            self._on_eef,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            String,
            f'/topic_arm_whole_body_and_gripper_current_joints_status_{suffix}',
            self._on_joints,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(String, 'hardware_check_status', self._on_hardware, 10)
        self.create_subscription(String, 'selected_arm', self._on_selected_arm, 10)
        self.create_subscription(PoseStamped, 'pbvs_target_pose', self._on_pbvs_pose, 10)
        self.create_service(
            ExecuteCandidate,
            'execute_grasp_candidate',
            self._execute_service,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            'prepare_grasp_hardware',
            self._prepare_hardware_service,
            callback_group=callback_group,
        )
        self.get_logger().warning(
            f'Motion executor ready: backend={self.get_parameter("planning_backend").value}, '
            f'dry_run={self.get_parameter("dry_run").value}, '
            f'hardware_unlock={self.get_parameter("allow_hardware_execution").value}'
        )

    def _on_eef(self, message):
        try:
            poses = parse_eef_feedback(message.data)
            with self._condition:
                self._current_poses.update(poses)
                self._condition.notify_all()
        except Exception as exc:
            self.get_logger().warning(f'EEF feedback rejected: {exc}')

    def _on_joints(self, message):
        try:
            parsed = parse_robot_joint_feedback(message.data, self._joint_names)
            payload = json.loads(message.data)
            target_errors = []
            for state_key, target_key in (
                ('left_arm_joint_state', 'left_arm_target_joint_state'),
                ('right_arm_joint_state', 'right_arm_target_joint_state'),
            ):
                current = payload.get(state_key, {}).get('position', [])
                target = payload.get(target_key, [])
                if len(current) == 7 and len(target) == 7:
                    target_errors.extend(abs(float(a) - float(b)) for a, b in zip(current, target))
            with self._condition:
                self._current_joints = {
                    group: [parsed[name] for name in joint_names]
                    for group, joint_names in self._joint_names.items()
                }
                self._last_joint_feedback_time = time.monotonic()
                self._joint_target_error_deg = max(target_errors) if target_errors else None
                self._condition.notify_all()
            self._publish_joint_state(parsed)
        except Exception as exc:
            self.get_logger().warning(f'joint feedback rejected: {exc}')

    def _publish_joint_state(self, parsed_degrees):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        for group in ('leg_waist', 'left_arm', 'right_arm', 'neck'):
            for name in self._joint_names[group]:
                joint_state.name.append(name)
                joint_state.position.append(float(parsed_degrees[name]) * 3.141592653589793 / 180.0)
        # The vendor feedback omits the planar base and gripper joints that are
        # present in the planning URDF. Keep them at a known state so MoveIt can
        # construct a complete current robot state.
        joint_state.name.extend([
            'Joint_Ground_Vehicle_X',
            'Joint_Ground_Vehicle_Y',
            'Joint_Ground_Vehicle_Z',
            'Joint_Left_Gripper',
            'Joint_Right_Gripper',
        ])
        joint_state.position.extend([0.0, 0.0, 0.0, 0.0, 0.0])
        self._joint_state_pub.publish(joint_state)

    def _prepare_hardware_service(self, _request, response):
        if not self._execution_unlocked():
            response.success = True
            response.message = 'hardware preparation skipped while execution is locked'
            return response
        if not bool(self.get_parameter('auto_prepare_hardware').value):
            response.success = True
            response.message = 'automatic hardware preparation disabled'
            return response

        self._publish_status('preparing_hardware', 'reset, enable and PID initialization')
        self._control_reset_pub.publish(String(data='{}'))
        time.sleep(1.5)
        enable = {
            name: True
            for group in ('left_arm', 'right_arm')
            for name in self._joint_names[group]
        }
        enable.update({'Joint_Left_Gripper': True, 'Joint_Right_Gripper': True})
        self._joint_enable_pub.publish(String(data=json.dumps(enable)))

        timeout = float(self.get_parameter('hardware_prepare_timeout').value)
        tolerance = float(self.get_parameter('hardware_prepare_joint_tolerance_deg').value)
        deadline = time.monotonic() + timeout
        stable_since = None
        with self._condition:
            while time.monotonic() < deadline:
                fresh = time.monotonic() - self._last_joint_feedback_time < 1.0
                error = self._joint_target_error_deg
                if fresh and error is not None and error <= tolerance:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= 1.0:
                        response.success = True
                        response.message = f'hardware prepared; maximum arm joint error={error:.2f}deg'
                        self._publish_status('hardware_ready', response.message)
                        return response
                else:
                    stable_since = None
                self._condition.wait(timeout=0.1)
        error = self._joint_target_error_deg
        response.success = False
        response.message = (
            'hardware preparation timed out; maximum arm joint error='
            + ('unknown' if error is None else f'{error:.2f}deg')
        )
        self._publish_status('failed', response.message)
        return response

    def _on_hardware(self, message):
        try:
            status = json.loads(message.data)
            self._hardware_ready = bool(status.get('ready', False))
            self._hardware_status_time = time.monotonic()
        except Exception:
            self._hardware_ready = False

    def _on_selected_arm(self, message):
        if message.data in ('left', 'right'):
            self._selected_arm = message.data

    def _on_pbvs_pose(self, message):
        if not bool(self.get_parameter('enable_pbvs_hardware_follow').value):
            return
        if not self._execution_unlocked():
            return
        now = time.monotonic()
        minimum_period = 1.0 / max(1.0, float(self.get_parameter('pbvs_command_rate').value))
        if now - self._last_pbvs_command < minimum_period:
            return
        self._last_pbvs_command = now
        backend = str(self.get_parameter('pbvs_follow_backend').value)
        if backend == 'moveit_servo':
            self._servo_pose_pub.publish(message)
        elif backend == 'vendor_task_space':
            self._publish_vendor_pose(self._selected_arm, message)

    def _execute_service(self, request, response):
        candidate = request.candidate
        error = self._validate_candidate(candidate)
        if error:
            response.message = error
            return response
        if not request.execute or bool(self.get_parameter('dry_run').value):
            try:
                backend = str(self.get_parameter('planning_backend').value)
                if backend == 'vendor_rrt':
                    self._validate_vendor_rrt_candidate(candidate)
                    response.message = (
                        'vendor IK/FK/RRT validated pregrasp, grasp and lift; '
                        'no hardware command sent'
                    )
                elif backend in ('moveit_py', 'moveit_py_vendor_execution'):
                    self._validate_moveit_candidate(candidate)
                    response.message = 'MoveIt2 planned pregrasp, grasp and lift; no hardware command sent'
                else:
                    response.message = 'candidate validated in dry-run; no hardware command sent'
                response.success = True
            except Exception as exc:
                response.message = f'dry-run planning failed: {exc}'
                self._publish_status('failed', response.message)
            return response
        if not self._execution_unlocked():
            response.message = 'hardware execution remains locked by configuration or hardware probe'
            return response
        try:
            backend = str(self.get_parameter('planning_backend').value)
            if backend == 'moveit_py':
                self._execute_moveit(candidate, execute_with_moveit=True)
            elif backend == 'moveit_py_vendor_execution':
                self._execute_moveit(candidate, execute_with_moveit=False)
            elif backend == 'vendor_rrt':
                self._execute_vendor_rrt(candidate)
            elif backend == 'vendor_task_space':
                self._execute_vendor(candidate)
            else:
                raise ValueError(f'unsupported planning_backend: {backend}')
            response.success = True
            response.message = f'grasp sequence completed using {backend}'
        except Exception as exc:
            response.message = f'grasp execution failed: {exc}'
            self._publish_status('failed', response.message)
        return response

    def _execution_unlocked(self):
        if bool(self.get_parameter('dry_run').value):
            return False
        if not bool(self.get_parameter('allow_hardware_execution').value):
            return False
        if bool(self.get_parameter('require_hardware_ready').value):
            return self._hardware_ready and time.monotonic() - self._hardware_status_time < 7.0
        return True

    def _validate_candidate(self, candidate):
        if candidate.arm not in ('left', 'right'):
            return 'candidate arm must be left or right'
        base_frame = str(self.get_parameter('base_frame').value)
        if candidate.grasp_pose.header.frame_id != base_frame:
            return f'candidate frame must be {base_frame}'
        lower = self.get_parameter('workspace_minimum').value
        upper = self.get_parameter('workspace_maximum').value
        for label, stamped in (
            ('pregrasp', candidate.pregrasp_pose),
            ('grasp', candidate.grasp_pose),
        ):
            pose = stamped.pose
            error = validate_pose(
                [pose.position.x, pose.position.y, pose.position.z],
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                lower,
                upper,
            )
            if error:
                return f'{label} pose rejected: {error}'
        if candidate.required_width > float(self.get_parameter('maximum_gripper_width').value):
            return 'required gripper width exceeds configured maximum'
        return ''

    def _execute_vendor(self, candidate):
        arm = candidate.arm
        self._publish_status('opening', arm)
        self._command_gripper(arm, float(self.get_parameter('gripper_open_position').value))
        time.sleep(float(self.get_parameter('gripper_wait').value))
        self._move_vendor_and_wait(arm, candidate.pregrasp_pose, 'pregrasp')
        self._move_vendor_and_wait(arm, candidate.grasp_pose, 'grasp')
        self._publish_status('closing', arm)
        position = width_to_gripper_position(
            candidate.required_width,
            float(self.get_parameter('maximum_gripper_width').value),
            float(self.get_parameter('gripper_open_position').value),
            float(self.get_parameter('gripper_closed_position').value),
        )
        self._command_gripper(arm, position)
        time.sleep(float(self.get_parameter('gripper_wait').value))
        lift = copy.deepcopy(candidate.grasp_pose)
        lift.pose.position.z += float(self.get_parameter('lift_distance').value)
        self._move_vendor_and_wait(arm, lift, 'lift')
        self._publish_status('completed', arm)

    def _execute_vendor_rrt(self, candidate):
        arm = candidate.arm
        self._publish_status('opening', arm)
        self._command_gripper(arm, float(self.get_parameter('gripper_open_position').value))
        time.sleep(float(self.get_parameter('gripper_wait').value))
        self._move_rrt_and_wait(arm, candidate.pregrasp_pose, 'pregrasp')
        self._move_rrt_and_wait(arm, candidate.grasp_pose, 'grasp')
        self._publish_status('closing', arm)
        position = width_to_gripper_position(
            candidate.required_width,
            float(self.get_parameter('maximum_gripper_width').value),
            float(self.get_parameter('gripper_open_position').value),
            float(self.get_parameter('gripper_closed_position').value),
        )
        self._command_gripper(arm, position)
        time.sleep(float(self.get_parameter('gripper_wait').value))
        lift = copy.deepcopy(candidate.grasp_pose)
        lift.pose.position.z += float(self.get_parameter('lift_distance').value)
        self._move_rrt_and_wait(arm, lift, 'lift')
        self._publish_status('completed', arm)

    def _move_rrt_and_wait(self, arm, pose, stage):
        target_position, target_orientation, trajectory = self._plan_vendor_rrt_pose(
            arm, pose, stage
        )
        self._publish_status(f'executing_{stage}', arm)
        self._vendor_trajectory_pub.publish(String(data=json.dumps({
            'traj_deg': trajectory,
            'duration': float(self.get_parameter('vendor_trajectory_duration').value),
            'excluded_joints_name': list(self.get_parameter('vendor_excluded_joints').value),
            'position_tolerance': 2.0,
        })))
        self._wait_for_eef(arm, target_position, target_orientation, stage)

    def _validate_vendor_rrt_candidate(self, candidate):
        lift = copy.deepcopy(candidate.grasp_pose)
        lift.pose.position.z += float(self.get_parameter('lift_distance').value)
        for stage, pose in (
            ('pregrasp', candidate.pregrasp_pose),
            ('grasp', candidate.grasp_pose),
            ('lift', lift),
        ):
            self._plan_vendor_rrt_pose(candidate.arm, pose, stage)
        self._publish_status('dry_run_planned', candidate.arm)

    def _plan_vendor_rrt_pose(self, arm, pose, stage):
        self._publish_status(f'ik_{stage}', arm)
        target_position, target_orientation = self._tcp_to_eef_pose_values(arm, pose)
        with self._condition:
            current_poses = copy.deepcopy(self._current_poses)
            current_joints = copy.deepcopy(self._current_joints)
        if not all(name in current_poses for name in ('left', 'right')) or current_joints is None:
            raise RuntimeError('current dual-arm pose and joint feedback are required for vendor planning')

        ik_payload = vendor_pose_payload(
            arm, target_position, target_orientation, current_poses
        )
        ik = self._call_vendor_service(self._ik_client, ik_payload, 'IK')
        left_body = [float(value) for value in ik['left_arm_body_target_joints']]
        right_body = [float(value) for value in ik['right_arm_body_target_joints']]
        neck_body = current_joints['leg_waist'] + current_joints['neck']
        fk = self._call_vendor_service(
            self._fk_client,
            {
                'left_arm_body_joints_deg': left_body,
                'right_arm_body_joints_deg': right_body,
                'neck_body_joints_deg': neck_body,
            },
            'FK',
        )
        self._validate_fk_solution(arm, fk, target_position, target_orientation, current_poses)

        q_end = compose_vendor_rrt_target(left_body, right_body)
        self._publish_status(f'planning_{stage}', arm)
        planned = self._call_vendor_service(
            self._rrt_client,
            {
                'max_step_size': float(self.get_parameter('vendor_rrt_max_step_size').value),
                'q_end': q_end,
            },
            'RRT',
        )
        trajectory = validate_vendor_trajectory(planned.get('trajectory'))
        return target_position, target_orientation, trajectory

    def _call_vendor_service(self, client, payload, label):
        timeout = float(self.get_parameter('vendor_service_timeout').value)
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            raise RuntimeError(f'vendor {label} service is unavailable')
        request = SetString.Request()
        request.data = json.dumps(payload)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise TimeoutError(f'vendor {label} service timed out after {timeout:.1f}s')
        response = future.result()
        if response is None or not response.success:
            detail = 'no response' if response is None else response.result
            raise RuntimeError(f'vendor {label} failed: {detail}')
        try:
            return json.loads(response.result)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f'vendor {label} returned invalid JSON') from exc

    def _validate_fk_solution(self, arm, fk, target_position, target_orientation, current_poses):
        selected = fk[f'{arm}_eef_pose']
        selected_pose = {
            'position': selected['position'],
            'orientation': selected['rotation'],
        }
        position_error, angle_error = pose_error(selected_pose, target_position, target_orientation)
        if position_error > float(self.get_parameter('position_tolerance').value) or angle_error > float(
            self.get_parameter('orientation_tolerance').value
        ):
            raise RuntimeError(
                f'vendor FK rejected IK for {arm}: position_error={position_error:.4f}, '
                f'orientation_error={angle_error:.4f}'
            )
        inactive = 'right' if arm == 'left' else 'left'
        inactive_fk = fk[f'{inactive}_eef_pose']
        inactive_pose = {
            'position': inactive_fk['position'],
            'orientation': inactive_fk['rotation'],
        }
        inactive_position_error, inactive_angle_error = pose_error(
            inactive_pose,
            current_poses[inactive]['position'],
            current_poses[inactive]['orientation'],
        )
        if inactive_position_error > float(self.get_parameter('inactive_arm_position_tolerance').value) or inactive_angle_error > float(
            self.get_parameter('inactive_arm_orientation_tolerance').value
        ):
            raise RuntimeError(
                f'vendor IK moves inactive {inactive} arm too far: '
                f'position_error={inactive_position_error:.4f}, '
                f'orientation_error={inactive_angle_error:.4f}'
            )

    def _wait_for_eef(self, arm, target_position, target_orientation, stage):
        timeout = float(self.get_parameter('motion_timeout').value)
        deadline = time.monotonic() + timeout
        last_position_error = None
        last_angle_error = None
        last_current = None
        with self._condition:
            while time.monotonic() < deadline:
                current = self._current_poses.get(arm)
                if current is not None:
                    position_error, angle_error = pose_error(current, target_position, target_orientation)
                    last_position_error = position_error
                    last_angle_error = angle_error
                    last_current = copy.deepcopy(current)
                    if position_error <= float(self.get_parameter('position_tolerance').value) and angle_error <= float(
                        self.get_parameter('orientation_tolerance').value
                    ):
                        return
                self._condition.wait(timeout=0.1)
        detail = f'{stage} pose did not converge within {timeout:.1f}s'
        if last_position_error is not None and last_angle_error is not None:
            detail += (
                f': position_error={last_position_error:.4f}m, '
                f'orientation_error={last_angle_error:.4f}rad, '
                f'target_position={[round(float(value), 4) for value in target_position]}, '
                f'current_position={[round(float(value), 4) for value in last_current["position"]]}'
            )
        raise TimeoutError(detail)

    def _move_vendor_and_wait(self, arm, pose, stage):
        self._publish_status(stage, arm)
        self._publish_vendor_pose(arm, pose)
        timeout = float(self.get_parameter('motion_timeout').value)
        deadline = time.monotonic() + timeout
        target_position, target_orientation = self._tcp_to_eef_pose_values(arm, pose)
        last_position_error = None
        last_angle_error = None
        last_current = None
        with self._condition:
            while time.monotonic() < deadline:
                current = self._current_poses.get(arm)
                if current is not None:
                    position_error, angle_error = pose_error(
                        current, target_position, target_orientation
                    )
                    last_position_error = position_error
                    last_angle_error = angle_error
                    last_current = copy.deepcopy(current)
                    if position_error <= float(self.get_parameter('position_tolerance').value) and angle_error <= float(
                        self.get_parameter('orientation_tolerance').value
                    ):
                        return
                self._condition.wait(timeout=0.1)
        detail = f'{stage} pose did not converge within {timeout:.1f}s'
        if last_position_error is not None and last_angle_error is not None:
            detail += (
                f': position_error={last_position_error:.4f}m, '
                f'orientation_error={last_angle_error:.4f}rad, '
                f'target_position={[round(float(value), 4) for value in target_position]}, '
                f'current_position={[round(float(value), 4) for value in last_current["position"]]}'
            )
        raise TimeoutError(detail)

    def _publish_vendor_pose(self, arm, pose):
        with self._condition:
            current = copy.deepcopy(self._current_poses)
        target_position, target_orientation = self._tcp_to_eef_pose_values(arm, pose)
        payload = vendor_pose_payload(
            arm,
            target_position,
            target_orientation,
            current,
        )
        self._vendor_pose_pub.publish(String(data=json.dumps(payload)))

    def _tcp_to_eef_pose_values(self, arm, pose):
        return tcp_target_to_eef(
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
            [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ],
            self.get_parameter(f'{arm}_tcp_translation').value,
            self.get_parameter(f'{arm}_tcp_quaternion').value,
        )

    def _validate_moveit_candidate(self, candidate):
        self._ensure_moveit()
        component = self._moveit_components[candidate.arm]
        tip = str(self.get_parameter(f'moveit_{candidate.arm}_tip').value)
        lift = copy.deepcopy(candidate.grasp_pose)
        lift.pose.position.z += float(self.get_parameter('lift_distance').value)
        for stage, pose in (
            ('pregrasp', candidate.pregrasp_pose),
            ('grasp', candidate.grasp_pose),
            ('lift', lift),
        ):
            result = self._plan_moveit(component, pose, tip, stage)
            self._moveit_result_to_vendor_trajectory(candidate.arm, result)
        self._publish_status('moveit_dry_run_planned', candidate.arm)

    def _execute_moveit(self, candidate, execute_with_moveit=False):
        self._ensure_moveit()
        component = self._moveit_components[candidate.arm]
        tip = str(self.get_parameter(f'moveit_{candidate.arm}_tip').value)
        self._command_gripper(candidate.arm, float(self.get_parameter('gripper_open_position').value))
        time.sleep(float(self.get_parameter('gripper_wait').value))
        self._plan_and_execute_moveit(component, candidate.arm, candidate.pregrasp_pose, tip, 'pregrasp', execute_with_moveit)
        self._plan_and_execute_moveit(component, candidate.arm, candidate.grasp_pose, tip, 'grasp', execute_with_moveit)
        close_position = width_to_gripper_position(
            candidate.required_width,
            float(self.get_parameter('maximum_gripper_width').value),
            float(self.get_parameter('gripper_open_position').value),
            float(self.get_parameter('gripper_closed_position').value),
        )
        self._command_gripper(candidate.arm, close_position)
        time.sleep(float(self.get_parameter('gripper_wait').value))
        lift = copy.deepcopy(candidate.grasp_pose)
        lift.pose.position.z += float(self.get_parameter('lift_distance').value)
        self._plan_and_execute_moveit(component, candidate.arm, lift, tip, 'lift', execute_with_moveit)

    def _ensure_moveit(self):
        if self._moveit is not None:
            return
        try:
            from moveit.planning import MoveItPy
        except ImportError as exc:
            raise RuntimeError(f'moveit_py import failed in motion executor environment: {exc}') from exc
        moveit_dir = Path(get_package_share_directory('adaptive_object_grasping')) / 'config' / 'moveit'
        config_dict = yaml.safe_load((moveit_dir / 'ompl_planning.yaml').read_text())
        config_dict.update({
            'robot_description': (moveit_dir / 'robot_v2_2.urdf').read_text(),
            'robot_description_semantic': (moveit_dir / 'autolife_s2.srdf').read_text(),
            'robot_description_kinematics': yaml.safe_load((moveit_dir / 'kinematics.yaml').read_text()),
        })
        self._moveit = MoveItPy(
            node_name='adaptive_grasp_moveit',
            config_dict=config_dict,
        )
        for arm in ('left', 'right'):
            group = str(self.get_parameter(f'moveit_{arm}_group').value)
            self._moveit_components[arm] = self._moveit.get_planning_component(group)

    def _plan_moveit(self, component, pose, tip, stage):
        self._publish_status(f'planning_{stage}', tip)
        component.set_start_state_to_current_state()
        component.set_goal_state(pose_stamped_msg=pose, pose_link=tip)
        try:
            from moveit.planning import PlanRequestParameters
            params = PlanRequestParameters(self._moveit, 'plan_request_params')
            params.planning_pipeline = 'ompl'
            params.planner_id = 'RRTConnectkConfigDefault'
            params.planning_time = float(self.get_parameter('moveit_planning_time').value)
            params.planning_attempts = int(self.get_parameter('moveit_planning_attempts').value)
            params.max_velocity_scaling_factor = float(self.get_parameter('moveit_max_velocity_scaling').value)
            params.max_acceleration_scaling_factor = float(self.get_parameter('moveit_max_acceleration_scaling').value)
            result = component.plan(single_plan_parameters=params)
        except Exception as exc:
            raise RuntimeError(f'MoveIt request setup or planning failed for {stage}: {exc}') from exc
        if not result:
            raise RuntimeError(f'MoveIt planning failed for {stage}')
        return result

    def _plan_and_execute_moveit(self, component, arm, pose, tip, stage, execute_with_moveit):
        result = self._plan_moveit(component, pose, tip, stage)
        if execute_with_moveit:
            self._moveit.execute(result.trajectory, controllers=[])
            return
        trajectory = self._moveit_result_to_vendor_trajectory(arm, result)
        self._publish_status(f'executing_{stage}', arm)
        self._vendor_trajectory_pub.publish(String(data=json.dumps({
            'traj_deg': trajectory,
            'duration': float(self.get_parameter('vendor_trajectory_duration').value),
            'excluded_joints_name': list(self.get_parameter('vendor_excluded_joints').value),
            'position_tolerance': 2.0,
        })))
        target_position, target_orientation = self._tcp_to_eef_pose_values(arm, pose)
        self._wait_for_eef(arm, target_position, target_orientation, stage)

    def _moveit_result_to_vendor_trajectory(self, arm, result):
        trajectory_message = result.trajectory.get_robot_trajectory_msg()
        joint_trajectory = trajectory_message.joint_trajectory
        if not joint_trajectory.points:
            raise RuntimeError('MoveIt2 returned an empty trajectory')
        names = list(joint_trajectory.joint_names)
        arm_group = 'left_arm' if arm == 'left' else 'right_arm'
        with self._condition:
            current_joints = copy.deepcopy(self._current_joints)
        if current_joints is None:
            raise RuntimeError('current joint feedback is required to execute MoveIt2 trajectory')
        trajectory = []
        for point in joint_trajectory.points:
            planned_degrees = dict(zip(names, [float(value) * 180.0 / 3.141592653589793 for value in point.positions]))
            leg_waist = list(current_joints['leg_waist'])
            left_arm = list(current_joints['left_arm'])
            right_arm = list(current_joints['right_arm'])
            target_arm = left_arm if arm == 'left' else right_arm
            for index, joint_name in enumerate(self._joint_names[arm_group]):
                if joint_name in planned_degrees:
                    target_arm[index] = planned_degrees[joint_name]
            trajectory.append([float(value) for value in (leg_waist + left_arm + right_arm)])
        return trajectory

    def _command_gripper(self, arm, position):
        key = f'{arm}_gripper_target_joints_position'
        self._gripper_pub.publish(String(data=json.dumps({key: [float(position)]})))

    def _publish_status(self, state, detail):
        self._status_pub.publish(String(data=json.dumps({'state': state, 'detail': detail})))


def main(args=None):
    rclpy.init(args=args)
    node = MotionExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
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
