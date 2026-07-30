import copy
import json
import math
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from adaptive_object_grasping.srv import ExecuteCandidate
from adaptive_object_grasping_nodes.dual_arm_core import (
    default_joint_groups,
    validate_joint_groups,
    vendor_trajectory_joint_names,
)
from adaptive_object_grasping_nodes.motion_core import (
    compose_vendor_rrt_target,
    motion_completion_timeout,
    parse_gripper_feedback,
    parse_eef_feedback,
    pose_error,
    resample_trajectory_for_uniform_timing,
    validate_gripper_width,
    validate_measurement_age,
    validate_pose,
    validate_vendor_trajectory,
    validated_gripper_command,
    vendor_gripper_to_urdf,
    vendor_pose_payload,
    width_to_gripper_position,
)
from adaptive_object_grasping_nodes.robot_geometry_core import tcp_target_to_eef
from adaptive_object_grasping_nodes.robot_geometry_core import parse_robot_joint_feedback
from adaptive_object_grasping_nodes.safety_core import (
    HARDWARE_EXECUTION_CONFIRMATION,
    execution_confirmation_valid,
    site_acceptance_blockers,
)


class MotionExecutor(Node):
    def __init__(self):
        super().__init__('adaptive_grasp_motion_executor')
        default_groups = default_joint_groups()
        self.declare_parameter(
            'planning_backend', 'moveit_py_vendor_execution'
        )
        self.declare_parameter('dry_run', True)
        self.declare_parameter('allow_hardware_execution', False)
        self.declare_parameter('require_hardware_ready', True)
        self.declare_parameter('hardware_status_maximum_age', 2.0)
        self.declare_parameter('require_site_acceptance', True)
        self.declare_parameter('site_acceptance_path', 'config/site_acceptance.yaml')
        self.declare_parameter(
            'hardware_execution_confirmation', HARDWARE_EXECUTION_CONFIRMATION
        )
        self.declare_parameter('base_frame', 'Link_Zero_Point')
        self.declare_parameter('topic_suffix', '0_306')
        self.declare_parameter('workspace_minimum', [0.10, -0.75, 0.20])
        self.declare_parameter('workspace_maximum', [0.85, 0.75, 1.45])
        self.declare_parameter('motion_timeout', 12.0)
        self.declare_parameter('motion_completion_margin', 3.0)
        self.declare_parameter('motion_settle_time', 0.30)
        self.declare_parameter('maximum_candidate_age', 1.0)
        self.declare_parameter('candidate_future_tolerance', 0.05)
        self.declare_parameter('position_tolerance', 0.025)
        self.declare_parameter('orientation_tolerance', 0.15)
        self.declare_parameter('maximum_gripper_width', 0.10)
        self.declare_parameter('grasp_width_scale', 0.5)
        self.declare_parameter('gripper_open_position', 10.0)
        self.declare_parameter('gripper_closed_position', 330.0)
        self.declare_parameter('gripper_feedback_open_position', 0.0)
        self.declare_parameter('gripper_feedback_closed_position', 360.0)
        self.declare_parameter('urdf_gripper_open_position', -1.333)
        self.declare_parameter('urdf_gripper_closed_position', 1.0)
        self.declare_parameter('gripper_wait', 1.0)
        self.declare_parameter('lift_distance', 0.10)
        self.declare_parameter('left_tcp_translation', [0.20187, -0.00014, -0.03035])
        self.declare_parameter('right_tcp_translation', [0.20188, 0.0, -0.03035])
        self.declare_parameter('left_tcp_quaternion', [-0.70710678, 0.0, 0.0, 0.70710678])
        self.declare_parameter('right_tcp_quaternion', [-0.70710678, 0.0, 0.0, 0.70710678])
        self.declare_parameter('moveit_left_group', 'Left_Arm')
        self.declare_parameter('moveit_right_group', 'Right_Arm')
        self.declare_parameter('moveit_dual_arm_group', 'Both_Arms')
        self.declare_parameter('moveit_left_tip', 'Link_Left_Wrist_Lower_to_Gripper')
        self.declare_parameter('moveit_right_tip', 'Link_Right_Wrist_Lower_to_Gripper')
        self.declare_parameter('moveit_planning_time', 5.0)
        self.declare_parameter('moveit_planning_attempts', 5)
        self.declare_parameter('moveit_max_velocity_scaling', 0.25)
        self.declare_parameter('moveit_max_acceleration_scaling', 0.25)
        self.declare_parameter('enable_pbvs_hardware_follow', False)
        self.declare_parameter('vendor_service_timeout', 15.0)
        self.declare_parameter('vendor_rrt_max_step_size', 0.06)
        self.declare_parameter('vendor_trajectory_duration', 4.0)
        self.declare_parameter('vendor_excluded_joints', ['Joint_Ankle', 'Joint_Knee'])
        self.declare_parameter('enable_joints_before_execute', True)
        self.declare_parameter('enable_pid_loop_before_execute', False)
        self.declare_parameter('inactive_arm_position_tolerance', 0.06)
        self.declare_parameter('inactive_arm_orientation_tolerance', 0.30)
        self.declare_parameter(
            'leg_waist_joint_names', default_groups['leg_waist']
        )
        self.declare_parameter('left_arm_joint_names', default_groups['left_arm'])
        self.declare_parameter('right_arm_joint_names', default_groups['right_arm'])
        self.declare_parameter('neck_joint_names', default_groups['neck'])

        suffix = str(self.get_parameter('topic_suffix').value)
        self._condition = threading.Condition()
        self._execution_lock = threading.Lock()
        self._command_state_lock = threading.Lock()
        self._active_hardware_execution = False
        self._unknown_motion_fault = None
        self._hardware_command_sequence = 0
        self._current_poses = {}
        self._eef_feedback_sequence = 0
        self._last_eef_feedback_time = 0.0
        self._current_joints = None
        self._current_grippers = None
        self._hardware_ready = False
        self._hardware_status_time = 0.0
        self._pbvs_unsupported_reported = False
        self._moveit = None
        self._moveit_components = {}
        self._last_joint_feedback_time = 0.0
        self._joint_target_error_deg = None
        self._target_joint_state = None

        # Cache frequently-accessed parameters to avoid repeated get_parameter calls.
        self._planning_backend = str(self.get_parameter('planning_backend').value)
        self._topic_suffix = suffix
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._allow_hardware_execution = bool(self.get_parameter('allow_hardware_execution').value)
        self._require_hardware_ready = bool(self.get_parameter('require_hardware_ready').value)
        self._hardware_status_maximum_age = float(
            self.get_parameter('hardware_status_maximum_age').value
        )
        self._require_site_acceptance = bool(
            self.get_parameter('require_site_acceptance').value
        )
        self._site_acceptance_path = str(
            self.get_parameter('site_acceptance_path').value
        )
        self._hardware_execution_confirmation = str(
            self.get_parameter('hardware_execution_confirmation').value
        )
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._workspace_min = list(self.get_parameter('workspace_minimum').value)
        self._workspace_max = list(self.get_parameter('workspace_maximum').value)
        self._motion_timeout = float(self.get_parameter('motion_timeout').value)
        self._motion_completion_margin = float(
            self.get_parameter('motion_completion_margin').value
        )
        self._motion_settle_time = float(
            self.get_parameter('motion_settle_time').value
        )
        if (
            not math.isfinite(self._motion_settle_time)
            or self._motion_settle_time < 0.0
        ):
            raise ValueError('motion_settle_time must be finite and non-negative')
        self._maximum_candidate_age = float(
            self.get_parameter('maximum_candidate_age').value
        )
        self._candidate_future_tolerance = float(
            self.get_parameter('candidate_future_tolerance').value
        )
        self._position_tolerance = float(self.get_parameter('position_tolerance').value)
        self._orientation_tolerance = float(self.get_parameter('orientation_tolerance').value)
        self._max_gripper_width = float(self.get_parameter('maximum_gripper_width').value)
        self._grasp_width_scale = float(self.get_parameter('grasp_width_scale').value)
        self._gripper_open_pos = float(self.get_parameter('gripper_open_position').value)
        self._gripper_closed_pos = float(self.get_parameter('gripper_closed_position').value)
        self._gripper_feedback_open_pos = float(
            self.get_parameter('gripper_feedback_open_position').value
        )
        self._gripper_feedback_closed_pos = float(
            self.get_parameter('gripper_feedback_closed_position').value
        )
        self._urdf_gripper_open_pos = float(
            self.get_parameter('urdf_gripper_open_position').value
        )
        self._urdf_gripper_closed_pos = float(
            self.get_parameter('urdf_gripper_closed_position').value
        )
        self._gripper_wait = float(self.get_parameter('gripper_wait').value)
        self._lift_distance = float(self.get_parameter('lift_distance').value)
        self._left_tcp_translation = list(self.get_parameter('left_tcp_translation').value)
        self._right_tcp_translation = list(self.get_parameter('right_tcp_translation').value)
        self._left_tcp_quaternion = list(self.get_parameter('left_tcp_quaternion').value)
        self._right_tcp_quaternion = list(self.get_parameter('right_tcp_quaternion').value)
        self._moveit_left_group = str(self.get_parameter('moveit_left_group').value)
        self._moveit_right_group = str(self.get_parameter('moveit_right_group').value)
        self._moveit_left_tip = str(self.get_parameter('moveit_left_tip').value)
        self._moveit_right_tip = str(self.get_parameter('moveit_right_tip').value)
        self._moveit_planning_time = float(self.get_parameter('moveit_planning_time').value)
        self._moveit_planning_attempts = int(self.get_parameter('moveit_planning_attempts').value)
        self._moveit_max_velocity_scaling = float(self.get_parameter('moveit_max_velocity_scaling').value)
        self._moveit_max_acceleration_scaling = float(self.get_parameter('moveit_max_acceleration_scaling').value)
        self._enable_pbvs_follow = bool(self.get_parameter('enable_pbvs_hardware_follow').value)
        self._vendor_service_timeout = float(self.get_parameter('vendor_service_timeout').value)
        self._vendor_rrt_max_step = float(self.get_parameter('vendor_rrt_max_step_size').value)
        self._vendor_trajectory_duration = float(self.get_parameter('vendor_trajectory_duration').value)
        self._vendor_excluded_joints = list(self.get_parameter('vendor_excluded_joints').value)
        self._enable_joints_before_exec = bool(self.get_parameter('enable_joints_before_execute').value)
        self._enable_pid_loop = bool(self.get_parameter('enable_pid_loop_before_execute').value)
        self._inactive_pos_tol = float(self.get_parameter('inactive_arm_position_tolerance').value)
        self._inactive_orient_tol = float(self.get_parameter('inactive_arm_orientation_tolerance').value)

        self._joint_names = validate_joint_groups({
            'leg_waist': list(self.get_parameter('leg_waist_joint_names').value),
            'left_arm': list(self.get_parameter('left_arm_joint_names').value),
            'right_arm': list(self.get_parameter('right_arm_joint_names').value),
            'neck': list(self.get_parameter('neck_joint_names').value),
        })
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
        self._target_joints_pub = self.create_publisher(
            String, f'/topic_arm_whole_body_target_joints_position_{suffix}', 10
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
        self._status_pub = self.create_publisher(String, 'motion_execution_status', 10)
        self._joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        target_state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._target_joint_state_pub = self.create_publisher(
            JointState, '/adaptive_grasp/target_joint_states', target_state_qos
        )
        self._joint_enable_pub = self.create_publisher(
            String, f'/topic_arm_joints_set_enable_state_{suffix}', 10
        )
        self._joint_clear_error_pub = self.create_publisher(
            String, f'/topic_arm_joints_clear_error_{suffix}', 10
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
        self.create_subscription(PoseStamped, 'pbvs_target_pose', self._on_pbvs_pose, 10)
        self.create_service(
            ExecuteCandidate,
            'execute_grasp_candidate',
            self._execute_service,
            callback_group=callback_group,
        )
        self.create_timer(0.5, self._republish_target_joint_state)
        self.get_logger().warning(
            f'Motion executor ready: backend={self._planning_backend}, '
            f'dry_run={self._dry_run}, '
            f'hardware_unlock={self._allow_hardware_execution}'
        )

    def _on_eef(self, message):
        try:
            poses = parse_eef_feedback(message.data)
            with self._condition:
                # The parser requires one complete, finite dual-arm snapshot.
                # Replace the snapshot atomically and advance a local sequence so
                # a command can never be acknowledged by pre-command feedback.
                self._current_poses = poses
                self._eef_feedback_sequence += 1
                self._last_eef_feedback_time = time.monotonic()
                self._condition.notify_all()
        except Exception as exc:
            self.get_logger().warning(f'EEF feedback rejected: {exc}')

    def _on_joints(self, message):
        try:
            parsed = parse_robot_joint_feedback(message.data, self._joint_names)
            payload = json.loads(message.data)
            grippers = parse_gripper_feedback(payload)
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
                self._current_grippers = grippers
                self._last_joint_feedback_time = time.monotonic()
                self._joint_target_error_deg = max(target_errors) if target_errors else None
                self._condition.notify_all()
            self._publish_joint_state(parsed, grippers)
        except Exception as exc:
            self.get_logger().warning(f'joint feedback rejected: {exc}')

    def _publish_joint_state(self, parsed_degrees, grippers):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        for group in ('leg_waist', 'left_arm', 'right_arm', 'neck'):
            for name in self._joint_names[group]:
                joint_state.name.append(name)
                joint_state.position.append(float(parsed_degrees[name]) * 3.141592653589793 / 180.0)
        # The vendor feedback omits the planar base joints. Gripper feedback is
        # mapped through an explicit vendor-to-URDF calibration instead of being
        # falsely published as zero.
        joint_state.name.extend([
            'Joint_Ground_Vehicle_X',
            'Joint_Ground_Vehicle_Y',
            'Joint_Ground_Vehicle_Z',
            'Joint_Left_Gripper',
            'Joint_Right_Gripper',
        ])
        joint_state.position.extend([
            0.0,
            0.0,
            0.0,
            self._gripper_to_urdf(grippers['left']),
            self._gripper_to_urdf(grippers['right']),
        ])
        self._joint_state_pub.publish(joint_state)

    def _publish_target_joint_state(self, planned_radians):
        with self._condition:
            current_joints = copy.deepcopy(self._current_joints)
            current_grippers = copy.deepcopy(self._current_grippers)
        if current_joints is None:
            raise RuntimeError('current joint feedback is required to publish target robot state')
        if current_grippers is None:
            raise RuntimeError('current gripper feedback is required to publish target robot state')
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        for group in ('leg_waist', 'left_arm', 'right_arm', 'neck'):
            for name, current_degrees in zip(self._joint_names[group], current_joints[group]):
                joint_state.name.append(name)
                joint_state.position.append(
                    float(planned_radians.get(
                        name,
                        float(current_degrees) * 3.141592653589793 / 180.0,
                    ))
                )
        joint_state.name.extend([
            'Joint_Ground_Vehicle_X',
            'Joint_Ground_Vehicle_Y',
            'Joint_Ground_Vehicle_Z',
            'Joint_Left_Gripper',
            'Joint_Right_Gripper',
        ])
        joint_state.position.extend([
            0.0,
            0.0,
            0.0,
            self._gripper_to_urdf(current_grippers['left']),
            self._gripper_to_urdf(current_grippers['right']),
        ])
        with self._condition:
            self._target_joint_state = copy.deepcopy(joint_state)
        self._target_joint_state_pub.publish(joint_state)

    def _gripper_to_urdf(self, vendor_position):
        return vendor_gripper_to_urdf(
            vendor_position,
            self._gripper_feedback_open_pos,
            self._gripper_feedback_closed_pos,
            self._urdf_gripper_open_pos,
            self._urdf_gripper_closed_pos,
        )

    def _republish_target_joint_state(self):
        with self._condition:
            joint_state = copy.deepcopy(self._target_joint_state)
        if joint_state is None:
            return
        joint_state.header.stamp = self.get_clock().now().to_msg()
        self._target_joint_state_pub.publish(joint_state)

    def _on_hardware(self, message):
        try:
            status = json.loads(message.data)
            self._hardware_ready = bool(status.get('ready', False))
            self._hardware_status_time = time.monotonic()
        except Exception:
            self._hardware_ready = False

    def _on_pbvs_pose(self, _message):
        if not self._enable_pbvs_follow:
            return
        # No verified controller-ownership or stop contract exists for PBVS.
        # Never make this subscription a second writer to hardware topics.
        if not self._pbvs_unsupported_reported:
            self._pbvs_unsupported_reported = True
            self.get_logger().error(
                'PBVS hardware follow is configured but unsupported; '
                'all PBVS hardware commands are dropped'
            )

    def _execute_service(self, request, response):
        if not self._execution_lock.acquire(blocking=False):
            response.message = 'motion executor is busy with another planning or execution request'
            return response
        try:
            return self._execute_service_locked(request, response)
        finally:
            self._execution_lock.release()

    def _execute_service_locked(self, request, response):
        candidate = request.candidate
        error = self._validate_candidate(candidate)
        if error:
            response.message = error
            return response
        if not request.execute:
            try:
                if self._planning_backend == 'vendor_rrt':
                    self._validate_vendor_rrt_candidate(candidate)
                    response.message = (
                        'vendor IK/FK/RRT independently validated pregrasp, grasp '
                        'and lift from the current state; no continuous execution '
                        'path was proven and no hardware command was sent'
                    )
                elif self._planning_backend in ('moveit_py', 'moveit_py_vendor_execution'):
                    self._validate_moveit_candidate(candidate)
                    response.message = (
                        'MoveIt2 independently planned pregrasp, grasp and lift '
                        'from the current state; no continuous execution path was '
                        'proven and no hardware command was sent'
                    )
                else:
                    response.message = 'candidate validated in dry-run; no hardware command sent'
                response.success = True
            except Exception as exc:
                response.message = f'dry-run planning failed: {exc}'
                self._publish_status('failed', response.message)
            return response
        if self._dry_run:
            response.message = (
                'hardware execution rejected: execute=true while dry_run is enabled'
            )
            return response
        confirmation = getattr(request, 'execution_confirmation', '')
        if not execution_confirmation_valid(
            confirmation, self._hardware_execution_confirmation
        ):
            response.message = (
                'hardware execution requires the exact operator confirmation token'
            )
            return response
        blockers = self._execution_blockers()
        if blockers:
            response.message = 'hardware execution remains locked: ' + '; '.join(blockers)
            return response
        with self._command_state_lock:
            starting_command_sequence = self._hardware_command_sequence
        self._active_hardware_execution = True
        try:
            self._enable_execution_joints(candidate.arm)
            backend = self._planning_backend
            if backend == 'moveit_py_vendor_execution':
                self._execute_moveit(candidate, execute_with_moveit=False)
            else:
                raise RuntimeError(
                    f'hardware backend {backend!r} is unsupported; only '
                    'moveit_py_vendor_execution may reach the vendor trajectory adapter'
                )
            response.success = True
            response.message = f'grasp sequence completed using {backend}'
        except Exception as exc:
            if self._hardware_commands_issued_after(starting_command_sequence):
                self._latch_unknown_motion_fault(
                    f'hardware command outcome became unknown after: {exc}'
                )
            fault = self._unknown_motion_fault_reason()
            response.message = f'grasp execution failed: {exc}'
            if fault:
                response.message += f'; executor fault latched: {fault}'
            self._publish_status('failed', response.message)
        finally:
            self._active_hardware_execution = False
        return response

    def _record_hardware_command(self):
        """Record intent immediately before a fire-and-forget vendor publish."""

        with self._command_state_lock:
            self._hardware_command_sequence += 1
            return self._hardware_command_sequence

    def _hardware_commands_issued_after(self, sequence):
        with self._command_state_lock:
            return self._hardware_command_sequence > int(sequence)

    def _unknown_motion_fault_reason(self):
        with self._command_state_lock:
            return self._unknown_motion_fault

    def _latch_unknown_motion_fault(self, reason):
        detail = str(reason).strip() or 'hardware command outcome is unknown'
        newly_latched = False
        with self._command_state_lock:
            if self._unknown_motion_fault is None:
                self._unknown_motion_fault = detail
                newly_latched = True
            latched = self._unknown_motion_fault
        if newly_latched:
            self._publish_status(
                'unknown_motion_fault_latched',
                latched
                + '; restart only after physical inspection and emergency-stop readiness',
            )
        return latched

    def _wait_with_execution_health(self, duration):
        duration = float(duration)
        if not math.isfinite(duration) or duration < 0.0:
            raise ValueError('execution wait duration must be finite and non-negative')
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._assert_execution_health()
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))

    def _enable_execution_joints(self, arm):
        self._assert_execution_health()
        if not self._enable_joints_before_exec:
            return
        with self._condition:
            current_joints = copy.deepcopy(self._current_joints)
        if current_joints is not None:
            self._publish_status('syncing_current_joint_targets', arm)
            self._assert_execution_health()
            self._record_hardware_command()
            self._target_joints_pub.publish(String(data=json.dumps({
                'leg_waist_target_joints_position': [float(value) for value in current_joints['leg_waist']],
                'left_arm_target_joints_position': [float(value) for value in current_joints['left_arm']],
                'right_arm_target_joints_position': [float(value) for value in current_joints['right_arm']],
            })))
            self._wait_with_execution_health(0.3)

        groups = [arm + '_arm']
        joint_names = [
            name
            for group in groups
            for name in self._joint_names[group]
        ]
        gripper_name = f'Joint_{arm.capitalize()}_Gripper'
        clear_names = list(joint_names)
        enable = {name: True for name in joint_names}
        enable[gripper_name] = True
        self._publish_status('clearing_joint_errors', arm)
        self._assert_execution_health()
        self._record_hardware_command()
        self._joint_clear_error_pub.publish(String(data=json.dumps(clear_names)))
        self._wait_with_execution_health(0.3)
        self._publish_status('enabling_joints', arm)
        self._assert_execution_health()
        self._record_hardware_command()
        self._joint_enable_pub.publish(String(data=json.dumps(enable)))
        self._wait_with_execution_health(0.8)
    def _execution_unlocked(self):
        return not self._execution_blockers()

    def _execution_blockers(self):
        blockers = []
        fault = self._unknown_motion_fault_reason()
        if fault:
            blockers.append(
                'unknown-motion fault is latched: '
                + fault
                + '; physical inspection and process restart are required'
            )
        if self._dry_run:
            blockers.append('dry_run is enabled')
        if not self._allow_hardware_execution:
            blockers.append('allow_hardware_execution is false')
        if self._enable_pbvs_follow:
            blockers.append(
                'PBVS hardware follow is unsupported because controller ownership '
                'and stop semantics are not verified'
            )
        if self._planning_backend != 'moveit_py_vendor_execution':
            blockers.append(
                f'hardware backend {self._planning_backend!r} is unsupported; '
                'vendor_rrt freezes joints used by its planner and '
                'vendor_task_space has no verified collision guarantee'
            )
        if self._enable_pid_loop:
            blockers.append(
                'direct vendor PID shared-memory control is unsupported; '
                'use the documented vendor startup procedure'
            )
        if self._require_hardware_ready:
            if not self._hardware_probe_ready():
                blockers.append('hardware probe is not ready or its status is stale')
        if self._require_site_acceptance:
            blockers.extend(self._site_acceptance_file_blockers())
        return blockers

    def _site_acceptance_file_blockers(self):
        configured = Path(self._site_acceptance_path).expanduser()
        if configured.is_absolute():
            path = configured
        else:
            package_share = Path(
                get_package_share_directory('adaptive_object_grasping')
            ).resolve()
            path = (package_share / configured).resolve()
            try:
                path.relative_to(package_share)
            except ValueError:
                return [
                    'relative site acceptance path escapes the installed package share'
                ]
        try:
            data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        except Exception as exc:
            return [f'cannot read site acceptance file {path}: {exc}']
        return [
            f'site acceptance: {blocker}'
            for blocker in site_acceptance_blockers(data)
        ]

    def _hardware_probe_ready(self):
        return (
            self._hardware_ready
            and time.monotonic() - self._hardware_status_time
            <= self._hardware_status_maximum_age
        )

    def _assert_execution_health(self):
        fault = self._unknown_motion_fault_reason()
        if fault:
            raise RuntimeError(
                'unknown-motion fault is latched; no further hardware command is allowed: '
                + fault
            )
        if (
            self._active_hardware_execution
            and self._require_hardware_ready
            and not self._hardware_probe_ready()
        ):
            self._publish_status(
                'safety_watchdog_blocked',
                'hardware probe became unsafe or stale; no further command will be sent',
            )
            raise RuntimeError(
                'hardware safety watchdog became unsafe or stale; '
                'use the physical emergency stop if the robot is still moving'
            )

    def _validate_candidate(self, candidate):
        if candidate.arm not in ('left', 'right'):
            return 'candidate arm must be left or right'
        if (
            candidate.header.frame_id != self._base_frame
            or candidate.grasp_pose.header.frame_id != self._base_frame
            or candidate.pregrasp_pose.header.frame_id != self._base_frame
        ):
            return f'candidate frame must be {self._base_frame}'
        candidate_stamp = (
            int(candidate.header.stamp.sec),
            int(candidate.header.stamp.nanosec),
        )
        for label, stamped in (
            ('pregrasp', candidate.pregrasp_pose),
            ('grasp', candidate.grasp_pose),
        ):
            pose_stamp = (
                int(stamped.header.stamp.sec),
                int(stamped.header.stamp.nanosec),
            )
            if pose_stamp != candidate_stamp:
                return (
                    f'{label} timestamp must exactly match the candidate '
                    'measurement timestamp'
                )
        stamp_seconds = (
            float(candidate.header.stamp.sec)
            + float(candidate.header.stamp.nanosec) / 1_000_000_000.0
        )
        now_seconds = self.get_clock().now().nanoseconds / 1_000_000_000.0
        age_error = validate_measurement_age(
            now_seconds,
            stamp_seconds,
            self._maximum_candidate_age,
            self._candidate_future_tolerance,
        )
        if age_error:
            return age_error
        if not math.isfinite(float(candidate.score)):
            return 'candidate score must be finite'
        lift = copy.deepcopy(candidate.grasp_pose)
        lift.pose.position.z += self._lift_distance
        for label, stamped in (
            ('pregrasp', candidate.pregrasp_pose),
            ('grasp', candidate.grasp_pose),
            ('lift', lift),
        ):
            pose = stamped.pose
            error = validate_pose(
                [pose.position.x, pose.position.y, pose.position.z],
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                self._workspace_min,
                self._workspace_max,
            )
            if error:
                return f'{label} pose rejected: {error}'
        width_error = validate_gripper_width(
            candidate.required_width,
            self._max_gripper_width,
            self._grasp_width_scale,
        )
        if width_error:
            return width_error
        return ''

    def _execute_vendor(self, candidate):
        arm = candidate.arm
        self._publish_status('opening', arm)
        self._command_gripper(arm, self._gripper_open_pos)
        self._wait_with_execution_health(self._gripper_wait)
        self._move_vendor_and_wait(arm, candidate.pregrasp_pose, 'pregrasp')
        self._move_vendor_and_wait(arm, candidate.grasp_pose, 'grasp')
        self._publish_status('closing', arm)
        position = width_to_gripper_position(
            candidate.required_width,
            self._max_gripper_width,
            self._gripper_open_pos,
            self._gripper_closed_pos,
            self._grasp_width_scale,
        )
        self._command_gripper(arm, position)
        self._wait_with_execution_health(self._gripper_wait)
        lift = copy.deepcopy(candidate.grasp_pose)
        lift.pose.position.z += self._lift_distance
        self._move_vendor_and_wait(arm, lift, 'lift')
        self._publish_status('completed', arm)

    def _execute_vendor_rrt(self, candidate):
        arm = candidate.arm
        self._publish_status('opening', arm)
        self._command_gripper(arm, self._gripper_open_pos)
        self._wait_with_execution_health(self._gripper_wait)
        self._move_rrt_and_wait(arm, candidate.pregrasp_pose, 'pregrasp')
        self._move_rrt_and_wait(arm, candidate.grasp_pose, 'grasp')
        self._publish_status('closing', arm)
        position = width_to_gripper_position(
            candidate.required_width,
            self._max_gripper_width,
            self._gripper_open_pos,
            self._gripper_closed_pos,
            self._grasp_width_scale,
        )
        self._command_gripper(arm, position)
        self._wait_with_execution_health(self._gripper_wait)
        lift = copy.deepcopy(candidate.grasp_pose)
        lift.pose.position.z += self._lift_distance
        self._move_rrt_and_wait(arm, lift, 'lift')
        self._publish_status('completed', arm)

    def _move_rrt_and_wait(self, arm, pose, stage):
        target_position, target_orientation, trajectory = self._plan_vendor_rrt_pose(
            arm, pose, stage
        )
        self._assert_execution_health()
        self._publish_status(f'executing_{stage}', arm)
        message = String(data=json.dumps({
            'traj_deg': trajectory,
            'duration': self._vendor_trajectory_duration,
            'excluded_joints_name': self._vendor_excluded_joints,
            'position_tolerance': 2.0,
        }))
        with self._condition:
            command_feedback_sequence = self._eef_feedback_sequence
            self._record_hardware_command()
            self._vendor_trajectory_pub.publish(message)
        self._wait_for_eef(
            arm,
            target_position,
            target_orientation,
            stage,
            command_feedback_sequence,
            self._vendor_trajectory_duration,
        )

    def _validate_vendor_rrt_candidate(self, candidate):
        lift = copy.deepcopy(candidate.grasp_pose)
        lift.pose.position.z += self._lift_distance
        grasp_trajectory = None
        for stage, pose in (
            ('pregrasp', candidate.pregrasp_pose),
            ('grasp', candidate.grasp_pose),
            ('lift', lift),
        ):
            _, _, trajectory = self._plan_vendor_rrt_pose(candidate.arm, pose, stage)
            if stage == 'grasp':
                grasp_trajectory = trajectory
        if grasp_trajectory:
            final = grasp_trajectory[-1]
            names = vendor_trajectory_joint_names(self._joint_names)
            if len(final) != len(names):
                raise RuntimeError(
                    f'vendor target trajectory has {len(final)} joints, expected {len(names)}'
                )
            self._publish_target_joint_state({
                name: float(value) * 3.141592653589793 / 180.0
                for name, value in zip(names, final)
            })
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
                'max_step_size': self._vendor_rrt_max_step,
                'q_end': q_end,
            },
            'RRT',
        )
        trajectory = validate_vendor_trajectory(planned.get('trajectory'))
        return target_position, target_orientation, trajectory

    def _call_vendor_service(self, client, payload, label):
        if not client.wait_for_service(timeout_sec=min(self._vendor_service_timeout, 2.0)):
            raise RuntimeError(f'vendor {label} service is unavailable')
        request = SetString.Request()
        request.data = json.dumps(payload)
        future = client.call_async(request)
        deadline = time.monotonic() + self._vendor_service_timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise TimeoutError(f'vendor {label} service timed out after {self._vendor_service_timeout:.1f}s')
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
        if position_error > self._position_tolerance or angle_error > self._orientation_tolerance:
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
        if inactive_position_error > self._inactive_pos_tol or inactive_angle_error > self._inactive_orient_tol:
            raise RuntimeError(
                f'vendor IK moves inactive {inactive} arm too far: '
                f'position_error={inactive_position_error:.4f}, '
                f'orientation_error={inactive_angle_error:.4f}'
            )

    def _wait_for_eef(
        self,
        arm,
        target_position,
        target_orientation,
        stage,
        command_feedback_sequence,
        commanded_duration,
    ):
        self._wait_for_eef_convergence(
            arm,
            target_position,
            target_orientation,
            stage,
            command_feedback_sequence,
            commanded_duration,
        )

    def _wait_for_eef_convergence(
        self,
        arm,
        target_position,
        target_orientation,
        stage,
        command_feedback_sequence,
        commanded_duration,
    ):
        timeout = motion_completion_timeout(
            self._motion_timeout,
            commanded_duration,
            self._motion_completion_margin,
        )
        deadline = time.monotonic() + timeout
        last_position_error = None
        last_angle_error = None
        last_current = None
        saw_post_command_feedback = False
        last_eef_sequence = int(command_feedback_sequence)
        within_tolerance_since = None
        with self._condition:
            while time.monotonic() < deadline:
                self._assert_execution_health()
                if self._eef_feedback_sequence > last_eef_sequence:
                    last_eef_sequence = self._eef_feedback_sequence
                    saw_post_command_feedback = True
                    current = self._current_poses.get(arm)
                    position_error, angle_error = pose_error(current, target_position, target_orientation)
                    last_position_error = position_error
                    last_angle_error = angle_error
                    last_current = copy.deepcopy(current)
                    if (
                        position_error <= self._position_tolerance
                        and angle_error <= self._orientation_tolerance
                    ):
                        now = time.monotonic()
                        if within_tolerance_since is None:
                            within_tolerance_since = now
                        elif (
                            now - within_tolerance_since
                            >= self._motion_settle_time
                        ):
                            return
                    else:
                        within_tolerance_since = None
                self._condition.wait(timeout=0.1)
        detail = f'{stage} pose did not converge within {timeout:.1f}s'
        if not saw_post_command_feedback:
            detail += '; no valid dual-arm EEF feedback arrived after the command'
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
        command_feedback_sequence = self._publish_vendor_pose(arm, pose)
        target_position, target_orientation = self._tcp_to_eef_pose_values(arm, pose)
        self._wait_for_eef_convergence(
            arm,
            target_position,
            target_orientation,
            stage,
            command_feedback_sequence,
            self._vendor_trajectory_duration,
        )

    def _publish_vendor_pose(self, arm, pose):
        self._assert_execution_health()
        with self._condition:
            current = copy.deepcopy(self._current_poses)
        target_position, target_orientation = self._tcp_to_eef_pose_values(arm, pose)
        payload = vendor_pose_payload(
            arm,
            target_position,
            target_orientation,
            current,
        )
        with self._condition:
            command_feedback_sequence = self._eef_feedback_sequence
            self._record_hardware_command()
            self._vendor_pose_pub.publish(String(data=json.dumps(payload)))
        return command_feedback_sequence

    def _tcp_to_eef_pose_values(self, arm, pose):
        tcp_translation = self._left_tcp_translation if arm == 'left' else self._right_tcp_translation
        tcp_quaternion = self._left_tcp_quaternion if arm == 'left' else self._right_tcp_quaternion
        return tcp_target_to_eef(
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
            [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ],
            tcp_translation,
            tcp_quaternion,
        )

    def _validate_moveit_candidate(self, candidate):
        self._ensure_moveit()
        component = self._moveit_components[candidate.arm]
        tip = self._moveit_left_tip if candidate.arm == 'left' else self._moveit_right_tip
        lift = copy.deepcopy(candidate.grasp_pose)
        lift.pose.position.z += self._lift_distance
        grasp_result = None
        for stage, pose in (
            ('pregrasp', candidate.pregrasp_pose),
            ('grasp', candidate.grasp_pose),
            ('lift', lift),
        ):
            result = self._plan_moveit(
                component, self._tcp_pose_to_eef_pose(candidate.arm, pose), tip, stage
            )
            self._moveit_result_to_vendor_trajectory(candidate.arm, result)
            if stage == 'grasp':
                grasp_result = result
        if grasp_result is not None:
            trajectory = grasp_result.trajectory.get_robot_trajectory_msg().joint_trajectory
            if not trajectory.points:
                raise RuntimeError('MoveIt2 returned an empty grasp trajectory')
            self._publish_target_joint_state(dict(zip(
                trajectory.joint_names,
                [float(value) for value in trajectory.points[-1].positions],
            )))
        self._publish_status('moveit_dry_run_planned', candidate.arm)

    def _execute_moveit(self, candidate, execute_with_moveit=False):
        self._ensure_moveit()
        component = self._moveit_components[candidate.arm]
        tip = self._moveit_left_tip if candidate.arm == 'left' else self._moveit_right_tip
        self._command_gripper(candidate.arm, self._gripper_open_pos)
        self._wait_with_execution_health(self._gripper_wait)
        self._plan_and_execute_moveit(component, candidate.arm, candidate.pregrasp_pose, tip, 'pregrasp', execute_with_moveit)
        self._plan_and_execute_moveit(component, candidate.arm, candidate.grasp_pose, tip, 'grasp', execute_with_moveit)
        close_position = width_to_gripper_position(
            candidate.required_width,
            self._max_gripper_width,
            self._gripper_open_pos,
            self._gripper_closed_pos,
            self._grasp_width_scale,
        )
        self._command_gripper(candidate.arm, close_position)
        self._wait_with_execution_health(self._gripper_wait)
        lift = copy.deepcopy(candidate.grasp_pose)
        lift.pose.position.z += self._lift_distance
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
        # The robot does not expose a FollowJointTrajectory action. MoveIt is
        # planning-only here; trajectories are converted below to the vendor's
        # 18-value JSON trajectory topic. Still load an explicit empty controller
        # manager so MoveItCpp does not attempt an unconfigured execution backend.
        config_dict.update(
            yaml.safe_load((moveit_dir / 'moveit_controllers.yaml').read_text())
        )
        self._moveit = MoveItPy(
            node_name='adaptive_grasp_moveit',
            config_dict=config_dict,
        )
        for arm in ('left', 'right'):
            group = self._moveit_left_group if arm == 'left' else self._moveit_right_group
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
            params.planning_time = self._moveit_planning_time
            params.planning_attempts = self._moveit_planning_attempts
            params.max_velocity_scaling_factor = self._moveit_max_velocity_scaling
            params.max_acceleration_scaling_factor = self._moveit_max_acceleration_scaling
            result = component.plan(single_plan_parameters=params)
        except Exception as exc:
            raise RuntimeError(f'MoveIt request setup or planning failed for {stage}: {exc}') from exc
        if not result:
            raise RuntimeError(f'MoveIt planning failed for {stage}')
        return result

    def _tcp_pose_to_eef_pose(self, arm, pose):
        target_position, target_orientation = self._tcp_to_eef_pose_values(arm, pose)
        result = PoseStamped()
        result.header = pose.header
        result.pose.position.x, result.pose.position.y, result.pose.position.z = [
            float(value) for value in target_position
        ]
        (
            result.pose.orientation.x,
            result.pose.orientation.y,
            result.pose.orientation.z,
            result.pose.orientation.w,
        ) = [float(value) for value in target_orientation]
        return result

    def _plan_and_execute_moveit(self, component, arm, pose, tip, stage, execute_with_moveit):
        result = self._plan_moveit(component, self._tcp_pose_to_eef_pose(arm, pose), tip, stage)
        if execute_with_moveit:
            self._moveit.execute(result.trajectory, controllers=[])
            return
        trajectory, planned_duration = self._moveit_result_to_vendor_trajectory(
            arm, result
        )
        vendor_duration = max(
            self._vendor_trajectory_duration, planned_duration
        )
        self._assert_execution_health()
        self._publish_status(f'executing_{stage}', arm)
        message = String(data=json.dumps({
            'traj_deg': trajectory,
            # Never execute faster than MoveIt's time-parameterized result.
            'duration': vendor_duration,
            'excluded_joints_name': self._vendor_excluded_joints,
            'position_tolerance': 2.0,
        }))
        with self._condition:
            command_feedback_sequence = self._eef_feedback_sequence
            self._record_hardware_command()
            self._vendor_trajectory_pub.publish(message)
        target_position, target_orientation = self._tcp_to_eef_pose_values(arm, pose)
        self._wait_for_eef(
            arm,
            target_position,
            target_orientation,
            stage,
            command_feedback_sequence,
            vendor_duration,
        )

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
        times = []
        current_whole_body = [
            float(value)
            for value in (
                current_joints['leg_waist']
                + current_joints['left_arm']
                + current_joints['right_arm']
            )
        ]
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
            times.append(
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) / 1_000_000_000.0
            )
        if times and times[0] > 1e-9:
            trajectory.insert(0, current_whole_body)
            times.insert(0, 0.0)
        return resample_trajectory_for_uniform_timing(trajectory, times)

    def _command_gripper(self, arm, position):
        self._assert_execution_health()
        position = validated_gripper_command(
            position,
            self._gripper_open_pos,
            self._gripper_closed_pos,
        )
        key = f'{arm}_gripper_target_joints_position'
        self._record_hardware_command()
        self._gripper_pub.publish(String(data=json.dumps({key: [position]})))

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
