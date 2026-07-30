import copy
import json
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from adaptive_object_grasping.action import PickObject
from adaptive_object_grasping.msg import TrackedObject
from adaptive_object_grasping.srv import (
    EstimateGrasps,
    ExecuteCandidate,
    SelectObject,
)
from adaptive_object_grasping_nodes.target_validation_core import (
    target_revalidation_error,
)


class GraspCoordinator(Node):
    def __init__(self):
        super().__init__('adaptive_grasp_coordinator')
        self.declare_parameter('service_timeout', 30.0)
        self.declare_parameter('execution_service_timeout', 180.0)
        self.declare_parameter('default_tracking_timeout', 12.0)
        self.declare_parameter('post_stable_confirmation', 0.35)
        self.declare_parameter('minimum_grasp_score', 0.25)
        self.declare_parameter('skip_pbvs_tracking', True)
        self.declare_parameter('fresh_selection_retry_timeout', 2.0)
        self.declare_parameter('fresh_selection_retry_period', 0.05)
        self.declare_parameter(
            'maximum_post_inference_target_shift_m',
            0.015,
        )
        self.declare_parameter(
            'maximum_post_inference_bbox_center_shift_px',
            12.0,
        )
        self.declare_parameter('minimum_post_inference_bbox_iou', 0.60)
        self._service_timeout = float(self.get_parameter('service_timeout').value)
        self._execution_service_timeout = float(
            self.get_parameter('execution_service_timeout').value
        )
        self._default_tracking_timeout = float(self.get_parameter('default_tracking_timeout').value)
        self._post_stable_confirmation = float(self.get_parameter('post_stable_confirmation').value)
        self._minimum_grasp_score = float(self.get_parameter('minimum_grasp_score').value)
        self._skip_pbvs_tracking = bool(self.get_parameter('skip_pbvs_tracking').value)
        self._fresh_selection_retry_timeout = max(
            0.0,
            float(
                self.get_parameter('fresh_selection_retry_timeout').value
            ),
        )
        self._fresh_selection_retry_period = max(
            0.01,
            float(
                self.get_parameter('fresh_selection_retry_period').value
            ),
        )
        self._maximum_post_inference_target_shift_m = max(
            0.0,
            float(
                self.get_parameter(
                    'maximum_post_inference_target_shift_m'
                ).value
            ),
        )
        self._maximum_post_inference_bbox_center_shift_px = max(
            0.0,
            float(
                self.get_parameter(
                    'maximum_post_inference_bbox_center_shift_px'
                ).value
            ),
        )
        self._minimum_post_inference_bbox_iou = float(
            self.get_parameter('minimum_post_inference_bbox_iou').value
        )
        self._lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._hardware_execution_lock = threading.Lock()
        self._goal_pending = False
        self._pre_hardware_cancel_accepted = False
        self._hardware_execution_active = False
        self._hardware_execution_unknown = False
        self._hardware_execution_unknown_reason = ''
        self._selected = None
        self._stability = {}
        callback_group = ReentrantCallbackGroup()
        self._select_client = self.create_client(
            SelectObject, 'select_grasp_object', callback_group=callback_group
        )
        self._estimate_client = self.create_client(
            EstimateGrasps, 'estimate_grasps', callback_group=callback_group
        )
        self._execute_client = self.create_client(
            ExecuteCandidate, 'execute_grasp_candidate', callback_group=callback_group
        )
        self._arm_pub = self.create_publisher(String, 'selected_arm', 10)
        self.create_subscription(TrackedObject, 'selected_object', self._on_selected, 10)
        self.create_subscription(String, 'target_stability', self._on_stability, 10)
        self._action_server = ActionServer(
            self,
            PickObject,
            'pick_object',
            execute_callback=self._execute_goal,
            goal_callback=self._accept_goal,
            cancel_callback=self._cancel_goal,
            callback_group=callback_group,
        )
        self.get_logger().info('Adaptive pick action ready')

    def _cancel_goal(self, _goal_handle):
        with self._hardware_execution_lock:
            if self._hardware_execution_unknown:
                self.get_logger().error(
                    'cancel rejected because hardware motion state is unknown: '
                    f'{self._hardware_execution_unknown_reason}; use the physical '
                    'emergency stop and restart the coordinator only after the '
                    'robot has been made safe'
                )
                return CancelResponse.REJECT
            if self._hardware_execution_active:
                self.get_logger().warning(
                    'cancel rejected after hardware execution started; use the '
                    'physical emergency stop if motion must stop immediately'
                )
                return CancelResponse.REJECT
            # The Action state may not expose is_cancel_requested immediately
            # after this callback accepts cancellation. Latch it under the same
            # lock used at the hardware-dispatch boundary to close that race.
            self._pre_hardware_cancel_accepted = True
        return CancelResponse.ACCEPT

    def _accept_goal(self, request):
        if request.track_id < 0 and not request.label.strip():
            return GoalResponse.REJECT
        if request.preferred_arm not in ('', 'auto', 'left', 'right'):
            return GoalResponse.REJECT
        if request.execute and not request.execution_confirmation.strip():
            self.get_logger().warning(
                'rejecting execute goal without an operator confirmation'
            )
            return GoalResponse.REJECT
        with self._hardware_execution_lock:
            if (
                self._goal_pending
                or self._goal_lock.locked()
                or self._hardware_execution_active
                or self._hardware_execution_unknown
            ):
                self.get_logger().warning(
                    'rejecting goal because another goal is active or hardware '
                    'motion state is unknown'
                )
                return GoalResponse.REJECT
            self._pre_hardware_cancel_accepted = False
            self._goal_pending = True
        return GoalResponse.ACCEPT

    def _on_selected(self, message):
        with self._lock:
            self._selected = message

    def _on_stability(self, message):
        try:
            status = json.loads(message.data)
            with self._lock:
                selected_track_id = (
                    None if self._selected is None else self._selected.track_id
                )
                if (
                    selected_track_id is not None
                    and status.get('track_id') != selected_track_id
                ):
                    return
                self._stability = status
            arm = status.get('arm')
            if arm in ('left', 'right'):
                self._arm_pub.publish(String(data=arm))
        except Exception as exc:
            self.get_logger().warning(f'invalid stability status: {exc}')

    def _execute_goal(self, goal_handle):
        result = PickObject.Result()
        if not self._goal_lock.acquire(blocking=False):
            with self._hardware_execution_lock:
                self._goal_pending = False
            goal_handle.abort()
            result.message = 'another grasp is already active'
            return result
        with self._hardware_execution_lock:
            self._goal_pending = False
        try:
            return self._run_goal(goal_handle, result)
        finally:
            self._goal_lock.release()

    def _run_goal(self, goal_handle, result):
        goal = goal_handle.request
        preferred_arm = goal.preferred_arm or 'auto'
        self._feedback(goal_handle, 'selecting', 'selecting requested object', 0.0, 0.0)
        select_request = SelectObject.Request()
        select_request.track_id = goal.track_id
        select_request.label = goal.label
        select_request.preferred_arm = preferred_arm
        select_request.include_observation = True
        select_status, selected, select_detail = self._select_fresh(
            select_request,
            retry_missing=True,
        )
        if goal_handle.is_cancel_requested:
            return self._cancel_at_boundary(
                goal_handle, result, 'grasp canceled after target selection'
            )
        if select_status != 'ok':
            return self._abort(
                goal_handle,
                result,
                'object selection failed: '
                + self._service_failure(select_status, select_detail),
            )
        if not selected.accepted:
            return self._abort(
                goal_handle, result, 'object selection failed: ' + selected.message
            )
        with self._lock:
            self._selected = selected.selected
            self._stability = {}
            selected_track_id = selected.selected.track_id
        selected_target = selected.selected
        selected_observation = selected.observation
        self._arm_pub.publish(String(data=preferred_arm))

        if self._skip_pbvs_tracking:
            self._feedback(goal_handle, 'selected', 'PBVS tracking skipped; estimating immediately', 0.0, 0.0)
        else:
            tracking_timeout = float(goal.maximum_tracking_time)
            if tracking_timeout <= 0.0:
                tracking_timeout = self._default_tracking_timeout
            deadline = time.monotonic() + tracking_timeout
            stable_since = None
            confirmation = max(
                self._post_stable_confirmation,
                float(goal.required_stable_duration),
            )
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.message = 'grasp canceled during target tracking'
                    return result
                with self._lock:
                    status = dict(self._stability)
                if status.get('track_id') != selected_track_id:
                    status = {}
                state = status.get('state', 'waiting')
                detail = status.get(
                    'detail',
                    f'waiting for PBVS stability for track [{selected_track_id}]',
                )
                motion = float(status.get('window_motion', 0.0))
                elapsed = float(status.get('stable_elapsed', 0.0))
                self._feedback(goal_handle, state, detail, motion, elapsed)
                if state == 'blocked' and 'exceeded maximum follow distance' in detail:
                    return self._abort(goal_handle, result, detail)
                if bool(status.get('stable', False)):
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= confirmation:
                        break
                else:
                    stable_since = None
                time.sleep(0.05)
            else:
                return self._abort(goal_handle, result, 'target did not become stable before timeout')

        if self._skip_pbvs_tracking:
            target = selected_target
            observation = selected_observation
        else:
            refresh_request = SelectObject.Request()
            refresh_request.track_id = selected_track_id
            refresh_request.label = ''
            refresh_request.preferred_arm = preferred_arm
            refresh_request.include_observation = True
            refresh_status, refreshed, refresh_detail = self._select_fresh(
                refresh_request,
                retry_missing=True,
            )
            if goal_handle.is_cancel_requested:
                return self._cancel_at_boundary(
                    goal_handle,
                    result,
                    'grasp canceled after refreshing the selected observation',
                )
            if refresh_status != 'ok':
                return self._abort(
                    goal_handle,
                    result,
                    'selected observation refresh failed: '
                    + self._service_failure(refresh_status, refresh_detail),
                )
            if not refreshed.accepted:
                return self._abort(
                    goal_handle,
                    result,
                    'selected observation refresh failed: ' + refreshed.message,
                )
            target = refreshed.selected
            observation = refreshed.observation
            with self._lock:
                self._selected = target
        if target is None or target.track_id != selected_track_id:
            return self._abort(goal_handle, result, 'selected target was lost before GraspNet')
        self._feedback(goal_handle, 'estimating', 'running GraspNet pose estimation', 0.0, 0.0)
        if goal_handle.is_cancel_requested:
            return self._cancel_at_boundary(
                goal_handle, result, 'grasp canceled before GraspNet'
            )
        estimate_request = EstimateGrasps.Request()
        estimate_request.target = target
        estimate_request.preferred_arm = preferred_arm
        estimate_request.observation = observation
        estimate_status, estimate, estimate_detail = self._call(
            self._estimate_client, estimate_request, return_status=True
        )
        if goal_handle.is_cancel_requested:
            return self._cancel_at_boundary(
                goal_handle, result, 'grasp canceled after GraspNet'
            )
        if estimate_status != 'ok':
            return self._abort(
                goal_handle,
                result,
                'GraspNet failed: '
                + self._service_failure(estimate_status, estimate_detail),
            )
        if not estimate.success or not estimate.candidates:
            return self._abort(
                goal_handle, result, 'GraspNet failed: ' + estimate.message
            )
        self._feedback(
            goal_handle,
            'revalidating',
            'checking that the target stayed fixed during GraspNet inference',
            0.0,
            0.0,
        )
        validation_request = SelectObject.Request()
        validation_request.track_id = selected_track_id
        validation_request.label = ''
        validation_request.preferred_arm = preferred_arm
        validation_request.include_observation = False
        validation_status, validation, validation_detail = self._select_fresh(
            validation_request,
            retry_missing=True,
        )
        if goal_handle.is_cancel_requested:
            return self._cancel_at_boundary(
                goal_handle,
                result,
                'grasp canceled after post-inference target validation',
            )
        if validation_status != 'ok':
            return self._abort(
                goal_handle,
                result,
                'post-inference target validation failed: '
                + self._service_failure(
                    validation_status, validation_detail
                ),
            )
        if not validation.accepted:
            return self._abort(
                goal_handle,
                result,
                'post-inference target validation failed: '
                + validation.message,
            )
        validation_error = target_revalidation_error(
            target,
            validation.selected,
            maximum_position_shift_m=(
                self._maximum_post_inference_target_shift_m
            ),
            maximum_bbox_center_shift_px=(
                self._maximum_post_inference_bbox_center_shift_px
            ),
            minimum_bbox_iou=self._minimum_post_inference_bbox_iou,
        )
        if validation_error:
            return self._abort(
                goal_handle,
                result,
                'target moved during GraspNet inference: '
                + validation_error,
            )
        validation_stamp = validation.selected.header.stamp
        for item in estimate.candidates:
            item.header.stamp = copy.deepcopy(validation_stamp)
            item.grasp_pose.header.stamp = copy.deepcopy(validation_stamp)
            item.pregrasp_pose.header.stamp = copy.deepcopy(validation_stamp)
        candidates = [item for item in estimate.candidates if item.score >= self._minimum_grasp_score]
        if not candidates:
            return self._abort(goal_handle, result, 'no GraspNet candidate passed score filter')
        candidate = None
        planning = None
        planning_errors = []
        for item in sorted(candidates, key=lambda value: value.score, reverse=True):
            if goal_handle.is_cancel_requested:
                return self._cancel_at_boundary(
                    goal_handle, result, 'grasp canceled during candidate planning'
                )
            self._feedback(
                goal_handle,
                'planning',
                f'checking score={item.score:.3f}, arm={item.arm}',
                0.0,
                0.0,
            )
            plan_request = ExecuteCandidate.Request()
            plan_request.candidate = item
            plan_request.execute = False
            plan_request.execution_confirmation = ''
            planning_status, planning, planning_detail = self._call(
                self._execute_client, plan_request, return_status=True
            )
            if goal_handle.is_cancel_requested:
                return self._cancel_at_boundary(
                    goal_handle, result, 'grasp canceled after candidate planning'
                )
            if planning_status == 'ok' and planning.success:
                candidate = item
                break
            planning_errors.append(
                self._service_failure(planning_status, planning_detail)
                if planning_status != 'ok'
                else planning.message
            )
        if candidate is None:
            detail = '; '.join(planning_errors[:3])
            return self._abort(goal_handle, result, f'no reachable grasp candidate: {detail}')
        result.selected_grasp = candidate
        if not goal.execute:
            result.success = True
            result.message = (
                f'dry-run independently validated one of {len(candidates)} '
                'GraspNet candidates from the current state; '
                f'score={candidate.score:.3f}, arm={candidate.arm}. '
                'The target robot ghost was published; no continuous path was '
                'proven and no hardware command was sent.'
            )
            self._feedback(
                goal_handle,
                'dry_run',
                result.message,
                0.0,
                0.0,
            )
            if goal_handle.is_cancel_requested:
                result.success = False
                return self._cancel_at_boundary(
                    goal_handle, result, 'grasp canceled after dry-run planning'
                )
            goal_handle.succeed()
            return result

        self._feedback(
            goal_handle,
            'executing',
            f'candidate score={candidate.score:.3f}, arm={candidate.arm}',
            0.0,
            0.0,
        )
        execution = planning
        execute_request = ExecuteCandidate.Request()
        execute_request.candidate = candidate
        execute_request.execute = True
        execute_request.execution_confirmation = goal.execution_confirmation
        with self._hardware_execution_lock:
            canceled_before_execution = (
                goal_handle.is_cancel_requested
                or self._pre_hardware_cancel_accepted
            )
            if not canceled_before_execution and not self._hardware_execution_unknown:
                self._hardware_execution_active = True
            elif self._hardware_execution_unknown:
                canceled_before_execution = True
        if canceled_before_execution:
            return self._cancel_at_boundary(
                goal_handle, result, 'grasp canceled before hardware execution'
            )
        # The vendor API exposes no documented software stop primitive. Once
        # the request is dispatched, cancellation is rejected until an explicit
        # response reaches an Action terminal state. A timeout or transport
        # error latches an unknown-motion fault and is never auto-cleared.
        execution_status, execution, execution_detail = self._call(
            self._execute_client,
            execute_request,
            timeout=self._execution_service_timeout,
            return_status=True,
        )
        if execution_status in ('timeout', 'error'):
            result.message = (
                'execution failed closed: '
                + self._service_failure(execution_status, execution_detail)
                + '; hardware motion state is unknown; use the physical emergency '
                'stop and verify the robot before restarting the coordinator'
            )
            with self._hardware_execution_lock:
                self._hardware_execution_unknown = True
                self._hardware_execution_unknown_reason = result.message
                # Keep _hardware_execution_active latched true. There is no
                # verified stop/completion signal that would make auto-unlock safe.
                goal_handle.abort()
            return result

        if execution_status == 'unavailable':
            result.message = (
                'execution failed: '
                + self._service_failure(execution_status, execution_detail)
            )
            with self._hardware_execution_lock:
                goal_handle.abort()
                self._hardware_execution_active = False
            return result

        if not execution.success:
            result.message = 'execution failed: ' + execution.message
            normalized_message = execution.message.lower()
            executor_reports_unknown_motion = any(
                marker in normalized_message
                for marker in (
                    'unknown-motion',
                    'unknown motion',
                    'fault latched',
                )
            )
            with self._hardware_execution_lock:
                if executor_reports_unknown_motion:
                    self._hardware_execution_unknown = True
                    self._hardware_execution_unknown_reason = result.message
                goal_handle.abort()
                if not executor_reports_unknown_motion:
                    self._hardware_execution_active = False
            return result

        result.success = True
        result.message = execution.message
        self._feedback(goal_handle, 'completed', result.message, 0.0, 0.0)
        with self._hardware_execution_lock:
            goal_handle.succeed()
            self._hardware_execution_active = False
        return result

    def _call(self, client, request, timeout=None, return_status=False):
        timeout = self._service_timeout if timeout is None else float(timeout)
        try:
            if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
                outcome = ('unavailable', None, 'service unavailable')
            else:
                future = client.call_async(request)
                event = threading.Event()
                future.add_done_callback(lambda _: event.set())
                if not event.wait(timeout):
                    outcome = (
                        'timeout',
                        None,
                        f'service response timed out after {timeout:.1f}s',
                    )
                else:
                    outcome = ('ok', future.result(), '')
        except Exception as exc:
            self.get_logger().error(f'service call failed: {exc}')
            outcome = ('error', None, str(exc))
        if return_status:
            return outcome
        return outcome[1] if outcome[0] == 'ok' else None

    def _select_fresh(self, request, *, retry_missing=False):
        deadline = time.monotonic() + self._fresh_selection_retry_timeout
        while True:
            status, response, detail = self._call(
                self._select_client,
                request,
                return_status=True,
            )
            retryable = (
                status == 'ok'
                and response is not None
                and not response.accepted
                and (
                    'tracked object frame is stale' in response.message
                    or (
                        retry_missing
                        and response.message
                        == 'requested object is not in the current frame'
                    )
                )
            )
            if not retryable or time.monotonic() >= deadline:
                return status, response, detail
            time.sleep(self._fresh_selection_retry_period)

    @staticmethod
    def _service_failure(status, detail):
        if status == 'unavailable':
            return detail or 'service unavailable'
        if status == 'timeout':
            return detail or 'service response timed out'
        if status == 'error':
            return 'service error: ' + (detail or 'unknown transport error')
        return detail or 'service returned an unknown failure'

    @staticmethod
    def _feedback(goal_handle, state, detail, motion, stable_elapsed):
        feedback = PickObject.Feedback()
        feedback.state = state
        feedback.detail = detail
        feedback.target_motion = float(motion)
        feedback.stable_elapsed = float(stable_elapsed)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _abort(goal_handle, result, message):
        goal_handle.abort()
        result.message = message
        return result

    @staticmethod
    def _cancel_at_boundary(goal_handle, result, message):
        goal_handle.canceled()
        result.message = message
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GraspCoordinator()
    executor = MultiThreadedExecutor(num_threads=6)
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
