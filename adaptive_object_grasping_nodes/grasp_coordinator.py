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


class GraspCoordinator(Node):
    def __init__(self):
        super().__init__('adaptive_grasp_coordinator')
        self.declare_parameter('service_timeout', 30.0)
        self.declare_parameter('default_tracking_timeout', 12.0)
        self.declare_parameter('post_stable_confirmation', 0.35)
        self.declare_parameter('minimum_grasp_score', 0.25)
        self._lock = threading.Lock()
        self._goal_lock = threading.Lock()
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
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=callback_group,
        )
        self.get_logger().info('Adaptive pick action ready')

    @staticmethod
    def _accept_goal(request):
        if request.track_id < 0 and not request.label.strip():
            return GoalResponse.REJECT
        if request.preferred_arm not in ('', 'auto', 'left', 'right'):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_selected(self, message):
        with self._lock:
            self._selected = message

    def _on_stability(self, message):
        try:
            status = json.loads(message.data)
            with self._lock:
                self._stability = status
            arm = status.get('arm')
            if arm in ('left', 'right'):
                self._arm_pub.publish(String(data=arm))
        except Exception as exc:
            self.get_logger().warning(f'invalid stability status: {exc}')

    def _execute_goal(self, goal_handle):
        result = PickObject.Result()
        if not self._goal_lock.acquire(blocking=False):
            goal_handle.abort()
            result.message = 'another grasp is already active'
            return result
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
        selected = self._call(self._select_client, select_request)
        if selected is None or not selected.accepted:
            return self._abort(goal_handle, result, 'object selection failed: ' + (
                'service unavailable' if selected is None else selected.message
            ))
        with self._lock:
            self._selected = selected.selected
            self._stability = {}
        self._arm_pub.publish(String(data=preferred_arm))

        tracking_timeout = float(goal.maximum_tracking_time)
        if tracking_timeout <= 0.0:
            tracking_timeout = float(self.get_parameter('default_tracking_timeout').value)
        deadline = time.monotonic() + tracking_timeout
        stable_since = None
        confirmation = max(
            float(self.get_parameter('post_stable_confirmation').value),
            float(goal.required_stable_duration),
        )
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message = 'grasp canceled during target tracking'
                return result
            with self._lock:
                status = dict(self._stability)
            state = status.get('state', 'waiting')
            detail = status.get('detail', 'waiting for PBVS target')
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

        with self._lock:
            target = self._selected
        if target is None:
            return self._abort(goal_handle, result, 'selected target was lost before GraspNet')
        self._feedback(goal_handle, 'estimating', 'running GraspNet pose estimation', 0.0, 0.0)
        estimate_request = EstimateGrasps.Request()
        estimate_request.target = target
        estimate_request.preferred_arm = preferred_arm
        estimate = self._call(self._estimate_client, estimate_request)
        if estimate is None or not estimate.success or not estimate.candidates:
            return self._abort(goal_handle, result, 'GraspNet failed: ' + (
                'service unavailable' if estimate is None else estimate.message
            ))
        minimum_score = float(self.get_parameter('minimum_grasp_score').value)
        candidates = [item for item in estimate.candidates if item.score >= minimum_score]
        if not candidates:
            return self._abort(goal_handle, result, 'no GraspNet candidate passed score filter')
        candidate = max(candidates, key=lambda item: item.score)
        result.selected_grasp = candidate

        self._feedback(
            goal_handle,
            'executing' if goal.execute else 'dry_run',
            f'candidate score={candidate.score:.3f}, arm={candidate.arm}',
            0.0,
            0.0,
        )
        execute_request = ExecuteCandidate.Request()
        execute_request.candidate = candidate
        execute_request.execute = goal.execute
        execution = self._call(self._execute_client, execute_request)
        if execution is None or not execution.success:
            return self._abort(goal_handle, result, 'execution failed: ' + (
                'service unavailable' if execution is None else execution.message
            ))
        goal_handle.succeed()
        result.success = True
        result.message = execution.message
        self._feedback(goal_handle, 'completed', result.message, 0.0, 0.0)
        return result

    def _call(self, client, request):
        timeout = float(self.get_parameter('service_timeout').value)
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return None
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f'service call failed: {exc}')
            return None

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
