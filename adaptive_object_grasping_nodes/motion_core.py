import json
import math
from collections.abc import Mapping

import numpy as np

from adaptive_object_grasping_nodes.dual_arm_core import (
    SHARED_BODY_DOF,
    VENDOR_IK_BODY_DOF,
    VENDOR_TRAJECTORY_DOF,
)


def parse_eef_feedback(payload):
    data = json.loads(payload) if isinstance(payload, str) else payload
    if not isinstance(data, Mapping):
        raise ValueError('EEF feedback must be a JSON object')
    result = {}
    for arm in ('left', 'right'):
        position = data.get(f'pos_{arm}_in_robot')
        orientation = data.get(f'quat_{arm}_in_robot')
        nested = data.get(f'{arm}_eef_pose')
        if isinstance(nested, Mapping) and (position is None or orientation is None):
            position = nested.get('position')
            orientation = nested.get('rotation') or nested.get('orientation')
        if not isinstance(position, (list, tuple)) or len(position) != 3:
            raise ValueError(f'{arm} EEF position must contain three values')
        if not isinstance(orientation, (list, tuple)) or len(orientation) != 4:
            raise ValueError(f'{arm} EEF orientation must contain four values')
        try:
            position = [float(value) for value in position]
            orientation = [float(value) for value in orientation]
        except (TypeError, ValueError) as exc:
            raise ValueError(f'{arm} EEF pose must contain numeric values') from exc
        if not all(math.isfinite(value) for value in position + orientation):
            raise ValueError(f'{arm} EEF pose must contain finite values')
        quaternion_norm = math.sqrt(sum(value * value for value in orientation))
        if not 0.98 <= quaternion_norm <= 1.02:
            raise ValueError(
                f'{arm} EEF quaternion norm {quaternion_norm:.4f} is invalid'
            )
        result[arm] = {
            'position': position,
            'orientation': orientation,
        }
    return result


def pose_error(current, target_position, target_orientation):
    current_position = np.asarray(current['position'], dtype=np.float64)
    target_position = np.asarray(target_position, dtype=np.float64)
    current_q = np.asarray(current['orientation'], dtype=np.float64)
    target_q = np.asarray(target_orientation, dtype=np.float64)
    if (
        current_position.shape != (3,)
        or target_position.shape != (3,)
        or current_q.shape != (4,)
        or target_q.shape != (4,)
        or not np.all(np.isfinite(current_position))
        or not np.all(np.isfinite(target_position))
        or not np.all(np.isfinite(current_q))
        or not np.all(np.isfinite(target_q))
    ):
        raise ValueError('pose error inputs must be finite 3D poses')
    position_error = float(np.linalg.norm(current_position - target_position))
    current_norm = float(np.linalg.norm(current_q))
    target_norm = float(np.linalg.norm(target_q))
    if current_norm < 1e-9 or target_norm < 1e-9:
        raise ValueError('pose error quaternion norm cannot be zero')
    current_q /= current_norm
    target_q /= target_norm
    dot = float(np.clip(abs(np.dot(current_q, target_q)), 0.0, 1.0))
    angular_error = 2.0 * math.acos(dot)
    return position_error, angular_error


def validate_pose(position, orientation, workspace_minimum, workspace_maximum):
    position = np.asarray(position, dtype=np.float64)
    orientation = np.asarray(orientation, dtype=np.float64)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        return 'position must contain three finite values'
    if orientation.shape != (4,) or not np.all(np.isfinite(orientation)):
        return 'orientation must contain four finite values'
    lower = np.asarray(workspace_minimum, dtype=np.float64)
    upper = np.asarray(workspace_maximum, dtype=np.float64)
    if np.any(position < lower) or np.any(position > upper):
        return f'position {position.tolist()} is outside configured workspace'
    norm = np.linalg.norm(orientation)
    if not 0.98 <= norm <= 1.02:
        return f'quaternion norm {norm:.4f} is invalid'
    return ''


def vendor_pose_payload(active_arm, position, orientation, current_poses):
    if active_arm not in ('left', 'right'):
        raise ValueError('active arm must be left or right')
    if not all(arm in current_poses for arm in ('left', 'right')):
        raise ValueError('both current EEF poses are required')
    poses = {
        arm: {
            'position': list(current_poses[arm]['position']),
            'orientation': list(current_poses[arm]['orientation']),
        }
        for arm in ('left', 'right')
    }
    poses[active_arm] = {
        'position': [float(value) for value in position],
        'orientation': [float(value) for value in orientation],
    }
    return {
        'pos_left_in_robot': poses['left']['position'],
        'quat_left_in_robot': poses['left']['orientation'],
        'pos_right_in_robot': poses['right']['position'],
        'quat_right_in_robot': poses['right']['orientation'],
    }


def effective_gripper_width(width, width_scale=1.0):
    return float(width) * float(width_scale)


def validate_gripper_width(width, maximum_width, width_scale=1.0):
    """Return a fail-closed validation error for one perceived grasp width."""

    try:
        raw_width = float(width)
        maximum = float(maximum_width)
        scale = float(width_scale)
    except (TypeError, ValueError):
        return 'gripper width, maximum and scale must be numeric'
    if not all(math.isfinite(value) for value in (raw_width, maximum, scale)):
        return 'gripper width, maximum and scale must be finite'
    if raw_width < 0.0:
        return 'gripper width must be non-negative'
    if maximum <= 0.0:
        return 'maximum gripper width must be positive'
    if scale < 0.0:
        return 'gripper width scale must be non-negative'
    effective = raw_width * scale
    if effective > maximum:
        return (
            f'effective gripper width {effective:.3f}m exceeds configured maximum '
            f'{maximum:.3f}m (raw={raw_width:.3f}m, scale={scale:.3f})'
        )
    return ''


def validate_measurement_age(
    now_seconds,
    stamp_seconds,
    maximum_age,
    future_tolerance=0.05,
):
    """Return an error when a perception measurement is missing, stale or future."""

    try:
        now = float(now_seconds)
        stamp = float(stamp_seconds)
        maximum = float(maximum_age)
        tolerance = float(future_tolerance)
    except (TypeError, ValueError):
        return 'candidate measurement timestamp configuration must be numeric'
    if not all(math.isfinite(value) for value in (now, stamp, maximum, tolerance)):
        return 'candidate measurement timestamp configuration must be finite'
    if stamp <= 0.0:
        return 'candidate measurement timestamp is missing'
    if maximum <= 0.0:
        return 'maximum candidate age must be positive'
    if tolerance < 0.0:
        return 'candidate future tolerance must be non-negative'
    age = now - stamp
    if age < -tolerance:
        return f'candidate measurement is {-age:.3f}s in the future'
    if age > maximum:
        return (
            f'candidate measurement is stale: age={age:.3f}s, '
            f'maximum={maximum:.3f}s'
        )
    return ''


def validated_gripper_command(position, open_position, closed_position):
    """Return one finite in-range vendor gripper command or raise."""

    try:
        command = float(position)
        open_value = float(open_position)
        closed_value = float(closed_position)
    except (TypeError, ValueError) as exc:
        raise ValueError('gripper command and limits must be numeric') from exc
    if not all(math.isfinite(value) for value in (command, open_value, closed_value)):
        raise ValueError('gripper command and limits must be finite')
    lower, upper = sorted((open_value, closed_value))
    if command < lower or command > upper:
        raise ValueError(
            f'gripper command {command:.3f} is outside [{lower:.3f}, {upper:.3f}]'
        )
    return command


def motion_completion_timeout(minimum_timeout, commanded_duration, margin):
    """Bind feedback waiting to the duration already sent to the controller."""

    try:
        minimum = float(minimum_timeout)
        duration = float(commanded_duration)
        completion_margin = float(margin)
    except (TypeError, ValueError) as exc:
        raise ValueError('motion timeout values must be numeric') from exc
    if not all(
        math.isfinite(value) for value in (minimum, duration, completion_margin)
    ):
        raise ValueError('motion timeout values must be finite')
    if minimum <= 0.0 or duration <= 0.0 or completion_margin < 0.0:
        raise ValueError(
            'minimum timeout and commanded duration must be positive; '
            'completion margin must be non-negative'
        )
    return max(minimum, duration + completion_margin)


def parse_gripper_feedback(payload):
    data = json.loads(payload) if isinstance(payload, str) else payload
    result = {}
    for arm in ('left', 'right'):
        position = (data.get(f'{arm}_gripper_state') or {}).get('position')
        if not isinstance(position, list) or len(position) != 1:
            raise ValueError(f'{arm} gripper feedback must contain one position')
        value = float(position[0])
        if not math.isfinite(value):
            raise ValueError(f'{arm} gripper feedback must be finite')
        result[arm] = value
    return result


def vendor_gripper_to_urdf(
    position,
    vendor_open_position,
    vendor_closed_position,
    urdf_open_position,
    urdf_closed_position,
):
    vendor_range = float(vendor_closed_position) - float(vendor_open_position)
    if abs(vendor_range) < 1e-9:
        raise ValueError('vendor gripper calibration range cannot be zero')
    ratio = np.clip(
        (float(position) - float(vendor_open_position)) / vendor_range,
        0.0,
        1.0,
    )
    return float(
        float(urdf_open_position)
        + ratio * (float(urdf_closed_position) - float(urdf_open_position))
    )


def width_to_gripper_position(
    width, maximum_width, open_position, closed_position, width_scale=1.0
):
    error = validate_gripper_width(width, maximum_width, width_scale)
    if error:
        raise ValueError(error)
    effective_width = effective_gripper_width(width, width_scale)
    ratio = np.clip(effective_width / float(maximum_width), 0.0, 1.0)
    command = float(closed_position + ratio * (open_position - closed_position))
    return validated_gripper_command(command, open_position, closed_position)


def compose_vendor_rrt_target(left_body, right_body):
    if len(left_body) != VENDOR_IK_BODY_DOF or len(right_body) != VENDOR_IK_BODY_DOF:
        raise ValueError(
            f'vendor IK must return {VENDOR_IK_BODY_DOF} left and '
            f'{VENDOR_IK_BODY_DOF} right body joints'
        )
    return [float(value) for value in left_body] + [
        float(value) for value in right_body[SHARED_BODY_DOF:]
    ]


def validate_vendor_trajectory(trajectory):
    if not isinstance(trajectory, list) or len(trajectory) < 2:
        raise ValueError('vendor RRT returned an empty or too-short trajectory')
    if any(
        not isinstance(point, list) or len(point) != VENDOR_TRAJECTORY_DOF
        for point in trajectory
    ):
        raise ValueError(
            f'vendor RRT trajectory points must contain {VENDOR_TRAJECTORY_DOF} joints'
        )
    array = np.asarray(trajectory, dtype=np.float64)
    if not np.all(np.isfinite(array)):
        raise ValueError('vendor RRT trajectory contains non-finite values')
    return array.tolist()


def resample_trajectory_for_uniform_timing(trajectory, times):
    """Resample a timed trajectory for a vendor API with one total duration.

    The AutoLife topic accepts positions plus one duration, not per-point
    timestamps. Resampling at uniformly spaced times preserves MoveIt's timing
    profile much more closely than discarding ``time_from_start`` outright.
    """

    array = np.asarray(trajectory, dtype=np.float64)
    stamps = np.asarray(times, dtype=np.float64)
    if array.ndim != 2 or len(array) < 2:
        raise ValueError('timed trajectory must contain at least two points')
    if stamps.shape != (len(array),):
        raise ValueError('trajectory timestamps must match trajectory points')
    if not np.all(np.isfinite(array)) or not np.all(np.isfinite(stamps)):
        raise ValueError('timed trajectory contains non-finite values')
    if abs(float(stamps[0])) > 1e-9:
        raise ValueError('timed trajectory must start at zero seconds')
    if np.any(np.diff(stamps) <= 0.0):
        raise ValueError('trajectory timestamps must be strictly increasing')
    duration = float(stamps[-1])
    sample_times = np.linspace(0.0, duration, len(array))
    resampled = np.column_stack([
        np.interp(sample_times, stamps, array[:, joint_index])
        for joint_index in range(array.shape[1])
    ])
    return resampled.tolist(), duration
