import json
import math

import numpy as np


def parse_eef_feedback(payload):
    data = json.loads(payload) if isinstance(payload, str) else payload
    result = {}
    for arm in ('left', 'right'):
        position = data.get(f'pos_{arm}_in_robot')
        orientation = data.get(f'quat_{arm}_in_robot')
        nested = data.get(f'{arm}_eef_pose')
        if nested and (position is None or orientation is None):
            position = nested.get('position')
            orientation = nested.get('rotation') or nested.get('orientation')
        if len(position or []) == 3 and len(orientation or []) == 4:
            result[arm] = {
                'position': [float(value) for value in position],
                'orientation': [float(value) for value in orientation],
            }
    return result


def pose_error(current, target_position, target_orientation):
    current_position = np.asarray(current['position'], dtype=np.float64)
    target_position = np.asarray(target_position, dtype=np.float64)
    position_error = float(np.linalg.norm(current_position - target_position))
    current_q = np.asarray(current['orientation'], dtype=np.float64)
    target_q = np.asarray(target_orientation, dtype=np.float64)
    current_q /= max(np.linalg.norm(current_q), 1e-9)
    target_q /= max(np.linalg.norm(target_q), 1e-9)
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


def width_to_gripper_position(
    width, maximum_width, open_position, closed_position, width_scale=1.0
):
    effective_width = effective_gripper_width(width, width_scale)
    ratio = np.clip(effective_width / float(maximum_width), 0.0, 1.0)
    return float(closed_position + ratio * (open_position - closed_position))


def compose_vendor_rrt_target(left_body, right_body):
    if len(left_body) != 11 or len(right_body) != 11:
        raise ValueError('vendor IK must return 11 left and 11 right body joints')
    return [float(value) for value in left_body] + [float(value) for value in right_body[4:]]


def validate_vendor_trajectory(trajectory):
    if not isinstance(trajectory, list) or len(trajectory) < 2:
        raise ValueError('vendor RRT returned an empty or too-short trajectory')
    if any(not isinstance(point, list) or len(point) != 18 for point in trajectory):
        raise ValueError('vendor RRT trajectory points must contain 18 joints')
    array = np.asarray(trajectory, dtype=np.float64)
    if not np.all(np.isfinite(array)):
        raise ValueError('vendor RRT trajectory contains non-finite values')
    return array.tolist()
