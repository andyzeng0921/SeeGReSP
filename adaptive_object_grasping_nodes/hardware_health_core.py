"""Validation for the AutoLife S2 status payloads used by the safety gate."""

from collections.abc import Mapping
import json
import math


JOINT_STATUS_GROUP_SIZES = {
    'leg_waist_joint_state': 4,
    'left_arm_joint_state': 7,
    'right_arm_joint_state': 7,
    'left_gripper_state': 1,
    'right_gripper_state': 1,
    'neck_joint_state': 3,
}

ROBOT_STATE_GROUP_SIZES = {
    'neck': 3,
    'leg_waist': 4,
    'left_arm': 7,
    'right_arm': 7,
    'left_gripper': 1,
    'right_gripper': 1,
}

DEFAULT_REQUIRED_HEARTBEAT_MASK = 31


def _payload_mapping(payload):
    data = json.loads(payload) if isinstance(payload, str) else payload
    if not isinstance(data, Mapping):
        raise ValueError('status payload must be a JSON object')
    return data


def _finite_array(value, expected_size):
    if not isinstance(value, list) or len(value) != expected_size:
        return False
    try:
        return all(math.isfinite(float(item)) for item in value)
    except (TypeError, ValueError):
        return False


def joint_status_blockers(payload):
    """Validate joint array shape, numeric content and communication flags."""

    try:
        data = _payload_mapping(payload)
    except Exception as exc:
        return [f'invalid joint status JSON: {exc}']

    blockers = []
    for group, expected_size in JOINT_STATUS_GROUP_SIZES.items():
        record = data.get(group)
        if not isinstance(record, Mapping):
            blockers.append(f'{group} is missing')
            continue
        for field in ('position', 'speed', 'torque', 'temperature'):
            if not _finite_array(record.get(field), expected_size):
                blockers.append(
                    f'{group}.{field} must contain {expected_size} finite values'
                )
        communication_lost = record.get('communication_lost')
        if (
            not isinstance(communication_lost, list)
            or len(communication_lost) != expected_size
        ):
            blockers.append(
                f'{group}.communication_lost must contain {expected_size} flags'
            )
        elif any(value is not False for value in communication_lost):
            blockers.append(f'{group} reports lost joint communication')
    return blockers


def robot_state_blockers(
    payload,
    required_heartbeat_mask=DEFAULT_REQUIRED_HEARTBEAT_MASK,
):
    """Validate the vendor protection heartbeat and collision state."""

    try:
        data = _payload_mapping(payload)
    except Exception as exc:
        return [f'invalid robot state JSON: {exc}']

    blockers = []
    joints = data.get('joints_pos')
    if not isinstance(joints, Mapping):
        blockers.append('robot state joints_pos is missing')
    else:
        for group, expected_size in ROBOT_STATE_GROUP_SIZES.items():
            if not _finite_array(joints.get(group), expected_size):
                blockers.append(
                    f'joints_pos.{group} must contain {expected_size} finite values'
                )

    if data.get('self_collide') != 0:
        blockers.append('robot reports self collision')

    protection = data.get('protection')
    if not isinstance(protection, Mapping):
        blockers.append('robot protection state is missing')
        return blockers
    if protection.get('state') != 'NORMAL':
        blockers.append(f'protection state is {protection.get("state")!r}, expected NORMAL')
    if protection.get('protected') is not False:
        blockers.append('robot protection is active')
    if protection.get('reason') != 'NONE':
        blockers.append(f'robot protection reason is {protection.get("reason")!r}')

    try:
        expected = int(protection.get('heartbeat_expected_mask', 0))
        ready = int(protection.get('heartbeat_ready_mask', 0))
        lost = int(protection.get('heartbeat_lost_mask', 0))
        required = int(required_heartbeat_mask)
    except (TypeError, ValueError):
        expected, ready, lost, required = 0, 0, -1, -1
        blockers.append('heartbeat masks are invalid')
    if required <= 0:
        blockers.append('required heartbeat mask must be positive')
    elif expected != required:
        blockers.append(
            f'heartbeat expected mask is {expected}, required exactly {required}'
        )
    if required > 0 and ready & required != required:
        blockers.append(
            f'heartbeat not ready: required_mask={required}, ready_mask={ready}'
        )
    if required > 0 and lost & required:
        blockers.append(
            f'heartbeat lost: required_mask={required}, lost_mask={lost}'
        )
    if protection.get('heartbeat_protection_armed') is not True:
        blockers.append('heartbeat protection is not armed')
    if data.get('notifications'):
        blockers.append('robot state contains active notifications')
    return blockers


def robot_state_summary(payload):
    """Return the protection fields needed for an auditable probe report."""

    data = _payload_mapping(payload)
    protection = data.get('protection')
    if not isinstance(protection, Mapping):
        raise ValueError('robot protection state is missing')
    return {
        'state': protection.get('state'),
        'protected': protection.get('protected'),
        'reason': protection.get('reason'),
        'heartbeat_expected_mask': protection.get('heartbeat_expected_mask'),
        'heartbeat_ready_mask': protection.get('heartbeat_ready_mask'),
        'heartbeat_lost_mask': protection.get('heartbeat_lost_mask'),
        'heartbeat_protection_armed': protection.get(
            'heartbeat_protection_armed'
        ),
        'self_collide': data.get('self_collide'),
    }
