"""Pure dual-arm embodiment rules shared by perception and execution.

This module intentionally has no ROS dependency so the 2 x 7-DoF contract can
be validated before any node starts or any hardware publisher is created.
"""

from dataclasses import dataclass
import math


ARM_DOF = 7
SHARED_BODY_DOF = 4
VENDOR_IK_BODY_DOF = SHARED_BODY_DOF + ARM_DOF
VENDOR_TRAJECTORY_DOF = SHARED_BODY_DOF + (2 * ARM_DOF)

DEFAULT_JOINT_GROUPS = {
    'leg_waist': (
        'Joint_Ankle',
        'Joint_Knee',
        'Joint_Waist_Pitch',
        'Joint_Waist_Yaw',
    ),
    'left_arm': (
        'Joint_Left_Shoulder_Inner',
        'Joint_Left_Shoulder_Outer',
        'Joint_Left_UpperArm',
        'Joint_Left_Elbow',
        'Joint_Left_Forearm',
        'Joint_Left_Wrist_Upper',
        'Joint_Left_Wrist_Lower',
    ),
    'right_arm': (
        'Joint_Right_Shoulder_Inner',
        'Joint_Right_Shoulder_Outer',
        'Joint_Right_UpperArm',
        'Joint_Right_Elbow',
        'Joint_Right_Forearm',
        'Joint_Right_Wrist_Upper',
        'Joint_Right_Wrist_Lower',
    ),
    'neck': (
        'Joint_Neck_Roll',
        'Joint_Neck_Pitch',
        'Joint_Neck_Yaw',
    ),
}

EXPECTED_GROUP_DOF = {
    'leg_waist': SHARED_BODY_DOF,
    'left_arm': ARM_DOF,
    'right_arm': ARM_DOF,
    'neck': 3,
}


@dataclass(frozen=True)
class ArmSelection:
    """Result of assigning one grasp to one arm."""

    arm: str
    object_side: str
    corrected_cross_body_request: bool


def select_arm(
    requested_arm,
    base_y,
    *,
    deadband_y=0.04,
    prevent_cross_body=True,
    neutral_arm='left',
):
    """Choose an arm deterministically from the target lateral position.

    Positive base-frame Y maps to the left arm and negative Y to the right.
    Inside the center deadband an explicit request wins, otherwise
    ``neutral_arm`` is used. An explicit cross-body request is corrected when
    ``prevent_cross_body`` is enabled.
    """

    requested = str(requested_arm or 'auto').strip().lower()
    if requested not in ('auto', 'left', 'right'):
        requested = 'auto'
    neutral = str(neutral_arm).strip().lower()
    if neutral not in ('left', 'right'):
        raise ValueError('neutral_arm must be left or right')
    y = float(base_y)
    deadband = abs(float(deadband_y))
    if not math.isfinite(y) or not math.isfinite(deadband):
        raise ValueError('base_y and deadband_y must be finite')

    if y > deadband:
        object_side = 'left'
    elif y < -deadband:
        object_side = 'right'
    else:
        object_side = 'center'

    if object_side == 'center':
        selected = requested if requested in ('left', 'right') else neutral
        return ArmSelection(selected, object_side, False)

    corrected = (
        bool(prevent_cross_body)
        and requested in ('left', 'right')
        and requested != object_side
    )
    if corrected:
        selected = object_side
    elif requested in ('left', 'right'):
        selected = requested
    else:
        selected = object_side
    return ArmSelection(selected, object_side, corrected)


def validate_joint_groups(joint_groups):
    """Return normalized joint groups or raise on an invalid robot profile."""

    normalized = {}
    all_names = []
    for group, expected_dof in EXPECTED_GROUP_DOF.items():
        if group not in joint_groups:
            raise ValueError(f'missing joint group: {group}')
        names = [str(value).strip() for value in joint_groups[group]]
        if len(names) != expected_dof:
            raise ValueError(
                f'{group} must contain {expected_dof} joints, got {len(names)}'
            )
        if any(not name for name in names):
            raise ValueError(f'{group} contains an empty joint name')
        if len(set(names)) != len(names):
            raise ValueError(f'{group} contains duplicate joint names')
        normalized[group] = names
        all_names.extend(names)

    if len(set(all_names)) != len(all_names):
        raise ValueError('joint names must be unique across robot groups')
    return normalized


def default_joint_groups():
    """Return mutable copies of the canonical AutoLife S2 joint groups."""

    return {group: list(names) for group, names in DEFAULT_JOINT_GROUPS.items()}


def vendor_trajectory_joint_names(joint_groups):
    """Return the vendor 18-value trajectory order after validating the profile."""

    groups = validate_joint_groups(joint_groups)
    names = groups['leg_waist'] + groups['left_arm'] + groups['right_arm']
    if len(names) != VENDOR_TRAJECTORY_DOF:
        raise ValueError(
            f'vendor trajectory must contain {VENDOR_TRAJECTORY_DOF} joints'
        )
    return names
