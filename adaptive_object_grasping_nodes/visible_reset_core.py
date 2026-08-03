"""Validation helpers for a camera-visible, site-captured reset pose."""

import math


ARM_JOINTS = {
    'left': [
        'Joint_Left_Shoulder_Inner', 'Joint_Left_Shoulder_Outer',
        'Joint_Left_UpperArm', 'Joint_Left_Elbow', 'Joint_Left_Forearm',
        'Joint_Left_Wrist_Upper', 'Joint_Left_Wrist_Lower',
    ],
    'right': [
        'Joint_Right_Shoulder_Inner', 'Joint_Right_Shoulder_Outer',
        'Joint_Right_UpperArm', 'Joint_Right_Elbow', 'Joint_Right_Forearm',
        'Joint_Right_Wrist_Upper', 'Joint_Right_Wrist_Lower',
    ],
}


def visible_pose_sample_blockers(report):
    blockers = []
    arms = report.get('arms', {}) if isinstance(report, dict) else {}
    for arm in ('left', 'right'):
        arm_report = arms.get(arm, {})
        keypoints = arm_report.get('keypoints', {})
        for name in ('wrist', 'tcp'):
            item = keypoints.get(name)
            if not isinstance(item, dict) or item.get('projected_inside_image') is not True:
                blockers.append(f'{arm} {name} is outside the head-camera image')
                continue
            state = item.get('depth_evidence', {}).get('state')
            if state != 'depth_consistent':
                blockers.append(f'{arm} {name} lacks depth-consistent visual evidence')
    return blockers


def validate_visible_reset_pose(data, *, require_verified=True):
    if not isinstance(data, dict):
        raise ValueError('visible reset pose must be a mapping')
    if require_verified and data.get('verified') is not True:
        raise ValueError('visible reset pose has not been verified on site')
    joints = data.get('joint_positions_deg')
    if not isinstance(joints, dict):
        raise ValueError('joint_positions_deg is required')
    normalized = {}
    for arm, names in ARM_JOINTS.items():
        values = joints.get(arm)
        if not isinstance(values, dict) or set(values) != set(names):
            raise ValueError(f'{arm} reset pose must contain exactly seven canonical joints')
        normalized[arm] = []
        for name in names:
            value = float(values[name])
            if not math.isfinite(value) or abs(value) > 190.0:
                raise ValueError(f'{name} reset value is invalid')
            normalized[arm].append(value)
    return normalized
