import json
import math

from adaptive_object_grasping_nodes.motion_core import (
    compose_vendor_rrt_target,
    effective_gripper_width,
    parse_eef_feedback,
    pose_error,
    validate_pose,
    validate_vendor_trajectory,
    vendor_pose_payload,
    width_to_gripper_position,
)


def test_feedback_parser_accepts_vendor_flat_format():
    payload = json.dumps({
        'pos_left_in_robot': [0.3, 0.2, 1.0],
        'quat_left_in_robot': [0.0, 0.0, 0.0, 1.0],
        'pos_right_in_robot': [0.3, -0.2, 1.0],
        'quat_right_in_robot': [0.0, 0.0, 0.0, 1.0],
    })
    parsed = parse_eef_feedback(payload)
    assert set(parsed) == {'left', 'right'}


def test_vendor_payload_holds_inactive_arm():
    current = {
        'left': {'position': [0.3, 0.2, 1.0], 'orientation': [0.0, 0.0, 0.0, 1.0]},
        'right': {'position': [0.3, -0.2, 1.0], 'orientation': [0.0, 0.0, 0.0, 1.0]},
    }
    payload = vendor_pose_payload('left', [0.4, 0.25, 0.9], [0.0, 0.0, 0.0, 1.0], current)
    assert payload['pos_left_in_robot'] == [0.4, 0.25, 0.9]
    assert payload['pos_right_in_robot'] == [0.3, -0.2, 1.0]


def test_pose_validation_and_error():
    assert validate_pose([0.3, 0.0, 0.8], [0.0, 0.0, 0.0, 1.0], [0.1, -0.5, 0.2], [0.8, 0.5, 1.4]) == ''
    assert 'outside' in validate_pose([2.0, 0.0, 0.8], [0.0, 0.0, 0.0, 1.0], [0.1, -0.5, 0.2], [0.8, 0.5, 1.4])
    position_error, angle_error = pose_error(
        {'position': [0.3, 0.0, 0.8], 'orientation': [0.0, 0.0, 0.0, 1.0]},
        [0.31, 0.0, 0.8],
        [0.0, 0.0, math.sin(0.05), math.cos(0.05)],
    )
    assert abs(position_error - 0.01) < 1e-9
    assert abs(angle_error - 0.1) < 1e-9


def test_gripper_mapping_opens_for_wider_object():
    narrow = width_to_gripper_position(0.02, 0.10, 0.0, 330.0)
    wide = width_to_gripper_position(0.08, 0.10, 0.0, 330.0)
    assert wide < narrow


def test_gripper_width_scale_maps_nine_centimeters_to_four_point_five():
    position = width_to_gripper_position(
        0.09, 0.095, 10.0, 330.0, width_scale=0.5
    )
    expected = 330.0 + (0.09 * 0.5 / 0.095) * (10.0 - 330.0)
    assert abs(position - expected) < 1e-9


def test_effective_gripper_width_applies_scale_before_limit_check():
    assert abs(effective_gripper_width(0.12, 0.5) - 0.06) < 1e-9
    assert effective_gripper_width(0.12, 0.5) < 0.095


def test_vendor_rrt_target_and_trajectory_contract():
    left = list(range(11))
    right = list(range(20, 31))
    target = compose_vendor_rrt_target(left, right)
    assert target == left + right[4:]
    assert validate_vendor_trajectory([target, [value + 1 for value in target]])[0] == target
