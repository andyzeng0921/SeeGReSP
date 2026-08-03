import pytest

from adaptive_object_grasping_nodes.dual_arm_core import (
    ARM_DOF,
    VENDOR_TRAJECTORY_DOF,
    default_joint_groups,
    select_arm,
    validate_joint_groups,
    vendor_trajectory_joint_names,
)


def test_robot_profile_is_two_independent_seven_dof_arms():
    groups = validate_joint_groups(default_joint_groups())
    assert len(groups['left_arm']) == ARM_DOF
    assert len(groups['right_arm']) == ARM_DOF
    assert len(vendor_trajectory_joint_names(groups)) == VENDOR_TRAJECTORY_DOF


def test_arm_selection_uses_robot_base_lateral_axis():
    assert select_arm('auto', 0.20).arm == 'left'
    assert select_arm('auto', -0.20).arm == 'right'


def test_center_deadband_preserves_request_or_uses_neutral_arm():
    assert select_arm('right', 0.01, deadband_y=0.04).arm == 'right'
    assert select_arm('auto', 0.01, deadband_y=0.04).arm == 'left'


def test_cross_body_request_is_corrected_when_guard_enabled():
    decision = select_arm('right', 0.20, prevent_cross_body=True)
    assert decision.arm == 'left'
    assert decision.corrected_cross_body_request is True


def test_cross_body_request_can_be_retained_for_offline_experiments():
    decision = select_arm('right', 0.20, prevent_cross_body=False)
    assert decision.arm == 'right'
    assert decision.corrected_cross_body_request is False


def test_joint_profile_rejects_wrong_dof_and_duplicate_names():
    groups = default_joint_groups()
    groups['left_arm'].pop()
    with pytest.raises(ValueError, match='7 joints'):
        validate_joint_groups(groups)

    groups = default_joint_groups()
    groups['right_arm'][0] = groups['left_arm'][0]
    with pytest.raises(ValueError, match='unique across'):
        validate_joint_groups(groups)
