import pytest

from adaptive_object_grasping_nodes.visible_reset_core import (
    ARM_JOINTS,
    validate_visible_reset_pose,
    visible_pose_sample_blockers,
)


def test_visibility_requires_both_wrists_and_tcps_with_depth():
    report = {'arms': {}}
    for arm in ('left', 'right'):
        report['arms'][arm] = {'keypoints': {
            name: {
                'projected_inside_image': True,
                'depth_evidence': {'state': 'depth_consistent'},
            }
            for name in ('wrist', 'tcp')
        }}
    assert visible_pose_sample_blockers(report) == []
    report['arms']['left']['keypoints']['tcp']['projected_inside_image'] = False
    assert visible_pose_sample_blockers(report)


def test_reset_pose_requires_site_verification_and_exact_joint_set():
    data = {
        'verified': True,
        'joint_positions_deg': {
            arm: {name: 0.0 for name in names}
            for arm, names in ARM_JOINTS.items()
        },
    }
    assert len(validate_visible_reset_pose(data)['left']) == 7
    data['verified'] = False
    with pytest.raises(ValueError, match='not been verified'):
        validate_visible_reset_pose(data)
