from adaptive_object_grasping_nodes.safety_core import (
    HARDWARE_EXECUTION_CONFIRMATION,
    execution_confirmation_valid,
    site_acceptance_blockers,
)


def complete_site_acceptance():
    common = {'verified': True, 'operator': 'site-operator', 'timestamp': '2026-07-30T15:00:00+08:00'}
    return {
        'controller_adapter': {'verified': True},
        'camera_extrinsic': dict(common),
        'left_tcp': dict(common),
        'right_tcp': dict(common),
        'gripper_mapping': dict(common),
        'emergency_stop': {
            **common,
            'physical_button_stopped_motion_test': True,
            'reset_and_reenable_test': True,
        },
        'workspace': {
            **common,
            'collision_zone_cleared': True,
            'observer_at_emergency_stop': True,
        },
    }


def test_complete_site_acceptance_has_no_blockers():
    assert site_acceptance_blockers(complete_site_acceptance()) == []


def test_missing_physical_estop_test_blocks_execution():
    data = complete_site_acceptance()
    data['emergency_stop']['physical_button_stopped_motion_test'] = False
    assert '未记录实体急停停止动作测试' in site_acceptance_blockers(data)


def test_confirmation_is_exact_and_explicit():
    assert execution_confirmation_valid(HARDWARE_EXECUTION_CONFIRMATION)
    assert not execution_confirmation_valid('')
    assert not execution_confirmation_valid(HARDWARE_EXECUTION_CONFIRMATION.lower())
