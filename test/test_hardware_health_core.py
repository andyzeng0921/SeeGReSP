import copy

from adaptive_object_grasping_nodes.hardware_health_core import (
    JOINT_STATUS_GROUP_SIZES,
    ROBOT_STATE_GROUP_SIZES,
    joint_status_blockers,
    robot_state_blockers,
    robot_state_summary,
)


def healthy_joint_status():
    data = {}
    for group, size in JOINT_STATUS_GROUP_SIZES.items():
        data[group] = {
            'position': [0.0] * size,
            'speed': [0.0] * size,
            'torque': [0.0] * size,
            # AutoLife currently disables temperature reading, so finite zeros
            # mean unavailable rather than proof of a low temperature.
            'temperature': [0.0] * size,
            'communication_lost': [False] * size,
        }
    return data


def healthy_robot_state():
    return {
        'joints_pos': {
            group: [0.0] * size
            for group, size in ROBOT_STATE_GROUP_SIZES.items()
        },
        'self_collide': 0,
        'notifications': {},
        'protection': {
            'state': 'NORMAL',
            'protected': False,
            'reason': 'NONE',
            'heartbeat_expected_mask': 31,
            'heartbeat_ready_mask': 31,
            'heartbeat_lost_mask': 0,
            'heartbeat_protection_armed': True,
        },
    }


def test_healthy_joint_status_passes():
    assert joint_status_blockers(healthy_joint_status()) == []


def test_lost_arm_joint_communication_blocks_execution():
    data = healthy_joint_status()
    data['left_arm_joint_state']['communication_lost'][3] = True
    assert 'lost joint communication' in '; '.join(joint_status_blockers(data))


def test_healthy_robot_protection_state_passes():
    data = healthy_robot_state()
    assert robot_state_blockers(data) == []
    assert robot_state_summary(data)['heartbeat_ready_mask'] == 31


def test_live_306_unarmed_heartbeat_sample_is_blocked():
    data = healthy_robot_state()
    data['protection']['heartbeat_ready_mask'] = 20
    data['protection']['heartbeat_protection_armed'] = False
    blockers = robot_state_blockers(data)
    assert any('heartbeat not ready' in item for item in blockers)
    assert 'heartbeat protection is not armed' in blockers


def test_reduced_expected_heartbeat_mask_is_not_accepted_as_healthy_subset():
    data = healthy_robot_state()
    data['protection']['heartbeat_expected_mask'] = 1
    data['protection']['heartbeat_ready_mask'] = 1
    blockers = robot_state_blockers(data)
    assert any('required exactly 31' in item for item in blockers)
    assert any('heartbeat not ready' in item for item in blockers)


def test_self_collision_blocks_execution():
    data = copy.deepcopy(healthy_robot_state())
    data['self_collide'] = 1
    assert 'robot reports self collision' in robot_state_blockers(data)
