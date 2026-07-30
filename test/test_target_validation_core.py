from types import SimpleNamespace

from adaptive_object_grasping_nodes.target_validation_core import (
    target_revalidation_error,
)


def _target(stamp_ns, *, x=0.1, bbox_x=100):
    return SimpleNamespace(
        track_id=7,
        label='bottle',
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            ),
            frame_id='camera',
        ),
        depth_valid=True,
        position_camera=SimpleNamespace(x=x, y=0.2, z=0.8),
        bbox_x=bbox_x,
        bbox_y=120,
        bbox_width=40,
        bbox_height=100,
    )


def test_target_revalidation_accepts_new_unchanged_observation():
    assert target_revalidation_error(_target(100), _target(200)) == ''


def test_target_revalidation_rejects_motion_and_old_stamp():
    assert 'shifted' in target_revalidation_error(
        _target(100), _target(200, x=0.13)
    )
    assert 'not newer' in target_revalidation_error(
        _target(200), _target(100)
    )
    assert 'bbox center shifted' in target_revalidation_error(
        _target(100), _target(200, bbox_x=140)
    )
