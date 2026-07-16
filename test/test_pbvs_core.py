import numpy as np

from adaptive_object_grasping_nodes.pbvs_core import (
    StabilityWindow,
    limited_low_pass,
    transform_point,
)


def test_stability_requires_full_time_window():
    window = StabilityWindow(1.0, 0.01, 0.25)
    assert not window.update(0.0, [0.5, 0.0, 0.8]).stable
    assert not window.update(0.5, [0.502, 0.0, 0.8]).stable
    result = window.update(1.01, [0.501, 0.0, 0.8])
    assert result.stable
    assert result.window_motion < 0.01


def test_follow_limit_rejects_large_target_move():
    window = StabilityWindow(1.0, 0.01, 0.20)
    window.update(0.0, [0.5, 0.0, 0.8])
    assert window.within_follow_limit([0.65, 0.0, 0.8])
    assert not window.within_follow_limit([0.75, 0.0, 0.8])


def test_limited_low_pass_caps_each_servo_step():
    output = limited_low_pass([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.5, 0.1)
    np.testing.assert_allclose(output, [0.1, 0.0, 0.0])


def test_transform_point_applies_rotation_then_translation():
    half = np.sqrt(0.5)
    result = transform_point([1.0, 0.0, 0.0], [0.5, 0.0, 0.0], [0.0, 0.0, half, half])
    np.testing.assert_allclose(result, [0.5, 1.0, 0.0], atol=1e-7)
