import numpy as np

from adaptive_object_grasping_nodes.perception_core import (
    CameraIntrinsics,
    deproject_pixel,
    object_depth_and_pixel,
    resize_mask_nearest,
)


def test_deproject_center_pixel_is_on_optical_axis():
    intrinsics = CameraIntrinsics(600.0, 600.0, 320.0, 240.0)
    point = deproject_pixel(320.0, 240.0, 1.2, intrinsics)
    np.testing.assert_allclose(point, [0.0, 0.0, 1.2])


def test_depth_uses_mask_and_rejects_background_outlier():
    depth = np.full((6, 8), 2000, dtype=np.uint16)
    depth[2:5, 3:6] = 750
    depth[3, 4] = 4000
    mask = np.zeros((6, 8), dtype=np.float32)
    mask[2:5, 3:6] = 1.0
    result = object_depth_and_pixel(depth, [1, 1, 7, 6], mask, maximum_depth=3.0)
    assert result is not None
    distance, u, v = result
    assert distance == 0.75
    assert 3.0 <= u <= 5.0
    assert 2.0 <= v <= 4.0


def test_mask_resize_preserves_binary_region():
    mask = np.array([[0.0, 1.0], [0.0, 1.0]])
    resized = resize_mask_nearest(mask, 4, 4)
    assert not resized[:, :2].any()
    assert resized[:, 2:].all()
