import numpy as np

from adaptive_object_grasping_nodes.graspnet_core import (
    matrix_to_quaternion,
    parse_grasp_array,
    sample_point_cloud,
    target_point_cloud,
    transform_grasp_pose,
)
from adaptive_object_grasping_nodes.perception_core import CameraIntrinsics


def test_target_cloud_uses_bbox_and_depth_band():
    depth = np.full((4, 5), 2000, dtype=np.uint16)
    depth[1:3, 2:4] = 800
    color = np.zeros((4, 5, 3), dtype=np.uint8)
    color[:, :, 2] = 255
    intrinsics = CameraIntrinsics(100.0, 100.0, 2.0, 2.0)
    points, colors = target_point_cloud(
        depth, color, [1, 0, 5, 4], intrinsics, target_depth=0.8, depth_band=0.03
    )
    assert points.shape == (4, 3)
    assert np.allclose(points[:, 2], 0.8)
    assert np.allclose(colors[:, 0], 1.0)


def test_sampling_repeats_when_cloud_is_small():
    points = np.arange(9, dtype=np.float32).reshape(3, 3)
    colors = np.zeros_like(points)
    sampled_points, sampled_colors = sample_point_cloud(
        points, colors, 10, np.random.default_rng(4)
    )
    assert sampled_points.shape == (10, 3)
    assert sampled_colors.shape == (10, 3)


def test_parse_and_transform_grasp_row():
    row = np.zeros(17)
    row[0:4] = [0.9, 0.06, 0.02, 0.03]
    row[4:13] = np.eye(3).reshape(-1)
    row[13:16] = [0.1, 0.2, 0.3]
    grasp = parse_grasp_array(row)
    position, rotation = transform_grasp_pose(
        grasp['translation'], grasp['rotation'], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
    )
    np.testing.assert_allclose(position, [1.1, 0.2, 0.3])
    np.testing.assert_allclose(rotation, np.eye(3))
    np.testing.assert_allclose(matrix_to_quaternion(rotation), [0.0, 0.0, 0.0, 1.0])
