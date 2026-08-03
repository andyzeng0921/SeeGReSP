import numpy as np

from adaptive_object_grasping_nodes.graspnet_core import (
    apply_grasp_depth_offset,
    axis_tilt_from_horizontal_degrees,
    collision_scene_cloud,
    horizontal_grasp_orientation_variants,
    is_horizontal_grasp,
    matrix_to_quaternion,
    parse_grasp_array,
    rotate_grasp_about_local_axis,
    rotation_distance_degrees,
    sample_point_cloud,
    target_point_cloud,
    transform_grasp_pose,
    wrist_aligned_side_grasp_rows,
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


def test_collision_scene_keeps_local_voxels_and_removes_far_geometry():
    target = np.array(
        [[0.0, 0.0, 1.0], [0.02, 0.02, 1.02]],
        dtype=np.float32,
    )
    scene = np.array(
        [
            [0.00, 0.00, 1.00],
            [0.001, 0.001, 1.001],
            [0.10, 0.00, 1.00],
            [1.00, 1.00, 1.00],
        ],
        dtype=np.float32,
    )
    local = collision_scene_cloud(
        scene, target, margin=0.20, voxel_size=0.01
    )
    assert len(local) == 2
    assert not np.any(np.all(np.isclose(local, [1.0, 1.0, 1.0]), axis=1))


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


def test_grasp_depth_moves_final_pose_along_approach_axis():
    rotation = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    shifted, applied = apply_grasp_depth_offset(
        [0.5, 0.1, 0.8], rotation, 0, 0.03, scale=1.0, maximum=0.05
    )
    np.testing.assert_allclose(shifted, [0.5, 0.13, 0.8])
    assert applied == 0.03


def test_grasp_depth_offset_is_safety_clamped():
    shifted, applied = apply_grasp_depth_offset(
        [0.0, 0.0, 0.0], np.eye(3), 0, 0.20, scale=1.0, maximum=0.05
    )
    np.testing.assert_allclose(shifted, [0.05, 0.0, 0.0])
    assert applied == 0.05


def test_horizontal_grasp_accepts_horizontal_approach_and_closing_axes():
    accepted, approach_tilt, closing_tilt = is_horizontal_grasp(np.eye(3))
    assert accepted
    assert approach_tilt == 0.0
    assert closing_tilt == 0.0


def test_horizontal_grasp_rejects_vertical_closing_axis():
    rotation = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ])
    accepted, approach_tilt, closing_tilt = is_horizontal_grasp(rotation)
    assert not accepted
    assert approach_tilt == 0.0
    assert closing_tilt == 90.0


def test_horizontal_tilt_threshold_allows_small_angle():
    angle = np.radians(10.0)
    rotation = np.array([
        [np.cos(angle), 0.0, -np.sin(angle)],
        [0.0, 1.0, 0.0],
        [np.sin(angle), 0.0, np.cos(angle)],
    ])
    assert np.isclose(axis_tilt_from_horizontal_degrees(rotation, 0), 10.0)
    accepted, _, _ = is_horizontal_grasp(
        rotation,
        maximum_approach_tilt_degrees=15.0,
        maximum_closing_tilt_degrees=15.0,
    )
    assert accepted


def test_local_roll_preserves_grasp_approach_axis():
    rotation = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ])
    rolled = rotate_grasp_about_local_axis(rotation, 0, 90.0)
    np.testing.assert_allclose(rolled[:, 0], rotation[:, 0], atol=1e-8)
    assert np.isclose(np.linalg.det(rolled), 1.0)


def test_bottle_roll_variants_recover_vertical_closing_direction():
    rotation = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ])
    variants = horizontal_grasp_orientation_variants(
        rotation,
        maximum_approach_tilt_degrees=30.0,
        maximum_closing_tilt_degrees=30.0,
        roll_degrees=(90.0, -90.0, 180.0),
    )
    assert [item['roll_degrees'] for item in variants] == [90.0, -90.0]
    for item in variants:
        accepted, approach_tilt, closing_tilt = is_horizontal_grasp(
            item['rotation'],
            maximum_approach_tilt_degrees=30.0,
            maximum_closing_tilt_degrees=30.0,
        )
        assert accepted
        assert approach_tilt == 0.0
        assert np.isclose(closing_tilt, 0.0, atol=1e-10)
        np.testing.assert_allclose(item['rotation'][:, 0], rotation[:, 0], atol=1e-8)


def test_bottle_roll_variants_do_not_repair_unsafe_vertical_approach():
    rotation = np.array([
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
    ])
    variants = horizontal_grasp_orientation_variants(
        rotation,
        maximum_approach_tilt_degrees=30.0,
        maximum_closing_tilt_degrees=30.0,
        roll_degrees=(90.0, -90.0, 180.0),
    )
    assert variants == []


def test_horizontal_grasp_keeps_parallel_jaw_direction_equivalent_branch():
    variants = horizontal_grasp_orientation_variants(
        np.eye(3),
        maximum_approach_tilt_degrees=30.0,
        maximum_closing_tilt_degrees=30.0,
    )
    assert [item['roll_degrees'] for item in variants] == [0.0, 180.0]
    np.testing.assert_allclose(
        variants[0]['rotation'][:, 1],
        -variants[1]['rotation'][:, 1],
        atol=1e-8,
    )


def test_wrist_aligned_bottle_row_points_from_tcp_to_grasp_center():
    row = np.zeros(17)
    row[0:4] = [0.8, 0.06, 0.02, 0.03]
    row[4:13] = np.eye(3).reshape(-1)
    row[13:16] = [0.60, 0.10, 0.80]
    tcp = np.array([0.30, 0.20, 0.90])
    rows = wrist_aligned_side_grasp_rows(
        [row], tcp, np.eye(3), [0.0, 0.0, 1.0]
    )
    assert rows.shape == (1, 17)
    generated = parse_grasp_array(rows[0])
    expected = (row[13:16] - tcp) / np.linalg.norm(row[13:16] - tcp)
    np.testing.assert_allclose(generated['rotation'][:, 0], expected, atol=1e-8)
    assert np.isclose(generated['rotation'][2, 1], 0.0, atol=1e-8)
    np.testing.assert_allclose(generated['translation'], row[13:16])
    assert generated['score'] == row[0]
    assert generated['width'] == row[1]


def test_wrist_aligned_bottle_row_selects_nearest_jaw_equivalent():
    row = np.zeros(17)
    row[0:4] = [0.8, 0.06, 0.02, 0.03]
    row[4:13] = np.eye(3).reshape(-1)
    row[13:16] = [0.60, 0.0, 0.80]
    tcp = np.array([0.30, 0.0, 0.80])
    preferred = np.diag([1.0, -1.0, -1.0])
    generated = parse_grasp_array(
        wrist_aligned_side_grasp_rows(
            [row], tcp, preferred, [0.0, 0.0, 1.0]
        )[0]
    )
    assert rotation_distance_degrees(
        preferred, generated['rotation']
    ) < 1e-6


def test_wrist_aligned_bottle_row_rejects_steep_tcp_approach():
    row = np.zeros(17)
    row[0:4] = [0.8, 0.06, 0.02, 0.03]
    row[4:13] = np.eye(3).reshape(-1)
    row[13:16] = [0.31, 0.0, 1.20]
    rows = wrist_aligned_side_grasp_rows(
        [row], [0.30, 0.0, 0.80], np.eye(3), [0.0, 0.0, 1.0],
        maximum_approach_tilt_degrees=30.0,
    )
    assert rows.shape == (0, 17)


def test_wrist_aligned_bottle_row_uses_exact_rgbd_target_center():
    row = np.zeros(17)
    row[0:4] = [0.8, 0.06, 0.02, 0.03]
    row[4:13] = np.eye(3).reshape(-1)
    row[13:16] = [0.40, -0.20, 0.75]
    tcp = np.array([0.30, 0.20, 0.90])
    exact_center = np.array([0.60, 0.10, 0.80])
    generated = parse_grasp_array(
        wrist_aligned_side_grasp_rows(
            [row],
            tcp,
            np.eye(3),
            [0.0, 0.0, 1.0],
            grasp_center=exact_center,
        )[0]
    )
    expected = (exact_center - tcp) / np.linalg.norm(exact_center - tcp)
    np.testing.assert_allclose(generated['translation'], exact_center)
    np.testing.assert_allclose(generated['rotation'][:, 0], expected, atol=1e-8)
    assert generated['score'] == row[0]
    assert generated['width'] == row[1]
