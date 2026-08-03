import numpy as np
import pytest

from adaptive_object_grasping_nodes.environment_scene_core import (
    estimate_horizontal_table,
    fuse_table_estimates,
    validate_environment_scene,
)


def test_horizontal_table_from_depth_and_identity_transform():
    depth = np.full((480, 640), 0.75, dtype=np.float32)
    table = estimate_horizontal_table(
        depth,
        [500.0, 500.0, 320.0, 240.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        roi_y_min_fraction=0.0,
        roi_y_max_fraction=1.0,
        minimum_inliers=1000,
    )
    assert table['measurement']['top_height_m'] == pytest.approx(0.75, abs=0.005)
    assert table['dimensions_m'][0] > 0.4
    assert table['dimensions_m'][1] > 0.2


def test_environment_requires_verified_and_normalized_boxes():
    data = {
        'verified': True,
        'collision_objects': [{
            'id': 'table',
            'frame_id': 'Link_Zero_Point',
            'shape': 'box',
            'dimensions_m': [1.0, 0.8, 0.06],
            'pose': {
                'position_m': [0.5, 0.0, 0.7],
                'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
            },
        }],
    }
    assert validate_environment_scene(data)[0]['id'] == 'table'
    data['verified'] = False
    with pytest.raises(ValueError, match='not been verified'):
        validate_environment_scene(data)


def test_fuse_table_estimates_uses_median_edges_and_rejects_height_instability():
    def table(x, y, width, depth, top):
        return {
            'shape': 'box',
            'dimensions_m': [width, depth, 0.06],
            'pose': {'position_m': [x, y, top - 0.03]},
            'measurement': {
                'top_height_m': top,
                'inlier_count': 100,
                'median_absolute_residual_m': 0.004,
            },
        }

    fused = fuse_table_estimates([
        table(0.8, 0.0, 0.6, 0.8, 0.77),
        table(0.81, 0.01, 0.6, 0.8, 0.771),
        table(1.5, 1.0, 2.0, 2.0, 0.769),
    ])
    assert fused['measurement']['frame_count'] == 3
    assert fused['measurement']['top_height_m'] == pytest.approx(0.77)
    assert fused['dimensions_m'][0] == pytest.approx(0.61)
    assert fused['pose']['position_m'][0] == pytest.approx(0.805)

    with pytest.raises(ValueError, match='height spread'):
        fuse_table_estimates([
            table(0.8, 0.0, 0.6, 0.8, 0.77),
            table(0.8, 0.0, 0.6, 0.8, 0.85),
        ])
