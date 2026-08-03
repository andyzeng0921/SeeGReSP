import numpy as np
import pytest

from adaptive_object_grasping_nodes.scene_understanding_core import (
    choose_scene_target,
    classify_depth_evidence,
    detection_quality,
    project_camera_point,
    relative_geometry,
    robust_depth_at_pixel,
)


def test_projection_and_image_bounds():
    result = project_camera_point([0.1, -0.05, 1.0], [600.0, 600.0, 320.0, 240.0], 640, 480)
    assert result['inside_image']
    assert result['pixel'] == pytest.approx([380.0, 210.0])
    assert not project_camera_point([0.0, 0.0, -1.0], [600.0, 600.0, 320.0, 240.0], 640, 480)['in_front']


def test_robust_depth_ignores_invalid_values():
    depth = np.zeros((9, 9), dtype=np.uint16)
    depth[3:6, 3:6] = 700
    depth[4, 4] = 2500
    assert robust_depth_at_pixel(depth, [4, 4], radius=1, maximum_depth=2.0) == pytest.approx(0.7)
    assert robust_depth_at_pixel(depth, [100, 100]) is None


@pytest.mark.parametrize(
    ('observed', 'state'),
    [(1.04, 'depth_consistent'), (0.75, 'occluded_by_nearer_surface'), (1.25, 'not_depth_confirmed'), (None, 'unknown')],
)
def test_depth_evidence_states(observed, state):
    assert classify_depth_evidence(1.0, observed, tolerance=0.1)['state'] == state


def test_target_selection_requires_depth_and_prefers_confidence():
    records = [
        {'track_id': 1, 'label': 'bottle', 'confidence': 0.9, 'depth_valid': False},
        {'track_id': 2, 'label': 'bottle', 'confidence': 0.6, 'depth_valid': True},
        {'track_id': 3, 'label': 'bottle', 'confidence': 0.8, 'depth_valid': True},
    ]
    assert choose_scene_target(records, 'bottle')['track_id'] == 3
    assert choose_scene_target(records, track_id=2)['track_id'] == 2
    assert choose_scene_target(records, 'cup') is None


def test_relative_geometry_proposes_standoff_but_requires_planning():
    result = relative_geometry([0.5, 0.0, 0.5], [0.0, 0.0, 0.5], standoff_distance=0.2)
    assert result['distance_m'] == pytest.approx(0.5)
    assert result['tcp_to_target_m'] == pytest.approx([0.5, 0.0, 0.0])
    assert result['line_of_sight_standoff_base_m'] == pytest.approx([0.3, 0.0, 0.5])
    assert result['planning_required'] is True


def test_detection_quality_reports_border_and_low_confidence():
    quality = detection_quality(
        {
            'confidence': 0.4,
            'depth_valid': True,
            'median_depth': 0.7,
            'bbox_x': 0,
            'bbox_y': 10,
            'bbox_width': 40,
            'bbox_height': 80,
        },
        640,
        480,
    )
    assert not quality['usable']
    assert quality['warnings'] == ['low_detection_confidence', 'bbox_touches_image_border']
