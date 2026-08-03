"""Pure geometry helpers for exact-frame RGB-D scene understanding."""

from __future__ import annotations

import math

import numpy as np


def _point3(value, name):
    point = np.asarray(value, dtype=np.float64).reshape(-1)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError(f'{name} must contain three finite values')
    return point


def project_camera_point(point_camera, intrinsics, image_width, image_height):
    """Project a camera-frame point and state whether it is inside the image."""
    point = _point3(point_camera, 'point_camera')
    fx, fy, cx, cy = [float(value) for value in intrinsics]
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
        raise ValueError('camera intrinsics must be finite')
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('camera focal lengths must be positive')
    width, height = int(image_width), int(image_height)
    if width <= 0 or height <= 0:
        raise ValueError('image dimensions must be positive')
    if point[2] <= 0.0:
        return {'pixel': None, 'in_front': False, 'inside_image': False}
    u = fx * point[0] / point[2] + cx
    v = fy * point[1] / point[2] + cy
    return {
        'pixel': [float(u), float(v)],
        'in_front': True,
        'inside_image': bool(0.0 <= u < width and 0.0 <= v < height),
    }


def robust_depth_at_pixel(
    depth_image,
    pixel,
    *,
    depth_scale=0.001,
    radius=4,
    minimum_depth=0.15,
    maximum_depth=3.0,
):
    """Return the median valid aligned depth around a projected pixel."""
    depth = np.asarray(depth_image)
    if depth.ndim != 2:
        raise ValueError('depth_image must be two-dimensional')
    if pixel is None:
        return None
    u, v = [int(round(float(value))) for value in pixel]
    radius = max(0, int(radius))
    x1, x2 = max(0, u - radius), min(depth.shape[1], u + radius + 1)
    y1, y2 = max(0, v - radius), min(depth.shape[0], v + radius + 1)
    if x1 >= x2 or y1 >= y2:
        return None
    values = depth[y1:y2, x1:x2].astype(np.float64) * float(depth_scale)
    valid = values[
        np.isfinite(values)
        & (values >= float(minimum_depth))
        & (values <= float(maximum_depth))
    ]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def classify_depth_evidence(predicted_depth, observed_depth, tolerance=0.10):
    """Classify whether aligned depth supports a projected robot keypoint."""
    predicted = float(predicted_depth)
    if not math.isfinite(predicted) or predicted <= 0.0:
        raise ValueError('predicted_depth must be finite and positive')
    if observed_depth is None:
        return {'state': 'unknown', 'depth_error_m': None}
    observed = float(observed_depth)
    if not math.isfinite(observed) or observed <= 0.0:
        return {'state': 'unknown', 'depth_error_m': None}
    error = observed - predicted
    if abs(error) <= float(tolerance):
        state = 'depth_consistent'
    elif error < 0.0:
        state = 'occluded_by_nearer_surface'
    else:
        state = 'not_depth_confirmed'
    return {'state': state, 'depth_error_m': float(error)}


def choose_scene_target(records, label='', track_id=-1):
    """Choose one valid tracked object without coupling to ROS message types."""
    requested_label = str(label).strip().lower()
    requested_track = int(track_id)
    candidates = []
    for record in records:
        if not bool(record.get('depth_valid', False)):
            continue
        if requested_track >= 0 and int(record.get('track_id', -1)) != requested_track:
            continue
        if requested_label and requested_label != '*' and str(record.get('label', '')).lower() != requested_label:
            continue
        candidates.append(record)
    if not candidates:
        return None
    return max(candidates, key=lambda item: float(item.get('confidence', 0.0)))


def relative_geometry(target_base, tcp_base, standoff_distance=0.25):
    """Describe target relative to a TCP and propose a non-executable standoff."""
    target = _point3(target_base, 'target_base')
    tcp = _point3(tcp_base, 'tcp_base')
    tcp_to_target = target - tcp
    distance = float(np.linalg.norm(tcp_to_target))
    if distance <= 1e-9:
        raise ValueError('target and TCP positions are coincident')
    standoff = float(standoff_distance)
    if not math.isfinite(standoff) or standoff <= 0.0:
        raise ValueError('standoff_distance must be finite and positive')
    object_to_tcp_direction = (tcp - target) / distance
    standoff_point = target + object_to_tcp_direction * standoff
    return {
        'tcp_to_target_m': tcp_to_target.tolist(),
        'distance_m': distance,
        'line_of_sight_standoff_base_m': standoff_point.tolist(),
        'tcp_to_standoff_m': (standoff_point - tcp).tolist(),
        'planning_required': True,
    }


def detection_quality(record, image_width, image_height, minimum_confidence=0.50):
    """Return explicit warnings for a target used by scene understanding."""
    warnings = []
    confidence = float(record.get('confidence', 0.0))
    if not math.isfinite(confidence) or confidence < float(minimum_confidence):
        warnings.append('low_detection_confidence')
    depth = float(record.get('median_depth', 0.0))
    if not bool(record.get('depth_valid', False)) or not math.isfinite(depth) or depth <= 0.0:
        warnings.append('invalid_target_depth')
    x = int(record.get('bbox_x', 0))
    y = int(record.get('bbox_y', 0))
    width = int(record.get('bbox_width', 0))
    height = int(record.get('bbox_height', 0))
    if width <= 0 or height <= 0:
        warnings.append('invalid_bbox')
    if x <= 1 or y <= 1 or x + width >= int(image_width) - 1 or y + height >= int(image_height) - 1:
        warnings.append('bbox_touches_image_border')
    return {'usable': not warnings, 'warnings': warnings}
