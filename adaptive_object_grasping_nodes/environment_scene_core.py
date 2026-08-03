"""Validated fixed-environment geometry for MoveIt planning scenes."""

from __future__ import annotations

import math

import numpy as np

from adaptive_object_grasping_nodes.robot_geometry_core import quaternion_matrix


def estimate_horizontal_table(
    depth,
    intrinsics,
    base_from_camera_translation,
    base_from_camera_quaternion,
    *,
    depth_scale=0.001,
    roi_y_min_fraction=0.0,
    roi_y_max_fraction=0.58,
    minimum_depth=0.25,
    maximum_depth=2.0,
    minimum_table_height=0.35,
    maximum_table_height=1.15,
    height_bin=0.008,
    inlier_tolerance=0.012,
    minimum_inliers=2000,
    thickness=0.06,
    footprint_margin=0.04,
):
    """Fit the dominant horizontal support surface in an aligned depth frame.

    The camera-to-base transform is measured for the same frame.  Working in
    the base frame makes the expected table normal explicit and avoids fitting
    a vertical wall that happens to dominate the image.
    """

    depth = np.asarray(depth)
    if depth.ndim != 2 or min(depth.shape) < 10:
        raise ValueError('depth must be a non-empty 2-D image')
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
        raise ValueError('camera intrinsics are invalid')
    scale = float(depth_scale)
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('depth_scale must be finite and positive')
    height, width = depth.shape
    y0 = int(round(height * float(roi_y_min_fraction)))
    y1 = int(round(height * float(roi_y_max_fraction)))
    if y0 < 0 or y1 > height or y0 >= y1:
        raise ValueError('table ROI excludes the complete image')

    # A 2x stride retains ample points while keeping calibration deterministic.
    vv, uu = np.mgrid[y0:y1:2, 0:width:2]
    raw = depth[y0:y1:2, 0:width:2].astype(np.float64)
    metres = raw if np.issubdtype(depth.dtype, np.floating) else raw * scale
    valid = np.isfinite(metres) & (metres >= minimum_depth) & (metres <= maximum_depth)
    if int(valid.sum()) < minimum_inliers:
        raise ValueError('not enough valid depth points in the table ROI')
    z = metres[valid]
    camera_points = np.column_stack((
        (uu[valid] - cx) * z / fx,
        (vv[valid] - cy) * z / fy,
        z,
    ))
    rotation = quaternion_matrix(base_from_camera_quaternion)
    translation = np.asarray(base_from_camera_translation, dtype=np.float64)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError('base_from_camera_translation must contain three finite values')
    base_points = camera_points @ rotation.T + translation
    plausible = (
        (base_points[:, 2] >= minimum_table_height)
        & (base_points[:, 2] <= maximum_table_height)
    )
    base_points = base_points[plausible]
    if len(base_points) < minimum_inliers:
        raise ValueError('not enough points at a plausible table height')

    bins = np.floor(base_points[:, 2] / float(height_bin)).astype(np.int64)
    values, counts = np.unique(bins, return_counts=True)
    dominant = values[int(np.argmax(counts))]
    seed_height = (float(dominant) + 0.5) * float(height_bin)
    inliers = base_points[np.abs(base_points[:, 2] - seed_height) <= inlier_tolerance]
    if len(inliers) < minimum_inliers:
        raise ValueError(
            f'table plane has only {len(inliers)} inliers; require {minimum_inliers}'
        )
    top = float(np.median(inliers[:, 2]))
    x_min, x_max = np.quantile(inliers[:, 0], [0.01, 0.99]).tolist()
    y_min, y_max = np.quantile(inliers[:, 1], [0.01, 0.99]).tolist()
    if x_max - x_min < 0.20 or y_max - y_min < 0.20:
        raise ValueError('estimated table footprint is implausibly small')
    margin = float(footprint_margin)
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError('footprint_margin must be finite and non-negative')
    x_min, x_max = x_min - margin, x_max + margin
    y_min, y_max = y_min - margin, y_max + margin
    residual = np.abs(inliers[:, 2] - top)
    return {
        'id': 'site_table',
        'frame_id': 'Link_Zero_Point',
        'shape': 'box',
        'dimensions_m': [
            float(x_max - x_min),
            float(y_max - y_min),
            float(thickness),
        ],
        'pose': {
            'position_m': [
                float((x_min + x_max) / 2.0),
                float((y_min + y_max) / 2.0),
                float(top - thickness / 2.0),
            ],
            'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
        },
        'measurement': {
            'top_height_m': top,
            'inlier_count': int(len(inliers)),
            'median_absolute_residual_m': float(np.median(residual)),
            'maximum_accepted_residual_m': float(inlier_tolerance),
            'footprint_quantiles': [0.01, 0.99],
        },
    }


def fuse_table_estimates(estimates, *, maximum_height_spread=0.03):
    """Robustly fuse repeated RGB-D table boxes in the base frame.

    Each input already includes the configured footprint margin.  Taking the
    median of the four footprint edges prevents one incomplete or noisy depth
    frame from expanding the PlanningScene box.  A large top-height spread is
    rejected because it usually means that different horizontal surfaces were
    selected or that the live camera transform is inconsistent.
    """

    if not isinstance(estimates, (list, tuple)) or not estimates:
        raise ValueError('at least one table estimate is required')
    rows = []
    for table in estimates:
        if not isinstance(table, dict) or table.get('shape') != 'box':
            raise ValueError('every table estimate must be a box')
        dimensions = np.asarray(table.get('dimensions_m', []), dtype=np.float64)
        position = np.asarray(table.get('pose', {}).get('position_m', []), dtype=np.float64)
        measurement = table.get('measurement', {})
        if dimensions.shape != (3,) or position.shape != (3,):
            raise ValueError('table estimate dimensions and position must have length three')
        if not np.isfinite(dimensions).all() or not np.isfinite(position).all():
            raise ValueError('table estimate contains non-finite geometry')
        if np.any(dimensions <= 0.0):
            raise ValueError('table estimate dimensions must be positive')
        top = float(measurement.get('top_height_m', position[2] + dimensions[2] / 2.0))
        if not math.isfinite(top):
            raise ValueError('table top height must be finite')
        rows.append({
            'x_min': float(position[0] - dimensions[0] / 2.0),
            'x_max': float(position[0] + dimensions[0] / 2.0),
            'y_min': float(position[1] - dimensions[1] / 2.0),
            'y_max': float(position[1] + dimensions[1] / 2.0),
            'top': top,
            'thickness': float(dimensions[2]),
            'inliers': int(measurement.get('inlier_count', 0)),
            'residual': float(measurement.get('median_absolute_residual_m', 0.0)),
        })
    heights = np.asarray([row['top'] for row in rows])
    height_spread = float(np.ptp(heights))
    if height_spread > float(maximum_height_spread):
        raise ValueError(
            f'table top height spread {height_spread:.4f} m exceeds '
            f'{float(maximum_height_spread):.4f} m'
        )
    x_min = float(np.median([row['x_min'] for row in rows]))
    x_max = float(np.median([row['x_max'] for row in rows]))
    y_min = float(np.median([row['y_min'] for row in rows]))
    y_max = float(np.median([row['y_max'] for row in rows]))
    top = float(np.median(heights))
    thickness = float(np.median([row['thickness'] for row in rows]))
    if x_max <= x_min or y_max <= y_min:
        raise ValueError('fused table footprint is empty')
    return {
        'id': 'site_table',
        'frame_id': 'Link_Zero_Point',
        'shape': 'box',
        'dimensions_m': [x_max - x_min, y_max - y_min, thickness],
        'pose': {
            'position_m': [
                (x_min + x_max) / 2.0,
                (y_min + y_max) / 2.0,
                top - thickness / 2.0,
            ],
            'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
        },
        'measurement': {
            'top_height_m': top,
            'frame_count': len(rows),
            'top_height_spread_m': height_spread,
            'total_inlier_count': sum(row['inliers'] for row in rows),
            'median_frame_residual_m': float(np.median([row['residual'] for row in rows])),
            'fusion_method': 'median_top_and_footprint_edges',
        },
    }


def validate_environment_scene(data, *, require_verified=True):
    """Return normalized collision boxes or raise on an unsafe scene file."""

    if not isinstance(data, dict):
        raise ValueError('environment scene must be a mapping')
    if require_verified and data.get('verified') is not True:
        raise ValueError('environment scene has not been verified on site')
    objects = data.get('collision_objects')
    if not isinstance(objects, list) or not objects:
        raise ValueError('environment scene must contain collision_objects')
    normalized = []
    seen = set()
    for item in objects:
        if not isinstance(item, dict) or item.get('shape') != 'box':
            raise ValueError('only explicit box collision objects are supported')
        object_id = str(item.get('id', '')).strip()
        if not object_id or object_id in seen:
            raise ValueError('collision object ids must be non-empty and unique')
        seen.add(object_id)
        frame_id = str(item.get('frame_id', '')).strip()
        dimensions = [float(value) for value in item.get('dimensions_m', [])]
        pose = item.get('pose', {})
        position = [float(value) for value in pose.get('position_m', [])]
        quaternion = [float(value) for value in pose.get('quaternion_xyzw', [])]
        if frame_id != 'Link_Zero_Point':
            raise ValueError(f'{object_id}: frame must be Link_Zero_Point')
        if len(dimensions) != 3 or any(not math.isfinite(v) or v <= 0 for v in dimensions):
            raise ValueError(f'{object_id}: dimensions must be three positive finite values')
        if len(position) != 3 or any(not math.isfinite(v) for v in position):
            raise ValueError(f'{object_id}: position must contain three finite values')
        if len(quaternion) != 4 or any(not math.isfinite(v) for v in quaternion):
            raise ValueError(f'{object_id}: quaternion must contain four finite values')
        norm = math.sqrt(sum(v * v for v in quaternion))
        if norm < 1e-9 or abs(norm - 1.0) > 1e-3:
            raise ValueError(f'{object_id}: quaternion must be normalized')
        normalized.append({
            'id': object_id,
            'frame_id': frame_id,
            'dimensions_m': dimensions,
            'position_m': position,
            'quaternion_xyzw': [value / norm for value in quaternion],
        })
    return normalized
