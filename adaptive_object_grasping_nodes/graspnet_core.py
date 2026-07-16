import numpy as np

from adaptive_object_grasping_nodes.pbvs_core import quaternion_matrix


def target_point_cloud(
    depth,
    color,
    box,
    intrinsics,
    *,
    target_depth,
    depth_scale=0.001,
    depth_band=0.08,
    minimum_depth=0.15,
    maximum_depth=2.5,
):
    height, width = depth.shape[:2]
    if color.shape[:2] != (height, width):
        raise ValueError('color and depth dimensions differ')
    x1, y1, x2, y2 = [int(round(value)) for value in box]
    x1, x2 = sorted((max(0, min(width - 1, x1)), max(1, min(width, x2))))
    y1, y2 = sorted((max(0, min(height - 1, y1)), max(1, min(height, y2))))
    scale = 1.0 if np.issubdtype(depth.dtype, np.floating) else float(depth_scale)
    depth_m = depth.astype(np.float64) * scale
    valid = np.zeros((height, width), dtype=bool)
    valid[y1:y2, x1:x2] = True
    valid &= np.isfinite(depth_m)
    valid &= (depth_m >= minimum_depth) & (depth_m <= maximum_depth)
    if target_depth > 0.0:
        valid &= np.abs(depth_m - float(target_depth)) <= float(depth_band)
    rows, cols = np.nonzero(valid)
    if rows.size == 0:
        return np.empty((0, 3), np.float32), np.empty((0, 3), np.float32)
    z = depth_m[rows, cols]
    x = (cols.astype(np.float64) - intrinsics.cx) * z / intrinsics.fx
    y = (rows.astype(np.float64) - intrinsics.cy) * z / intrinsics.fy
    points = np.column_stack((x, y, z)).astype(np.float32)
    colors = color[rows, cols, ::-1].astype(np.float32) / 255.0
    return points, colors


def sample_point_cloud(points, colors, number, random_generator=None):
    if len(points) == 0:
        raise ValueError('target point cloud is empty')
    random_generator = random_generator or np.random.default_rng()
    replace = len(points) < int(number)
    indices = random_generator.choice(len(points), int(number), replace=replace)
    return points[indices], colors[indices]


def matrix_to_quaternion(matrix):
    matrix = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array([
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            ])
        elif index == 1:
            scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array([
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            ])
        else:
            scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array([
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ])
    return quaternion / max(np.linalg.norm(quaternion), 1e-9)


def transform_grasp_pose(position, rotation, tf_translation, tf_quaternion):
    tf_rotation = quaternion_matrix(tf_quaternion)
    position = tf_rotation @ np.asarray(position, dtype=np.float64) + np.asarray(
        tf_translation, dtype=np.float64
    )
    rotation = tf_rotation @ np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    return position, rotation


def parse_grasp_array(row):
    row = np.asarray(row, dtype=np.float64).reshape(-1)
    if row.size < 16:
        raise ValueError(f'GraspNet row has {row.size} fields, expected at least 16')
    return {
        'score': float(row[0]),
        'width': float(row[1]),
        'height': float(row[2]),
        'depth': float(row[3]),
        'rotation': row[4:13].reshape(3, 3),
        'translation': row[13:16],
    }
