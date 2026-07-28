from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


def image_buffer_to_array(message):
    encodings = {
        'bgr8': (np.uint8, 3),
        'rgb8': (np.uint8, 3),
        'mono8': (np.uint8, 1),
        '16UC1': (np.uint16, 1),
        '32FC1': (np.float32, 1),
    }
    if message.encoding not in encodings:
        raise ValueError(f'unsupported image encoding: {message.encoding}')
    dtype, channels = encodings[message.encoding]
    array = np.frombuffer(message.data, dtype=dtype)
    expected = int(message.height) * int(message.width) * channels
    if array.size < expected:
        raise ValueError(f'image buffer has {array.size} elements, expected {expected}')
    if channels == 1:
        return array[:expected].reshape(int(message.height), int(message.width)).copy()
    result = array[:expected].reshape(int(message.height), int(message.width), channels).copy()
    if message.encoding == 'rgb8':
        result = result[:, :, ::-1]
    return result


def resize_mask_nearest(mask, height, width):
    if mask.shape == (height, width):
        return mask.astype(bool)
    rows = np.minimum(
        (np.arange(height, dtype=np.float64) * mask.shape[0] / height).astype(int),
        mask.shape[0] - 1,
    )
    cols = np.minimum(
        (np.arange(width, dtype=np.float64) * mask.shape[1] / width).astype(int),
        mask.shape[1] - 1,
    )
    return mask[np.ix_(rows, cols)] > 0.5


def mask_to_image_message(mask, header):
    from sensor_msgs.msg import Image

    result = (np.asarray(mask) > 0).astype(np.uint8) * 255
    if result.ndim != 2:
        raise ValueError(f'mask must be 2D, got {result.shape}')
    message = Image()
    message.header = header
    message.height = int(result.shape[0])
    message.width = int(result.shape[1])
    message.encoding = 'mono8'
    message.is_bigendian = False
    message.step = int(result.shape[1])
    message.data = np.ascontiguousarray(result).tobytes()
    return message


def object_depth_and_pixel(
    depth,
    box,
    mask=None,
    *,
    depth_scale=0.001,
    minimum_depth=0.15,
    maximum_depth=2.5,
):
    height, width = depth.shape[:2]
    x1, y1, x2, y2 = [int(round(value)) for value in box]
    x1, x2 = sorted((max(0, min(width - 1, x1)), max(1, min(width, x2))))
    y1, y2 = sorted((max(0, min(height - 1, y1)), max(1, min(height, y2))))
    selection = np.zeros((height, width), dtype=bool)
    selection[y1:y2, x1:x2] = True
    if mask is not None:
        selection &= resize_mask_nearest(np.asarray(mask), height, width)
    scale = 1.0 if np.issubdtype(depth.dtype, np.floating) else float(depth_scale)
    depth_m = depth.astype(np.float64) * scale
    valid = selection & np.isfinite(depth_m)
    valid &= (depth_m >= minimum_depth) & (depth_m <= maximum_depth)
    rows, cols = np.nonzero(valid)
    if rows.size == 0:
        return None
    values = depth_m[rows, cols]
    median = float(np.median(values))
    close = np.abs(values - median) <= max(0.02, median * 0.04)
    if np.any(close):
        rows, cols = rows[close], cols[close]
    return median, float(np.median(cols)), float(np.median(rows))


def deproject_pixel(u, v, depth, intrinsics):
    if min(intrinsics.fx, intrinsics.fy) <= 0.0:
        raise ValueError('camera focal lengths must be positive')
    return np.array([
        (float(u) - intrinsics.cx) * float(depth) / intrinsics.fx,
        (float(v) - intrinsics.cy) * float(depth) / intrinsics.fy,
        float(depth),
    ])


def object_menu_lines(objects):
    if not objects:
        return ['当前画面没有检测到可选物体']
    lines = ['当前可抓取物体：']
    for item in sorted(objects, key=lambda value: (value.label, value.track_id)):
        depth = f'{item.median_depth:.2f}m' if item.depth_valid else '无有效深度'
        lines.append(
            f'[{item.track_id}] {item.label} 置信度{item.confidence:.2f} 距离{depth}'
        )
    return lines
