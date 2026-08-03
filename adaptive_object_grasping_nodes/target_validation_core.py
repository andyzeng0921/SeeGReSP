import math


def _stamp_ns(message):
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


def _bbox(message):
    x1 = float(message.bbox_x)
    y1 = float(message.bbox_y)
    x2 = x1 + float(message.bbox_width)
    y2 = y1 + float(message.bbox_height)
    return x1, y1, x2, y2


def _bbox_iou(first, second):
    ax1, ay1, ax2, ay2 = _bbox(first)
    bx1, by1, bx2, by2 = _bbox(second)
    intersection = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * max(
        0.0, min(ay2, by2) - max(ay1, by1)
    )
    first_area = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    second_area = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = first_area + second_area - intersection
    return 0.0 if union <= 0.0 else intersection / union


def target_revalidation_error(
    previous,
    current,
    *,
    maximum_position_shift_m=0.015,
    maximum_bbox_center_shift_px=12.0,
    minimum_bbox_iou=0.60,
):
    if int(previous.track_id) != int(current.track_id):
        return (
            f'track changed from {previous.track_id} to {current.track_id}'
        )
    if str(previous.label) != str(current.label):
        return f'label changed from {previous.label} to {current.label}'
    if str(previous.header.frame_id) != str(current.header.frame_id):
        return (
            f'camera frame changed from {previous.header.frame_id} '
            f'to {current.header.frame_id}'
        )
    if _stamp_ns(current) <= _stamp_ns(previous):
        return 'validation observation is not newer than the inference source'
    if not previous.depth_valid or not current.depth_valid:
        return 'target depth is invalid during revalidation'
    previous_xyz = (
        float(previous.position_camera.x),
        float(previous.position_camera.y),
        float(previous.position_camera.z),
    )
    current_xyz = (
        float(current.position_camera.x),
        float(current.position_camera.y),
        float(current.position_camera.z),
    )
    if not all(math.isfinite(value) for value in previous_xyz + current_xyz):
        return 'target position is not finite during revalidation'
    position_shift = math.dist(previous_xyz, current_xyz)
    if position_shift > float(maximum_position_shift_m):
        return (
            f'target shifted {position_shift:.3f}m; maximum='
            f'{float(maximum_position_shift_m):.3f}m'
        )
    previous_center = (
        float(previous.bbox_x) + float(previous.bbox_width) / 2.0,
        float(previous.bbox_y) + float(previous.bbox_height) / 2.0,
    )
    current_center = (
        float(current.bbox_x) + float(current.bbox_width) / 2.0,
        float(current.bbox_y) + float(current.bbox_height) / 2.0,
    )
    center_shift = math.dist(previous_center, current_center)
    if center_shift > float(maximum_bbox_center_shift_px):
        return (
            f'target bbox center shifted {center_shift:.1f}px; maximum='
            f'{float(maximum_bbox_center_shift_px):.1f}px'
        )
    overlap = _bbox_iou(previous, current)
    if overlap < float(minimum_bbox_iou):
        return (
            f'target bbox IoU is {overlap:.3f}; minimum='
            f'{float(minimum_bbox_iou):.3f}'
        )
    return ''
