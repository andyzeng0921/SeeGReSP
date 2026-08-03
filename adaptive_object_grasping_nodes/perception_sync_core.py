from dataclasses import dataclass


NANOSECONDS_PER_SECOND = 1_000_000_000


def stamp_nanoseconds(stamp):
    """Return a stable integer key for a ROS-like ``sec``/``nanosec`` stamp."""
    return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)


def frame_delta_decision(
    color_frame_id,
    depth_frame_id,
    *,
    expected_delta=0,
    allowed_deviation=0,
):
    """Decide whether two vendor frames are the configured physical pair.

    The delta convention is ``color_frame_id - depth_frame_id``.  A mismatch
    identifies which pending stream is older and should be discarded.
    """
    deviation = int(allowed_deviation)
    if deviation < 0:
        raise ValueError('allowed frame-id deviation must be non-negative')
    actual = int(color_frame_id) - int(depth_frame_id)
    lower = int(expected_delta) - deviation
    upper = int(expected_delta) + deviation
    if lower <= actual <= upper:
        return 'match'
    return 'drop_color' if actual < lower else 'drop_depth'


def metadata_pair_decision(
    color_frame_id,
    depth_frame_id,
    color_meta,
    depth_meta,
    *,
    expected_frame_delta=0,
    allowed_frame_deviation=0,
    maximum_pts_delta_seconds=0.06,
):
    """Validate a physical RGB-D pair using vendor sequence, ID and PTS.

    Positive publish sequences are authoritative and must match exactly. Frame
    IDs are checked as a second independent invariant. PTS values may differ
    slightly between the RealSense color and depth sensors, but an excessive
    delta rejects both samples.
    """
    color_meta = color_meta or {}
    depth_meta = depth_meta or {}
    color_sequence = int(color_meta.get('publish_seq') or 0)
    depth_sequence = int(depth_meta.get('publish_seq') or 0)
    if color_sequence > 0 and depth_sequence > 0:
        if color_sequence < depth_sequence:
            return 'drop_color'
        if color_sequence > depth_sequence:
            return 'drop_depth'

    frame_decision = frame_delta_decision(
        color_frame_id,
        depth_frame_id,
        expected_delta=expected_frame_delta,
        allowed_deviation=allowed_frame_deviation,
    )
    if frame_decision != 'match':
        # Equal producer sequences with inconsistent frame IDs are corrupt as
        # a pair; consuming only one side could manufacture a later match.
        if color_sequence > 0 and color_sequence == depth_sequence:
            return 'drop_both'
        return frame_decision

    color_pts = int(color_meta.get('pts_ns') or 0)
    depth_pts = int(depth_meta.get('pts_ns') or 0)
    maximum_pts_delta_ns = int(
        max(0.0, float(maximum_pts_delta_seconds)) * NANOSECONDS_PER_SECOND
    )
    if (
        color_pts > 0
        and depth_pts > 0
        and abs(color_pts - depth_pts) > maximum_pts_delta_ns
    ):
        return 'drop_both'
    return 'match'


def source_stamp_nanoseconds(
    color_meta,
    depth_meta,
    fallback_now_ns,
    *,
    maximum_pts_clock_error_seconds=5.0,
):
    """Choose the closest trustworthy source timestamp in the ROS epoch.

    PTS is preferable for TF because it describes capture rather than bridge
    publication, but the SDK documents its clock domain as source-specific.
    It is used only when both PTS values are close to their producer epoch
    timestamps. Otherwise the later producer publish epoch is used.
    """
    color_meta = color_meta or {}
    depth_meta = depth_meta or {}
    color_pts = int(color_meta.get('pts_ns') or 0)
    depth_pts = int(depth_meta.get('pts_ns') or 0)
    color_epoch = int(color_meta.get('publish_epoch_ns') or 0)
    depth_epoch = int(depth_meta.get('publish_epoch_ns') or 0)
    maximum_clock_error_ns = int(
        max(0.0, float(maximum_pts_clock_error_seconds))
        * NANOSECONDS_PER_SECOND
    )
    if (
        color_pts > 0
        and depth_pts > 0
        and color_epoch > 0
        and depth_epoch > 0
        and abs(color_pts - color_epoch) <= maximum_clock_error_ns
        and abs(depth_pts - depth_epoch) <= maximum_clock_error_ns
    ):
        # Detection runs on color pixels; use the color capture time.
        return color_pts
    publish_epochs = [
        value for value in (color_epoch, depth_epoch) if value > 0
    ]
    if publish_epochs:
        return max(publish_epochs)
    return int(fallback_now_ns)


@dataclass(frozen=True)
class RgbdSnapshot:
    stamp_ns: int
    frame_id: str
    color: object
    depth: object
    intrinsics: object


class ExactRgbdCache:
    """Small bounded cache that exposes only exact-stamp RGB-D-info triples."""

    _STREAMS = ('color', 'depth', 'intrinsics')

    def __init__(self, capacity=16):
        self._capacity = max(1, int(capacity))
        self._values = {name: {} for name in self._STREAMS}

    def add(self, stream, stamp_ns, frame_id, value):
        if stream not in self._values:
            raise ValueError(f'unknown RGB-D stream: {stream}')
        stamp_ns = int(stamp_ns)
        if stamp_ns <= 0:
            raise ValueError('RGB-D timestamp must be positive')
        self._values[stream][stamp_ns] = (str(frame_id), value)
        self._trim(self._values[stream])
        return self.get(stamp_ns)

    def get(self, stamp_ns):
        stamp_ns = int(stamp_ns)
        if any(stamp_ns not in self._values[name] for name in self._STREAMS):
            return None
        entries = [self._values[name][stamp_ns] for name in self._STREAMS]
        frame_ids = {entry[0] for entry in entries}
        if len(frame_ids) != 1:
            raise ValueError(
                f'RGB-D frame IDs differ at stamp {stamp_ns}: {sorted(frame_ids)}'
            )
        return RgbdSnapshot(
            stamp_ns=stamp_ns,
            frame_id=entries[0][0],
            color=entries[0][1],
            depth=entries[1][1],
            intrinsics=entries[2][1],
        )

    def latest(self, after_stamp_ns=None):
        stamps = set(self._values['color'])
        stamps.intersection_update(self._values['depth'])
        stamps.intersection_update(self._values['intrinsics'])
        if after_stamp_ns is not None:
            stamps = {stamp for stamp in stamps if stamp > int(after_stamp_ns)}
        if not stamps:
            return None
        return self.get(max(stamps))

    def coverage(self):
        """Return timestamp coverage without exposing cached image payloads."""
        complete = set(self._values['color'])
        complete.intersection_update(self._values['depth'])
        complete.intersection_update(self._values['intrinsics'])
        return {
            'complete_count': len(complete),
            'oldest_complete_stamp_ns': min(complete) if complete else 0,
            'newest_complete_stamp_ns': max(complete) if complete else 0,
            'stream_counts': {
                name: len(self._values[name]) for name in self._STREAMS
            },
            'stream_ranges': {
                name: (
                    min(self._values[name]) if self._values[name] else 0,
                    max(self._values[name]) if self._values[name] else 0,
                )
                for name in self._STREAMS
            },
        }

    def _trim(self, values):
        while len(values) > self._capacity:
            del values[min(values)]


@dataclass(frozen=True)
class IdentitySnapshot:
    track_id: int
    stamp_ns: int
    frame_id: str
    value: object


class ExactIdentityCache:
    """Bounded cache addressed by both semantic identity and exact timestamp."""

    def __init__(self, capacity=32):
        self._capacity = max(1, int(capacity))
        self._values = {}

    def add(self, track_id, stamp_ns, frame_id, value):
        track_id = int(track_id)
        stamp_ns = int(stamp_ns)
        if stamp_ns <= 0:
            raise ValueError('identity timestamp must be positive')
        key = (track_id, stamp_ns)
        snapshot = IdentitySnapshot(track_id, stamp_ns, str(frame_id), value)
        self._values[key] = snapshot
        while len(self._values) > self._capacity:
            oldest = min(self._values, key=lambda item: item[1])
            del self._values[oldest]
        return snapshot

    def get(self, track_id, stamp_ns):
        return self._values.get((int(track_id), int(stamp_ns)))


def freshness_error(
    now_ns,
    stamp_ns,
    maximum_age_seconds,
    *,
    maximum_future_seconds=0.02,
):
    """Return a fail-closed freshness error, or an empty string when valid."""
    now_ns = int(now_ns)
    stamp_ns = int(stamp_ns)
    if stamp_ns <= 0:
        return 'timestamp is zero or negative'
    maximum_age = float(maximum_age_seconds)
    maximum_future = float(maximum_future_seconds)
    if maximum_age < 0.0 or maximum_future < 0.0:
        return 'freshness limits must be non-negative'
    age = (now_ns - stamp_ns) / NANOSECONDS_PER_SECOND
    if age < -maximum_future:
        return (
            f'timestamp is {-age:.3f}s in the future; '
            f'maximum={maximum_future:.3f}s'
        )
    if age > maximum_age:
        return f'data age is {age:.3f}s; maximum={maximum_age:.3f}s'
    return ''
