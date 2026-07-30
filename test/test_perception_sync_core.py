import pytest

from adaptive_object_grasping_nodes.perception_sync_core import (
    ExactIdentityCache,
    ExactRgbdCache,
    frame_delta_decision,
    freshness_error,
    metadata_pair_decision,
    source_stamp_nanoseconds,
)


def test_vendor_frame_delta_accepts_only_configured_pair():
    assert frame_delta_decision(100, 100, expected_delta=0) == 'match'
    assert frame_delta_decision(99, 100, expected_delta=0) == 'drop_color'
    assert frame_delta_decision(101, 100, expected_delta=0) == 'drop_depth'
    assert (
        frame_delta_decision(
            99, 100, expected_delta=0, allowed_deviation=1
        )
        == 'match'
    )
    with pytest.raises(ValueError):
        frame_delta_decision(99, 100, allowed_deviation=-1)


def test_rgbd_cache_never_pairs_different_timestamps():
    cache = ExactRgbdCache(capacity=4)
    assert cache.add('color', 100, 'camera', 'color-100') is None
    assert cache.add('depth', 101, 'camera', 'depth-101') is None
    assert cache.add('intrinsics', 100, 'camera', 'info-100') is None
    assert cache.latest() is None

    assert cache.add('depth', 100, 'camera', 'depth-100') is not None
    snapshot = cache.latest()
    assert snapshot.stamp_ns == 100
    assert snapshot.color == 'color-100'
    assert snapshot.depth == 'depth-100'
    assert snapshot.intrinsics == 'info-100'


def test_rgbd_cache_reports_complete_coverage_without_payloads():
    cache = ExactRgbdCache(capacity=2)
    for stamp in (100, 200, 300):
        cache.add('color', stamp, 'camera', f'color-{stamp}')
        cache.add('depth', stamp, 'camera', f'depth-{stamp}')
        cache.add('intrinsics', stamp, 'camera', f'info-{stamp}')
    coverage = cache.coverage()
    assert coverage == {
        'complete_count': 2,
        'oldest_complete_stamp_ns': 200,
        'newest_complete_stamp_ns': 300,
        'stream_counts': {'color': 2, 'depth': 2, 'intrinsics': 2},
        'stream_ranges': {
            'color': (200, 300),
            'depth': (200, 300),
            'intrinsics': (200, 300),
        },
    }


def test_vendor_metadata_requires_matching_sequence_id_and_bounded_pts():
    color = {'publish_seq': 20, 'pts_ns': 1_010_000_000}
    depth = {'publish_seq': 20, 'pts_ns': 1_000_000_000}
    assert metadata_pair_decision(10, 10, color, depth) == 'match'
    assert (
        metadata_pair_decision(
            10, 10, color, {**depth, 'publish_seq': 22}
        )
        == 'drop_color'
    )
    assert metadata_pair_decision(10, 11, color, color) == 'drop_both'
    assert (
        metadata_pair_decision(
            10,
            10,
            color,
            {**depth, 'pts_ns': 2_000_000_000},
        )
        == 'drop_both'
    )


def test_source_stamp_uses_capture_pts_only_in_epoch_clock_domain():
    color = {'pts_ns': 900, 'publish_epoch_ns': 1_000}
    depth = {'pts_ns': 890, 'publish_epoch_ns': 990}
    assert source_stamp_nanoseconds(color, depth, 2_000) == 900
    far_clock = {
        'pts_ns': 1,
        'publish_epoch_ns': 20_000_000_000,
    }
    assert (
        source_stamp_nanoseconds(
            far_clock,
            far_clock,
            30_000_000_000,
            maximum_pts_clock_error_seconds=1.0,
        )
        == 20_000_000_000
    )


def test_rgbd_cache_rejects_frame_mismatch_at_same_stamp():
    cache = ExactRgbdCache()
    cache.add('color', 100, 'color_frame', object())
    cache.add('depth', 100, 'depth_frame', object())
    with pytest.raises(ValueError, match='frame IDs differ'):
        cache.add('intrinsics', 100, 'color_frame', object())


def test_mask_cache_requires_both_track_identity_and_exact_stamp():
    cache = ExactIdentityCache()
    cache.add(7, 100, 'camera', 'mask-7-100')
    cache.add(8, 100, 'camera', 'mask-8-100')
    assert cache.get(7, 100).value == 'mask-7-100'
    assert cache.get(8, 100).value == 'mask-8-100'
    assert cache.get(7, 101) is None
    assert cache.get(9, 100) is None


def test_freshness_is_absolute_and_fail_closed():
    second = 1_000_000_000
    assert freshness_error(10 * second, 9_900_000_000, 0.2) == ''
    assert 'data age' in freshness_error(10 * second, 9 * second, 0.2)
    assert 'future' in freshness_error(10 * second, 11 * second, 0.2)
    assert 'zero' in freshness_error(10 * second, 0, 0.2)
