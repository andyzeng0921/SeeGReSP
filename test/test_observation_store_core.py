import os

import numpy as np
import pytest

from adaptive_object_grasping_nodes.observation_store_core import (
    load_observation,
    observation_path,
    resolve_store_directory,
    write_observation,
)


def _sample():
    color = np.zeros((8, 10, 3), dtype=np.uint8)
    color[:, :, 1] = 127
    depth = np.full((8, 10), 850, dtype=np.uint16)
    mask = np.zeros((8, 10), dtype=bool)
    mask[2:6, 3:8] = True
    return color, depth, mask


def test_observation_round_trip_is_hashed_atomic_and_consumed(tmp_path):
    project = tmp_path / 'project'
    project.mkdir()
    store = resolve_store_directory(project, 'runtime/selected_observations')
    color, depth, mask = _sample()
    token, size, digest = write_observation(
        store,
        color=color,
        depth=depth,
        mask=mask,
        intrinsics=(600.0, 610.0, 5.0, 4.0),
        track_id=7,
        stamp_ns=123456789,
        frame_id='camera',
    )
    path = observation_path(store, token)
    assert os.path.isfile(path)
    assert size == os.path.getsize(path)
    assert len(digest) == 64
    restored = load_observation(
        store,
        token=token,
        expected_size=size,
        expected_sha256=digest,
    )
    np.testing.assert_array_equal(restored['color'], color)
    np.testing.assert_array_equal(restored['depth'], depth)
    np.testing.assert_array_equal(restored['mask'], mask)
    np.testing.assert_allclose(
        restored['intrinsics'], [600.0, 610.0, 5.0, 4.0]
    )
    assert restored['track_id'] == 7
    assert restored['stamp_ns'] == 123456789
    assert restored['frame_id'] == 'camera'
    assert not os.path.exists(path)


def test_observation_store_rejects_escape_and_tampering(tmp_path):
    project = tmp_path / 'project'
    project.mkdir()
    with pytest.raises(ValueError, match='inside project root'):
        resolve_store_directory(project, tmp_path / 'outside')
    store = resolve_store_directory(project, 'runtime/selected_observations')
    with pytest.raises(ValueError, match='invalid observation storage token'):
        observation_path(store, '../escape.npz')
    color, depth, mask = _sample()
    token, size, digest = write_observation(
        store,
        color=color,
        depth=depth,
        mask=mask,
        intrinsics=(600.0, 610.0, 5.0, 4.0),
        track_id=7,
        stamp_ns=123456789,
        frame_id='camera',
    )
    with open(observation_path(store, token), 'ab') as stream:
        stream.write(b'tamper')
    with pytest.raises(ValueError, match='size changed'):
        load_observation(
            store,
            token=token,
            expected_size=size,
            expected_sha256=digest,
        )
