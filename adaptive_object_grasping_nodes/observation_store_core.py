import hashlib
import os
import re
import secrets
import stat

import numpy as np


_TOKEN_PATTERN = re.compile(r'^obs-[0-9a-f]{32}\.npz$')
_SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')


def resolve_store_directory(project_root, configured_directory):
    root = os.path.realpath(str(project_root))
    if not root or not os.path.isdir(root):
        raise ValueError(f'project root does not exist: {project_root}')
    configured = str(configured_directory).strip()
    if not configured:
        raise ValueError('observation store directory is empty')
    candidate = (
        configured
        if os.path.isabs(configured)
        else os.path.join(root, configured)
    )
    candidate = os.path.realpath(candidate)
    if os.path.commonpath((root, candidate)) != root:
        raise ValueError(
            f'observation store must stay inside project root {root}: '
            f'{candidate}'
        )
    os.makedirs(candidate, mode=0o700, exist_ok=True)
    candidate = os.path.realpath(candidate)
    if os.path.commonpath((root, candidate)) != root:
        raise ValueError('observation store resolved outside the project root')
    os.chmod(candidate, 0o700)
    return candidate


def observation_path(store_directory, token):
    token = str(token)
    if not _TOKEN_PATTERN.fullmatch(token):
        raise ValueError('invalid observation storage token')
    store = os.path.realpath(str(store_directory))
    path = os.path.realpath(os.path.join(store, token))
    if os.path.commonpath((store, path)) != store:
        raise ValueError('observation token resolved outside the store')
    return path


def _validate_arrays(color, depth, mask, intrinsics):
    color = np.asarray(color)
    depth = np.asarray(depth)
    mask = np.asarray(mask)
    intrinsics = np.asarray(intrinsics, dtype=np.float64)
    if color.dtype != np.uint8 or color.ndim != 3 or color.shape[2] != 3:
        raise ValueError(
            f'color must be uint8 HxWx3, got {color.dtype}/{color.shape}'
        )
    if depth.dtype not in (np.dtype(np.uint16), np.dtype(np.float32)):
        raise ValueError(f'depth must be uint16 or float32, got {depth.dtype}')
    if depth.ndim != 2 or depth.shape != color.shape[:2]:
        raise ValueError(
            f'depth {depth.shape} does not match color {color.shape[:2]}'
        )
    if mask.ndim != 2 or mask.shape != depth.shape:
        raise ValueError(f'mask {mask.shape} does not match depth {depth.shape}')
    if intrinsics.shape != (4,) or not np.isfinite(intrinsics).all():
        raise ValueError('intrinsics must contain four finite values')
    if min(intrinsics[0], intrinsics[1]) <= 0.0:
        raise ValueError('camera focal lengths must be positive')
    return (
        np.ascontiguousarray(color),
        np.ascontiguousarray(depth),
        np.ascontiguousarray(mask > 0, dtype=np.uint8),
        intrinsics,
    )


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def write_observation(
    store_directory,
    *,
    color,
    depth,
    mask,
    intrinsics,
    track_id,
    stamp_ns,
    frame_id,
    maximum_files=32,
):
    color, depth, mask, intrinsics = _validate_arrays(
        color, depth, mask, intrinsics
    )
    track_id = int(track_id)
    stamp_ns = int(stamp_ns)
    frame_id = str(frame_id)
    if stamp_ns <= 0:
        raise ValueError('observation timestamp must be positive')
    if not frame_id:
        raise ValueError('observation frame ID is empty')
    token = f'obs-{secrets.token_hex(16)}.npz'
    final_path = observation_path(store_directory, token)
    temporary_path = final_path + f'.tmp-{secrets.token_hex(8)}'
    descriptor = os.open(
        temporary_path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL,
        0o600,
    )
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            np.savez(
                stream,
                color=color,
                depth=depth,
                mask=mask,
                intrinsics=intrinsics,
                track_id=np.asarray([track_id], dtype=np.int64),
                stamp_ns=np.asarray([stamp_ns], dtype=np.int64),
                frame_id=np.asarray([frame_id]),
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, final_path)
        os.chmod(final_path, 0o600)
    except Exception:
        try:
            os.remove(temporary_path)
        except FileNotFoundError:
            pass
        raise
    payload_size = os.path.getsize(final_path)
    digest = _sha256(final_path)
    prune_observations(
        store_directory,
        maximum_files=maximum_files,
        protected_token=token,
    )
    return token, payload_size, digest


def load_observation(
    store_directory,
    *,
    token,
    expected_size,
    expected_sha256,
    maximum_payload_bytes=64 * 1024 * 1024,
    consume=True,
):
    expected_size = int(expected_size)
    expected_sha256 = str(expected_sha256).lower()
    if expected_size <= 0 or expected_size > int(maximum_payload_bytes):
        raise ValueError(
            f'observation payload size {expected_size} is outside limits'
        )
    if not _SHA256_PATTERN.fullmatch(expected_sha256):
        raise ValueError('invalid observation SHA-256')
    path = observation_path(store_directory, token)
    metadata = os.lstat(path)
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
        raise ValueError('observation payload is not a regular file')
    if metadata.st_size != expected_size:
        raise ValueError(
            f'observation payload size changed: '
            f'{metadata.st_size} != {expected_size}'
        )
    actual_sha256 = _sha256(path)
    if not secrets.compare_digest(actual_sha256, expected_sha256):
        raise ValueError('observation SHA-256 mismatch')
    with np.load(path, allow_pickle=False) as payload:
        required = {
            'color',
            'depth',
            'mask',
            'intrinsics',
            'track_id',
            'stamp_ns',
            'frame_id',
        }
        missing = sorted(required.difference(payload.files))
        if missing:
            raise ValueError(
                'observation payload is missing: ' + ', '.join(missing)
            )
        color, depth, mask, intrinsics = _validate_arrays(
            np.asarray(payload['color']).copy(),
            np.asarray(payload['depth']).copy(),
            np.asarray(payload['mask']).copy(),
            np.asarray(payload['intrinsics']).copy(),
        )
        result = {
            'color': color,
            'depth': depth,
            'mask': mask > 0,
            'intrinsics': intrinsics,
            'track_id': int(np.asarray(payload['track_id']).reshape(-1)[0]),
            'stamp_ns': int(np.asarray(payload['stamp_ns']).reshape(-1)[0]),
            'frame_id': str(np.asarray(payload['frame_id']).reshape(-1)[0]),
        }
    if consume:
        os.remove(path)
    return result


def prune_observations(
    store_directory,
    *,
    maximum_files=32,
    protected_token='',
):
    maximum_files = max(1, int(maximum_files))
    entries = []
    with os.scandir(store_directory) as iterator:
        for entry in iterator:
            if (
                entry.name == protected_token
                or not _TOKEN_PATTERN.fullmatch(entry.name)
                or entry.is_symlink()
                or not entry.is_file(follow_symlinks=False)
            ):
                continue
            entries.append((entry.stat(follow_symlinks=False).st_mtime_ns, entry.name))
    entries.sort(reverse=True)
    allowance = maximum_files - (1 if protected_token else 0)
    for _, token in entries[max(0, allowance) :]:
        os.remove(observation_path(store_directory, token))
