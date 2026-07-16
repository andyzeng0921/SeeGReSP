from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StabilityResult:
    stable: bool
    stable_elapsed: float
    window_motion: float
    total_displacement: float


class StabilityWindow:
    def __init__(self, required_duration, motion_threshold, maximum_follow_distance):
        self.required_duration = float(required_duration)
        self.motion_threshold = float(motion_threshold)
        self.maximum_follow_distance = float(maximum_follow_distance)
        self.samples = deque()
        self.origin = None

    def reset(self):
        self.samples.clear()
        self.origin = None

    def invalidate_stability(self):
        self.samples.clear()

    def update(self, timestamp, position):
        timestamp = float(timestamp)
        position = np.asarray(position, dtype=np.float64).reshape(3)
        if self.origin is None:
            self.origin = position.copy()
        self.samples.append((timestamp, position.copy()))
        cutoff = timestamp - self.required_duration
        # Keep the last sample just before the cutoff so elapsed time is measurable.
        while len(self.samples) > 1 and self.samples[1][0] <= cutoff:
            self.samples.popleft()
        points = np.asarray([item[1] for item in self.samples])
        center = np.median(points, axis=0)
        window_motion = float(np.max(np.linalg.norm(points - center, axis=1)))
        elapsed = max(0.0, timestamp - self.samples[0][0])
        total = float(np.linalg.norm(position - self.origin))
        stable = elapsed >= self.required_duration and window_motion <= self.motion_threshold
        return StabilityResult(stable, elapsed, window_motion, total)

    def within_follow_limit(self, position):
        if self.origin is None:
            return True
        distance = np.linalg.norm(np.asarray(position, dtype=np.float64) - self.origin)
        return bool(distance <= self.maximum_follow_distance)


def quaternion_matrix(quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm([x, y, z, w])
    if norm < 1e-9:
        return np.eye(3)
    x, y, z, w = np.array([x, y, z, w]) / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def transform_point(point, translation, quaternion):
    point = np.asarray(point, dtype=np.float64).reshape(3)
    translation = np.asarray(translation, dtype=np.float64).reshape(3)
    return quaternion_matrix(quaternion) @ point + translation


def limited_low_pass(previous, target, alpha, maximum_step):
    target = np.asarray(target, dtype=np.float64).reshape(3)
    if previous is None:
        return target.copy()
    previous = np.asarray(previous, dtype=np.float64).reshape(3)
    filtered = previous + float(alpha) * (target - previous)
    delta = filtered - previous
    distance = np.linalg.norm(delta)
    if maximum_step > 0.0 and distance > maximum_step:
        delta *= float(maximum_step) / distance
    return previous + delta
