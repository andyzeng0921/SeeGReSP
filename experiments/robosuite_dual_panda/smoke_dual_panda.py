#!/usr/bin/env python3
"""Headless, hardware-free smoke test for two independent 7-DoF Panda arms."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import robosuite as suite
from robosuite.controllers import load_composite_controller_config


EXPECTED_ARM_DOF = 7
STEPS = 10


def _reset(env):
    result = env.reset()
    if isinstance(result, tuple):
        return result[0]
    return result


def _step(env, action):
    result = env.step(action)
    if len(result) == 5:
        observation, reward, terminated, truncated, info = result
        return observation, reward, bool(terminated or truncated), info
    return result


def _joint_vector(observation, robot_index):
    key = f"robot{robot_index}_joint_pos"
    if key not in observation:
        available = sorted(k for k in observation if "joint" in k)
        raise KeyError(f"missing {key}; joint observation keys={available}")
    vector = np.asarray(observation[key], dtype=np.float64)
    if vector.shape != (EXPECTED_ARM_DOF,):
        raise AssertionError(
            f"{key} shape={vector.shape}, expected ({EXPECTED_ARM_DOF},)"
        )
    if not np.all(np.isfinite(vector)):
        raise AssertionError(f"{key} contains a non-finite value")
    return vector


def main() -> int:
    controller_config = load_composite_controller_config(controller="BASIC")
    env = suite.make(
        "TwoArmLift",
        robots=["Panda", "Panda"],
        gripper_types="default",
        controller_configs=controller_config,
        env_configuration="parallel",
        has_renderer=False,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        use_object_obs=True,
        reward_shaping=True,
        control_freq=20,
        horizon=STEPS + 1,
    )

    report = {
        "framework": "robosuite",
        "framework_version": getattr(suite, "__version__", "unknown"),
        "environment": "TwoArmLift",
        "configuration": "parallel",
        "robots": ["Panda", "Panda"],
        "expected_arm_dof": EXPECTED_ARM_DOF,
        "hardware_connected": False,
        "steps": 0,
        "passed": False,
    }

    try:
        observation = _reset(env)
        left = _joint_vector(observation, 0)
        right = _joint_vector(observation, 1)

        action_low, action_high = env.action_spec
        action_low = np.asarray(action_low, dtype=np.float64)
        action_high = np.asarray(action_high, dtype=np.float64)
        if action_low.shape != action_high.shape or action_low.ndim != 1:
            raise AssertionError(
                f"invalid action bounds: low={action_low.shape}, "
                f"high={action_high.shape}"
            )
        action = np.clip(np.zeros_like(action_low), action_low, action_high)

        rewards = []
        for _ in range(STEPS):
            observation, reward, done, _ = _step(env, action)
            _joint_vector(observation, 0)
            _joint_vector(observation, 1)
            if not math.isfinite(float(reward)):
                raise AssertionError("reward is non-finite")
            rewards.append(float(reward))
            report["steps"] += 1
            if done:
                break

        report.update(
            {
                "initial_joint_shapes": [list(left.shape), list(right.shape)],
                "action_dimension": int(action.size),
                "reward_min": min(rewards),
                "reward_max": max(rewards),
                "passed": report["steps"] > 0,
            }
        )
    finally:
        env.close()

    output_path = Path(__file__).with_name("smoke-result.json")
    output_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
