#!/usr/bin/env python3
"""Static 2 x 7-DoF profile check; never connects to or commands the robot."""

import argparse
from pathlib import Path
import sys

import yaml

SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from adaptive_object_grasping_nodes.robot_profile_core import robot_profile_blockers


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--root',
        type=Path,
        default=SOURCE_ROOT,
        help='adaptive_object_grasping source root',
    )
    args = parser.parse_args()
    root = args.root.resolve()
    try:
        parameters = (
            yaml.safe_load((root / 'config' / 'motion.yaml').read_text(encoding='utf-8'))
            ['adaptive_grasp_motion_executor']['ros__parameters']
        )
        urdf_text = (
            root / 'config' / 'moveit' / 'robot_v2_2.urdf'
        ).read_text(encoding='utf-8')
        srdf_text = (
            root / 'config' / 'moveit' / 'autolife_s2.srdf'
        ).read_text(encoding='utf-8')
    except Exception as exc:
        print(f'PROFILE INVALID: cannot load profile inputs: {exc}', file=sys.stderr)
        return 2

    blockers = robot_profile_blockers(parameters, urdf_text, srdf_text)
    if blockers:
        print('PROFILE INVALID:', file=sys.stderr)
        for blocker in blockers:
            print(f'  - {blocker}', file=sys.stderr)
        return 2
    print(
        'PROFILE OK: AutoLife S2 = left 7 DoF + right 7 DoF; '
        'vendor trajectory = waist/leg 4 + left 7 + right 7 = 18'
    )
    print(
        'Both_Arms is a planning/collision group only; synchronized bimanual '
        'hardware execution is not enabled.'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
