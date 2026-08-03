#!/usr/bin/env python3
"""Estimate a table collision box from an exact RGB-D scene sample.

The output is deliberately unverified.  An operator must compare the measured
top height and footprint against the physical table before setting verified.
No ROS command or motion topic is used by this tool.
"""

import argparse
import json
from pathlib import Path
import sys

import cv2
import yaml

from adaptive_object_grasping_nodes.environment_scene_core import (
    estimate_horizontal_table,
    fuse_table_estimates,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'sample_directory', nargs='+',
        help='one or more exact RGB-D scene sample directories',
    )
    parser.add_argument('--output', default='config/environment_scene.yaml')
    parser.add_argument('--minimum-inliers', type=int, default=2000)
    args = parser.parse_args()

    samples = [Path(value).expanduser().resolve() for value in args.sample_directory]
    estimates = []
    for sample in samples:
        report = json.loads((sample / 'scene.json').read_text(encoding='utf-8'))
        capture = report['capture']
        transform = capture.get('base_from_camera')
        if not isinstance(transform, dict):
            raise RuntimeError(
                'scene sample predates camera-transform recording; capture a new sample first'
            )
        depth = cv2.imread(str(sample / 'aligned_depth.png'), cv2.IMREAD_UNCHANGED)
        if depth is None:
            raise RuntimeError(f'{sample}/aligned_depth.png cannot be read')
        estimates.append(estimate_horizontal_table(
            depth,
            capture['intrinsics_fx_fy_cx_cy'],
            transform['translation_m'],
            transform['quaternion_xyzw'],
            depth_scale=float(capture['depth_scale']),
            minimum_inliers=args.minimum_inliers,
        ))
    table = estimates[0] if len(estimates) == 1 else fuse_table_estimates(estimates)
    result = {
        'robot_id': 306,
        'frame_id': 'Link_Zero_Point',
        'source_sample': str(samples[0]),
        'source_samples': [str(sample) for sample in samples],
        'operator': None,
        'timestamp': None,
        'verified': False,
        'verification_note': (
            'Measure table top height and footprint on site; set verified true only '
            'after the YAML values agree and RViz shows no robot/environment overlap.'
        ),
        'collision_objects': [table],
    }
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(result, allow_unicode=True, sort_keys=False),
        encoding='utf-8',
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
