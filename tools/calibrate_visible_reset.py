#!/usr/bin/env python3
"""Capture joint feedback for a visually verified reset-pose sample.

This tool is read-only with respect to the robot.  It rejects samples unless
both wrists and TCPs are inside the image and supported by aligned depth.
"""

import argparse
import json
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from adaptive_object_grasping_nodes.visible_reset_core import (
    ARM_JOINTS,
    visible_pose_sample_blockers,
)


class JointSnapshot(Node):
    def __init__(self, topic):
        super().__init__('visible_reset_joint_snapshot')
        self.payload = None
        self.create_subscription(String, topic, self._on_message, 10)

    def _on_message(self, message):
        try:
            data = json.loads(message.data)
            groups = {
                'left': data['left_arm_joint_state']['position'],
                'right': data['right_arm_joint_state']['position'],
            }
            if all(len(values) == 7 for values in groups.values()):
                self.payload = groups
        except Exception as exc:
            self.get_logger().warning(f'joint snapshot rejected: {exc}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sample_directory')
    parser.add_argument('--output', default='config/visible_reset_pose.yaml')
    parser.add_argument(
        '--joint-topic',
        default='/topic_arm_whole_body_and_gripper_current_joints_status_0_306',
    )
    parser.add_argument('--timeout', type=float, default=5.0)
    args = parser.parse_args()

    sample = Path(args.sample_directory).expanduser().resolve()
    report = json.loads((sample / 'scene.json').read_text(encoding='utf-8'))
    blockers = visible_pose_sample_blockers(report)
    if blockers:
        raise RuntimeError('sample is not a visible reset pose: ' + '; '.join(blockers))

    rclpy.init()
    node = JointSnapshot(args.joint_topic)
    deadline = time.monotonic() + max(1.0, args.timeout)
    try:
        while rclpy.ok() and node.payload is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.payload is None:
            raise RuntimeError('fresh joint feedback was not received')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    result = {
        'robot_id': 306,
        'source_sample': str(sample),
        'source_stamp_ns': int(report['capture']['stamp_ns']),
        'operator': None,
        'timestamp': None,
        'verified': False,
        'verification_note': (
            'Set verified true only after MoveIt plans to this pose with the '
            'calibrated environment loaded and an operator reviews the RViz trajectory.'
        ),
        'joint_positions_deg': {
            arm: {
                name: float(value)
                for name, value in zip(ARM_JOINTS[arm], node.payload[arm])
            }
            for arm in ('left', 'right')
        },
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
