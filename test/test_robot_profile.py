from pathlib import Path

import yaml

from adaptive_object_grasping_nodes.robot_profile_core import robot_profile_blockers


def test_motion_config_matches_urdf_and_srdf():
    root = Path(__file__).resolve().parents[1]
    parameters = yaml.safe_load(
        (root / 'config' / 'motion.yaml').read_text(encoding='utf-8')
    )['adaptive_grasp_motion_executor']['ros__parameters']
    urdf_text = (
        root / 'config' / 'moveit' / 'robot_v2_2.urdf'
    ).read_text(encoding='utf-8')
    srdf_text = (
        root / 'config' / 'moveit' / 'autolife_s2.srdf'
    ).read_text(encoding='utf-8')
    assert robot_profile_blockers(parameters, urdf_text, srdf_text) == []
