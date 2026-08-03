"""Cross-check the ROS parameters, URDF and SRDF for the dual-arm robot."""

import xml.etree.ElementTree as ET

from adaptive_object_grasping_nodes.dual_arm_core import validate_joint_groups


def robot_profile_blockers(parameters, urdf_text, srdf_text):
    """Return inconsistencies that would make joint ordering unsafe."""

    blockers = []
    joint_groups = {
        'leg_waist': parameters.get('leg_waist_joint_names', []),
        'left_arm': parameters.get('left_arm_joint_names', []),
        'right_arm': parameters.get('right_arm_joint_names', []),
        'neck': parameters.get('neck_joint_names', []),
    }
    try:
        joint_groups = validate_joint_groups(joint_groups)
    except ValueError as exc:
        return [str(exc)]

    try:
        urdf = ET.fromstring(urdf_text)
        srdf = ET.fromstring(srdf_text)
    except ET.ParseError as exc:
        return [f'invalid robot XML: {exc}']

    urdf_joint_names = {
        element.get('name') for element in urdf.findall('joint')
    }
    urdf_link_names = {
        element.get('name') for element in urdf.findall('link')
    }
    configured_joint_names = {
        name for names in joint_groups.values() for name in names
    }
    missing_joints = sorted(configured_joint_names - urdf_joint_names)
    if missing_joints:
        blockers.append(f'configured joints missing from URDF: {missing_joints}')

    srdf_groups = {
        group.get('name'): group
        for group in srdf.findall('group')
    }
    for arm, parameter_key, joint_key in (
        ('left', 'moveit_left_group', 'left_arm'),
        ('right', 'moveit_right_group', 'right_arm'),
    ):
        group_name = str(parameters.get(parameter_key, ''))
        group = srdf_groups.get(group_name)
        if group is None:
            blockers.append(f'{arm} MoveIt group {group_name!r} is missing from SRDF')
            continue
        srdf_joints = [item.get('name') for item in group.findall('joint')]
        if srdf_joints != joint_groups[joint_key]:
            blockers.append(
                f'{arm} SRDF joint order does not match {joint_key}_joint_names'
            )

    dual_group_name = str(parameters.get('moveit_dual_arm_group', ''))
    dual_group = srdf_groups.get(dual_group_name)
    if dual_group is None:
        blockers.append(f'dual-arm MoveIt group {dual_group_name!r} is missing from SRDF')
    else:
        nested_groups = [item.get('name') for item in dual_group.findall('group')]
        expected_nested = [
            str(parameters.get('moveit_left_group', '')),
            str(parameters.get('moveit_right_group', '')),
        ]
        if nested_groups != expected_nested:
            blockers.append(
                f'dual-arm SRDF group must contain {expected_nested}, got {nested_groups}'
            )

    for parameter_key in ('base_frame', 'moveit_left_tip', 'moveit_right_tip'):
        link_name = str(parameters.get(parameter_key, ''))
        if link_name not in urdf_link_names:
            blockers.append(f'{parameter_key} link {link_name!r} is missing from URDF')
    return blockers
