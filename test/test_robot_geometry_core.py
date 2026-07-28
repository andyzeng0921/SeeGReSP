import math

import numpy as np

from adaptive_object_grasping_nodes.robot_geometry_core import (
    matrix_pose,
    pose_matrix,
    quaternion_matrix,
    tcp_target_to_eef,
)


def test_pose_matrix_round_trip():
    position = [0.3, -0.2, 1.0]
    quaternion = [0.0, 0.0, math.sin(0.25), math.cos(0.25)]
    restored_position, restored_quaternion = matrix_pose(pose_matrix(position, quaternion))
    assert np.allclose(restored_position, position)
    assert abs(np.dot(restored_quaternion, quaternion)) > 1.0 - 1e-9


def test_tcp_target_is_converted_to_eef_target():
    position, quaternion = tcp_target_to_eef(
        [0.5, 0.0, 0.8],
        [0.0, 0.0, 0.0, 1.0],
        [0.2, 0.0, -0.03],
        [0.0, 0.0, 0.0, 1.0],
    )
    assert np.allclose(position, [0.3, 0.0, 0.83])
    assert np.allclose(quaternion, [0.0, 0.0, 0.0, 1.0])


def test_graspnet_width_axis_maps_to_robot_gripper_opening_axis():
    _, eef_quaternion = tcp_target_to_eef(
        [0.5, 0.0, 0.8],
        [0.0, 0.0, 0.0, 1.0],
        [0.2, 0.0, -0.03],
        [-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)],
    )
    base_to_eef = quaternion_matrix(eef_quaternion)
    robot_opening_axis = base_to_eef @ np.array([0.0, 0.0, 1.0])
    # A parallel gripper's opening axis is an unoriented line: +Y and -Y are equivalent.
    assert np.allclose(np.abs(robot_opening_axis), [0.0, 1.0, 0.0])
