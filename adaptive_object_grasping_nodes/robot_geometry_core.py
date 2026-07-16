import json
import math
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(rpy):
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def quaternion_matrix(quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('quaternion norm is zero')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def matrix_quaternion(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        result = [
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ]
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            result = [0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale,
                      (matrix[0, 2] + matrix[2, 0]) / scale,
                      (matrix[2, 1] - matrix[1, 2]) / scale]
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            result = [(matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                      (matrix[1, 2] + matrix[2, 1]) / scale,
                      (matrix[0, 2] - matrix[2, 0]) / scale]
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            result = [(matrix[0, 2] + matrix[2, 0]) / scale,
                      (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale,
                      (matrix[1, 0] - matrix[0, 1]) / scale]
    result = np.asarray(result, dtype=np.float64)
    return (result / np.linalg.norm(result)).tolist()


def pose_matrix(position, orientation):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_matrix(orientation)
    transform[:3, 3] = np.asarray(position, dtype=np.float64)
    return transform


def matrix_pose(transform):
    transform = np.asarray(transform, dtype=np.float64)
    return transform[:3, 3].tolist(), matrix_quaternion(transform[:3, :3])


def tcp_target_to_eef(target_position, target_orientation, tcp_translation, tcp_rotation):
    base_to_tcp = pose_matrix(target_position, target_orientation)
    eef_to_tcp = pose_matrix(tcp_translation, tcp_rotation)
    return matrix_pose(base_to_tcp @ np.linalg.inv(eef_to_tcp))


def parse_robot_joint_feedback(payload, names):
    data = json.loads(payload) if isinstance(payload, str) else payload
    groups = (
        ('leg_waist_joint_state', names['leg_waist']),
        ('left_arm_joint_state', names['left_arm']),
        ('right_arm_joint_state', names['right_arm']),
        ('neck_joint_state', names['neck']),
    )
    result = {}
    for key, joint_names in groups:
        positions = data[key]['position']
        if len(positions) != len(joint_names):
            raise ValueError(f'{key} expected {len(joint_names)} positions, got {len(positions)}')
        result.update(zip(joint_names, (float(value) for value in positions)))
    return result


class UrdfForwardKinematics:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self._joint_by_child = {}
        for joint in root.findall('joint'):
            child = joint.find('child').get('link')
            origin = joint.find('origin')
            axis = joint.find('axis')
            self._joint_by_child[child] = {
                'name': joint.get('name'),
                'type': joint.get('type'),
                'parent': joint.find('parent').get('link'),
                'xyz': _values(None if origin is None else origin.get('xyz'), [0.0, 0.0, 0.0]),
                'rpy': _values(None if origin is None else origin.get('rpy'), [0.0, 0.0, 0.0]),
                'axis': _values(None if axis is None else axis.get('xyz'), [1.0, 0.0, 0.0]),
            }

    def transform(self, base_link, target_link, positions_radians):
        chain = []
        current = target_link
        while current != base_link:
            if current not in self._joint_by_child:
                raise KeyError(f'URDF has no chain from {base_link} to {target_link}')
            joint = self._joint_by_child[current]
            chain.append(joint)
            current = joint['parent']
        transform = np.eye(4, dtype=np.float64)
        for joint in reversed(chain):
            origin = np.eye(4, dtype=np.float64)
            origin[:3, :3] = rpy_matrix(joint['rpy'])
            origin[:3, 3] = joint['xyz']
            transform = transform @ origin
            value = float(positions_radians.get(joint['name'], 0.0))
            if joint['type'] in ('revolute', 'continuous'):
                transform = transform @ _axis_rotation(joint['axis'], value)
            elif joint['type'] == 'prismatic':
                translation = np.eye(4, dtype=np.float64)
                translation[:3, 3] = np.asarray(joint['axis']) * value
                transform = transform @ translation
        return transform


def _axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cosine, sine = math.cos(angle), math.sin(angle)
    rotation = np.array([
        [cosine + x * x * (1 - cosine), x * y * (1 - cosine) - z * sine,
         x * z * (1 - cosine) + y * sine],
        [y * x * (1 - cosine) + z * sine, cosine + y * y * (1 - cosine),
         y * z * (1 - cosine) - x * sine],
        [z * x * (1 - cosine) - y * sine, z * y * (1 - cosine) + x * sine,
         cosine + z * z * (1 - cosine)],
    ], dtype=np.float64)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    return transform


def _values(text, default):
    return list(default) if text is None else [float(value) for value in text.split()]
