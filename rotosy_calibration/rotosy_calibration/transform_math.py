import math

import numpy as np


def normalize_quaternion(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


def quaternion_to_matrix(q):
    x, y, z, w = normalize_quaternion(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ])
    return matrix


def matrix_to_quaternion(matrix):
    m = np.asarray(matrix, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s

    return normalize_quaternion([x, y, z, w])


def rpy_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return normalize_quaternion([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def make_transform(translation, quaternion):
    transform = quaternion_to_matrix(quaternion)
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def make_transform_from_rotation_translation(rotation, translation):
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = np.asarray(rotation, dtype=float)
    transform[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return transform


def transform_from_msg(msg):
    return make_transform(
        [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z],
        [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w],
    )


def invert_transform(transform):
    transform = np.asarray(transform, dtype=float)
    inverse = np.eye(4, dtype=float)
    rotation = transform[:3, :3]
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ transform[:3, 3]
    return inverse


def transform_from_rvec_tvec(rvec, tvec):
    import cv2

    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
    return transform


def average_transforms(transforms):
    if not transforms:
        raise ValueError('No transforms to average')

    translations = np.array([t[:3, 3] for t in transforms], dtype=float)
    quaternions = [matrix_to_quaternion(t[:3, :3]) for t in transforms]
    reference = quaternions[0]
    aligned = [(-q if np.dot(reference, q) < 0.0 else q) for q in quaternions]

    mean_translation = translations.mean(axis=0)
    mean_quaternion = normalize_quaternion(np.array(aligned).mean(axis=0))
    return make_transform(mean_translation, mean_quaternion)


def rotation_angle(transform):
    rotation = np.asarray(transform, dtype=float)[:3, :3]
    value = (np.trace(rotation) - 1.0) * 0.5
    value = max(-1.0, min(1.0, float(value)))
    return math.acos(value)
