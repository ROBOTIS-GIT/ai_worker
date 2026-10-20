"""Rotation and angle helpers shared by robot_io and robot_tools.

Angles are radians in the tf2 convention R = Rz(yaw) * Ry(pitch) * Rx(roll).
Quaternions are (x, y, z, w).
"""

import math

import numpy as np


def rpy_to_quat(roll, pitch, yaw):
    """Return (x, y, z, w) for R = Rz(yaw) * Ry(pitch) * Rx(roll)"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_to_matrix(q):
    """Return the 3x3 rotation matrix of quaternion (x, y, z, w)"""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
    ])


def quat_to_rpy(q):
    """Return (roll, pitch, yaw) in the same convention as rpy_to_quat"""
    m = quat_to_matrix(q)
    return (math.atan2(m[2, 1], m[2, 2]),
            math.asin(max(-1.0, min(1.0, -m[2, 0]))),
            math.atan2(m[1, 0], m[0, 0]))


def quat_angle(q1, q2):
    """Smallest rotation angle in degrees between two orientations"""
    a = np.asarray(q1, dtype=float) / np.linalg.norm(q1)
    b = np.asarray(q2, dtype=float) / np.linalg.norm(q2)
    dot = abs(float(np.dot(a, b)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def yaw_from_quat(x, y, z, w):
    """Yaw of a quaternion, for a body that stays level"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(angle):
    """Same angle in (-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))


def base_delta(start, end):
    """Net odom displacement from start to end, in the start base_link frame"""
    dx, dy = end['x'] - start['x'], end['y'] - start['y']
    c, s = math.cos(start['yaw']), math.sin(start['yaw'])
    return {
        'dx': c * dx + s * dy,
        'dy': -s * dx + c * dy,
        'dyaw': wrap_angle(end['yaw'] - start['yaw']),
    }