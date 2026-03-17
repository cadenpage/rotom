import math
from typing import Iterable

import numpy as np
from geometry_msgs.msg import Transform


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def pose_xyzrpy_to_matrix(values: Iterable[float]) -> np.ndarray:
    xyzrpy = list(values)
    if len(xyzrpy) != 6:
        raise ValueError(f"Expected 6 values [x,y,z,roll,pitch,yaw], got: {xyzrpy}")

    x, y, z, roll, pitch, yaw = [float(v) for v in xyzrpy]
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rpy_to_matrix(roll, pitch, yaw)
    transform[:3, 3] = np.array([x, y, z], dtype=np.float64)
    return transform


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / norm

    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s

    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(rot: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s

    quat = np.array([x, y, z, w], dtype=np.float64)
    quat_norm = np.linalg.norm(quat)
    if quat_norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quat / quat_norm


def transform_msg_to_matrix(transform: Transform) -> np.ndarray:
    t = transform.translation
    q = transform.rotation

    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    out[:3, 3] = np.array([t.x, t.y, t.z], dtype=np.float64)
    return out


def rotation_matrix_to_rotvec(rot: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rot))
    cos_theta = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    theta = math.acos(cos_theta)

    if theta < 1e-9:
        return np.zeros(3, dtype=np.float64)

    sin_theta = math.sin(theta)
    if abs(sin_theta) < 1e-7:
        axis = np.array(
            [
                math.sqrt(max((rot[0, 0] + 1.0) * 0.5, 0.0)),
                math.sqrt(max((rot[1, 1] + 1.0) * 0.5, 0.0)),
                math.sqrt(max((rot[2, 2] + 1.0) * 0.5, 0.0)),
            ],
            dtype=np.float64,
        )
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-12:
            return np.zeros(3, dtype=np.float64)
        axis = axis / axis_norm
        return axis * theta

    axis = np.array(
        [
            rot[2, 1] - rot[1, 2],
            rot[0, 2] - rot[2, 0],
            rot[1, 0] - rot[0, 1],
        ],
        dtype=np.float64,
    ) / (2.0 * sin_theta)
    return axis * theta


def clamp_vector(vec: np.ndarray, max_norm: float) -> np.ndarray:
    if max_norm <= 0.0:
        return np.zeros_like(vec)

    norm = float(np.linalg.norm(vec))
    if norm <= max_norm:
        return vec
    return vec * (max_norm / norm)
