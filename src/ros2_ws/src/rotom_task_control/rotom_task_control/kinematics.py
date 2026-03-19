import math
from dataclasses import dataclass

import numpy as np


def _rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _rotation_from_axis_angle(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    axis_np = np.asarray(axis, dtype=float)
    axis_norm = np.linalg.norm(axis_np)
    if axis_norm <= 1e-12:
        return np.eye(3, dtype=float)
    x, y, z = axis_np / axis_norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_minus_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
            [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
            [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
        ],
        dtype=float,
    )


def _make_transform(translation: tuple[float, float, float], rotation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


@dataclass(frozen=True)
class FKResult:
    position: np.ndarray
    rotation: np.ndarray


class RotomKinematics:
    JOINT_NAMES = ("O", "A", "B", "C")
    JOINT_LOWER = np.array([-1.787524, -3.141593, -0.0421948, -3.141593], dtype=float)
    JOINT_UPPER = np.array([1.787524, 0.405070, 3.141593, 0.377451], dtype=float)

    _JOINT_SEGMENTS = (
        ((0.0, 0.02, 0.0292), (0.0, 0.0, 1.570796), (0.0, 0.0, 1.0)),
        ((0.0344, 0.0, 0.0615), (0.0, 0.0, 1.570796), (-1.0, 0.0, 0.0)),
        ((0.0, 0.0996, 0.0290), (3.141593, 0.0, 0.0), (-1.0, 0.0, 0.0)),
        ((0.0, 0.1215, 0.0), (-1.570796, 0.0, 0.0), (-1.0, 0.0, 0.0)),
    )
    _TOOL_FIXED = ((0.0, 0.064, 0.0), (-1.570796, 0.0, -1.570796))

    def __init__(self) -> None:
        self.base_origin = np.array((0.0, 0.02, 0.0292), dtype=float)

    def clamp(self, q: np.ndarray) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=float), self.JOINT_LOWER, self.JOINT_UPPER)

    def joint_midpoints(self) -> np.ndarray:
        return 0.5 * (self.JOINT_LOWER + self.JOINT_UPPER)

    def forward_kinematics(self, q: np.ndarray) -> FKResult:
        q_np = self.clamp(q)
        transform = np.eye(4, dtype=float)
        for joint_idx, (translation, rpy, axis) in enumerate(self._JOINT_SEGMENTS):
            transform = transform @ _make_transform(translation, _rotation_from_rpy(*rpy))
            transform = transform @ _make_transform((0.0, 0.0, 0.0), _rotation_from_axis_angle(axis, q_np[joint_idx]))

        tool_translation, tool_rpy = self._TOOL_FIXED
        transform = transform @ _make_transform(tool_translation, _rotation_from_rpy(*tool_rpy))
        return FKResult(position=transform[:3, 3].copy(), rotation=transform[:3, :3].copy())

