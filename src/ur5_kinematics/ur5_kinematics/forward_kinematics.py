"""UR5 forward (direct) kinematics using standard DH parameters."""

from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

# Standard DH (Universal Robots documentation)
_A = np.array([0.0, -0.425, -0.39225, 0.0, 0.0, 0.0])
_D = np.array([0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823])
_ALPHA = np.array([math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0])


def _dh_transform(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    cth = math.cos(theta)
    sth = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    return np.array(
        [
            [cth, -sth * ca, sth * sa, a * cth],
            [sth, cth * ca, -cth * sa, a * sth],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def rotm_to_axang(R: np.ndarray) -> np.ndarray:
    """Axis-angle [ux, uy, uz, theta] from a 3x3 rotation matrix."""
    trace_r = float(np.trace(R))
    cos_theta = max(-1.0, min(1.0, (trace_r - 1.0) / 2.0))
    theta = math.acos(cos_theta)

    if theta < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])

    if abs(theta - math.pi) < 1e-6:
        axis_vec = np.sqrt(np.maximum(0.0, (np.diag(R) + 1.0) / 2.0))
        if R[2, 1] - R[1, 2] < 0:
            axis_vec[0] = -axis_vec[0]
        if R[0, 2] - R[2, 0] < 0:
            axis_vec[1] = -axis_vec[1]
        if R[1, 0] - R[0, 1] < 0:
            axis_vec[2] = -axis_vec[2]
        n = float(np.linalg.norm(axis_vec))
        if n < 1e-10:
            return np.array([1.0, 0.0, 0.0, math.pi])
        return np.concatenate((axis_vec / n, [math.pi]))

    axis_vec = np.array(
        [
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ],
        dtype=float,
    ) / (2.0 * math.sin(theta))
    return np.concatenate((axis_vec, [theta]))


def ur5_dirkin(
    q: Sequence[float],
) -> Tuple[float, float, float, np.ndarray, np.ndarray]:
    """Direct kinematics for the Universal Robots UR5.

    Args:
        q: Six joint angles [rad].

    Returns:
        x, y, z: End-effector position in the base frame [m].
        axang: Unit axis and angle [ux, uy, uz, theta] with theta in [rad].
        frames: ``frames[:, :, i]`` is the 4x4 transform ^0T_i.
    """
    q_arr = np.asarray(q, dtype=float).reshape(-1)
    if q_arr.size != 6:
        raise ValueError('q must contain 6 joint values')

    frames = np.zeros((4, 4, 6))
    t_current = np.eye(4)

    for i in range(6):
        a_i = _dh_transform(_A[i], _D[i], _ALPHA[i], float(q_arr[i]))
        t_current = t_current @ a_i
        frames[:, :, i] = t_current

    p = t_current[:3, 3]
    r = t_current[:3, :3]
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    axang = rotm_to_axang(r)
    return x, y, z, axang, frames


Ur5DirectKinematicsResult = Tuple[float, float, float, np.ndarray, np.ndarray]

__all__ = ['ur5_dirkin', 'rotm_to_axang', 'Ur5DirectKinematicsResult']
