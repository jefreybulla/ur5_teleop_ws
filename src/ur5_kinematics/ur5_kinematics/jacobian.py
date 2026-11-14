"""Geometric Jacobian for serial manipulators (UR5 link frames)."""

from __future__ import annotations

from typing import Sequence

import numpy as np


def compute_jacobian(frames: np.ndarray, p_tip: Sequence[float]) -> np.ndarray:
    """Geometric Jacobian at ``p_tip`` using link frames from forward kinematics.

    Args:
        frames: ``frames[:, :, i]`` is the 4x4 transform ^0T_{i+1} for ``i`` in 0..5.
        p_tip: Tool-tip position in the world (base) frame [m], shape (3,).

    Returns:
        6x6 Jacobian ``[J_p; J_w]`` for revolute joints.
    """
    p_tip = np.asarray(p_tip, dtype=float).reshape(3)

    # All joints are revolute
    joint_type = np.ones(6, dtype=float)

    # Initialize Jacobian
    j = np.zeros((6, 6), dtype=float)

    # Compute each column of Jacobian
    for i in range(6):
        if i == 0:
            t_prev = np.eye(4)
        else:
            t_prev = frames[:, :, i - 1]

        # Get position of joint i
        p_i = t_prev[:3, 3]

        # Get z-axis (joint axis) of joint i
        z_i = t_prev[:3, 2]

        # For revolute joint
        if joint_type[i] == 1.0:
            # Linear velocity contribution: J_p = z_i x (p_tip - p_i)
            j[:3, i] = np.cross(z_i, p_tip - p_i)

            # Angular velocity contribution: J_w = z_i
            j[3:6, i] = z_i

    return j


__all__ = ['compute_jacobian']
