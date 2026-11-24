"""Clutched relative-pose mapping from controller to UR5 tool goal (no ROS).

While the clutch (squeeze) is held, the tool goal is

    p_goal = p_ee0 + s * R_a (p_c - p_c0)
    R_goal = (R_a  R_c R_c0^T  R_a^T) R_ee0

where (p_c0, R_c0) and (p_ee0, R_ee0) are the controller and tool poses at the
moment the clutch closed, R_a rotates controller-world (quest_world, REP-103)
vectors into the UR kinematics ``base`` frame, and s is a motion scale.
Releasing the clutch freezes the robot, so the operator can re-grip and
"ratchet" to cover the whole workspace (indexing, as in surgical teleop).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from quest_teleop.frames import rot_z

# UR 'base' is 'base_link' rotated 180 deg about z.
R_BASE_FROM_BASE_LINK = rot_z(math.pi)

UR5_D1 = 0.089159  # shoulder height above the base frame [m]


def alignment_rotation(operator_yaw_deg: float) -> np.ndarray:
    """R_a: quest_world -> UR ``base``.

    ``operator_yaw_deg`` = 0 maps "controller forward" (quest_world +x, the
    direction the headset faced at session start) to ``base_link`` +x.
    Rotate in 90 deg steps until pushing the controller away from you moves
    the tool away from you.
    """
    return R_BASE_FROM_BASE_LINK @ rot_z(math.radians(operator_yaw_deg))


@dataclass
class WorkspaceLimits:
    r_max: float = 0.80    # max distance from shoulder [m] (UR5 reach ~0.85)
    r_min_xy: float = 0.15  # keep out of the cylinder around joint 1 axis [m]
    z_min: float = 0.05    # min tool height in base frame [m]
    z_max: float = 0.90

    def clamp(self, p: np.ndarray) -> np.ndarray:
        p = np.asarray(p, dtype=float).copy()
        p[2] = min(max(p[2], self.z_min), self.z_max)
        shoulder = np.array([0.0, 0.0, UR5_D1])
        d = p - shoulder
        n = float(np.linalg.norm(d))
        if n > self.r_max:
            p = shoulder + d * (self.r_max / n)
        rxy = math.hypot(p[0], p[1])
        if rxy < self.r_min_xy:
            if rxy < 1e-9:
                p[0] = self.r_min_xy
            else:
                p[:2] *= self.r_min_xy / rxy
        return p


class ClutchMapper:
    def __init__(
        self,
        r_align: np.ndarray,
        scale: float = 1.0,
        enable_orientation: bool = True,
        limits: Optional[WorkspaceLimits] = None,
    ) -> None:
        self.r_align = np.asarray(r_align, dtype=float).reshape(3, 3)
        self.scale = float(scale)
        self.enable_orientation = enable_orientation
        self.limits = limits or WorkspaceLimits()
        self._c0: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._ee0: Optional[Tuple[np.ndarray, np.ndarray]] = None

    @property
    def engaged(self) -> bool:
        return self._c0 is not None

    def engage(self, p_c, r_c, p_ee, r_ee) -> None:
        self._c0 = (np.asarray(p_c, float).copy(), np.asarray(r_c, float).copy())
        self._ee0 = (np.asarray(p_ee, float).copy(), np.asarray(r_ee, float).copy())

    def release(self) -> None:
        self._c0 = None
        self._ee0 = None

    def target(self, p_c, r_c) -> Tuple[np.ndarray, np.ndarray]:
        if self._c0 is None or self._ee0 is None:
            raise RuntimeError('target() called while clutch is released')
        p_c0, r_c0 = self._c0
        p_ee0, r_ee0 = self._ee0
        dp = self.scale * (self.r_align @ (np.asarray(p_c, float) - p_c0))
        p_goal = self.limits.clamp(p_ee0 + dp)
        if self.enable_orientation:
            dr_world = np.asarray(r_c, float) @ r_c0.T
            dr_base = self.r_align @ dr_world @ self.r_align.T
            r_goal = dr_base @ r_ee0
        else:
            r_goal = r_ee0.copy()
        return p_goal, r_goal


def clip_joint_step(dq: np.ndarray, max_step: float) -> np.ndarray:
    """Uniformly scale dq so no joint moves more than max_step (keeps direction)."""
    dq = np.asarray(dq, dtype=float)
    peak = float(np.max(np.abs(dq))) if dq.size else 0.0
    if peak <= max_step or peak == 0.0:
        return dq
    return dq * (max_step / peak)


def manipulability(j: np.ndarray) -> float:
    """Yoshikawa manipulability sqrt(det(J J^T)); -> 0 near singularities."""
    return math.sqrt(max(0.0, float(np.linalg.det(j @ j.T))))


__all__ = [
    'R_BASE_FROM_BASE_LINK', 'alignment_rotation', 'WorkspaceLimits',
    'ClutchMapper', 'clip_joint_step', 'manipulability',
]
