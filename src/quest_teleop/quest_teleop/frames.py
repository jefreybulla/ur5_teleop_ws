"""Frame conventions and small SO(3) helpers (pure numpy, no ROS).

WebXR reference spaces are right-handed with +X right, +Y up and +Z pointing
back toward the user (-Z is "forward"). ROS REP-103 uses +X forward, +Y left,
+Z up. The fixed change of basis C maps a WebXR vector into ROS axes:

    ros_x = -xr_z,  ros_y = -xr_x,  ros_z = xr_y

C is a proper rotation (det = +1), so a WebXR rotation R_xr becomes
R_ros = C @ R_xr @ C.T, which for a quaternion means rotating its vector
part by C and keeping w. Both the world axes and the controller body axes are
re-expressed in ROS convention by this.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

C_XR_TO_ROS = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
)


def xr_position_to_ros(p_xr: Sequence[float]) -> np.ndarray:
    return C_XR_TO_ROS @ np.asarray(p_xr, dtype=float).reshape(3)


def xr_quat_to_ros(q_xr: Sequence[float]) -> np.ndarray:
    """Quaternion [x, y, z, w] in WebXR axes -> [x, y, z, w] in ROS axes."""
    q = np.asarray(q_xr, dtype=float).reshape(4)
    v = C_XR_TO_ROS @ q[:3]
    out = np.array([v[0], v[1], v[2], q[3]])
    n = np.linalg.norm(out)
    return out / n if n > 0 else np.array([0.0, 0.0, 0.0, 1.0])


def quat_to_rotm(q: Sequence[float]) -> np.ndarray:
    """[x, y, z, w] -> 3x3 rotation matrix."""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotm_to_quat(r: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> [x, y, z, w] (Shepperd's method)."""
    r = np.asarray(r, dtype=float)
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
             (r[1, 0] - r[0, 1]) / s, 0.25 * s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        q = [0.25 * s, (r[0, 1] + r[1, 0]) / s,
             (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        q = [(r[0, 1] + r[1, 0]) / s, 0.25 * s,
             (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        q = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s,
             0.25 * s, (r[1, 0] - r[0, 1]) / s]
    q = np.array(q)
    if q[3] < 0:  # canonical hemisphere
        q = -q
    return q / np.linalg.norm(q)


def rot_z(angle_rad: float) -> np.ndarray:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotation_angle(r: np.ndarray) -> float:
    """Angle [rad] of a rotation matrix."""
    return math.acos(max(-1.0, min(1.0, (float(np.trace(r)) - 1.0) / 2.0)))


__all__ = [
    'C_XR_TO_ROS',
    'xr_position_to_ros',
    'xr_quat_to_ros',
    'quat_to_rotm',
    'rotm_to_quat',
    'rot_z',
    'rotation_angle',
]
