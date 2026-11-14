"""Resolved-rates (velocity-based) inverse kinematics."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Sequence, Tuple, Union

import numpy as np

from ur5_kinematics.forward_kinematics import ur5_dirkin
from ur5_kinematics.jacobian import compute_jacobian

DirkinFunc = Callable[[Sequence[float]], np.ndarray]
GoalPose = Union[Mapping[str, Any], Any]


class TrajectoryControlError(RuntimeError):
    """Resolved-rates IK failed to reach the goal within ``max_iter``."""

    def __init__(
        self,
        message: str,
        *,
        iter_count: int,
        delta_p: float,
        delta_w: float,
        q_traj: np.ndarray,
    ) -> None:
        super().__init__(message)
        self.iter_count = iter_count
        self.delta_p = delta_p
        self.delta_w = delta_w
        self.q_traj = q_traj


@dataclass
class TrajectoryControlParams:
    """Parameters for :func:`trajectory_control`."""

    eps_p: float    # Position error threshold [n]
    eps_w: float    # Orientation error threshold [rad]
    v_min: float    # Minimum linear velocity [m/s]
    v_max: float    # Maximum linear velocity [m/s]
    w_min: float    # Minimum angular velocity [rad/s]
    w_max: float    # Maximum angular velocity [rad/s]
    lambda_p: float     # Position decay factor
    lambda_w: float     # Orientation decay factor
    dt: float           # Time step [s]. Must align with frequency at which resolved rates is called
    rho: float = 1e-4   # Singularity robustness parameter
    max_iter: int = 1000
    tool_offset: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=float)
    )

    def __post_init__(self) -> None:
        self.tool_offset = np.asarray(self.tool_offset, dtype=float).reshape(3)


def _default_dirkin(q: Sequence[float]) -> np.ndarray:
    _, _, _, _, frames = ur5_dirkin(q)
    return frames


def tool_pose(t_06: np.ndarray, tool_offset: np.ndarray) -> np.ndarray:
    """Tool-tip position in the world frame."""
    tool_offset = np.asarray(tool_offset, dtype=float).reshape(3)
    return t_06[:3, 3] + t_06[:3, :3] @ tool_offset


def _goal_rotation(x_goal: GoalPose) -> np.ndarray:
    if isinstance(x_goal, Mapping):
        if 'R' in x_goal:
            return np.asarray(x_goal['R'], dtype=float).reshape(3, 3)
        n = np.asarray(x_goal['n'], dtype=float).reshape(3)
        o = np.asarray(x_goal['o'], dtype=float).reshape(3)
        a = np.asarray(x_goal['a'], dtype=float).reshape(3)
        return np.column_stack((n, o, a))

    if hasattr(x_goal, 'R'):
        return np.asarray(x_goal.R, dtype=float).reshape(3, 3)
    n = np.asarray(x_goal.n, dtype=float).reshape(3)
    o = np.asarray(x_goal.o, dtype=float).reshape(3)
    a = np.asarray(x_goal.a, dtype=float).reshape(3)
    return np.column_stack((n, o, a))


def _goal_position(x_goal: GoalPose) -> np.ndarray:
    if isinstance(x_goal, Mapping):
        return np.asarray(x_goal['p'], dtype=float).reshape(3)
    return np.asarray(x_goal.p, dtype=float).reshape(3)


@dataclass
class ResolvedRatesStepResult:
    """Result of one resolved-rates integration step."""

    q: np.ndarray
    delta_p: float
    delta_w: float
    at_goal: bool


def resolved_rates_step(
    q: Sequence[float],
    x_goal: GoalPose,
    params: TrajectoryControlParams,
    dirkin_func: DirkinFunc | None = None,
) -> ResolvedRatesStepResult:
    """One resolved-rates tick: FK, error check, optional ``q += q_dot * dt``."""
    if dirkin_func is None:
        dirkin_func = _default_dirkin

    q_arr = np.asarray(q, dtype=float).reshape(6)
    rd = _goal_rotation(x_goal)
    pd = _goal_position(x_goal)
    tool_offset = params.tool_offset

    frames = dirkin_func(q_arr)
    t_06 = frames[:, :, 5]
    rc = t_06[:3, :3]
    pc_tool = tool_pose(t_06, tool_offset)

    delta_p = float(np.linalg.norm(pd - pc_tool))

    re = rd @ rc.T
    trace_re = float(np.trace(re))
    cos_theta_e = max(-1.0, min(1.0, (trace_re - 1.0) / 2.0))
    theta_e = math.acos(cos_theta_e)

    if abs(math.sin(theta_e)) < 1e-6:
        m_e = np.array(
            [re[2, 1] - re[1, 2], re[0, 2] - re[2, 0], re[1, 0] - re[0, 1]],
            dtype=float,
        ) / 2.0
        m_e = m_e / (float(np.linalg.norm(m_e)) + 1e-10)
    else:
        m_e = np.array(
            [re[2, 1] - re[1, 2], re[0, 2] - re[2, 0], re[1, 0] - re[0, 1]],
            dtype=float,
        ) / (2.0 * math.sin(theta_e))

    delta_w = theta_e

    if delta_p <= params.eps_p and delta_w <= params.eps_w:
        return ResolvedRatesStepResult(
            q=q_arr.copy(),
            delta_p=delta_p,
            delta_w=delta_w,
            at_goal=True,
        )

    if delta_p > 1e-10:
        n_hat = (pd - pc_tool) / delta_p
    else:
        n_hat = np.zeros(3, dtype=float)

    if (delta_p / params.eps_p) > params.lambda_p:
        v_norm = params.v_max
    else:
        v_norm = params.v_min + (params.v_max - params.v_min) * (
            (delta_p - params.eps_p) / (params.eps_p * (params.lambda_p - 1.0))
        )

    p_dot = v_norm * n_hat

    if (delta_w / params.eps_w) > params.lambda_w:
        w_norm = params.w_max
    else:
        w_norm = params.w_min + (params.w_max - params.w_min) * (
            (delta_w - params.eps_w) / (params.eps_w * (params.lambda_w - 1.0))
        )

    if delta_w > 1e-10:
        w_d = w_norm * m_e
    else:
        w_d = np.zeros(3, dtype=float)

    x_dot_d = np.concatenate((p_dot, w_d))

    eye6 = np.eye(6)
    j = compute_jacobian(frames, pc_tool)
    jjt = j @ j.T
    j_pinv = j.T @ np.linalg.inv(jjt + params.rho * eye6)
    q_dot_d = j_pinv @ x_dot_d

    q_next = q_arr + q_dot_d * params.dt
    return ResolvedRatesStepResult(
        q=q_next,
        delta_p=delta_p,
        delta_w=delta_w,
        at_goal=False,
    )


def trajectory_control(
    q_start: Sequence[float],
    x_goal: GoalPose,
    params: TrajectoryControlParams,
    dirkin_func: DirkinFunc | None = None,
) -> Tuple[np.ndarray, bool, int]:
    """Resolved-rates IK: integrate joint angles toward a desired tool pose.

    Args:
        q_start: Starting joint angles [rad], length 6.
        x_goal: Desired pose with ``p`` (3,) and ``R`` (3x3) or ``n``, ``o``, ``a``.
        params: Algorithm thresholds, velocity limits, and step size.
        dirkin_func: Forward kinematics returning ``frames`` (4, 4, 6); defaults to UR5.

    Returns:
        ``q_traj`` with shape (6, num_steps), ``True``, and iteration count.

    Raises:
        TrajectoryControlError: Goal not reached within ``params.max_iter``.
    """
    if dirkin_func is None:
        dirkin_func = _default_dirkin

    q = np.asarray(q_start, dtype=float).reshape(6)

    q_traj_list = [q.copy()]
    converged = False
    iter_count = 0
    delta_p = float('inf')
    delta_w = float('inf')

    for iter_count in range(1, params.max_iter + 1):
        step = resolved_rates_step(q, x_goal, params, dirkin_func)
        delta_p = step.delta_p
        delta_w = step.delta_w

        if step.at_goal:
            converged = True
            break

        q = step.q
        q_traj_list.append(q.copy())

    q_traj = np.column_stack(q_traj_list)

    # Raise error if algorithm does not converge
    if not converged:
        raise TrajectoryControlError(
            (
                f'resolved-rates IK did not converge within max_iter={params.max_iter}: '
                f'position error {delta_p:.6g} m (eps_p={params.eps_p}), '
                f'orientation error {delta_w:.6g} rad (eps_w={params.eps_w})'
            ),
            iter_count=iter_count,
            delta_p=delta_p,
            delta_w=delta_w,
            q_traj=q_traj,
        )

    return q_traj, True, iter_count


__all__ = [
    'ResolvedRatesStepResult',
    'TrajectoryControlError',
    'TrajectoryControlParams',
    'resolved_rates_step',
    'trajectory_control',
    'tool_pose',
]
