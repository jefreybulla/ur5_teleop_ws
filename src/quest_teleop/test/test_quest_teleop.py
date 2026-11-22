"""Headless tests (no ROS needed): frames, mapping, closed-loop IK, server."""

import asyncio
import math
import socket
import time
import urllib.request

import numpy as np
import pytest

from quest_teleop.fake_quest import run as fake_run
from quest_teleop.frames import (
    C_XR_TO_ROS, quat_to_rotm, rot_z, rotation_angle, rotm_to_quat,
    xr_position_to_ros, xr_quat_to_ros,
)
from quest_teleop.pose_server import PoseServer
from quest_teleop.teleop_core import (
    ClutchMapper, WorkspaceLimits, alignment_rotation, clip_joint_step, manipulability,
)

RNG = np.random.default_rng(0)


def random_quat():
    q = RNG.normal(size=4)
    return q / np.linalg.norm(q)


# ------------------------------------------------------------------ frames
def test_change_of_basis_is_proper_rotation():
    assert np.allclose(C_XR_TO_ROS @ C_XR_TO_ROS.T, np.eye(3))
    assert np.isclose(np.linalg.det(C_XR_TO_ROS), 1.0)


@pytest.mark.parametrize('xr, ros', [
    ([0, 0, -1], [1, 0, 0]),   # WebXR forward -> ROS +x
    ([1, 0, 0], [0, -1, 0]),   # WebXR right   -> ROS -y
    ([0, 1, 0], [0, 0, 1]),    # WebXR up      -> ROS +z
])
def test_axes(xr, ros):
    assert np.allclose(xr_position_to_ros(xr), ros)


def test_quaternion_conversion_matches_similarity_transform():
    for _ in range(200):
        q = random_quat()
        r_expected = C_XR_TO_ROS @ quat_to_rotm(q) @ C_XR_TO_ROS.T
        assert np.allclose(quat_to_rotm(xr_quat_to_ros(q)), r_expected, atol=1e-12)


def test_quat_rotm_round_trip():
    for _ in range(200):
        r = quat_to_rotm(random_quat())
        assert np.allclose(quat_to_rotm(rotm_to_quat(r)), r, atol=1e-12)


# ------------------------------------------------------------------ mapping
def test_translation_mapping_yaw0():
    m = ClutchMapper(alignment_rotation(0.0))
    p_ee0 = np.array([-0.4, -0.1, 0.4])
    m.engage([0, 0, 1.0], np.eye(3), p_ee0, np.eye(3))
    p, r = m.target([0.05, 0, 1.0], np.eye(3))   # controller 5 cm forward
    # +x in base_link == -x in UR 'base'
    assert np.allclose(p - p_ee0, [-0.05, 0, 0])
    assert np.allclose(r, np.eye(3))


def test_rotation_mapping_is_relative():
    m = ClutchMapper(alignment_rotation(90.0))
    r_ee0 = quat_to_rotm(random_quat())
    r_c0 = quat_to_rotm(random_quat())
    m.engage([0, 0, 1], r_c0, [-0.4, 0, 0.4], r_ee0)
    _, r_goal = m.target([0, 0, 1], r_c0)          # no controller rotation
    assert np.allclose(r_goal, r_ee0)
    _, r_goal = m.target([0, 0, 1], rot_z(0.3) @ r_c0)   # 0.3 rad yaw in world
    assert np.isclose(rotation_angle(r_goal @ r_ee0.T), 0.3)


def test_workspace_clamp():
    lim = WorkspaceLimits(r_max=0.8, r_min_xy=0.15, z_min=0.05)
    p = lim.clamp([2.0, 0.0, 0.5])
    assert np.linalg.norm(p - [0, 0, 0.089159]) <= 0.8 + 1e-9
    assert lim.clamp([0.0, 0.0, 0.5])[0] == pytest.approx(0.15)
    assert lim.clamp([0.4, 0.0, -1.0])[2] == pytest.approx(0.05)


def test_clip_joint_step_keeps_direction():
    dq = np.array([0.1, -0.4, 0.0, 0.0, 0.2, 0.0])
    out = clip_joint_step(dq, 0.02)
    assert np.max(np.abs(out)) == pytest.approx(0.02)
    assert np.allclose(out / np.linalg.norm(out), dq / np.linalg.norm(dq))


# ------------------------------------------------------- closed loop with our IK
def test_closed_loop_tracks_controller_motion():
    from ur5_kinematics.forward_kinematics import ur5_dirkin
    from ur5_kinematics.resolved_rates import TrajectoryControlParams, resolved_rates_step

    dt = 1 / 50
    params = TrajectoryControlParams(eps_p=0.003, eps_w=0.03, v_min=0.005, v_max=0.25,
                                     w_min=0.05, w_max=1.0, lambda_p=5, lambda_w=5,
                                     dt=dt, rho=1e-3)
    q = np.array([0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0])
    t06 = ur5_dirkin(q)[4][:, :, 5]
    m = ClutchMapper(alignment_rotation(0.0))
    r_c0 = np.eye(3)
    m.engage([0, 0, 1.0], r_c0, t06[:3, 3], t06[:3, :3])
    # Controller: 10 cm forward, 5 cm up, 20 deg yaw.
    p_c, r_c = np.array([0.10, 0, 1.05]), rot_z(math.radians(20))
    p_goal, r_goal = m.target(p_c, r_c)
    for _ in range(int(3.0 / dt)):
        step = resolved_rates_step(q, {'p': p_goal, 'R': r_goal}, params)
        if step.at_goal:
            break
        q = q + clip_joint_step(step.q - q, 1.0 * dt)
    t = ur5_dirkin(q)[4][:, :, 5]
    assert np.linalg.norm(t[:3, 3] - p_goal) < 0.005
    assert rotation_angle(r_goal @ t[:3, :3].T) < 0.05


def test_home_pose_singularity_check():
    """ur5_kinematics HOME_Q has q5 = 0 (wrist singularity); teleop home does not."""
    from ur5_kinematics.forward_kinematics import ur5_dirkin
    from ur5_kinematics.jacobian import compute_jacobian

    def w(q):
        f = ur5_dirkin(q)[4]
        return manipulability(compute_jacobian(f, f[:3, 3, 5]))

    demo_home = [0, -math.pi / 2, math.pi / 2, 0, 0, 0]
    teleop_home = [0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0]
    assert w(demo_home) < 1e-6
    assert w(teleop_home) > 1e-3


# ------------------------------------------------------------------ server
def _free_port():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_server_end_to_end_with_fake_quest():
    frames = []
    port = _free_port()
    server = PoseServer(frames.append, host='127.0.0.1', port=port, log=lambda s: None)
    server.start()
    try:
        html = urllib.request.urlopen(f'http://127.0.0.1:{port}/').read().decode()
        assert 'navigator.xr' in html
        sent = asyncio.run(fake_run(f'ws://127.0.0.1:{port}/ws', 72.0, 1.0, 0.1, 8.0))
        time.sleep(0.2)
    finally:
        server.stop()
    assert sent >= 65
    assert len(frames) == sent
    assert server.bad_frames == 0
    c = frames[0].controllers['right']
    # fake starts at xr p = [0.1, 1.1, -0.4]  ->  ros [0.4, -0.1, 1.1]
    assert np.allclose(c.grip.p, [0.4, -0.1, 1.1])
    assert c.pressed(1)  # squeeze held at t = 0
