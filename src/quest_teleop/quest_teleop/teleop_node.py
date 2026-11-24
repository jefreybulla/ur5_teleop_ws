"""Quest controller -> UR5 teleoperation using our resolved-rates IK.

Hold the squeeze (grip) button on the selected controller to engage the
clutch; the tool then follows the controller's motion relative to where the
clutch closed. Release to freeze the arm.

Modes
  sim       integrate q internally and publish /joint_states (RViz only).
  hardware  read the driver's /joint_states, publish joint positions to a
            forward position controller (Float64MultiArray). Try URSim first.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Empty, Float64MultiArray

from quest_teleop.frames import quat_to_rotm, rotm_to_quat
from quest_teleop.teleop_core import (
    R_BASE_FROM_BASE_LINK, ClutchMapper, WorkspaceLimits, alignment_rotation,
    clip_joint_step, manipulability,
)
from ur5_kinematics.forward_kinematics import ur5_dirkin
from ur5_kinematics.jacobian import compute_jacobian
from ur5_kinematics.resolved_rates import TrajectoryControlParams, resolved_rates_step

UR5_JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
HOME_Q = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]


class QuestTeleop(Node):
    def __init__(self) -> None:
        super().__init__('quest_teleop')
        p = self.declare_parameter
        p('hand', 'right')
        p('mode', 'sim')                       # 'sim' | 'hardware'
        p('control_rate_hz', 50.0)
        p('scale', 1.0)                        # controller metres -> tool metres
        p('operator_yaw_deg', 0.0)
        p('enable_orientation', True)
        p('input_timeout_s', 0.25)             # stale Quest data -> clutch opens
        p('max_joint_speed', 1.0)              # [rad/s] per-joint cap
        p('home_q', HOME_Q)
        p('joint_prefix', '')
        p('command_topic', '/forward_position_controller/commands')
        p('joint_state_topic', '/joint_states')
        p('visualization_frame', 'base_link')
        # Workspace limits in UR 'base' frame
        p('r_max', 0.80)
        p('r_min_xy', 0.15)
        p('z_min', 0.05)
        p('z_max', 0.90)
        # Resolved-rates gains (same algorithm as resolved_rates_demo)
        p('eps_p', 0.003)
        p('eps_w', 0.03)
        p('v_min', 0.005)
        p('v_max', 0.25)
        p('w_min', 0.05)
        p('w_max', 1.0)
        p('lambda_p', 5.0)
        p('lambda_w', 5.0)
        p('rho', 1e-3)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.hand = str(g('hand'))
        self.mode = str(g('mode'))
        if self.mode not in ('sim', 'hardware'):
            raise ValueError("mode must be 'sim' or 'hardware'")
        rate = float(g('control_rate_hz'))
        self.dt = 1.0 / rate
        self.timeout = float(g('input_timeout_s'))
        self.max_step = float(g('max_joint_speed')) * self.dt
        self.joint_names = [str(g('joint_prefix')) + n for n in UR5_JOINT_NAMES]
        self.viz_frame = str(g('visualization_frame'))

        self.params = TrajectoryControlParams(
            eps_p=float(g('eps_p')), eps_w=float(g('eps_w')),
            v_min=float(g('v_min')), v_max=float(g('v_max')),
            w_min=float(g('w_min')), w_max=float(g('w_max')),
            lambda_p=float(g('lambda_p')), lambda_w=float(g('lambda_w')),
            dt=self.dt, rho=float(g('rho')),
        )
        self.mapper = ClutchMapper(
            alignment_rotation(float(g('operator_yaw_deg'))),
            scale=float(g('scale')),
            enable_orientation=bool(g('enable_orientation')),
            limits=WorkspaceLimits(float(g('r_max')), float(g('r_min_xy')),
                                   float(g('z_min')), float(g('z_max'))),
        )

        self._lock = threading.Lock()
        self._ctrl_p: Optional[np.ndarray] = None
        self._ctrl_r: Optional[np.ndarray] = None
        self._ctrl_t = 0.0
        self._squeeze = False

        self.q: Optional[np.ndarray] = None
        self.q_meas: Optional[np.ndarray] = None
        if self.mode == 'sim':
            self.q = np.asarray(g('home_q'), dtype=float).reshape(6)

        self.create_subscription(PoseStamped, f'/quest/{self.hand}/pose', self._on_pose, 10)
        self.create_subscription(Joy, f'/quest/{self.hand}/joy', self._on_joy, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/quest_teleop/goal_pose', 10)
        self.metrics_pub = self.create_publisher(Float64MultiArray, '/quest_teleop/metrics', 10)

        if self.mode == 'sim':
            self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
            # Stop ur5_kinematics/joint_state_idle so we own /joint_states.
            self.takeover_pub = self.create_publisher(Empty, '/resolved_rates/takeover', 10)
            self.create_timer(0.5, lambda: self.takeover_pub.publish(Empty()))
        else:
            self.cmd_pub = self.create_publisher(Float64MultiArray, str(g('command_topic')), 10)
            self.create_subscription(JointState, str(g('joint_state_topic')), self._on_js, 10)

        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'Quest teleop ready: mode={self.mode}, hand={self.hand}, {rate:.0f} Hz. '
            'Hold SQUEEZE to move the arm.')

    # ------------------------------------------------------------ inputs
    def _on_pose(self, msg: PoseStamped) -> None:
        o, pos = msg.pose.orientation, msg.pose.position
        with self._lock:
            self._ctrl_p = np.array([pos.x, pos.y, pos.z])
            self._ctrl_r = quat_to_rotm([o.x, o.y, o.z, o.w])
            self._ctrl_t = time.monotonic()

    def _on_joy(self, msg: Joy) -> None:
        with self._lock:
            self._squeeze = len(msg.buttons) > 1 and bool(msg.buttons[1])

    def _on_js(self, msg: JointState) -> None:
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in idx for n in self.joint_names):
            return
        self.q_meas = np.array([msg.position[idx[n]] for n in self.joint_names])

    # ------------------------------------------------------------ control
    def _tick(self) -> None:
        with self._lock:
            p_c = None if self._ctrl_p is None else self._ctrl_p.copy()
            r_c = None if self._ctrl_r is None else self._ctrl_r.copy()
            fresh = (time.monotonic() - self._ctrl_t) < self.timeout
            squeeze = self._squeeze

        if self.mode == 'hardware' and not self.mapper.engaged:
            if self.q_meas is None:
                return
            self.q = self.q_meas.copy()   # track the real robot while idle

        clutch = squeeze and fresh and p_c is not None and self.q is not None
        if clutch and not self.mapper.engaged:
            t06 = ur5_dirkin(self.q)[4][:, :, 5]
            self.mapper.engage(p_c, r_c, t06[:3, 3], t06[:3, :3])
            self.get_logger().info('Clutch engaged')
        elif not clutch and self.mapper.engaged:
            self.mapper.release()
            self.get_logger().info('Clutch released' + ('' if fresh else ' (Quest data stale)'))

        if self.mapper.engaged:
            p_goal, r_goal = self.mapper.target(p_c, r_c)
            step = resolved_rates_step(self.q, {'p': p_goal, 'R': r_goal}, self.params)
            dq = clip_joint_step(step.q - self.q, self.max_step)
            self.q = self.q + dq
            if self.mode == 'hardware':
                self.cmd_pub.publish(Float64MultiArray(data=[float(v) for v in self.q]))
            self._publish_goal(p_goal, r_goal)
            frames = ur5_dirkin(self.q)[4]
            j = compute_jacobian(frames, frames[:3, 3, 5])
            self.metrics_pub.publish(Float64MultiArray(
                data=[step.delta_p, step.delta_w, manipulability(j)]))

        if self.mode == 'sim':
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.joint_names
            js.position = [float(v) for v in self.q]
            self.js_pub.publish(js)

    def _publish_goal(self, p_base: np.ndarray, r_base: np.ndarray) -> None:
        # base -> base_link is the same 180 deg z rotation (its own inverse).
        p = R_BASE_FROM_BASE_LINK @ p_base
        r = R_BASE_FROM_BASE_LINK @ r_base
        q = rotm_to_quat(r)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.viz_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in p)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = (float(v) for v in q)
        self.goal_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QuestTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
