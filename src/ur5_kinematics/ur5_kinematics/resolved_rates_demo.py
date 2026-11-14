"""Online resolved-rates IK demo: publish /joint_states and tool path for RViz."""

from __future__ import annotations

import math
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener   #  used only for drawing the tool path in RViz, not for IK
from visualization_msgs.msg import Marker

from ur5_kinematics.frame_utils import transform_point
from ur5_kinematics.forward_kinematics import ur5_dirkin
from ur5_kinematics.resolved_rates import (
    TrajectoryControlParams,
    resolved_rates_step,
    tool_pose,
)

UR5_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

HOME_Q = [
    0.0,
    -math.pi / 2,
    math.pi / 2,
    0.0,
    0.0,
    0.0,
]


def _rotation_matrix_to_quaternion(r: np.ndarray) -> List[float]:
    """Quaternion [x, y, z, w] from 3x3 rotation matrix."""
    tr = float(r[0, 0] + r[1, 1] + r[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return [x, y, z, w]


class ResolvedRatesDemo(Node):
    def __init__(self) -> None:
        super().__init__('ur5_resolved_rates_demo')

        self.declare_parameter('kinematics_frame', 'base')
        self.declare_parameter('visualization_frame', 'base_link')
        self.declare_parameter('goal_frame', 'base')
        self.declare_parameter('tool_frame', 'tool0')
        self.declare_parameter('use_tf_tool_path', True)
        self.declare_parameter('control_rate_hz', 10.0)     # Frequency at which we compute a new q_new
        self.declare_parameter('max_ticks', 1000)
        self.declare_parameter('auto_goal', True)       # # Default IK target: tool pose at HOME_Q, with position += offset in base_link
        self.declare_parameter('auto_goal_offset', [0.02, 0.0, 0.08])  # [m] Δx, Δy, Δz. 
        self.declare_parameter('goal_position', [0.0, 0.0, 0.0])
        self.declare_parameter('use_goal_position_param', False)
        self.declare_parameter('eps_p', 1e-2)
        self.declare_parameter('eps_w', 1e-1)
        self.declare_parameter('v_min', 0.01)
        self.declare_parameter('v_max', 0.15)
        self.declare_parameter('w_min', 0.05)
        self.declare_parameter('w_max', 0.5)
        self.declare_parameter('lambda_p', 5.0)
        self.declare_parameter('lambda_w', 5.0)
        self.declare_parameter('rho', 1e-4)

        self.kinematics_frame = str(self.get_parameter('kinematics_frame').value)
        self.visualization_frame = str(
            self.get_parameter('visualization_frame').value
        )
        self.goal_frame = str(self.get_parameter('goal_frame').value)
        self.tool_frame = str(self.get_parameter('tool_frame').value)
        self.use_tf_tool_path = bool(self.get_parameter('use_tf_tool_path').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._takeover_pub = self.create_publisher(
            Empty, '/resolved_rates/takeover', 10
        )
        self._takeover_pub.publish(Empty())
        self._takeover_resend = self.create_timer(
            0.3, self._republish_takeover
        )

        rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.max_ticks = int(self.get_parameter('max_ticks').value)

        initial_q = np.array(HOME_Q, dtype=float)
        self.q = initial_q.copy()

        self.goal = self._build_goal(initial_q)
        self.get_logger().info(
            f'Goal position [m]: {self.goal["p"].tolist()}'
        )

        dt_nominal = 1.0 / rate_hz
        self.params = TrajectoryControlParams(
            eps_p=float(self.get_parameter('eps_p').value),
            eps_w=float(self.get_parameter('eps_w').value),
            v_min=float(self.get_parameter('v_min').value),
            v_max=float(self.get_parameter('v_max').value),
            w_min=float(self.get_parameter('w_min').value),
            w_max=float(self.get_parameter('w_max').value),
            lambda_p=float(self.get_parameter('lambda_p').value),
            lambda_w=float(self.get_parameter('lambda_w').value),
            dt=dt_nominal,
            rho=float(self.get_parameter('rho').value),
        )

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.path_pub = self.create_publisher(Path, '/resolved_rates/tool_path', 10)
        self.goal_pub = self.create_publisher(Marker, '/resolved_rates/goal_marker', 10)

        self.tool_path = Path()
        self.tool_path.header.frame_id = self.visualization_frame

        self.servo_active = True
        self.tick_count = 0
        self.last_time = self.get_clock().now()

        self._publish_joint_states()
        self._append_tool_path()
        self._publish_goal_marker()

        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self._control_step)
        self.get_logger().info(
            f'Resolved-rates servo at {rate_hz:.1f} Hz (dt≈{dt_nominal:.3f} s)'
        )

    def _republish_takeover(self) -> None:
        self._takeover_pub.publish(Empty())
        self._takeover_resend.cancel()

    def _build_goal(self, q: np.ndarray) -> dict:
        _, _, _, _, frames = ur5_dirkin(q)
        t_06 = frames[:, :, 5]
        r_curr = t_06[:3, :3].copy()
        p_curr = tool_pose(t_06, np.zeros(3))

        if bool(self.get_parameter('use_goal_position_param').value):
            goal_p = np.asarray(
                self.get_parameter('goal_position').value,
                dtype=float,
            ).reshape(3)
            goal_p = transform_point(
                goal_p, self.goal_frame, self.kinematics_frame
            )
        elif bool(self.get_parameter('auto_goal').value):
            offset = np.asarray(
                self.get_parameter('auto_goal_offset').value,
                dtype=float,
            ).reshape(3)
            goal_p = p_curr + offset
        else:
            goal_p = np.asarray(
                self.get_parameter('goal_position').value,
                dtype=float,
            ).reshape(3)
            goal_p = transform_point(
                goal_p, self.goal_frame, self.kinematics_frame
            )

        return {'p': goal_p, 'R': r_curr}

    def _tool_position_fk(self) -> np.ndarray:
        _, _, _, _, frames = ur5_dirkin(self.q)
        t_06 = frames[:, :, 5]
        return tool_pose(t_06, np.zeros(3))

    def _tool_position_for_visualization(self) -> np.ndarray:
        if self.use_tf_tool_path:
            tf_point = self._tool_position_from_tf()
            if tf_point is not None:
                return tf_point

        p_fk = self._tool_position_fk()
        return transform_point(
            p_fk, self.kinematics_frame, self.visualization_frame
        )

    def _tool_position_from_tf(self) -> Optional[np.ndarray]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.visualization_frame,
                self.tool_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            return None

        t = transform.transform.translation
        return np.array([t.x, t.y, t.z], dtype=float)

    def _goal_position_for_visualization(self) -> np.ndarray:
        goal_p = np.asarray(self.goal['p'], dtype=float).reshape(3)
        return transform_point(
            goal_p, self.kinematics_frame, self.visualization_frame
        )

    def _append_tool_path(self) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.visualization_frame
        p = self._tool_position_for_visualization()
        pose.pose.position.x = float(p[0])
        pose.pose.position.y = float(p[1])
        pose.pose.position.z = float(p[2])
        pose.pose.orientation.w = 1.0
        self.tool_path.poses.append(pose)
        self.tool_path.header.stamp = pose.header.stamp

    def _publish_goal_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = self.visualization_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'resolved_rates'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        goal_p = self._goal_position_for_visualization()
        marker.pose.position.x = float(goal_p[0])
        marker.pose.position.y = float(goal_p[1])
        marker.pose.position.z = float(goal_p[2])
        q_xyzw = _rotation_matrix_to_quaternion(self.goal['R'])
        marker.pose.orientation.x = q_xyzw[0]
        marker.pose.orientation.y = q_xyzw[1]
        marker.pose.orientation.z = q_xyzw[2]
        marker.pose.orientation.w = q_xyzw[3]
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.9
        self.goal_pub.publish(marker)

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = UR5_JOINT_NAMES
        msg.position = self.q.tolist()
        self.joint_pub.publish(msg)

    def _control_step(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = self.params.dt
        self.params.dt = dt

        record_path = False
        if self.servo_active:
            self.tick_count += 1
            step = resolved_rates_step(self.q, self.goal, self.params)

            if step.at_goal:
                self.get_logger().info(
                    f'Goal reached after {self.tick_count} ticks '
                    f'(Δp={step.delta_p:.4e}, Δω={step.delta_w:.4e})'
                )
                record_path = True
                self.servo_active = False
            elif self.tick_count >= self.max_ticks:
                self.get_logger().error(
                    'Resolved-rates did not converge before max_ticks: '
                    f'Δp={step.delta_p:.6g} m, Δω={step.delta_w:.6g} rad'
                )
                self.servo_active = False
            else:
                self.q = step.q
                record_path = True

        self._publish_joint_states()
        if record_path:
            self._append_tool_path()

        if self.tool_path.poses:
            self.tool_path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.tool_path)
        self._publish_goal_marker()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ResolvedRatesDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()