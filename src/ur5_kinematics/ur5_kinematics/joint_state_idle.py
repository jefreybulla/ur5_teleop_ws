"""Publish HOME_Q on /joint_states until the resolved-rates demo takes over."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

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


class JointStateIdle(Node):
    def __init__(self) -> None:
        super().__init__('ur5_joint_state_idle')
        self._active = True
        self._publisher = self.create_publisher(JointState, '/joint_states', 10)
        self._takeover_sub = self.create_subscription(
            Empty,
            '/resolved_rates/takeover',
            self._on_takeover,
            10,
        )
        self._timer = self.create_timer(0.1, self._publish)
        self.get_logger().info(
            'Publishing idle joint states until resolved_rates_demo starts'
        )

    def _on_takeover(self, _msg: Empty) -> None:
        if not self._active:
            return
        self._active = False
        self._timer.cancel()
        self.get_logger().info('Stopped idle publisher; servo owns /joint_states')

    def _publish(self) -> None:
        if not self._active:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = UR5_JOINT_NAMES
        msg.position = HOME_Q
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateIdle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()