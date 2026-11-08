
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateDemo(Node):
    def __init__(self):
        super().__init__('ur5_joint_state_demo')

        self.publisher = self.create_publisher(
            JointState, '/joint_states', 10
        )

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        # Example joint configuration in radians
        self.q = [
            0.0,
            -math.pi / 2,
            math.pi / 2,
            -math.pi / 2,
            -math.pi / 2,
            0.0,
        ]

        # Publish at 10 Hz
        self.timer = self.create_timer(
            0.1, self.publish_joint_states
        )

        self.get_logger().info(
            'UR5 joint state demo started'
        )

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.q

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
