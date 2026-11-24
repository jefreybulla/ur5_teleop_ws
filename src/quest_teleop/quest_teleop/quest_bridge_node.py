"""ROS 2 node: serve the WebXR page and publish Quest controller poses.

Topics (all poses in ``frame_id``, REP-103 axes: x forward, y left, z up,
origin = floor below the headset at session start for ``local-floor``):

    /quest/<hand>/pose   geometry_msgs/PoseStamped   controller grip pose
    /quest/<hand>/twist  geometry_msgs/TwistStamped  if the runtime reports velocities
    /quest/<hand>/joy    sensor_msgs/Joy
                         axes    = [thumb_x, thumb_y, trigger, squeeze]
                         buttons = [trigger, squeeze, thumbstick, A/X, B/Y]
    /quest/head/pose     geometry_msgs/PoseStamped
    /quest/rtt_ms        std_msgs/Float32   page <-> bridge round-trip time
    TF: frame_id -> quest_<hand>_grip, frame_id -> quest_head
"""

from __future__ import annotations

import threading
import time
from typing import Dict

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from quest_teleop.pose_server import PoseServer
from quest_teleop.protocol import (
    BTN_A_X, BTN_B_Y, BTN_SQUEEZE, BTN_THUMBSTICK, BTN_TRIGGER, Pose, QuestFrame,
)

HANDS = ('right', 'left')


class QuestBridge(Node):
    def __init__(self) -> None:
        super().__init__('quest_bridge')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('certfile', '')
        self.declare_parameter('keyfile', '')
        self.declare_parameter('frame_id', 'quest_world')
        self.declare_parameter('publish_tf', True)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.pose_pubs: Dict[str, object] = {}
        self.twist_pubs: Dict[str, object] = {}
        self.joy_pubs: Dict[str, object] = {}
        for hand in HANDS:
            self.pose_pubs[hand] = self.create_publisher(PoseStamped, f'/quest/{hand}/pose', 10)
            self.twist_pubs[hand] = self.create_publisher(TwistStamped, f'/quest/{hand}/twist', 10)
            self.joy_pubs[hand] = self.create_publisher(Joy, f'/quest/{hand}/joy', 10)
        self.head_pub = self.create_publisher(PoseStamped, '/quest/head/pose', 10)
        self.rtt_pub = self.create_publisher(Float32, '/quest/rtt_ms', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self._lock = threading.Lock()
        self._count = 0
        self._last_rx = 0.0
        self._last_seq = None
        self._dropped = 0

        self.server = PoseServer(
            self._on_frame,
            host=str(self.get_parameter('host').value),
            port=int(self.get_parameter('port').value),
            certfile=str(self.get_parameter('certfile').value),
            keyfile=str(self.get_parameter('keyfile').value),
            log=lambda s: self.get_logger().info(s),
        )
        self.server.start()
        self.create_timer(2.0, self._report)

    # Called from the server thread.
    def _on_frame(self, frame: QuestFrame) -> None:
        stamp = self.get_clock().now().to_msg()
        with self._lock:
            self._count += 1
            self._last_rx = time.monotonic()
            if self._last_seq is not None and frame.seq > self._last_seq + 1:
                self._dropped += frame.seq - self._last_seq - 1
            self._last_seq = frame.seq

        tfs = []
        for hand, c in frame.controllers.items():
            if hand not in self.pose_pubs:
                continue
            self.pose_pubs[hand].publish(self._pose_msg(c.grip, stamp))
            if self.tf_broadcaster is not None:
                tfs.append(self._tf_msg(c.grip, stamp, f'quest_{hand}_grip'))
            if c.lin_vel is not None and c.ang_vel is not None:
                tw = TwistStamped()
                tw.header.stamp = stamp
                tw.header.frame_id = self.frame_id
                tw.twist.linear.x, tw.twist.linear.y, tw.twist.linear.z = (float(v) for v in c.lin_vel)
                tw.twist.angular.x, tw.twist.angular.y, tw.twist.angular.z = (float(v) for v in c.ang_vel)
                self.twist_pubs[hand].publish(tw)
            joy = Joy()
            joy.header.stamp = stamp
            joy.header.frame_id = f'quest_{hand}_grip'
            axes = c.axes + [0.0] * (4 - len(c.axes))
            joy.axes = [float(axes[2]), float(axes[3]),
                        float(c.value(BTN_TRIGGER)), float(c.value(BTN_SQUEEZE))]
            joy.buttons = [int(c.pressed(i)) for i in
                           (BTN_TRIGGER, BTN_SQUEEZE, BTN_THUMBSTICK, BTN_A_X, BTN_B_Y)]
            self.joy_pubs[hand].publish(joy)

        if frame.head is not None:
            self.head_pub.publish(self._pose_msg(frame.head, stamp))
            if self.tf_broadcaster is not None:
                tfs.append(self._tf_msg(frame.head, stamp, 'quest_head'))
        if tfs:
            self.tf_broadcaster.sendTransform(tfs)
        if frame.rtt_ms is not None:
            self.rtt_pub.publish(Float32(data=float(frame.rtt_ms)))

    def _pose_msg(self, pose: Pose, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in pose.p)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = (float(v) for v in pose.q)
        return msg

    def _tf_msg(self, pose: Pose, stamp, child: str) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (
            float(v) for v in pose.p)
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = (float(v) for v in pose.q)
        return t

    def _report(self) -> None:
        with self._lock:
            count, self._count = self._count, 0
            dropped, self._dropped = self._dropped, 0
            age = time.monotonic() - self._last_rx if self._last_rx else None
        if age is None:
            self.get_logger().info(
                f'Waiting for the Quest page at {self.server.url} '
                f'(clients connected: {self.server.clients})')
        elif age > 1.0:
            self.get_logger().warn(f'No controller data for {age:.1f} s')
        else:
            self.get_logger().info(
                f'{count / 2.0:.1f} Hz from Quest, {dropped} seq gaps (untracked or lost frames), '
                f'bad msgs total {self.server.bad_frames}')

    def destroy_node(self) -> None:
        self.server.stop()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QuestBridge()
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
