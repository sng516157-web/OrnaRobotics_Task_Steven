#!/usr/bin/env python3
"""Bridge Gazebo pose/info to ROS with entity names preserved.

ros_gz_bridge converts gz.msgs.Pose_V to tf2_msgs/TFMessage but drops the
entity names, which makes it useless for per-model validation. This node
republishes the same world poses as a TFMessage with child_frame_id set to
the Gazebo entity name (e.g. "plug", "sc_tip_link", "sc_port_base_link_entrance").

Used for task validation ground truth only, never for commanding motion.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class PoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("gz_pose_bridge")
        self.declare_parameter("gz_topic", "/world/insertion/pose/info")
        self.declare_parameter("ros_topic", "/world/insertion/pose/info_named")
        self.declare_parameter("frame_id", "gazebo_world")
        self._gz = GzNode()
        self._pub = self.create_publisher(
            TFMessage, self.get_parameter("ros_topic").value, 10
        )
        if not self._gz.subscribe(
            Pose_V, self.get_parameter("gz_topic").value, self._cb
        ):
            raise RuntimeError(
                f"Failed to subscribe to {self.get_parameter('gz_topic').value}"
            )
        self.get_logger().info(
            f"Bridging {self.get_parameter('gz_topic').value} -> "
            f"{self.get_parameter('ros_topic').value} (named poses)"
        )

    def _cb(self, msg: Pose_V) -> None:
        out = TFMessage()
        stamp = self.get_clock().now().to_msg()
        frame = self.get_parameter("frame_id").value
        for pose in msg.pose:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = frame
            t.child_frame_id = pose.name
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.y
            t.transform.rotation.z = pose.orientation.z
            t.transform.rotation.w = pose.orientation.w
            out.transforms.append(t)
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = PoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
