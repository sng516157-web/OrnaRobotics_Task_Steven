#!/usr/bin/env python3
"""Open and close the DH AG95, or hold /gripper_cmd (finger angle, radians)."""

import time

import rclpy
from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from std_msgs.msg import Float64

# Vendor mimic ratios relative to gripper_finger1_joint.
JOINT_SCALE = {
    "gripper_finger1_joint": 1.0,
    "gripper_finger2_joint": 1.0,
    "gripper_finger1_inner_knuckle_joint": 1.49462955,
    "gripper_finger2_inner_knuckle_joint": 1.49462955,
    "gripper_finger1_finger_tip_joint": 1.49462955,
    "gripper_finger2_finger_tip_joint": 1.49462955,
    "gripper_finger1_finger_joint": 0.4563942,
    "gripper_finger2_finger_joint": 0.4563942,
}

CLOSED = 0.0
OPEN = 0.62
CYCLE_S = 6.0


class Gripper(Node):
    def __init__(self) -> None:
        super().__init__("dh_gripper")
        gz = GzNode()
        self._pubs = {
            name: gz.advertise(f"/gripper/{name}", Double) for name in JOINT_SCALE
        }
        self._held: float | None = None
        self._t0 = time.monotonic()
        self.create_subscription(Float64, "/gripper_cmd", self._on_cmd, 10)
        self.create_timer(0.02, self._tick)
        self.get_logger().info(
            "DH gripper is moving. Hold a finger angle with: "
            "ros2 topic pub /gripper_cmd std_msgs/msg/Float64 \"{data: 0.3}\" --once"
        )

    def _on_cmd(self, msg: Float64) -> None:
        self._held = min(OPEN, max(CLOSED, float(msg.data)))
        self.get_logger().info(f"Holding gripper finger at {self._held:.3f} rad")

    def _tick(self) -> None:
        if self._held is None:
            phase = ((time.monotonic() - self._t0) % CYCLE_S) / CYCLE_S
            if phase < 0.5:
                driver = OPEN * (phase / 0.5)
            else:
                driver = OPEN * (1.0 - (phase - 0.5) / 0.5)
        else:
            driver = self._held
        for name, scale in JOINT_SCALE.items():
            command = Double()
            command.data = driver * scale
            self._pubs[name].publish(command)


def main() -> None:
    rclpy.init()
    node = Gripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
