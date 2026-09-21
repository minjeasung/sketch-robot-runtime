"""
Bridge RB controller external F/T values to a standard WrenchStamped topic.

RB controller SystemState.eft is populated when an external force/torque sensor
is connected to the robot controller/tool chain. This node exposes that stream as
/aft200/ft so the existing FT normal controller can consume it.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rbpodo_msgs.msg import SystemState
from std_msgs.msg import String


class RBPodoEFTBridge(Node):
    def __init__(self):
        super().__init__("rbpodo_eft_bridge")

        self.declare_parameter("system_state_topic", "/rbpodo_hardware/system_state")
        self.declare_parameter("wrench_topic", "/aft200/ft")
        self.declare_parameter("status_topic", "/aft200/status")
        self.declare_parameter("frame_id", "tcp")
        self.declare_parameter("publish_zero_values", True)
        self.declare_parameter("zero_epsilon", 1e-6)

        self.system_state_topic = str(
            self.get_parameter("system_state_topic").value).strip()
        self.wrench_topic = str(self.get_parameter("wrench_topic").value).strip()
        self.status_topic = str(self.get_parameter("status_topic").value).strip()
        self.frame_id = str(self.get_parameter("frame_id").value).strip() or "tcp"
        self.publish_zero_values = bool(
            self.get_parameter("publish_zero_values").value)
        self.zero_epsilon = float(self.get_parameter("zero_epsilon").value)

        self.last_msg_time = 0.0
        self.samples = 0
        self.last_eft = None

        self.create_subscription(
            SystemState, self.system_state_topic, self._on_state, 10)
        self.wrench_pub = self.create_publisher(
            WrenchStamped, self.wrench_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            "RB PODO EFT bridge 시작\n"
            f"  state : {self.system_state_topic}\n"
            f"  wrench: {self.wrench_topic}\n"
            f"  frame : {self.frame_id}")

    def _on_state(self, msg):
        eft = [float(v) for v in msg.eft]
        if not all(math.isfinite(v) for v in eft):
            return
        if (
            not self.publish_zero_values
            and max(abs(v) for v in eft) <= self.zero_epsilon
        ):
            return

        wrench = WrenchStamped()
        wrench.header.stamp = self.get_clock().now().to_msg()
        wrench.header.frame_id = self.frame_id
        wrench.wrench.force.x = eft[0]
        wrench.wrench.force.y = eft[1]
        wrench.wrench.force.z = eft[2]
        wrench.wrench.torque.x = eft[3]
        wrench.wrench.torque.y = eft[4]
        wrench.wrench.torque.z = eft[5]
        self.wrench_pub.publish(wrench)

        self.samples += 1
        self.last_msg_time = time.monotonic()
        self.last_eft = eft

    def _publish_status(self):
        payload = {
            "source": "rbpodo_system_state",
            "system_state_topic": self.system_state_topic,
            "connected": self.last_msg_time > 0.0,
            "fresh": (
                self.last_msg_time > 0.0
                and time.monotonic() - self.last_msg_time < 1.0
            ),
            "samples": self.samples,
        }
        if self.last_eft is not None:
            payload["force_n"] = self.last_eft[:3]
            payload["torque_nm"] = self.last_eft[3:6]
        status = String()
        status.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = RBPodoEFTBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
