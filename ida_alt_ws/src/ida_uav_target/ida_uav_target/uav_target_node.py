import time
from collections import deque
from typing import Deque, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ida_planning.contracts import dumps, loads


class UavTargetNode(Node):
    def __init__(self) -> None:
        super().__init__("ida_uav_target")
        self.declare_parameter("sim_target_color", "")
        self.declare_parameter("min_confidence", 0.65)
        self.declare_parameter("stable_frames", 3)
        self.declare_parameter("publish_hz", 1.0)

        self.sim_target_color = str(self.get_parameter("sim_target_color").value).lower()
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.stable_frames = int(self.get_parameter("stable_frames").value)
        self.history: Deque[str] = deque(maxlen=max(1, self.stable_frames))
        self.last_published = ""

        self.create_subscription(String, "/uav/color_candidates", self.on_candidate, 10)
        self.pub = self.create_publisher(String, "/mission/target_color", 10)
        publish_hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / publish_hz, self.tick)
        self.get_logger().info("UAV target color node started")

    def on_candidate(self, msg: String) -> None:
        payload = loads(msg.data, {})
        if not isinstance(payload, dict):
            return
        color = str(payload.get("color", payload.get("target_color", ""))).lower()
        confidence = float(payload.get("confidence", 0.0))
        if color in {"red", "green", "black"} and confidence >= self.min_confidence:
            self.history.append(color)

    def tick(self) -> None:
        if self.sim_target_color in {"red", "green", "black"} and not self.history:
            self.history.append(self.sim_target_color)

        if len(self.history) < self.stable_frames:
            return
        colors = list(self.history)
        if len(set(colors)) != 1:
            return
        color = colors[-1]
        if color == self.last_published:
            return
        msg = String()
        msg.data = dumps({"stamp": time.time(), "target_color": color, "source": "uav_target"})
        self.pub.publish(msg)
        self.last_published = color
        self.get_logger().info(f"Published target color: {color}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UavTargetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
