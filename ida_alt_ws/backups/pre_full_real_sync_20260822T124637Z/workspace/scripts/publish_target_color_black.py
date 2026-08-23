#!/usr/bin/env python3
"""Target color'ı 'black' olarak publish eder (P3 test için).

Kullanım:
    python3 publish_target_color_black.py          # 1 kere publish edip çıkar
    python3 publish_target_color_black.py --loop   # 1 Hz'de sürekli publish
"""
import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = "/mission/target_color"


class TargetColorPublisher(Node):
    def __init__(self, color: str, loop: bool):
        super().__init__("target_color_publisher")
        self.pub = self.create_publisher(String, TOPIC, 10)
        self.color = color
        self.loop = loop

    def publish_once(self) -> None:
        msg = String()
        msg.data = json.dumps({"target_color": self.color})
        self.pub.publish(msg)
        self.get_logger().info(f"Published target_color={self.color} to {TOPIC}")

    def run(self) -> None:
        if not self.loop:
            self.publish_once()
            rclpy.shutdown()
            return
        timer = self.create_timer(1.0, self.publish_once)
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description="Publish target color (P3 test)")
    parser.add_argument("--color", default="black",
                        help="Hedef renk (default: black)")
    parser.add_argument("--loop", action="store_true",
                        help="1 Hz'de sürekli publish et")
    args = parser.parse_args()

    color = args.color.lower()
    if color not in ("black", "red", "green"):
        print(f"Hatalı renk: '{color}' (geçerli: black, red, green)", file=sys.stderr)
        return 1

    rclpy.init()
    node = TargetColorPublisher(color, args.loop)
    node.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
