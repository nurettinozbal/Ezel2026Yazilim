#!/usr/bin/env python3
"""Print aggregate-only health for three JSON String fusion topics."""

from __future__ import annotations

import argparse
import json
import time
from typing import Any


def summarize(status: dict[str, Any], buoys: dict[str, Any], obstacles: dict[str, Any]) -> dict[str, Any]:
    detections = buoys.get("detections", [])
    obstacle_rows = obstacles.get("obstacles", [])
    if not isinstance(detections, list) or not isinstance(obstacle_rows, list):
        raise ValueError("fusion list schema invalid")
    valid_obstacles = [row for row in obstacle_rows if isinstance(row, dict)]
    return {
        "source_health": status.get("source_health"),
        "temporal_health": status.get("temporal_health"),
        "accepted": status.get("accepted"),
        "camera_fresh": status.get("camera_fresh"),
        "lidar_fresh": status.get("lidar_fresh"),
        "ready": status.get("ready"),
        "matched_count": status.get("matched_count"),
        "colored_buoy_count": len(detections),
        "obstacle_count": len(valid_obstacles),
        "hard_obstacle_count": sum(row.get("hard_obstacle") is True for row in valid_obstacles),
        "unknown_obstacle_count": sum(row.get("color") == "unknown" for row in valid_obstacles),
        "buoys_stale": buoys.get("stale", False),
        "obstacles_stale": obstacles.get("stale", False),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--status", default="/perception/fusion/status")
    parser.add_argument("--buoys", default="/perception/fusion/shadow/buoys")
    parser.add_argument("--obstacles", default="/perception/fusion/shadow/obstacles")
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()
    if not 0.1 <= args.timeout <= 60.0:
        raise ValueError("timeout out of bounds")

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = Node("fusion_health_summary_once")
    payloads: dict[str, dict[str, Any]] = {}

    def callback(key: str):
        def receive(message: String) -> None:
            if len(message.data) > 2_000_000:
                return
            try:
                decoded = json.loads(message.data)
            except (TypeError, ValueError):
                return
            if isinstance(decoded, dict):
                payloads[key] = decoded

        return receive

    node.create_subscription(String, args.status, callback("status"), 10)
    node.create_subscription(String, args.buoys, callback("buoys"), 10)
    node.create_subscription(String, args.obstacles, callback("obstacles"), 10)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and len(payloads) < 3:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(payloads) != 3:
            print(json.dumps({"error": "timeout", "received": sorted(payloads)}))
            return 2
        print(json.dumps(summarize(payloads["status"], payloads["buoys"], payloads["obstacles"]), sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
