#!/usr/bin/env python3
"""send_mission.py — /mission/waypoints + /mission/start yayınlar (sim).

Zincirdeki ros2 topic pub escape karmaşası waypoint'leri bozuyordu (5'ten 2'si
kayboluyordu — autonomy "3 waypoints" alıyordu). Bu script JSON'u Python'da
temiz üretir: 5 hakem waypoint'i + start sinyali.

Kullanım: python3 send_mission.py [--rate 1] [--scenario-file PATH]
"""

import argparse
import json
import math
import time
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ida_planning.geo import local_m_to_latlon
from ida_planning.scenario import load_scenario

WAYPOINTS = [
    {"lat": 40.7236638, "lon": 29.8249893, "parkur": 1},
    {"lat": 40.7237441, "lon": 29.8248786, "parkur": 1},
    {"lat": 40.7238497, "lon": 29.8249284, "parkur": 1},
    {"lat": 40.7240384, "lon": 29.8251016, "parkur": 1},
    {"lat": 40.7235886, "lon": 29.8254987, "parkur": 2},
]


def waypoints_from_scenario(path: str) -> List[Dict[str, Any]]:
    """Load and validate a scenario before any ROS publisher is created."""

    scenario = load_scenario(path)
    if not isinstance(scenario, dict):
        raise ValueError("scenario root must be an object")
    origin = scenario.get("origin")
    raw_waypoints = scenario.get("waypoints")
    if not isinstance(origin, dict) or not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("scenario requires origin and non-empty waypoints")
    try:
        if isinstance(origin.get("lat"), bool) or isinstance(origin.get("lon"), bool):
            raise ValueError
        origin_lat = float(origin["lat"])
        origin_lon = float(origin["lon"])
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError("scenario origin lat/lon must be numeric") from exc
    if (
        not math.isfinite(origin_lat)
        or not math.isfinite(origin_lon)
        or not -90.0 <= origin_lat <= 90.0
        or not -180.0 <= origin_lon <= 180.0
        or abs(math.cos(math.radians(origin_lat))) < 1e-12
    ):
        raise ValueError("scenario origin lat/lon is invalid")

    converted: List[Dict[str, Any]] = []
    for index, raw in enumerate(raw_waypoints):
        if not isinstance(raw, dict):
            raise ValueError(f"waypoint {index} must be an object")
        try:
            if isinstance(raw.get("x"), bool) or isinstance(raw.get("y"), bool):
                raise ValueError
            x_m = float(raw["x"])
            y_m = float(raw["y"])
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"waypoint {index} x/y must be numeric") from exc
        parkur = raw.get("parkur")
        if (
            not math.isfinite(x_m)
            or not math.isfinite(y_m)
            or isinstance(parkur, bool)
            or not isinstance(parkur, int)
            or parkur not in (1, 2, 3)
        ):
            raise ValueError(f"waypoint {index} is invalid")
        try:
            lat, lon = local_m_to_latlon(origin_lat, origin_lon, x_m, y_m)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"waypoint {index} conversion is invalid") from exc
        if (
            not math.isfinite(lat)
            or not math.isfinite(lon)
            or not -90.0 <= lat <= 90.0
            or not -180.0 <= lon <= 180.0
        ):
            raise ValueError(f"waypoint {index} conversion is invalid")
        converted.append({"lat": lat, "lon": lon, "parkur": parkur})
    return converted


class MissionSender(Node):
    def __init__(self, rate: float, waypoints: List[Dict[str, Any]] | None = None) -> None:
        super().__init__("sim_mission_sender")
        self.waypoints_pub = self.create_publisher(String, "/mission/waypoints", 10)
        self.start_pub = self.create_publisher(String, "/mission/start", 10)
        self.rate = rate
        self.waypoints = list(WAYPOINTS if waypoints is None else waypoints)
        self._sent = 0

    def send(self) -> bool:
        # Waypoint'leri birkaç kez yayınla (autonomy abone olduktan sonra yakalasın).
        if self._sent < 5:
            msg = String()
            msg.data = json.dumps({"waypoints": self.waypoints})
            self.waypoints_pub.publish(msg)
            self._sent += 1
            self.get_logger().info(
                f"Waypoint yayınlandı ({len(self.waypoints)} wp, deneme {self._sent}/5)"
            )
            return True
        # Start yayınla (3 kez).
        if self._sent < 8:
            msg = String()
            msg.data = json.dumps({"start": True})
            self.start_pub.publish(msg)
            self._sent += 1
            self.get_logger().info("Mission start yayınlandı")
            return True
        return False


def main(args=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--scenario-file", default="")
    ns_args, ros_args = parser.parse_known_args(args)
    waypoints = (
        waypoints_from_scenario(ns_args.scenario_file)
        if ns_args.scenario_file else list(WAYPOINTS)
    )
    rclpy.init(args=ros_args)
    node = MissionSender(ns_args.rate, waypoints)
    try:
        while rclpy.ok() and node.send():
            time.sleep(1.0 / max(0.1, ns_args.rate))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
