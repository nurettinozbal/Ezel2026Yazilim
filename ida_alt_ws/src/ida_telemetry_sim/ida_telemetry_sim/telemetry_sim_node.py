import math
import time
from typing import Any, Dict, List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from ida_planning.contracts import dumps, mission_waypoints, telemetry_state
from ida_planning.geo import body_to_world, local_m_to_latlon, normalize_angle_deg
from ida_planning.scenario import load_scenario


class TelemetrySimNode(Node):
    def __init__(self) -> None:
        super().__init__("ida_telemetry_sim")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("start_delay_s", 1.0)
        self.declare_parameter("command_timeout_s", 0.7)
        self.declare_parameter("drag_time_constant_s", 0.8)

        self.scenario_file = str(self.get_parameter("scenario_file").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.start_delay_s = float(self.get_parameter("start_delay_s").value)
        self.command_timeout = float(self.get_parameter("command_timeout_s").value)
        self.drag_tau = float(self.get_parameter("drag_time_constant_s").value)

        self.scenario = self.load_or_default_scenario()
        origin = self.scenario.get("origin", {"lat": 40.86305, "lon": 29.25995})
        self.origin_lat = float(origin["lat"])
        self.origin_lon = float(origin["lon"])
        initial = self.scenario.get("initial_pose", {"x": 0.0, "y": 0.0, "heading_deg": 0.0})
        self.x = float(initial.get("x", 0.0))
        self.y = float(initial.get("y", 0.0))
        self.heading_deg = float(initial.get("heading_deg", 0.0))

        self.cmd = Twist()
        self.last_cmd_time = time.time()
        self.speed = 0.0
        self.last_update = time.time()
        self.started_sent = False
        self.mission_sent_count = 0
        self.start_time = time.time()

        self.create_subscription(Twist, "/control/cmd_vel_body", self.on_cmd, 20)
        self.telemetry_pub = self.create_publisher(String, "/telemetry/state", 20)
        self.waypoints_pub = self.create_publisher(String, "/mission/waypoints", 10)
        self.start_pub = self.create_publisher(String, "/mission/start", 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info("Telemetry simulator started")

    def load_or_default_scenario(self) -> Dict[str, Any]:
        if self.scenario_file:
            return load_scenario(self.scenario_file)
        return {
            "origin": {"lat": 40.86305, "lon": 29.25995},
            "initial_pose": {"x": 0.0, "y": 0.0, "heading_deg": 0.0},
            "waypoints": [{"x": 15.0, "y": 0.0, "parkur": 1}],
        }

    def on_cmd(self, msg: Twist) -> None:
        self.cmd = msg
        self.last_cmd_time = time.time()

    def tick(self) -> None:
        now = time.time()
        dt = max(0.001, now - self.last_update)
        self.last_update = now

        if now - self.last_cmd_time > self.command_timeout:
            target_vx = 0.0
            target_vy = 0.0
            yaw_rate = 0.0
        else:
            target_vx = float(self.cmd.linear.x)
            target_vy = float(self.cmd.linear.y)
            yaw_rate = float(self.cmd.angular.z)

        alpha = min(1.0, dt / max(0.001, self.drag_tau))
        self.speed += (target_vx - self.speed) * alpha
        self.heading_deg = normalize_angle_deg(self.heading_deg + math.degrees(yaw_rate) * dt)
        world = body_to_world(self.speed * dt, target_vy * dt, self.heading_deg)
        self.x += world.x
        self.y += world.y

        self.publish_telemetry()
        self.publish_mission_if_needed(now)

    def publish_telemetry(self) -> None:
        lat, lon = local_m_to_latlon(self.origin_lat, self.origin_lon, self.x, self.y)
        msg = String()
        payload = telemetry_state(lat, lon, self.heading_deg, abs(self.speed), "SIM")
        payload["x_m"] = self.x
        payload["y_m"] = self.y
        msg.data = dumps(payload)
        self.telemetry_pub.publish(msg)

    def publish_mission_if_needed(self, now: float) -> None:
        if self.mission_sent_count < 20:
            msg = String()
            msg.data = dumps(mission_waypoints(self.scenario_waypoints_latlon()))
            self.waypoints_pub.publish(msg)
            self.mission_sent_count += 1

        if self.auto_start and not self.started_sent and now - self.start_time >= self.start_delay_s:
            msg = String()
            msg.data = dumps({"start": True})
            self.start_pub.publish(msg)
            self.started_sent = True

    def scenario_waypoints_latlon(self) -> List[Dict[str, Any]]:
        out = []
        for wp in self.scenario.get("waypoints", []):
            if "lat" in wp and "lon" in wp:
                item = dict(wp)
            else:
                lat, lon = local_m_to_latlon(self.origin_lat, self.origin_lon, float(wp["x"]), float(wp["y"]))
                item = {"lat": lat, "lon": lon}
            item["parkur"] = int(wp.get("parkur", item.get("parkur", 1)))
            out.append(item)
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetrySimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
