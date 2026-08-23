import math
import re
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ida_planning.contracts import dumps, loads
from ida_planning.geo import clamp, latlon_to_local_m, world_to_body
from ida_planning.scenario import load_scenario


class PerceptionSimNode(Node):
    def __init__(self) -> None:
        super().__init__("ida_perception_sim")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("camera_range_m", 35.0)
        self.declare_parameter("camera_fov_deg", 90.0)
        self.declare_parameter("lidar_range_m", 18.0)
        self.declare_parameter("gate_truth_range_m", 45.0)
        self.declare_parameter("gate_truth_behind_m", 2.0)
        self.declare_parameter("publish_canonical", True)
        self.declare_parameter("publish_raw", False)
        self.declare_parameter("raw_p1p2_topic", "/perception/camera/p1p2/raw")
        self.declare_parameter("raw_p3_topic", "/perception/camera/p3/raw")
        self.declare_parameter("raw_lidar_topic", "/perception/lidar/raw_obstacles")

        self.scenario_file = str(self.get_parameter("scenario_file").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.camera_range = float(self.get_parameter("camera_range_m").value)
        self.camera_fov = float(self.get_parameter("camera_fov_deg").value)
        self.lidar_range = float(self.get_parameter("lidar_range_m").value)
        self.gate_truth_range = float(self.get_parameter("gate_truth_range_m").value)
        self.gate_truth_behind = float(self.get_parameter("gate_truth_behind_m").value)
        self.publish_canonical = bool(self.get_parameter("publish_canonical").value)
        self.publish_raw = bool(self.get_parameter("publish_raw").value)
        self.scenario = load_scenario(self.scenario_file) if self.scenario_file else {}
        origin = self.scenario.get("origin", {"lat": 40.86305, "lon": 29.25995})
        self.origin_lat = float(origin["lat"])
        self.origin_lon = float(origin["lon"])
        self.telemetry: Dict[str, Any] = {}

        self.create_subscription(String, "/telemetry/state", self.on_telemetry, 20)
        self._create_output_publishers(
            str(self.get_parameter("raw_p1p2_topic").value),
            str(self.get_parameter("raw_p3_topic").value),
            str(self.get_parameter("raw_lidar_topic").value),
        )
        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info("Perception simulator started")

    def _create_output_publishers(
        self, raw_p1p2_topic: str, raw_p3_topic: str, raw_lidar_topic: str
    ) -> None:
        """Create only enabled writers so ROS graph ownership stays unambiguous."""

        self.buoy_pub = None
        self.obstacle_pub = None
        self.raw_p1p2_pub = None
        self.raw_p3_pub = None
        self.raw_lidar_pub = None
        if self.publish_canonical:
            self.buoy_pub = self.create_publisher(String, "/perception/buoys", 20)
            self.obstacle_pub = self.create_publisher(
                String, "/perception/obstacles", 20
            )
        self.gate_truth_pub = self.create_publisher(
            String, "/perception_sim/gate_truth", 20
        )
        if self.publish_raw:
            self.raw_p1p2_pub = self.create_publisher(String, raw_p1p2_topic, 20)
            self.raw_p3_pub = self.create_publisher(String, raw_p3_topic, 20)
            self.raw_lidar_pub = self.create_publisher(String, raw_lidar_topic, 20)

    def on_telemetry(self, msg: String) -> None:
        payload = loads(msg.data, {})
        if isinstance(payload, dict):
            self.telemetry = payload

    def tick(self) -> None:
        acquisition_stamp = self.now_seconds()
        if not self.telemetry:
            self.publish([], [], [], [], [], [], acquisition_stamp, raw_stale=True)
            return

        boat_x, boat_y = self.boat_local_xy()
        heading = float(self.telemetry.get("heading_deg", 0.0))

        physical_objects = (
            self.scenario.get("buoys", [])
            + self.scenario.get("targets", [])
            + self.scenario.get("obstacles", [])
        )
        raw_lidar = []
        physical_by_id: Dict[str, Dict[str, Any]] = {}
        for obj in physical_objects:
            object_id = str(obj.get("id", ""))
            if object_id:
                physical_by_id[object_id] = obj
            cluster = self.object_to_raw_lidar(
                obj, boat_x, boat_y, heading, acquisition_stamp
            )
            if cluster is not None:
                raw_lidar.append(cluster)
        lidar_by_id = {str(item["id"]): item for item in raw_lidar}

        buoy_detections = []
        for obj in self.scenario.get("buoys", []) + self.scenario.get("targets", []):
            det = self.object_to_detection(
                obj, boat_x, boat_y, heading, self.camera_range, acquisition_stamp
            )
            if det and abs(det["bearing_deg"]) <= self.camera_fov / 2.0:
                det["bbox_norm_x"] = clamp(det["bearing_deg"] / (self.camera_fov / 2.0), -1.0, 1.0)
                det["bbox_size"] = clamp(0.6 / max(det["distance"], 0.5), 0.01, 0.35)
                # Canonical sim yolu gerçek fusion kontratını taklit eder:
                # renkli kamera tespiti yalnız aynı fiziksel nesne lidar
                # menzilindeyse stabil lidar track kimliği taşır. P3
                # fail-closed kontrolü böylece production'da gevşetilmez.
                lidar_id = str(obj.get("id", ""))
                if lidar_id in lidar_by_id:
                    det["lidar_obstacle_id"] = lidar_id
                    det["source"] = "sim_camera_lidar_fused"
                buoy_detections.append(det)

        camera_by_id = {str(item.get("id", "")): item for item in buoy_detections}
        obstacles = [
            self.raw_lidar_to_canonical_obstacle(
                cluster,
                physical_by_id.get(str(cluster["id"]), {}),
                camera_by_id.get(str(cluster["id"])),
                acquisition_stamp,
            )
            for cluster in raw_lidar
        ]

        gate_truth = self.build_gate_truth(
            boat_x, boat_y, heading, acquisition_stamp
        )

        raw_p1p2 = self.build_raw_camera(
            self.scenario.get("buoys", []),
            {"orange", "yellow"},
            boat_x,
            boat_y,
            heading,
            acquisition_stamp,
        )
        raw_p3 = self.build_raw_camera(
            self.scenario.get("targets", []),
            {"red", "green", "black"},
            boat_x,
            boat_y,
            heading,
            acquisition_stamp,
        )
        self.publish(
            buoy_detections,
            obstacles,
            gate_truth,
            raw_p1p2,
            raw_p3,
            raw_lidar,
            acquisition_stamp,
        )

    @staticmethod
    def raw_lidar_to_canonical_obstacle(
        cluster: Dict[str, Any],
        physical_object: Dict[str, Any],
        camera_detection: Optional[Dict[str, Any]],
        acquisition_stamp: float,
    ) -> Dict[str, Any]:
        """Project one physical lidar return onto the canonical hard-obstacle wire.

        The stable scenario object ID is deliberately shared by raw lidar,
        canonical obstacle and a matched canonical buoy.  Objects outside the
        camera FOV remain color-unknown hard obstacles, as on the real stack.
        """

        forward_m = float(cluster["forward_m"])
        lateral_m = -float(cluster["lateral_left_m"])
        color = "unknown"
        source = "sim_lidar"
        confidence = 1.0
        if camera_detection is not None:
            color = str(camera_detection.get("color", "unknown")).lower()
            source = "sim_camera_lidar_fused"
            confidence = float(camera_detection.get("confidence", 1.0))
        object_id = str(cluster["id"])
        return {
            "stamp": acquisition_stamp,
            "id": object_id,
            "lidar_obstacle_id": object_id,
            "distance": float(cluster["distance_m"]),
            "forward_m": forward_m,
            "lateral_m": lateral_m,
            "bearing_deg": math.degrees(math.atan2(lateral_m, forward_m)),
            "color": color,
            "class": str(physical_object.get("class", color)).lower(),
            "confidence": confidence,
            "hard_obstacle": True,
            "source": source,
        }

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def build_raw_camera(
        self,
        objects: List[Dict[str, Any]],
        accepted_colors: set[str],
        boat_x: float,
        boat_y: float,
        heading_deg: float,
        acquisition_stamp: float,
    ) -> List[Dict[str, Any]]:
        detections: List[Dict[str, Any]] = []
        for obj in objects:
            if str(obj.get("color", "")).lower() not in accepted_colors:
                continue
            det = self.object_to_detection(
                obj, boat_x, boat_y, heading_deg, self.camera_range, acquisition_stamp
            )
            if det is None or abs(det["bearing_deg"]) > self.camera_fov / 2.0:
                continue
            det["bbox_norm_x"] = clamp(
                det["bearing_deg"] / (self.camera_fov / 2.0), -1.0, 1.0
            )
            det["bbox_size"] = clamp(0.6 / max(det["distance"], 0.5), 0.01, 0.35)
            detections.append(det)
        return detections

    def object_to_raw_lidar(
        self,
        obj: Dict[str, Any],
        boat_x: float,
        boat_y: float,
        heading_deg: float,
        acquisition_stamp: float,
    ) -> Optional[Dict[str, Any]]:
        try:
            ox = float(obj["x"])
            oy = float(obj["y"])
        except (KeyError, TypeError, ValueError):
            return None
        object_id = str(obj.get("id", ""))
        if not object_id or not math.isfinite(ox) or not math.isfinite(oy):
            return None
        body = world_to_body(ox - boat_x, oy - boat_y, heading_deg)
        distance = math.hypot(body.x, body.y)
        if (
            not math.isfinite(distance)
            or not math.isfinite(body.x)
            or not math.isfinite(body.y)
            or distance > self.lidar_range
        ):
            return None
        return {
            "acquisition_stamp": acquisition_stamp,
            "id": object_id,
            "forward_m": body.x,
            # ROS base_link +y is left; the autonomy stack's lateral_m is +right.
            "lateral_left_m": -body.y,
            "distance_m": distance,
        }

    def build_gate_truth(
        self,
        boat_x: float,
        boat_y: float,
        heading_deg: float,
        acquisition_stamp: Optional[float] = None,
    ) -> List[Dict[str, Any]]:
        """Tam stable turuncu çiftleri FOV'dan bağımsız sim kanalına üret."""

        groups: Dict[str, Dict[str, Dict[str, Any]]] = {}
        for obj in self.scenario.get("buoys", []):
            if str(obj.get("color", "")).lower() != "orange":
                continue
            match = re.match(r"^(.*)_([lr])(\d+)$", str(obj.get("id", "")), re.IGNORECASE)
            if match is None:
                continue
            key = f"{match.group(1)}:{int(match.group(3))}"
            groups.setdefault(key, {})[match.group(2).lower()] = obj

        truth: List[Dict[str, Any]] = []
        for key in sorted(groups):
            group = groups[key]
            if "l" not in group or "r" not in group:
                continue
            pair: List[Dict[str, Any]] = []
            for side in ("l", "r"):
                det = self.object_to_gate_truth(
                    group[side], boat_x, boat_y, heading_deg, acquisition_stamp
                )
                if det is not None:
                    pair.append(det)
            # Stable pair atomiktir: heading offset'te bir uç body-forward
            # eşiğinin arkasına diğerinden erken geçer. İki uç de menzil/
            # finite kontrolünü geçiyor ve gate merkezi truth-behind penceresi
            # içindeyse ikisini birlikte yayınla.
            center_forward = (
                0.5 * (pair[0]["forward_m"] + pair[1]["forward_m"])
                if len(pair) == 2 else float("-inf")
            )
            if len(pair) == 2 and center_forward >= -self.gate_truth_behind:
                truth.extend(pair)
        return truth

    def object_to_gate_truth(
        self,
        obj: Dict[str, Any],
        boat_x: float,
        boat_y: float,
        heading_deg: float,
        acquisition_stamp: Optional[float] = None,
    ) -> Dict[str, Any] | None:
        ox = float(obj.get("x", 0.0))
        oy = float(obj.get("y", 0.0))
        body = world_to_body(ox - boat_x, oy - boat_y, heading_deg)
        distance = math.hypot(body.x, body.y)
        if (
            not math.isfinite(distance)
            or not math.isfinite(body.x)
            or not math.isfinite(body.y)
            or distance > self.gate_truth_range
        ):
            return None
        return {
            "stamp": time.time() if acquisition_stamp is None else acquisition_stamp,
            "id": str(obj.get("id", "")),
            "color": "orange",
            "class": "orange",
            "distance": distance,
            "bearing_deg": math.degrees(math.atan2(body.y, max(0.001, body.x))),
            "forward_m": body.x,
            "lateral_m": body.y,
            "confidence": 0.99,
            "source": "sim_gate_truth",
        }

    def boat_local_xy(self) -> tuple[float, float]:
        if "x_m" in self.telemetry and "y_m" in self.telemetry:
            return float(self.telemetry["x_m"]), float(self.telemetry["y_m"])
        point = latlon_to_local_m(
            self.origin_lat,
            self.origin_lon,
            float(self.telemetry["lat"]),
            float(self.telemetry["lon"]),
        )
        return point.x, point.y

    def object_to_detection(
        self,
        obj: Dict[str, Any],
        boat_x: float,
        boat_y: float,
        heading_deg: float,
        max_range: float,
        acquisition_stamp: Optional[float] = None,
    ) -> Dict[str, Any] | None:
        ox = float(obj.get("x", 0.0))
        oy = float(obj.get("y", 0.0))
        body = world_to_body(ox - boat_x, oy - boat_y, heading_deg)
        distance = math.hypot(body.x, body.y)
        if distance > max_range or body.x < -0.5:
            return None
        bearing = math.degrees(math.atan2(body.y, max(0.001, body.x)))
        confidence = clamp(1.0 - distance / max_range + 0.25, 0.0, 0.99)
        return {
            "stamp": time.time() if acquisition_stamp is None else acquisition_stamp,
            "id": obj.get("id", ""),
            "color": str(obj.get("color", obj.get("class", "unknown"))).lower(),
            "class": str(obj.get("class", obj.get("color", "unknown"))).lower(),
            "distance": distance,
            "bearing_deg": bearing,
            "forward_m": body.x,
            "lateral_m": body.y,
            "confidence": confidence,
        }

    def publish(
        self,
        buoy_detections: List[Dict[str, Any]],
        obstacles: List[Dict[str, Any]],
        gate_truth: List[Dict[str, Any]],
        raw_p1p2: Optional[List[Dict[str, Any]]] = None,
        raw_p3: Optional[List[Dict[str, Any]]] = None,
        raw_lidar: Optional[List[Dict[str, Any]]] = None,
        acquisition_stamp: Optional[float] = None,
        raw_stale: bool = False,
    ) -> None:
        stamp = time.time() if acquisition_stamp is None else acquisition_stamp
        buoy_pub = getattr(self, "buoy_pub", None)
        obstacle_pub = getattr(self, "obstacle_pub", None)
        if self.publish_canonical and buoy_pub is not None and obstacle_pub is not None:
            buoy_msg = String()
            buoy_msg.data = dumps({"stamp": stamp, "detections": buoy_detections})
            buoy_pub.publish(buoy_msg)

            obstacle_msg = String()
            obstacle_msg.data = dumps({"stamp": stamp, "obstacles": obstacles})
            obstacle_pub.publish(obstacle_msg)

        truth_msg = String()
        truth_msg.data = dumps({"stamp": stamp, "detections": gate_truth})
        self.gate_truth_pub.publish(truth_msg)

        if self.publish_raw:
            camera_p1p2_payload = {
                "acquisition_stamp": stamp, "detections": raw_p1p2 or []
            }
            camera_p3_payload = {
                "acquisition_stamp": stamp, "detections": raw_p3 or []
            }
            lidar_payload = {
                "acquisition_stamp": stamp, "clusters": raw_lidar or []
            }
            if raw_stale:
                camera_p1p2_payload["stale"] = True
                camera_p3_payload["stale"] = True
                lidar_payload["stale"] = True
            self._publish_payload(
                getattr(self, "raw_p1p2_pub", None), camera_p1p2_payload
            )
            self._publish_payload(
                getattr(self, "raw_p3_pub", None), camera_p3_payload
            )
            self._publish_payload(
                getattr(self, "raw_lidar_pub", None), lidar_payload
            )

    @staticmethod
    def _publish_payload(publisher: Any, payload: Dict[str, Any]) -> None:
        if publisher is None:
            return
        msg = String()
        msg.data = dumps(payload)
        publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
