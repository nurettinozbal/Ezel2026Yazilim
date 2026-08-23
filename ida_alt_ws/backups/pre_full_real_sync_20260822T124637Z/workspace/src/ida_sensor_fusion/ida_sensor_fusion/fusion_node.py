"""ROS 2 camera-lidar fusion node with shadow and canonical output modes."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ida_planning.contracts import dumps, loads

from .contracts import camera_payload, lidar_payload, merge_camera_roles, result_payloads
from .core import FusionCore, ReadinessFlags, fuse_frames, sanitize_camera_frame


class SensorFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("ida_sensor_fusion")
        self.declare_parameter("camera_topic_p1p2", "/perception/camera/p1p2/raw")
        self.declare_parameter("camera_topic_p3", "/perception/camera/p3/raw")
        self.declare_parameter("lidar_topic", "/perception/lidar/raw_obstacles")
        self.declare_parameter("shadow_mode", True)
        self.declare_parameter("buoy_output_topic", "")
        self.declare_parameter("obstacle_output_topic", "")
        self.declare_parameter("status_output_topic", "/perception/fusion/status")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("camera_timeout_s", 0.5)
        self.declare_parameter("lidar_timeout_s", 0.5)
        self.declare_parameter("camera_role_merge_s", 0.0)
        self.declare_parameter("max_bearing_error_deg", 6.0)
        self.declare_parameter("ambiguity_margin_deg", 0.25)
        self.declare_parameter("max_association_items", 10)
        self.declare_parameter("buffer_size", 20)
        self.declare_parameter("clock_rollback_threshold_s", 0.5)
        self.declare_parameter("model_loaded", False)
        self.declare_parameter("camera_calibrated", False)
        self.declare_parameter("lidar_calibrated", False)
        self.declare_parameter("extrinsics_calibrated", False)

        self.shadow_mode = bool(self.get_parameter("shadow_mode").value)
        default_buoys = "/perception/fusion/shadow/buoys" if self.shadow_mode else "/perception/buoys"
        default_obstacles = (
            "/perception/fusion/shadow/obstacles" if self.shadow_mode else "/perception/obstacles"
        )
        self.buoy_output_topic = str(self.get_parameter("buoy_output_topic").value) or default_buoys
        self.obstacle_output_topic = (
            str(self.get_parameter("obstacle_output_topic").value) or default_obstacles
        )
        self.status_output_topic = str(self.get_parameter("status_output_topic").value)
        self.publish_hz = self._positive_float("publish_hz")
        self.camera_timeout_s = self._positive_float("camera_timeout_s")
        self.lidar_timeout_s = self._positive_float("lidar_timeout_s")
        self.camera_role_merge_s = self._nonnegative_float("camera_role_merge_s")
        self.max_bearing_error_deg = self._nonnegative_float("max_bearing_error_deg")
        self.ambiguity_margin_deg = self._nonnegative_float("ambiguity_margin_deg")
        raw_max_items = self.get_parameter("max_association_items").value
        if isinstance(raw_max_items, bool):
            raise ValueError("max_association_items must be a positive integer")
        self.max_association_items = int(raw_max_items)
        if self.max_association_items <= 0 or float(raw_max_items) != self.max_association_items:
            raise ValueError("max_association_items must be > 0")
        raw_buffer_size = self.get_parameter("buffer_size").value
        if isinstance(raw_buffer_size, bool):
            raise ValueError("buffer_size must be a positive integer")
        buffer_size = int(raw_buffer_size)
        if buffer_size <= 0 or float(raw_buffer_size) != buffer_size:
            raise ValueError("buffer_size must be a positive integer")
        rollback_s = self._nonnegative_float("clock_rollback_threshold_s")
        self.core = FusionCore(buffer_size, rollback_s)
        self.readiness = ReadinessFlags(
            bool(self.get_parameter("model_loaded").value),
            bool(self.get_parameter("camera_calibrated").value),
            bool(self.get_parameter("lidar_calibrated").value),
            bool(self.get_parameter("extrinsics_calibrated").value),
        )

        camera_p1p2 = str(self.get_parameter("camera_topic_p1p2").value)
        camera_p3 = str(self.get_parameter("camera_topic_p3").value)
        lidar_topic = str(self.get_parameter("lidar_topic").value)
        self.create_subscription(String, camera_p1p2, lambda msg: self.on_camera("p1p2", msg), 20)
        self.create_subscription(String, camera_p3, lambda msg: self.on_camera("p3", msg), 20)
        self.create_subscription(String, lidar_topic, self.on_lidar, 20)
        self.buoy_pub = self.create_publisher(String, self.buoy_output_topic, 20)
        self.obstacle_pub = self.create_publisher(String, self.obstacle_output_topic, 20)
        self.status_pub = self.create_publisher(String, self.status_output_topic, 20)
        self.create_timer(1.0 / self.publish_hz, self.tick)

        self._camera_roles: Dict[str, Tuple[float, List[Dict[str, Any]], float]] = {}
        self._last_camera_receive = 0.0
        self._last_lidar_receive = 0.0
        self._last_camera_acquisition = 0.0
        self._last_lidar_acquisition = 0.0
        self.get_logger().info(
            f"sensor fusion started (shadow={self.shadow_mode}, ready={self.readiness.ready}, "
            f"buoys={self.buoy_output_topic}, obstacles={self.obstacle_output_topic})"
        )

    def _positive_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or not value > 0.0:
            raise ValueError(f"{name} must be finite and > 0")
        return value

    def _nonnegative_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or not value >= 0.0:
            raise ValueError(f"{name} must be finite and >= 0")
        return value

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_camera(self, role: str, msg: String) -> None:
        parsed = camera_payload(loads(msg.data, None))
        if parsed is None:
            return
        stamp, detections = parsed
        received = self.now_seconds()
        self._camera_roles[role] = (stamp, detections, received)
        if stamp > self._last_camera_acquisition:
            self._last_camera_acquisition = stamp
            self._last_camera_receive = received
        current = [
            (source_role, source_stamp, source_detections)
            for source_role, (source_stamp, source_detections, receive_stamp)
            in self._camera_roles.items()
            if received - receive_stamp <= self.camera_timeout_s
        ]
        merged = merge_camera_roles(current, self.camera_role_merge_s)
        if merged is not None:
            self.core.ingest_camera(*merged)

    def on_lidar(self, msg: String) -> None:
        parsed = lidar_payload(loads(msg.data, None))
        if parsed is None:
            return
        if self.core.ingest_lidar(*parsed) and parsed[0] > self._last_lidar_acquisition:
            self._last_lidar_acquisition = parsed[0]
            self._last_lidar_receive = self.now_seconds()

    def tick(self) -> None:
        now = self.now_seconds()
        if self.core.observe_clock(now):
            self._camera_roles.clear()
            self._last_camera_receive = 0.0
            self._last_lidar_receive = 0.0
            self._last_camera_acquisition = 0.0
            self._last_lidar_acquisition = 0.0
        if not self.shadow_mode and (
            self.count_publishers(self.buoy_output_topic) > 1
            or self.count_publishers(self.obstacle_output_topic) > 1
        ):
            # Canonical mode must be single-writer. Do not join an accidental
            # publisher race; expose it on status and leave the old writer alone.
            self._publish(
                self.status_pub,
                self._status_base(
                    now,
                    "canonical_publisher_conflict",
                    camera_fresh=False,
                    lidar_fresh=False,
                ),
            )
            return
        camera_receive_age = now - self._last_camera_receive
        lidar_receive_age = now - self._last_lidar_receive
        camera_acquisition_age = now - self._last_camera_acquisition
        lidar_acquisition_age = now - self._last_lidar_acquisition
        camera_fresh = (
            self._last_camera_receive > 0.0
            and 0.0 <= camera_receive_age <= self.camera_timeout_s
            and 0.0 <= camera_acquisition_age <= self.camera_timeout_s
        )
        lidar_fresh = (
            self._last_lidar_receive > 0.0
            and 0.0 <= lidar_receive_age <= self.lidar_timeout_s
            and 0.0 <= lidar_acquisition_age <= self.lidar_timeout_s
        )
        if not lidar_fresh:
            self._publish_stale(now, "lidar_stale", camera_fresh, lidar_fresh)
            return

        if camera_fresh:
            result = self.core.fuse_best(
                self.readiness,
                now=now,
                max_frame_age_s=max(self.camera_timeout_s, self.lidar_timeout_s),
                max_bearing_error_deg=self.max_bearing_error_deg,
                ambiguity_margin_deg=self.ambiguity_margin_deg,
                max_association_items=self.max_association_items,
            )
        else:
            lidars = self.core.lidar_buffer.snapshot()
            if not lidars:
                self._publish_stale(now, "lidar_missing", camera_fresh, False)
                return
            empty_camera = sanitize_camera_frame(lidars[-1].stamp, [])
            not_ready = ReadinessFlags(False, False, self.readiness.lidar_calibrated, False)
            result = fuse_frames(
                empty_camera,
                lidars[-1],
                not_ready,
                max_association_items=self.max_association_items,
                clock_rollback_resets=self.core.clock_rollback_resets,
            )
        if result is None:
            self._publish_stale(now, "pair_missing", camera_fresh, lidar_fresh)
            return
        buoys, obstacles, status = result_payloads(result)
        if not camera_fresh:
            source_health = "camera_stale"
        elif result.status.association_overflow:
            source_health = "association_overflow"
        elif not self.readiness.ready:
            source_health = "fusion_not_ready"
        elif result.status.temporal_health == "reject":
            source_health = "time_rejected"
        elif result.status.accepted:
            source_health = "ok"
        else:
            source_health = "not_accepted"
        status.update({
            "shadow_mode": self.shadow_mode,
            "camera_fresh": camera_fresh,
            "lidar_fresh": lidar_fresh,
            "source_health": source_health,
        })
        if not camera_fresh:
            buoys["stale"] = True
        self._publish(self.buoy_pub, buoys)
        self._publish(self.obstacle_pub, obstacles)
        self._publish(self.status_pub, status)

    def _publish_stale(
        self, stamp: float, reason: str, camera_fresh: bool, lidar_fresh: bool
    ) -> None:
        self._publish(self.buoy_pub, {"stamp": stamp, "detections": [], "stale": True})
        self._publish(self.obstacle_pub, {"stamp": stamp, "obstacles": [], "stale": True})
        self._publish(
            self.status_pub,
            self._status_base(stamp, reason, camera_fresh, lidar_fresh),
        )

    def _status_base(
        self, stamp: float, reason: str, camera_fresh: bool, lidar_fresh: bool
    ) -> Dict[str, Any]:
        return {
            "stamp": stamp,
            "shadow_mode": self.shadow_mode,
            "accepted": False,
            "source_health": reason,
            "camera_fresh": camera_fresh,
            "lidar_fresh": lidar_fresh,
            "temporal_health": "unavailable",
            "dt_ms": None,
            "camera_stamp": None,
            "lidar_stamp": None,
            "matched_count": 0,
            "ambiguous_camera_count": 0,
            "ambiguous_lidar_count": 0,
            "invalid_camera_count": 0,
            "invalid_lidar_count": 0,
            "association_overflow": False,
            "overflow_component_count": 0,
            "overflow_camera_count": 0,
            "overflow_lidar_count": 0,
            "clock_rollback_resets": self.core.clock_rollback_resets,
            **self.readiness.as_dict(),
        }

    @staticmethod
    def _publish(publisher: Any, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = dumps(payload)
        publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
