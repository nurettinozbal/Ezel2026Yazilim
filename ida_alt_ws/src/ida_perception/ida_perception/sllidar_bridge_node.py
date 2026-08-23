"""sllidar_ros2 köprü düğümü (ida_sllidar_bridge).

sllidar_ros2 sürücüsünün ``/scan`` topic'inden sensor_msgs/LaserScan mesajlarını
alır, ``sllidar_bridge.scan_to_obstacles`` ile engel listesine çevirir ve
``/perception/obstacles`` topic'ine README kontratına uygun JSON yayınlar.

Kontrat hiç susmaz: scan belirli bir süre gelmezse (ör. sürücü yeniden başlarken)
canary davranışı olarak son engeller tekrarlanır. Abonelik default Reliable QoS
kullanır — sllidar_ros2 Reliable KeepLast(10) yayınladığı için
qos_profile_sensor_data (BestEffort) KULLANILMAZ.
"""

import math
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from ida_perception.sllidar_bridge import (
    laserscan_to_points,
    obstacles_to_raw_clusters,
    points_to_raw_body_contract,
    scan_to_obstacles,
)
from ida_planning.contracts import dumps

# Canary eşiği: bu süreden uzun scan gelmezse son engeller tekrarlanır.
_CANARY_TIMEOUT_S = 0.5


def _scan_to_dict(msg: LaserScan) -> Dict[str, Any]:
    """sensor_msgs/LaserScan -> dict (sllidar_bridge saf fonksiyonları için).

    NumPy olmadan tek tek float'a çevirir; NaN/inf sllidar_bridge tarafında
    elenir. Bozuk eleman (None/string) tek scan'i düşürmez — None'a çevrilir,
    laserscan_to_points tarafından elenir (review bulgusu: sessiz veri kaybı).
    """
    ranges: List[Any] = []
    for r in msg.ranges:
        try:
            ranges.append(float(r))
        except (TypeError, ValueError):
            ranges.append(None)
    return {
        "angle_min": float(msg.angle_min),
        "angle_max": float(msg.angle_max),
        "angle_increment": float(msg.angle_increment),
        "ranges": ranges,
        "range_min": float(msg.range_min),
        "range_max": float(msg.range_max),
    }


class SllidarBridgeNode(Node):
    """sllidar_ros2 /scan aboneliğini /perception/obstacles kontratına bağlar."""

    def __init__(self) -> None:
        super().__init__("ida_sllidar_bridge")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("output_topic", "/perception/obstacles")
        self.declare_parameter("raw_contract", False)
        self.declare_parameter("angle_offset_deg", 0.0)
        self.declare_parameter("mirror_scan", False)
        self.declare_parameter("front_fov_deg", 360.0)
        self.declare_parameter("sensor_forward_offset_m", 0.0)
        self.declare_parameter("sensor_lateral_right_offset_m", 0.0)
        self.declare_parameter("lidar_range_m", 18.0)
        self.declare_parameter("min_distance_m", 0.3)
        self.declare_parameter("cluster_gap_m", 0.35)
        self.declare_parameter("min_cluster_points", 3)
        self.declare_parameter("max_angle_gap_deg", 8.0)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("raw_point_limit", 720)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.raw_contract = bool(self.get_parameter("raw_contract").value)
        self.angle_offset_deg = float(self.get_parameter("angle_offset_deg").value)
        self.mirror_scan = bool(self.get_parameter("mirror_scan").value)
        self.front_fov_deg = float(self.get_parameter("front_fov_deg").value)
        self.sensor_forward_offset_m = float(
            self.get_parameter("sensor_forward_offset_m").value
        )
        self.sensor_lateral_right_offset_m = float(
            self.get_parameter("sensor_lateral_right_offset_m").value
        )
        self.lidar_range_m = float(self.get_parameter("lidar_range_m").value)
        self.min_distance_m = float(self.get_parameter("min_distance_m").value)
        self.cluster_gap_m = float(self.get_parameter("cluster_gap_m").value)
        self.min_cluster_points = int(self.get_parameter("min_cluster_points").value)
        self.max_angle_gap_deg = float(self.get_parameter("max_angle_gap_deg").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.raw_point_limit = self.get_parameter("raw_point_limit").value
        if (
            not math.isfinite(self.angle_offset_deg)
            or not math.isfinite(self.sensor_forward_offset_m)
            or not math.isfinite(self.sensor_lateral_right_offset_m)
            or not math.isfinite(self.front_fov_deg)
            or not 0.0 < self.front_fov_deg <= 360.0
            or isinstance(self.raw_point_limit, bool)
            or not isinstance(self.raw_point_limit, int)
            or not 1 <= self.raw_point_limit <= 720
        ):
            raise ValueError("invalid S2 mounting/front-FOV configuration")

        # /scan: sllidar_ros2 Reliable KeepLast(10) yayınlar -> default Reliable.
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, 10
        )
        self.obstacle_pub = self.create_publisher(String, self.output_topic, 20)
        self.create_timer(1.0 / max(0.1, self.publish_hz), self.publish_timer)

        self._last_obstacles: List[Dict[str, Any]] = []
        self._last_raw_points: List[Dict[str, float]] = []
        self._last_scan_time: float = 0.0
        self._last_scan_acquisition_stamp: float = 0.0
        self.get_logger().info(
            f"sllidar bridge started (scan_topic={self.scan_topic}, "
            f"angle_offset={self.angle_offset_deg} deg, mirror={self.mirror_scan}, "
            f"front_fov={self.front_fov_deg} deg, "
            f"sensor_offset=({self.sensor_forward_offset_m}, "
            f"{self.sensor_lateral_right_offset_m}) m)"
        )

    def on_scan(self, msg: LaserScan) -> None:
        """Yeni LaserScan geldiğinde engelleri hesaplar ve yayınlar."""
        scan = _scan_to_dict(msg)
        received_stamp = self.now_seconds()
        stamp = self._message_stamp_seconds(msg, received_stamp)
        obstacles = scan_to_obstacles(
            scan,
            angle_offset_deg=self.angle_offset_deg,
            mirror_scan=self.mirror_scan,
            front_fov_deg=self.front_fov_deg,
            sensor_forward_offset_m=self.sensor_forward_offset_m,
            sensor_lateral_right_offset_m=self.sensor_lateral_right_offset_m,
            lidar_range_m=self.lidar_range_m,
            min_distance_m=self.min_distance_m,
            cluster_gap_m=self.cluster_gap_m,
            min_cluster_points=self.min_cluster_points,
            max_angle_gap_deg=self.max_angle_gap_deg,
            stamp=stamp,
        )
        raw_points = points_to_raw_body_contract(
            laserscan_to_points(
                scan,
                angle_offset_deg=self.angle_offset_deg,
                mirror_scan=self.mirror_scan,
                front_fov_deg=self.front_fov_deg,
                min_distance_m=self.min_distance_m,
                max_distance_m=self.lidar_range_m,
            ),
            sensor_forward_offset_m=self.sensor_forward_offset_m,
            sensor_lateral_right_offset_m=self.sensor_lateral_right_offset_m,
            limit=self.raw_point_limit,
        )
        self._last_obstacles = obstacles
        self._last_raw_points = raw_points
        self._last_scan_time = received_stamp
        self._last_scan_acquisition_stamp = stamp
        self._publish(obstacles, stale=False, acquisition_stamp=stamp, raw_points=raw_points)

    def publish_timer(self) -> None:
        """Canary: scan 0.5 s'den uzun gelmezse son engelleri BAYAT işaretli yayınla.

        ``stale: true`` alanı autonomy'nin ``on_obstacles``'ında ``last_obstacle_time``
        güncellememesini sağlar — böylece lidar kaybında ``obstacle_timeout``
        failsafe'i tetiklenir (bayat veriyle araç sürülmez). Review bulgusu #1.
        """
        if self.now_seconds() - self._last_scan_time >= _CANARY_TIMEOUT_S:
            self._publish(
                self._last_obstacles,
                stale=True,
                acquisition_stamp=self._last_scan_acquisition_stamp,
                raw_points=self._last_raw_points,
            )

    def _publish(
        self,
        obstacles: List[Dict[str, Any]],
        stale: bool,
        acquisition_stamp: float,
        raw_points: List[Dict[str, float]] | None = None,
    ) -> None:
        msg = String()
        published_stamp = self.now_seconds()
        if self.raw_contract:
            payload: Dict[str, Any] = {
                "stamp": acquisition_stamp,
                "acquisition_stamp": acquisition_stamp,
                "published_stamp": published_stamp,
                "points": list(raw_points or []),
                "clusters": obstacles_to_raw_clusters(obstacles),
            }
        else:
            payload = {"stamp": acquisition_stamp, "obstacles": obstacles}
        if stale:
            payload["stale"] = True
        msg.data = dumps(payload)
        self.obstacle_pub.publish(msg)

    @staticmethod
    def _message_stamp_seconds(msg: LaserScan, fallback: float) -> float:
        """Use the LaserScan acquisition stamp, falling back only for zero stamps."""
        sec = int(msg.header.stamp.sec)
        nanosec = int(msg.header.stamp.nanosec)
        if sec == 0 and nanosec == 0:
            return fallback
        return sec + nanosec / 1e9

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SllidarBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
