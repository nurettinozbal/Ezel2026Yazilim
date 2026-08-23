"""RPLidar algı düğümü (ida_rplidar).

LiDAR tarama noktalarını işleyip /perception/obstacles JSON kontratına uygun
engel listesi yayınlar. rplidar kütüphanesi (veya gerçek seri port) yoksa
dry_run modunda örnek nokta bulutu üretir; her durumda kontrat bozulmaz.

Bu düğüm ida_perception_sim'den bağımsızdır: aynı topic kontratına gerçek
donanımdan yayın yapar.
"""

from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ida_perception.rplidar import demo_scan, process_scan
from ida_planning.contracts import dumps

try:  # rplidar kütüphanesi opsiyonel: yoksa dry_run modunda çalış.
    from rplidar import RPLidar as _RPLidar  # type: ignore

    RPLIDAR_AVAILABLE = True
except Exception:  # pragma: no cover - dev makinesinde kurulu olmayabilir
    _RPLidar = None  # type: ignore
    RPLIDAR_AVAILABLE = False


class LidarNode(Node):
    """RPLidar tabanlı engel tespit düğümü."""

    def __init__(self) -> None:
        super().__init__("ida_rplidar")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("lidar_range_m", 18.0)
        self.declare_parameter("min_distance_m", 0.3)
        self.declare_parameter("cluster_gap_m", 0.35)
        self.declare_parameter("min_cluster_points", 3)
        self.declare_parameter("dry_run", True)

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.lidar_range_m = float(self.get_parameter("lidar_range_m").value)
        self.min_distance_m = float(self.get_parameter("min_distance_m").value)
        self.cluster_gap_m = float(self.get_parameter("cluster_gap_m").value)
        self.min_cluster_points = int(self.get_parameter("min_cluster_points").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.obstacle_pub = self.create_publisher(String, "/perception/obstacles", 20)
        self.create_timer(1.0 / max(0.1, self.publish_hz), self.tick)

        self._lidar: Any = None
        if not self.dry_run:
            self._connect_real_lidar()

        if self.dry_run:
            self.get_logger().warn("RPLidar dry_run modunda (örnek nokta bulutu)")
        self.get_logger().info("RPLidar node started")

    def _connect_real_lidar(self) -> None:
        """Gerçek RPLidar bağlantısını kurar; başarısızsa dry_run'a düşer."""
        if not RPLIDAR_AVAILABLE:
            self.get_logger().warn("rplidar kütüphanesi yok; dry_run moduna geçiliyor")
            self.dry_run = True
            return
        try:
            self._lidar = _RPLidar(self.serial_port, baudrate=self.baudrate)
            self._lidar.connect()
            self._lidar.start_motor()
            self.get_logger().info(f"RPLidar bağlandı: {self.serial_port}")
        except Exception as exc:
            self.get_logger().error(f"RPLidar bağlanamadı ({exc}); dry_run moduna geçiliyor")
            self._lidar = None
            self.dry_run = True

    def tick(self) -> None:
        if self.dry_run:
            points = demo_scan(self.lidar_range_m)
        else:
            points = self._read_scan()
        stamp = self.now_seconds()
        obstacles = process_scan(
            points,
            lidar_range_m=self.lidar_range_m,
            min_distance_m=self.min_distance_m,
            cluster_gap_m=self.cluster_gap_m,
            min_cluster_points=self.min_cluster_points,
            stamp=stamp,
        )
        self._publish(obstacles)

    def _read_scan(self) -> List[Tuple[float, float]]:
        """Gerçek RPLidar'dan (angle_deg, distance_m) noktalarını toplar."""
        if self._lidar is None:
            return []
        points: List[Tuple[float, float]] = []
        try:
            for _scan in self._lidar.iter_scans(max_buf_meas=3000):
                for quality, angle, distance in _scan:
                    if distance == 0.0:
                        continue
                    points.append((angle, distance / 1000.0))  # mm -> m
                if points:
                    break
        except Exception:
            # Okuma hatası anlık; boş liste sonraki tick'te yeniden denenir.
            return []
        return points

    def _publish(self, obstacles: List[Dict[str, Any]]) -> None:
        msg = String()
        msg.data = dumps({"stamp": self.now_seconds(), "obstacles": obstacles})
        self.obstacle_pub.publish(msg)

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def destroy_node(self) -> None:
        if self._lidar is not None:
            try:
                self._lidar.stop_motor()
                self._lidar.disconnect()
            except Exception:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
