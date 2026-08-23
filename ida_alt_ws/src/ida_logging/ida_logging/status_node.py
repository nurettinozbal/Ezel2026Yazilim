"""Log durumu yayıncısı (ida_logging_status).

Şartname zorunlu 3 dosyayı (telemetri CSV, kamera MP4, lokal harita) üreten
loglayıcı node'ların durumunu özetler ve ``/logging/status`` topic'ine yayınlar.
Gateway bu topic'ten LOG_ACT/LOG_CNT NAMED_VALUE_INT'lerini türetir (K2 kararı:
gerçek zamanlı doğruluk, statik parametre yerine).

Kontrat (std_msgs/String JSON):
    {"stamp": ..., "active": true, "logger_count": 3,
     "files": ["<log_dir>/telemetry.csv", "<log_dir>/processed_video.mp4",
               "<log_dir>/map.mp4"]}

``active`` yalnızca CSV'de veri satırı, kamera ``frames.csv`` kanıtı ve her iki
video için teslim MP4'ü veya fsync edilmiş kurtarılabilir segment varsa true
olur. ``files`` panelde yol göstermek için hazır tutulur; şimdilik
NAMED_VALUE_INT ile taşınmaz (yalnızca LOG_ACT/LOG_CNT gider).
"""

import json
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ida_logging.judge_contract import REQUIRED_FILES, evaluate_referee_files
from ida_logging.run_dir import make_run_dir, make_run_dir_same

class LoggingStatusNode(Node):
    """Loglayıcı durumunu /logging/status topic'ine yayınlar."""

    def __init__(self) -> None:
        super().__init__("ida_logging_status")
        self.declare_parameter("log_dir", "./logs")
        self.declare_parameter("run_name", "")
        self.declare_parameter("status_hz", 1.0)

        self.log_dir = str(self.get_parameter("log_dir").value)
        self.run_name = str(self.get_parameter("run_name").value).strip()
        self.status_hz = float(self.get_parameter("status_hz").value)

        # Loglayıcı node'larla aynı run klasörü paylaşılır (launch'ta aynı log_dir).
        self.run_dir = (
            make_run_dir_same(self.log_dir, self.run_name)
            if self.run_name
            else make_run_dir(self.log_dir)
        )

        self.status_pub = self.create_publisher(String, "/logging/status", 10)
        self.create_timer(1.0 / max(0.1, self.status_hz), self.tick)
        self.get_logger().info(f"Logging status started -> {self.run_dir}")

    def _current_files(self, ready_names: List[str]) -> List[str]:
        """Only return files that contain usable referee evidence."""
        return [str((self.run_dir / name).resolve()) for name in ready_names]

    def tick(self) -> None:
        readiness = evaluate_referee_files(self.run_dir)
        files = self._current_files(readiness["ready_files"])
        payload: Dict[str, Any] = {
            "stamp": self.now_seconds(),
            "active": readiness["active"],
            "logger_count": readiness["logger_count"],
            "files": files,
            "file_status": readiness["file_status"],
            "expected_files": list(REQUIRED_FILES),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
        self.status_pub.publish(msg)

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoggingStatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
