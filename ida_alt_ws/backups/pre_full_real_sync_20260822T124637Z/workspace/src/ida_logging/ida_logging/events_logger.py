"""Olay (contact/out-of-course) CSV kaydedicisi — itiraz adayı izleme.

Puan ceza modülünün (ida_planning.scoring) ürettiği zaman damgalı olayları
``events.csv`` dosyasına yazar. Sütunlar:

    t, type, parkur, key, detail

- ``t``: olay zamanı (ROS2 clock, saniye).
- ``type``: ``contact`` (duba/enme teması) ya da ``out_of_course`` (parkur
  dışına çıkış).
- ``parkur``: 1/2/3.
- ``key``: duba anahtarı (contact) ya da ``outside`` (parkur dışı).
- ``detail``: olay detayı (ör. ``counted=2 sustained=True``).

Süreklilik gereksinimi: her olay BİR satırdır; 30 sn sürekli temas 2 çarpma
sayıldığında da yalnız tek satır yazılır (detail'te ``counted=2`` işaretlenir).
Dosya, ``CsvWriter`` (ida_logging.csv_writer) ile yazılır: header sabit,
NaN/Inf elle temizlenir, kapatıldıktan sonra satır yazılmaz.
"""

from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ida_logging.csv_writer import CsvWriter
from ida_logging.run_dir import make_run_dir, make_run_dir_same
from ida_planning.contracts import dumps, loads

# events.csv sütun sırası (sabit — analiz araçları buna güvenir).
EVENTS_CSV_HEADER = ["t", "type", "parkur", "key", "detail"]


class EventsLoggerNode(Node):
    """Puan ceza modülü olaylarını (/autonomy/score) events.csv'ye yazar.

    ``/autonomy/score`` (String/JSON) topic'inden ``events`` listesini dinler:
    her olay {t, type, parkur, key, detail} CSV satırı olarak yazılır.
    Aynı olay birden çok yayında tekrar gelirse ``seen`` setiyle filtrelenir
    (mükerrer satır yazılmaz). Rosbag ya da canlı koşuda ``--events-only``
    benzeri filtre YOKTUR; tüm olaylar tek dosyaya gider.
    """

    def __init__(self) -> None:
        super().__init__("ida_events_logger")
        self.declare_parameter("log_dir", "./logs")
        self.declare_parameter("run_name", "")
        self.declare_parameter("flush_every_lines", 100)

        self.log_dir = str(self.get_parameter("log_dir").value)
        self.run_name = str(self.get_parameter("run_name").value).strip()
        self.flush_every_lines = int(self.get_parameter("flush_every_lines").value)

        # {parkur: set(key)} — aynı olayı tekrar yazmamak için.
        self._seen: Dict[str, set] = {}
        self._closed = False

        self.create_subscription(String, "/autonomy/score", self.on_score, 30)

        run_dir = (
            make_run_dir_same(self.log_dir, self.run_name)
            if self.run_name
            else make_run_dir(self.log_dir)
        )
        self.csv_path = run_dir / "events.csv"
        self._writer = CsvWriter(self.csv_path, EVENTS_CSV_HEADER, self.flush_every_lines)

        self.get_logger().info(f"Events logger started -> {self.csv_path}")

    def on_score(self, msg: String) -> None:
        """/autonomy/score mesajını işler; yeni olayları CSV'ye yazar."""
        payload = loads(msg.data, {})
        if not isinstance(payload, dict):
            return
        events = payload.get("events", [])
        if not isinstance(events, list):
            return
        for event in events:
            if not isinstance(event, dict):
                continue
            key = str(event.get("key", ""))
            ev_type = str(event.get("type", "contact"))
            parkur = str(event.get("parkur", "0"))
            detail = str(event.get("detail", ""))
            seen_for = self._seen.setdefault(parkur, set())
            if key in seen_for:
                continue  # aynı olay tekrarı yazılmaz
            seen_for.add(key)
            row = {
                "t": self._stamp_value(event, self.now_seconds()),
                "type": ev_type,
                "parkur": parkur,
                "key": key,
                "detail": detail,
            }
            self._writer.add_row(row)

    def _stamp_value(self, event: Dict[str, Any], fallback: float) -> Any:
        """Olaydaki t damgasını kullan; yoksa (fallback) timer zamanını kullan."""
        stamp = event.get("t")
        if stamp is None:
            return fallback
        try:
            return float(stamp)
        except (TypeError, ValueError):
            return fallback

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def destroy_node(self) -> None:
        if not self._closed:
            self._writer.close()
            self._closed = True
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EventsLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
