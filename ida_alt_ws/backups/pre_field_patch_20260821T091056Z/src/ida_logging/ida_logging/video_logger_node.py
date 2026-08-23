"""Video loglayıcı düğümü (ida_video_logger).

Yapılandırılmış işlenmiş kamera topic'ini (varsayılan saha general
modeli: /perception/processed_image/p1p2) dinler
ve her frame'i cv2 VideoWriter ile mp4'e (fourcc "mp4v") kaydeder. cv2 yoksa
düğüm uyarıyla pasif kalır; sahte frame kanıtı üretmez.

Frame zaman damgaları ayrıca ``frames.csv``'ye yazılır (stamp, frame_index),
böylece video ile telemetri CSV'si senkronize edilebilir. İlk frame'de boyut
bilinmiyorsa ``frame_width``/``frame_height`` parametreleri kullanılır.
"""

import csv
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from ida_logging.run_dir import make_run_dir, make_run_dir_same

try:  # cv2 opsiyonel: yoksa video kaydı pasif, düğüm çalışmaya devam eder.
    import cv2  # type: ignore

    CV2_AVAILABLE = True
except Exception:  # pragma: no cover - dev makinesinde cv2 olmayabilir
    cv2 = None  # type: ignore
    CV2_AVAILABLE = False


class VideoLoggerNode(Node):
    """İşlenmiş görüntü akışını mp4 + frame CSV olarak kaydeder."""

    def __init__(self) -> None:
        super().__init__("ida_video_logger")
        self.declare_parameter("log_dir", "./logs")
        self.declare_parameter("run_name", "")
        self.declare_parameter("log_hz", 1.0)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("fourcc", "mp4v")
        self.declare_parameter("frame_width", -1)
        self.declare_parameter("frame_height", -1)
        self.declare_parameter("record_path", "")
        self.declare_parameter(
            "processed_image_topic", "/perception/processed_image/p1p2"
        )

        self.log_dir = str(self.get_parameter("log_dir").value)
        self.run_name = str(self.get_parameter("run_name").value).strip()
        self.log_hz = float(self.get_parameter("log_hz").value)
        self.fps = float(self.get_parameter("fps").value)
        self.fourcc = str(self.get_parameter("fourcc").value)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.record_path = str(self.get_parameter("record_path").value)
        self.processed_image_topic = str(
            self.get_parameter("processed_image_topic").value
        ).strip()
        if not self.processed_image_topic.startswith("/"):
            raise ValueError("processed_image_topic must be an absolute ROS topic")

        self._writer: Any = None
        self._video_path: Optional[str] = None
        self._frames_csv_path = None
        self._frames_csv_file = None
        self._frames_csv: Any = None
        self._frame_index = 0

        run_dir = (
            make_run_dir_same(self.log_dir, self.run_name)
            if self.run_name
            else make_run_dir(self.log_dir)
        )
        self._run_dir = run_dir
        if self.record_path:
            self._video_path = self.record_path
        else:
            self._video_path = str(run_dir / "processed_video.mp4")
        self._frames_csv_path = run_dir / "frames.csv"
        self._open_frames_csv()

        self.create_subscription(
            CompressedImage, self.processed_image_topic, self.on_frame, 10
        )
        self.create_timer(1.0 / max(0.1, self.log_hz), self.tick)
        if not CV2_AVAILABLE:
            self.get_logger().warn("cv2 bulunamadı; video ve frame kanıtı pasif")
        self.get_logger().info(
            f"Video logger started: {self.processed_image_topic} -> {self._video_path}"
        )

    def _open_frames_csv(self) -> None:
        """Frame zaman damgası CSV'sini açar (header: stamp, frame_index)."""
        self._frames_csv_file = open(self._frames_csv_path, "w", newline="", encoding="utf-8")
        self._frames_csv = csv.writer(self._frames_csv_file)
        self._frames_csv.writerow(["stamp", "frame_index"])

    def on_frame(self, msg: CompressedImage) -> None:
        """Gelen sıkıştırılmış frame'i mp4'e ve frames.csv'ye yazar."""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        if not CV2_AVAILABLE:
            return
        try:
            import numpy as np  # type: ignore

            buffer = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
            if frame is None:
                return
            self._ensure_writer(frame)
            if self._writer is not None:
                self._writer.write(frame)
                # Yalnız gerçekten VideoWriter'a verilen kare kanıt sayılır.
                self._log_frame_meta(stamp)
        except Exception as exc:
            self.get_logger().warn(f"Frame işlenemedi: {exc}")

    def _log_frame_meta(self, stamp: float) -> None:
        self._frame_index += 1
        if self._frames_csv is not None:
            self._frames_csv.writerow([stamp, self._frame_index])
            self._frames_csv_file.flush()

    def _ensure_writer(self, frame: Any) -> None:
        """İlk frame'de VideoWriter'ı boyut bilgisiyle açar (parametre ya da frame)."""
        if self._writer is not None:
            return
        height, width = frame.shape[:2]
        if self.frame_width > 0 and self.frame_height > 0:
            width, height = self.frame_width, self.frame_height
        fourcc = cv2.VideoWriter_fourcc(*self.fourcc)
        self._writer = cv2.VideoWriter(self._video_path, fourcc, self.fps, (width, height))
        if not self._writer.isOpened():
            self.get_logger().error(f"VideoWriter açılamadı: {self._video_path}")
            self._writer = None
        else:
            self.get_logger().info(f"Video kaydı başladı: {self._video_path} ({width}x{height})")

    def tick(self) -> None:
        """Yapılandırılmış hızda düzenli satır yazımı (boşta timer korunur)."""
        # Frame'ler subscription callback'inde yazılır; timer yalnızca periyodu
        # sabit tutar (>=1 Hz şartnamesi). Ek iş yok.
        pass

    def destroy_node(self) -> None:
        if self._writer is not None:
            self._writer.release()
        if self._frames_csv_file is not None:
            self._frames_csv_file.flush()
            self._frames_csv_file.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VideoLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
