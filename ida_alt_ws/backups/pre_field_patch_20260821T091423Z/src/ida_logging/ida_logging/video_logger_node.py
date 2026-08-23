"""Kesintiye dayanıklı video loglayıcı düğümü (ida_video_logger).

Yapılandırılmış işlenmiş kamera topic'ini (varsayılan saha general
modeli: /perception/processed_image/p1p2) dinler
ve her frame'i kısa, bağımsız finalize edilen MP4 segmentlerine kaydeder. Kamera
akışı kesildiğinde açık segment zaman aşımıyla kapatılır. Temiz kapanışta ffmpeg
stream-copy ile segmentler tek ``processed_video.mp4`` teslim dosyası yapılır.
cv2 yoksa düğüm uyarıyla pasif kalır; sahte frame kanıtı üretmez.

Frame zaman damgaları ayrıca ``frames.csv``'ye yazılır (stamp, frame_index),
böylece video ile telemetri CSV'si senkronize edilebilir. İlk frame'de boyut
bilinmiyorsa ``frame_width``/``frame_height`` parametreleri kullanılır.
"""

import csv
import math
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from ida_logging.run_dir import make_run_dir, make_run_dir_same
from ida_logging.video_segments import SegmentJournal, finalize_delivery_video

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
        self.declare_parameter("segment_duration_s", 10.0)
        self.declare_parameter("frame_stall_timeout_s", 2.0)
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
        self.segment_duration_s = float(self.get_parameter("segment_duration_s").value)
        self.frame_stall_timeout_s = float(
            self.get_parameter("frame_stall_timeout_s").value
        )
        if not math.isfinite(self.segment_duration_s) or self.segment_duration_s < 2.0:
            raise ValueError("segment_duration_s must be finite and >= 2.0")
        if not math.isfinite(self.frame_stall_timeout_s) or not (
            0.2 <= self.frame_stall_timeout_s < self.segment_duration_s
        ):
            raise ValueError(
                "frame_stall_timeout_s must be finite, >= 0.2 and smaller than segment_duration_s"
            )
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
        self._journal: Optional[SegmentJournal] = None
        self._segment_index = 0
        self._segment_partial_path = None
        self._segment_final_path = None
        self._segment_started_monotonic: Optional[float] = None
        self._last_frame_monotonic: Optional[float] = None
        self._segment_first_stamp: Optional[float] = None
        self._segment_last_stamp: Optional[float] = None
        self._segment_frame_count = 0
        self._segment_width = 0
        self._segment_height = 0

        run_dir = (
            make_run_dir_same(self.log_dir, self.run_name)
            if self.run_name
            else make_run_dir(self.log_dir)
        )
        self._run_dir = run_dir
        if self.record_path:
            self.get_logger().warn(
                "record_path segmented recording ile kullanılmaz; run klasörü esas alındı"
            )
        self._video_path = str(run_dir / "processed_video.mp4")
        self._journal = SegmentJournal(run_dir)
        self._frames_csv_path = run_dir / "frames.csv"
        self._open_frames_csv()

        self.create_subscription(
            CompressedImage, self.processed_image_topic, self.on_frame, 10
        )
        self.create_timer(min(0.5, 1.0 / max(0.1, self.log_hz)), self.tick)
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
            now_monotonic = time.monotonic()
            frame_height, frame_width = frame.shape[:2]
            expected_width = self.frame_width if self.frame_width > 0 else frame_width
            expected_height = self.frame_height if self.frame_height > 0 else frame_height
            if (
                self._writer is not None
                and (expected_width != self._segment_width or expected_height != self._segment_height)
            ):
                self.get_logger().warn(
                    "Kamera çözünürlüğü değişti; açık video segmenti kapatılıyor"
                )
                self._finalize_segment("resolution_change")
            if (
                self._writer is not None
                and self._segment_started_monotonic is not None
                and now_monotonic - self._segment_started_monotonic >= self.segment_duration_s
            ):
                self._finalize_segment("duration")
            self._ensure_writer(frame, now_monotonic)
            if self._writer is not None:
                if frame.shape[1] != self._segment_width or frame.shape[0] != self._segment_height:
                    frame = cv2.resize(frame, (self._segment_width, self._segment_height))
                self._writer.write(frame)
                self._last_frame_monotonic = now_monotonic
                if self._segment_first_stamp is None:
                    self._segment_first_stamp = stamp
                self._segment_last_stamp = stamp
                self._segment_frame_count += 1
                # Yalnız gerçekten VideoWriter'a verilen kare kanıt sayılır.
                self._log_frame_meta(stamp)
        except Exception as exc:
            self.get_logger().warn(f"Frame işlenemedi: {exc}")

    def _log_frame_meta(self, stamp: float) -> None:
        self._frame_index += 1
        if self._frames_csv is not None:
            self._frames_csv.writerow([stamp, self._frame_index])
            self._frames_csv_file.flush()

    def _ensure_writer(self, frame: Any, now_monotonic: float) -> None:
        """İlk frame'de VideoWriter'ı boyut bilgisiyle açar (parametre ya da frame)."""
        if self._writer is not None:
            return
        height, width = frame.shape[:2]
        if self.frame_width > 0 and self.frame_height > 0:
            width, height = self.frame_width, self.frame_height
        if self._journal is None:
            return
        index, partial_path, final_path = self._journal.allocate()
        fourcc = cv2.VideoWriter_fourcc(*self.fourcc)
        self._writer = cv2.VideoWriter(str(partial_path), fourcc, self.fps, (width, height))
        if not self._writer.isOpened():
            self.get_logger().error(f"VideoWriter açılamadı: {partial_path}")
            self._writer.release()
            self._writer = None
        else:
            self._segment_index = index
            self._segment_partial_path = partial_path
            self._segment_final_path = final_path
            self._segment_started_monotonic = now_monotonic
            self._last_frame_monotonic = now_monotonic
            self._segment_first_stamp = None
            self._segment_last_stamp = None
            self._segment_frame_count = 0
            self._segment_width = width
            self._segment_height = height
            self.get_logger().info(
                f"Video segmenti başladı: {partial_path.name} ({width}x{height})"
            )

    def _finalize_segment(self, reason: str) -> None:
        writer = self._writer
        if writer is None:
            return
        self._writer = None
        writer.release()
        partial_path = self._segment_partial_path
        final_path = self._segment_final_path
        frame_count = self._segment_frame_count
        try:
            if (
                frame_count > 0
                and partial_path is not None
                and final_path is not None
                and self._journal is not None
            ):
                committed = self._journal.commit(
                    partial_path,
                    final_path,
                    {
                        "segment_index": self._segment_index,
                        "first_stamp": self._segment_first_stamp,
                        "last_stamp": self._segment_last_stamp,
                        "frame_count": frame_count,
                        "width": self._segment_width,
                        "height": self._segment_height,
                        "fps": self.fps,
                        "close_reason": reason,
                    },
                )
                self.get_logger().info(
                    f"Video segmenti güvenle kapatıldı: {committed.name} ({frame_count} kare)"
                )
        except Exception as exc:
            self.get_logger().error(f"Video segmenti finalize edilemedi: {exc}")
        finally:
            self._segment_partial_path = None
            self._segment_final_path = None
            self._segment_started_monotonic = None
            self._last_frame_monotonic = None
            self._segment_first_stamp = None
            self._segment_last_stamp = None
            self._segment_frame_count = 0

    def tick(self) -> None:
        """Yapılandırılmış hızda düzenli satır yazımı (boşta timer korunur)."""
        if self._writer is None or self._last_frame_monotonic is None:
            return
        if time.monotonic() - self._last_frame_monotonic >= self.frame_stall_timeout_s:
            self.get_logger().warn("Kamera akışı kesildi; açık video segmenti kapatılıyor")
            self._finalize_segment("camera_stall")

    def destroy_node(self) -> None:
        self._finalize_segment("shutdown")
        if self._frames_csv_file is not None:
            self._frames_csv_file.flush()
            self._frames_csv_file.close()
        try:
            output = finalize_delivery_video(self._run_dir)
            if output is None:
                self.get_logger().warn(
                    "Tek processed_video.mp4 üretilemedi; finalize edilmiş segmentler korunuyor"
                )
            else:
                self.get_logger().info(f"Teslim videosu hazır: {output}")
        except Exception as exc:
            self.get_logger().error(
                f"Teslim videosu birleştirilemedi; segmentler korunuyor: {exc}"
            )
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
