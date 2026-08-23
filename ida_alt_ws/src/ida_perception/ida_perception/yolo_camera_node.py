"""YOLO kamera algı düğümü (ida_yolo_camera).

Kameradan gelen kareleri YOLO modeliyle işleyip /perception/buoys JSON kontratına
uygun tespitler yayınlar. İşlenmiş görüntü (tespit çerçeveleri çizilmiş JPEG)
/perception/processed_image_comp topic'ine sensor_msgs/CompressedImage olarak
gönderilir (logger tarafından dinlenir).

Model kütüphanesi, kamera ya da cv2 yoksa düğüm boş/eksik yayınlarla çalışmaya
devam eder — iskelet aşamasında tespit kaynağının yokluğu düğümü çökertmez.
"""

import math
import json
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from ida_perception.class_mapping import map_class_name, parse_class_name_map_strict
from ida_perception.model_loader import DEFAULT_CLASS_NAMES, load_model, parse_class_names
from ida_planning.contracts import dumps
from ida_planning.geo import clamp

try:  # cv2 opsiyonel: yoksa görüntü işleme pas geçilir.
    import cv2  # type: ignore

    CV2_AVAILABLE = True
except Exception:  # pragma: no cover - dev makinesinde cv2 olmayabilir
    cv2 = None  # type: ignore
    CV2_AVAILABLE = False


def open_v4l2_yuyv_camera(
    cv2_module: Any,
    camera_index: int,
    image_width: int,
    image_height: int,
    camera_fps: float,
) -> Any:
    """Open the B0495 through the verified low-latency V4L2/YUYV path.

    OpenCV's automatic backend selected GStreamer on the Jetson and produced a
    materially different capture path from the validated bench viewer.  When
    V4L2 is available, pin both backend and native YUYV FourCC; a non-Linux
    development build may fall back to the platform default backend.
    """
    backend = getattr(cv2_module, "CAP_V4L2", None)
    if backend is None:
        capture = cv2_module.VideoCapture(camera_index)
    else:
        capture = cv2_module.VideoCapture(camera_index, backend)
    fourcc_factory = getattr(cv2_module, "VideoWriter_fourcc", None)
    fourcc_property = getattr(cv2_module, "CAP_PROP_FOURCC", None)
    if callable(fourcc_factory) and fourcc_property is not None:
        capture.set(fourcc_property, fourcc_factory(*"YUYV"))
    capture.set(cv2_module.CAP_PROP_FRAME_WIDTH, image_width)
    capture.set(cv2_module.CAP_PROP_FRAME_HEIGHT, image_height)
    capture.set(cv2_module.CAP_PROP_FPS, camera_fps)
    capture.set(cv2_module.CAP_PROP_BUFFERSIZE, 1)
    return capture


def validate_yolo_config(
    imgsz: int,
    device: str,
    confidence: float,
    iou: float,
    fov_deg: float,
    publish_hz: float = 10.0,
    focal_length_px: float = 600.0,
    target_height_m: float = 0.5,
    image_width: int = 640,
    image_height: int = 480,
    model_type: str = "auto",
    model_path: str = "",
    camera_fps: float = 30.0,
) -> None:
    model_type = str(model_type).strip().lower()
    suffix = ""
    if model_path:
        from pathlib import Path

        suffix = Path(model_path).suffix.lower().lstrip(".")
    if (
        isinstance(imgsz, bool)
        or not isinstance(imgsz, int)
        or imgsz <= 0
        or not isinstance(device, str)
        or device != device.strip()
        or not math.isfinite(confidence)
        or not 0.0 <= confidence <= 1.0
        or not math.isfinite(iou)
        or not 0.0 <= iou <= 1.0
        or not math.isfinite(fov_deg)
        or not 0.0 < fov_deg < 180.0
        or not math.isfinite(publish_hz)
        or publish_hz <= 0.0
        or not math.isfinite(focal_length_px)
        or focal_length_px <= 0.0
        or not math.isfinite(target_height_m)
        or target_height_m <= 0.0
        or isinstance(image_width, bool)
        or not isinstance(image_width, int)
        or image_width <= 0
        or isinstance(image_height, bool)
        or not isinstance(image_height, int)
        or image_height <= 0
        or isinstance(camera_fps, bool)
        or not isinstance(camera_fps, (int, float))
        or not math.isfinite(float(camera_fps))
        or camera_fps <= 0.0
        or model_type not in {"auto", "pt", "engine", "onnx"}
        or (model_type != "auto" and suffix != model_type)
    ):
        raise ValueError("invalid YOLO runtime/model configuration")


def parse_allowed_colors(value: str, available_colors: List[str]) -> frozenset[str]:
    """Parse an exact role-level output allowlist.

    ``class_names`` must continue to describe the checkpoint's complete native
    class order.  This separate allowlist lets a five-colour general model feed
    only orange/yellow into the P1/P2 role without lying about model metadata.
    """
    normalized_available = [str(item).strip().lower() for item in available_colors]
    if not normalized_available or any(not item for item in normalized_available):
        raise ValueError("available colors must be non-empty strings")
    if not isinstance(value, str):
        raise ValueError("allowed_colors must be a string")
    raw = value.strip()
    if not raw:
        return frozenset(normalized_available)
    try:
        parsed = json.loads(raw) if raw.startswith("[") else raw.split(",")
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError("allowed_colors is malformed") from exc
    if not isinstance(parsed, list) or not parsed:
        raise ValueError("allowed_colors must contain at least one color")
    colors = [str(item).strip().lower() for item in parsed]
    if any(not item for item in colors) or len(set(colors)) != len(colors):
        raise ValueError("allowed_colors must be unique non-empty names")
    unknown = set(colors).difference(normalized_available)
    if unknown:
        raise ValueError(
            "allowed_colors must be a subset of mapped model colors: "
            + ",".join(sorted(unknown))
        )
    return frozenset(colors)


class YoloCameraNode(Node):
    """Kamera + YOLO tabanlı şamandıra/hedef tespit düğümü."""

    def __init__(self) -> None:
        super().__init__("ida_yolo_camera")
        self.declare_parameter("model_path", "")
        self.declare_parameter("model_type", "auto")
        # class_names STRING olarak declare edilir (virgülle ayrılmış: "orange,yellow").
        # parse_class_names ile listeye çevrilir. STRING_ARRAY olursa config'teki
        # STRING değer ("orange,yellow") ile uyumsuzluk olur (InvalidParameterTypeException).
        self.declare_parameter("class_names", "")
        self.declare_parameter(
            "class_name_map", ""
        )  # JSON {model_class: contract_color}; boş -> kimlik (İngilizce model)
        self.declare_parameter("allowed_colors", "")
        self.declare_parameter("secondary_output_topic", "")
        self.declare_parameter("secondary_allowed_colors", "")
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "")
        self.declare_parameter("camera_index", -1)
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("camera_topic_type", "compressed")
        self.declare_parameter("output_topic", "/perception/buoys")
        self.declare_parameter(
            "processed_image_topic", "/perception/processed_image_comp"
        )
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("camera_fps", 30.0)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("fov_deg", 90.0)
        self.declare_parameter("record_video", False)
        self.declare_parameter("record_path", "")
        self.declare_parameter("focal_length_px", 600.0)
        self.declare_parameter("target_height_m", 0.5)

        self.model_path = str(self.get_parameter("model_path").value)
        self.model_type = str(self.get_parameter("model_type").value)
        self.class_names = parse_class_names(str(self.get_parameter("class_names").value))
        # Model dili -> kontrat rengi eşlemesi. P3 İngilizce model (red/green/black)
        # zaten kontrat diliyle çıktı verir -> boş (kimlik). P1/P2 İngilizce model
        # (orange,yellow): boş bırakılır -> kimlik. Türkçe sınıf adlı model gerekirse
        # (geriye dönük) class_name_map parametresiyle eşleme verilir.
        self.class_name_map = parse_class_name_map_strict(
            str(self.get_parameter("class_name_map").value), self.class_names
        )
        mapped_colors = [
            map_class_name(name, self.class_name_map) for name in self.class_names
        ]
        if any(color is None for color in mapped_colors):
            raise ValueError("class_name_map does not cover the native model manifest")
        self.allowed_colors = parse_allowed_colors(
            str(self.get_parameter("allowed_colors").value),
            [str(color) for color in mapped_colors],
        )
        self.secondary_output_topic = str(
            self.get_parameter("secondary_output_topic").value
        ).strip()
        secondary_value = str(
            self.get_parameter("secondary_allowed_colors").value
        ).strip()
        self.secondary_allowed_colors = (
            parse_allowed_colors(
                secondary_value,
                [str(color) for color in mapped_colors],
            )
            if self.secondary_output_topic
            else frozenset()
        )
        if self.secondary_output_topic and not secondary_value:
            raise ValueError(
                "secondary_allowed_colors must be explicit when secondary output is enabled"
            )
        self.inference_allowed_colors = (
            self.allowed_colors | self.secondary_allowed_colors
        )
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)
        self.camera_index = int(self.get_parameter("camera_index").value)
        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.camera_topic_type = str(
            self.get_parameter("camera_topic_type").value
        ).strip().lower()
        if self.camera_topic_type not in {"compressed", "raw"}:
            raise ValueError("camera_topic_type must be compressed or raw")
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.processed_image_topic = str(
            self.get_parameter("processed_image_topic").value
        )
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.camera_fps = float(self.get_parameter("camera_fps").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.fov_deg = float(self.get_parameter("fov_deg").value)
        self.record_video = bool(self.get_parameter("record_video").value)
        self.record_path = str(self.get_parameter("record_path").value)
        self.focal_length_px = float(self.get_parameter("focal_length_px").value)
        self.target_height_m = float(self.get_parameter("target_height_m").value)
        validate_yolo_config(
            self.imgsz,
            self.device,
            self.confidence_threshold,
            self.iou_threshold,
            self.fov_deg,
            self.publish_hz,
            self.focal_length_px,
            self.target_height_m,
            self.image_width,
            self.image_height,
            self.model_type,
            self.model_path,
            self.camera_fps,
        )

        self.buoy_pub = self.create_publisher(String, self.output_topic, 20)
        self.secondary_buoy_pub = (
            self.create_publisher(String, self.secondary_output_topic, 20)
            if self.secondary_output_topic
            else None
        )
        self.image_pub = self.create_publisher(
            CompressedImage, self.processed_image_topic, 10
        )
        self.create_timer(1.0 / max(0.1, self.publish_hz), self.tick)

        self._video_writer: Any = None
        self._cap: Any = None
        self._topic_frame: Optional[Any] = None
        self._topic_acquisition_stamp: Optional[float] = None
        self._topic_header_fields: Optional[tuple[int, int, str]] = None
        self._cv_bridge: Any = None
        self._model = None
        self._preprocess = None
        self._infer = None
        self._inference_status = "model_unavailable"
        self._setup_model()
        self._setup_camera()

        if not CV2_AVAILABLE:
            self.get_logger().warn("cv2 bulunamadı; görüntü işleme ve video kaydı pasif")
        self.get_logger().info(
            f"YOLO camera node started (model={'none' if self._model is None else 'loaded'}, "
            f"classes={self.class_names})"
        )

    def _setup_model(self) -> None:
        """Modeli yükler; başarısızsa boş tespit moduna geçer."""
        if not self.model_path:
            self.get_logger().warn("model_path boş; boş detection yayınlanacak")
            return
        loaded = load_model(
            self.model_path,
            self.class_names,
            self.confidence_threshold,
            self.iou_threshold,
            self.imgsz,
            self.device,
        )
        if loaded is None:
            self.get_logger().warn(
                f"Model yüklenemedi (uzantı/model kütüphanesi eksik): {self.model_path}"
            )
            return
        self._model, self._preprocess, self._infer = loaded

    def _setup_camera(self) -> None:
        """Kamera kaynağını açar: index veya topic (topic tercih edilir)."""
        if not CV2_AVAILABLE:
            return
        if self.camera_topic:
            # Kamera yerine topic'ten frame almak subscriber ile olur.
            if self.camera_topic_type == "raw":
                self.frame_sub = self.create_subscription(
                    Image, self.camera_topic, self.on_frame_raw_topic, 10
                )
            else:
                self.frame_sub = self.create_subscription(
                    CompressedImage, self.camera_topic, self.on_frame_topic, 10
                )
            self.get_logger().info(
                "Kamera kaynağı olarak "
                f"{self.camera_topic_type} topic kullanılıyor: {self.camera_topic}"
            )
        elif self.camera_index >= 0:
            self._cap = open_v4l2_yuyv_camera(
                cv2,
                self.camera_index,
                self.image_width,
                self.image_height,
                self.camera_fps,
            )
            if not self._cap.isOpened():
                self.get_logger().warn(f"Kamera açılamadı (index={self.camera_index})")
                self._cap = None
        else:
            self.get_logger().warn("camera_index=-1 ve camera_topic boş; görüntü kaynağı yok")

    def _read_frame(
        self,
    ) -> tuple[Optional[Any], Optional[float], Optional[tuple[int, int, str]]]:
        """Return frame, acquisition seconds and original ROS header fields."""
        if self._cap is not None:
            ok, frame = self._cap.read()
            if ok:
                # VideoCapture has no source header; node time is the acquisition
                # time and is reused by detections, payload and annotated image.
                return frame, self.now_seconds(), None
            return None, None, None
        if self._topic_frame is not None:
            frame = self._topic_frame
            acquisition_stamp = self._topic_acquisition_stamp
            header_fields = self._topic_header_fields
            self._topic_frame = None
            self._topic_acquisition_stamp = None
            self._topic_header_fields = None
            return frame, acquisition_stamp, header_fields
        return None, None, None

    def on_frame_topic(self, msg: CompressedImage) -> None:
        """camera_topic'ten gelen sıkıştırılmış frame'i buffer'a alır."""
        if not CV2_AVAILABLE:
            return
        try:
            import numpy as np  # type: ignore

            buffer = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
            if frame is not None:
                self._topic_frame = frame
                self._store_topic_header(msg)
        except Exception:
            self._topic_frame = None
            self._topic_acquisition_stamp = None
            self._topic_header_fields = None

    def on_frame_raw_topic(self, msg: Image) -> None:
        """Buffer a raw sensor_msgs/Image while preserving acquisition time."""
        if not CV2_AVAILABLE:
            return
        try:
            if self._cv_bridge is None:
                from cv_bridge import CvBridge  # type: ignore

                self._cv_bridge = CvBridge()
            frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if frame is None:
                raise ValueError("raw camera conversion returned no frame")
            self._topic_frame = frame
            self._store_topic_header(msg)
        except Exception:
            self._topic_frame = None
            self._topic_acquisition_stamp = None
            self._topic_header_fields = None

    def _store_topic_header(self, msg: Any) -> None:
        sec = int(msg.header.stamp.sec)
        nanosec = int(msg.header.stamp.nanosec)
        acquisition_stamp = sec + nanosec / 1e9
        if acquisition_stamp <= 0.0:
            acquisition_stamp = self.now_seconds()
            sec = math.floor(acquisition_stamp)
            nanosec = round((acquisition_stamp - sec) * 1e9)
        self._topic_acquisition_stamp = acquisition_stamp
        self._topic_header_fields = (sec, nanosec, str(msg.header.frame_id))

    def tick(self) -> None:
        frame, acquisition_stamp, header_fields = self._read_frame()
        detections: List[Dict[str, Any]] = []
        annotated: Optional[Any] = None

        if acquisition_stamp is None:
            acquisition_stamp = self.now_seconds()

        if frame is not None:
            detections = self._run_inference(frame, acquisition_stamp)
            annotated = self._annotate(frame, detections) if CV2_AVAILABLE else None

        stale = frame is None or self._inference_status != "ok"
        self._publish_buoys(
            detections, frame is not None, acquisition_stamp, stale=stale
        )
        if self.secondary_buoy_pub is not None:
            self._publish_buoys(
                detections,
                frame is not None,
                acquisition_stamp,
                stale=stale,
                publisher=self.secondary_buoy_pub,
                allowed_colors=self.secondary_allowed_colors,
            )
        if annotated is not None:
            self._publish_processed_image(annotated, acquisition_stamp, header_fields)

    def _run_inference(
        self, frame: Any, acquisition_stamp: Optional[float] = None
    ) -> List[Dict[str, Any]]:
        """Model varsa inference çalıştırıp README detection listesi üretir.

        Sınıf eşlemesi: model sınıf adı (model dili) -> kontrat rengi (İngilizce).
        ``class_name_map`` boşsa kimlik eşleme (P1/P2 ve P3 modelleri zaten İngilizce
        sınıf adları döndürür: orange/yellow ve red/green/black). Eşlenemeyen sınıf
        yayından atlanır.
        """
        if self._infer is None:
            self._inference_status = "model_unavailable"
            return []
        if acquisition_stamp is None:
            acquisition_stamp = self.now_seconds()
        try:
            raw = self._infer(frame)
        except Exception:
            self._inference_status = "inference_error"
            return []
        if not isinstance(raw, list):
            self._inference_status = "inference_error"
            return []
        self._inference_status = "ok"
        detections: List[Dict[str, Any]] = []
        try:
            frame_height, frame_width = [int(value) for value in frame.shape[:2]]
        except (AttributeError, TypeError, ValueError):
            frame_width, frame_height = self.image_width, self.image_height
        if frame_width <= 0 or frame_height <= 0:
            return []
        for det in raw:
            if not isinstance(det, dict):
                continue
            model_class = str(det.get("class", "unknown")).lower()
            if model_class not in self.class_names:
                continue
            color = map_class_name(model_class, self.class_name_map)
            if color is None:
                # Eşleme dolu ama model sınıfı haritada yok: kontrat rengi üretilemez.
                continue
            if color not in self.inference_allowed_colors:
                continue
            try:
                confidence = float(det.get("confidence", 0.0))
                bbox = det.get("bbox", ())
                if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
                    continue
                x, y, w, h = [float(v) for v in bbox]
            except (TypeError, ValueError, OverflowError):
                continue
            if (
                not math.isfinite(confidence)
                or not 0.0 <= confidence <= 1.0
                or confidence < self.confidence_threshold
            ):
                continue
            if not all(math.isfinite(value) for value in (x, y, w, h)):
                continue
            x1 = clamp(x, 0.0, float(frame_width))
            y1 = clamp(y, 0.0, float(frame_height))
            x2 = clamp(x + w, 0.0, float(frame_width))
            y2 = clamp(y + h, 0.0, float(frame_height))
            clipped_w = x2 - x1
            clipped_h = y2 - y1
            if clipped_w <= 0.0 or clipped_h <= 0.0:
                continue
            bbox_width = clipped_w / frame_width
            bbox_height = clipped_h / frame_height
            bbox_size = max(bbox_width, bbox_height)
            # Mesafe tahmini: bilinen hedef yüksekliğinden ters-orantı.
            distance = self._estimate_distance(clipped_h)
            bearing = self._bbox_to_bearing(x1 + clipped_w / 2.0, frame_width)
            bbox_norm_x = clamp(bearing / (self.fov_deg / 2.0), -1.0, 1.0)
            detections.append(
                {
                    "stamp": acquisition_stamp,
                    "acquisition_stamp": acquisition_stamp,
                    # "color" kontrat rengidir (İngilizce; autonomy VALID_TARGET_COLORS
                    # ile doğrular). "class" yardımcı: modelin özgün sınıf adı tutulur
                    # (red/green/black ya da orange/yellow) — hata ayıklama için.
                    "color": color,
                    "class": model_class,
                    "distance": round(distance, 3),
                    "confidence": round(confidence, 3),
                    "bbox_norm_x": round(bbox_norm_x, 3),
                    "bbox_size": round(bbox_size, 3),
                    "bbox_x": round(x1 / frame_width, 6),
                    "bbox_y": round(y1 / frame_height, 6),
                    "bbox_width": round(bbox_width, 6),
                    "bbox_height": round(bbox_height, 6),
                    # Costmap füzyonu (ida_planning.costmap) için bearing-only
                    # kontrat girdisi: her detection'da garantidir. Costmap,
                    # turuncu/sarı detection'ı 6° eşiğindeki lidar engeline bu
                    # alanla eşler; füzyon mantığı costmap tarafındadır.
                    "bearing_deg": round(bearing, 3),
                }
            )
        return detections

    def _estimate_distance(self, bbox_size_px: float) -> float:
        """Bilinen hedef yüksekliğinden mesafe tahmini (f/px * H/m / h/px)."""
        if bbox_size_px <= 1e-6:
            return 0.0
        return self.focal_length_px * self.target_height_m / bbox_size_px

    def _bbox_to_bearing(
        self, bbox_center_x_px: float, image_width: Optional[int] = None
    ) -> float:
        """Bbox merkezi (px) -> gövde koordinatında yatak açısı (derece)."""
        width = self.image_width if image_width is None else image_width
        dx = bbox_center_x_px - width / 2.0
        # Görüntü genişliği başına piksel yoğunluğu; fov_deg merkezli.
        rad_per_px = math.radians(self.fov_deg) / max(1, width)
        return clamp(math.degrees(math.atan(math.tan(rad_per_px * width / 2.0) * (dx / max(1, width / 2.0)))), -90.0, 90.0)

    def _annotate(self, frame: Any, detections: List[Dict[str, Any]]) -> Any:
        """Detection çerçevelerini kareye çizer (cv2)."""
        frame_height, frame_width = frame.shape[:2]
        for det in detections:
            x1 = round(clamp(float(det.get("bbox_x", 0.0)), 0.0, 1.0) * frame_width)
            y1 = round(clamp(float(det.get("bbox_y", 0.0)), 0.0, 1.0) * frame_height)
            x2 = round(clamp(float(det.get("bbox_x", 0.0)) + float(det.get("bbox_width", 0.0)), 0.0, 1.0) * frame_width)
            y2 = round(clamp(float(det.get("bbox_y", 0.0)) + float(det.get("bbox_height", 0.0)), 0.0, 1.0) * frame_height)
            color = (0, 255, 0) if det.get("color") == "green" else (0, 0, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, str(det.get("color", "")), (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return frame

    def _publish_buoys(
        self,
        detections: List[Dict[str, Any]],
        frame_available: bool,
        acquisition_stamp: Optional[float] = None,
        stale: bool = False,
        publisher: Optional[Any] = None,
        allowed_colors: Optional[frozenset[str]] = None,
    ) -> None:
        """/perception/buoys kontratıyla tespitleri yayınlar (boş da olsa)."""
        msg = String()
        if acquisition_stamp is None:
            acquisition_stamp = self.now_seconds()
        selected_colors = self.allowed_colors if allowed_colors is None else allowed_colors
        selected_detections = [
            detection
            for detection in detections
            if str(detection.get("color", "")).lower() in selected_colors
        ]
        payload: Dict[str, Any] = {
            "stamp": acquisition_stamp,
            "acquisition_stamp": acquisition_stamp,
            "detections": selected_detections,
        }
        if not frame_available:
            payload["source"] = "no_camera"
        elif stale:
            payload["source"] = self._inference_status
        if stale:
            payload["stale"] = True
        msg.data = dumps(payload)
        (self.buoy_pub if publisher is None else publisher).publish(msg)

    def _publish_processed_image(
        self,
        frame: Any,
        acquisition_stamp: Optional[float] = None,
        header_fields: Optional[tuple[int, int, str]] = None,
    ) -> None:
        """İşlenmiş kareyi JPEG olarak /perception/processed_image_comp'a yayınlar."""
        try:
            ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                return
            comp = CompressedImage()
            comp.format = "jpeg"
            comp.data = jpeg.tobytes()
            if header_fields is not None:
                sec, nanosec, frame_id = header_fields
                comp.header.stamp.sec = sec
                comp.header.stamp.nanosec = nanosec
                comp.header.frame_id = frame_id
            else:
                if acquisition_stamp is None:
                    acquisition_stamp = self.now_seconds()
                sec = math.floor(acquisition_stamp)
                nanosec = round((acquisition_stamp - sec) * 1e9)
                if nanosec >= 1_000_000_000:
                    sec += 1
                    nanosec -= 1_000_000_000
                comp.header.stamp.sec = int(sec)
                comp.header.stamp.nanosec = int(nanosec)
            self.image_pub.publish(comp)
            self._write_video_frame(frame)
        except Exception as exc:
            self.get_logger().warn(f"Görüntü yayını başarısız: {exc}")

    def _write_video_frame(self, frame: Any) -> None:
        """record_video=True ise frame'i mp4'e yazar (ilk frame boyutunu kullanır)."""
        if not self.record_video or not CV2_AVAILABLE:
            return
        if self._video_writer is None:
            path = self.record_path or "perception_output.mp4"
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            h, w = frame.shape[:2]
            self._video_writer = cv2.VideoWriter(path, fourcc, 15.0, (w, h))
            self.get_logger().info(f"Video kaydı başladı: {path}")
        self._video_writer.write(frame)

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def destroy_node(self) -> None:
        if self._cap is not None:
            self._cap.release()
        if self._video_writer is not None:
            self._video_writer.release()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
