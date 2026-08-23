"""
yolo_vision_node
────────────────
Kamera → görüntü düzeltme (filtre) → YOLO tespiti → çizim → yayın + kayıt.

TASARIM: ÜÇ AYRI HIZ
────────────────────
Tek döngüde okuma+tespit+kayıt yapılırsa hepsi en yavaş adıma (TensorRT
çıkarımı, ~40 ms) iner ve görüntü 20-24 FPS'e takılır. Burada üç iş ayrıldı:

  1. YAKALAMA thread'i   — kameradan sürekli okur, filtreyi uygular ve son
                           kareyi saklar. Kamera ne veriyorsa o hızda (50 FPS).
  2. TESPİT thread'i     — son kareyi alıp YOLO çalıştırır, kutuları saklar.
                           Engine ne kadar hızlıysa o kadar (~24 FPS).
  3. ÇIKIŞ timer'ı       — son kare + son kutuları birleştirip çizer, yayınlar
                           ve kaydeder. Yakalama hızında.

Böylece görüntü 50 FPS akıcı kalırken tespit 24 Hz güncellenir. Kutular en
fazla bir tespit periyodu kadar (~40 ms) eskidir — gözle fark edilmez.
Tek döngüde bunu yapmak, kamerada biriken kareler yüzünden görüntünün
giderek GECİKMESİNE de yol açıyordu; yakalama thread'i her zaman en son
kareyi tuttuğu için o gecikme de ortadan kalkar.

GÜNEŞ ALTINDA BEYAZLAMA
───────────────────────
İki katman var ve ikisi de arayüzden CANLI ayarlanır:

  * Kamera (V4L2) kontrolleri — asıl çözüm. Otomatik pozlama güneşte kareyi
    yakar; `auto_exposure=1` (manuel) + düşük `exposure` ile düzelir.
  * Yazılım filtresi — gamma, CLAHE (yerel kontrast), parlaklık/kontrast,
    doygunluk. `filter_before_detect` açıkken filtre YOLO'ya giden kareye de
    uygulanır, yani kalibrasyon tespiti doğrudan iyileştirir.

KAYIT
─────
Kayıt `record_segment_sec` saniyede bir YENİ dosyaya geçer. mp4 ancak dosya
düzgün kapatıldığında (moov atom) oynatılabilir; Jetson çökerse yalnızca
o anki segment kaybolur, öncekiler sağlam kalır. Tek uzun dosyada çökme
kaydın TAMAMINI okunamaz hâle getiriyordu.
"""

import json
import os
import threading
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Header, String, Int32
from sensor_msgs.msg import Image, CompressedImage

from idaws_msgs.msg import Buoy, BuoyArray

# telemetry/user_params (SCR_USER1..4) durum taşır, olay değil — latched
# abonelik olmazsa sonradan başlayan node mevcut SCR_USER4'ü kaçırır.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


# Arayüzden canlı ayarlanabilen V4L2 kamera kontrolleri.
# DİKKAT (V4L2): auto_exposure 3 = OTOMATİK, 1 = MANUEL. Manuel'e almadan
# `exposure` yazmak çoğu sürücüde sessizce yok sayılır.
V4L2_CONTROLS = {
    'auto_exposure':   cv2.CAP_PROP_AUTO_EXPOSURE,
    'exposure':        cv2.CAP_PROP_EXPOSURE,
    'brightness':      cv2.CAP_PROP_BRIGHTNESS,
    'contrast':        cv2.CAP_PROP_CONTRAST,
    'saturation':      cv2.CAP_PROP_SATURATION,
    'gain':            cv2.CAP_PROP_GAIN,
    'sharpness':       cv2.CAP_PROP_SHARPNESS,
    'auto_wb':         cv2.CAP_PROP_AUTO_WB,
    'wb_temperature':  cv2.CAP_PROP_WB_TEMPERATURE,
    'autofocus':       cv2.CAP_PROP_AUTOFOCUS,
    'focus':           cv2.CAP_PROP_FOCUS,
    'zoom':            cv2.CAP_PROP_ZOOM,
}

# Yazılım filtresi parametreleri (hepsi canlı ayarlanır).
FILTER_PARAMS = (
    'filter_enabled', 'filter_before_detect', 'gamma', 'brightness_gain',
    'brightness_offset', 'saturation_scale', 'clahe_enabled', 'clahe_clip',
    'clahe_grid',
)

# -1 = "dokunma" (kameranın kendi varsayılanını koru). V4L2 kontrollerinin
# geçerli aralıkları kameradan kameraya değişiyor; sabit bir varsayılan
# dayatmak bazı kameralarda görüntüyü tamamen bozuyordu.
UNSET = -1.0


class YoloVisionNode(Node):

    def __init__(self):
        super().__init__('yolo_vision_node')

        # ---------- Kamera ----------
        self.declare_parameter('video_source', 'device')   # device | gstreamer | ros
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('gstreamer_port', 5400)
        self.declare_parameter('input_width', 1920)
        self.declare_parameter('input_height', 1080)
        self.declare_parameter('input_fps', 50)
        # MJPG ŞART: V4L2 varsayılanı YUYV'dir ve 1920x1080'de çoğu USB kamera
        # YUYV ile yalnızca 5 FPS verir. MJPG'e geçmeden yüksek FPS alınamaz.
        self.declare_parameter('fourcc', 'MJPG')

        # ---------- V4L2 kontrolleri (canlı) ----------
        for name in V4L2_CONTROLS:
            self.declare_parameter(name, UNSET)

        # ---------- Yazılım filtresi (canlı) ----------
        self.declare_parameter('filter_enabled', True)
        # Filtre YOLO'ya giden kareye de uygulansın mı? Açık olması güneşte
        # tespiti doğrudan iyileştirir; kapalıyken filtre yalnızca görüntüyü
        # ve kaydı etkiler, model ham kareyi görür.
        self.declare_parameter('filter_before_detect', True)
        self.declare_parameter('gamma', 1.0)               # <1 koyulaştırır, >1 açar
        self.declare_parameter('brightness_gain', 1.0)     # kontrast (alpha)
        self.declare_parameter('brightness_offset', 0.0)   # parlaklık (beta)
        self.declare_parameter('saturation_scale', 1.0)
        # CLAHE: güneşte yanmış/arkadan ışıklı sahnelerde en etkili adım, ama
        # 1080p'de ~10 ms CPU yer — FPS düşerse ilk bunu kapat.
        self.declare_parameter('clahe_enabled', False)
        self.declare_parameter('clahe_clip', 2.0)
        self.declare_parameter('clahe_grid', 8)

        # ---------- Tespit ----------
        self.declare_parameter('use_yolo', True)
        self.declare_parameter('model_path', 'best.engine')
        # Sahada Jetson'a terminal/ağ erişimi olmadığı için model değiştirmenin
        # TEK yolu bu otopilot parametresi (yer istasyonundan yazılabilir).
        # 0 = değişiklik yok (yukarıdaki model_path kullanılır). 1/2/3... —
        # model_dir İÇİNDEKİ *.engine/*.pt dosyalarının, isme göre alfabetik
        # sıralanmış, 1-tabanlı indeksi. Liste HER SCR_USER4 değişiminde
        # yeniden taranır (bkz. _scan_model_dir) — dizine yeni bir dosya
        # atmak (scp) yeniden derleme/restart GEREKTİRMEZ. Panelde hem seçili
        # modelin adı hem TÜM dizin listesi (index → dosya adı) gösterilir ki
        # operatör hangi SCR_USER4 değerinin hangi dosyaya karşılık geldiğini
        # SSH olmadan görebilsin.
        self.declare_parameter('model_select_user_param', 'SCR_USER4')
        self.declare_parameter('model_dir', '/home/ezelproject/idaws_models')
        self.declare_parameter('confidence_threshold', 0.45)
        # Çıkarıma giren karenin genişliği. TensorRT engine'in giriş boyutu
        # SABİTTİR (export'ta ne verildiyse, genelde 640), bu yüzden burayı
        # büyütmek çıkarım SÜRESİNİ değiştirmez — sadece engine'e gitmeden
        # önceki yeniden boyutlandırma maliyetini ve küçük/uzak dubaların
        # ayrıntısını etkiler. Arayüzden canlı denenebilir.
        self.declare_parameter('infer_width', 960)
        self.declare_parameter('detect_hz', 30.0)   # üst sınır; engine daha yavaşsa o belirler

        # ---------- Yayın ----------
        self.declare_parameter('annotated_topic', 'vision/image_annotated')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('output_hz', 50.0)
        self.declare_parameter('stream_max_width', 960)
        # Akış JPEG (CompressedImage) olarak gider. ÖLÇÜLDÜ: 960x540 HAM Image
        # yayınlamak çıkışı 31 FPS'ten 5.4 FPS'e düşürüyordu — kare başına
        # 1.5 MB'ı DDS'ten geçirmek, işin geri kalanının (çizim+ölçek+kayıt,
        # toplam ~3.5 ms) 50 katı. JPEG ~60 KB ve encode yalnızca ~1 ms; ayrıca
        # panel baytları tarayıcıya OLDUĞU GİBİ verir, hiç decode etmez.
        self.declare_parameter('stream_jpeg_quality', 70)
        # Panel zaten ~20-25 FPS gösteriyor; akışı çıkış hızından ayırmak
        # kaydı/tespiti yavaşlatmadan bant genişliğini düşürür.
        self.declare_parameter('stream_hz', 20.0)
        # Ham Image'a ihtiyaç duyan bir tüketici olursa (varsayılan: yok).
        self.declare_parameter('publish_raw_annotated', False)

        # ---------- Kayıt ----------
        self.declare_parameter('record_enabled', True)
        self.declare_parameter('record_dir', '/home/jetson/idaws_logs')
        # Kayıt çözünürlüğü/hızı yayından AYRI: 1080p50 CPU encode Jetson'ı
        # boğuyor ve asıl işi (tespit) yavaşlatıyor.
        self.declare_parameter('record_width', 1280)
        self.declare_parameter('record_fps', 25.0)
        # Her segment ayrı mp4 — çökmede yalnızca son segment kaybolur.
        self.declare_parameter('record_segment_sec', 30.0)
        # Diskte tutulacak azami segment sayısı (0 = sınırsız). Uzun testte
        # kartın dolup Jetson'ı kilitlemesini önler.
        self.declare_parameter('record_max_segments', 240)   # 240 x 30 sn = 2 saat

        p = self.get_parameter
        self._video_source = p('video_source').value
        self._camera_idx = int(p('camera_index').value)
        self._camera_topic = p('camera_topic').value
        self._gst_port = int(p('gstreamer_port').value)
        self._width = int(p('input_width').value)
        self._height = int(p('input_height').value)
        self._fps = int(p('input_fps').value)
        self._fourcc = str(p('fourcc').value)

        self._use_yolo = bool(p('use_yolo').value)
        self._model_path = p('model_path').value
        self._model_select_param = p('model_select_user_param').value
        self._model_dir = p('model_dir').value
        self._model_select_index = 0
        self._model_display_name = os.path.basename(self._model_path)
        self._model_available = []   # [(index, name)] — panel için (bkz. _scan_model_dir)
        self._conf = float(p('confidence_threshold').value)
        self._infer_width = int(p('infer_width').value)
        self._detect_hz = max(1.0, float(p('detect_hz').value))

        self._annotated_topic = p('annotated_topic').value
        self._publish_annotated = bool(p('publish_annotated').value)
        self._output_hz = max(1.0, float(p('output_hz').value))
        self._stream_max_width = int(p('stream_max_width').value)
        self._stream_quality = int(p('stream_jpeg_quality').value)
        self._stream_hz = max(1.0, float(p('stream_hz').value))
        self._publish_raw = bool(p('publish_raw_annotated').value)

        self._record_enabled = bool(p('record_enabled').value)
        self._record_dir = p('record_dir').value
        self._record_width = int(p('record_width').value)
        self._record_fps = max(1.0, float(p('record_fps').value))
        self._segment_sec = max(5.0, float(p('record_segment_sec').value))
        self._max_segments = int(p('record_max_segments').value)

        # ---------- Durum ----------
        self._lock = threading.Lock()
        self._running = True
        self._latest_frame = None       # filtre UYGULANMIŞ son kare
        self._latest_raw = None         # filtresiz (filter_before_detect=False için)
        self._frame_seq = 0
        self._boxes = []                # [(x1,y1,x2,y2,label,conf)] — orijinal kare koordinatı
        self._boxes_seq = -1
        self._ros_frame = None
        self._cap = None
        self._model = None
        self._clahe = None

        self._writer = None
        self._writer_path = None
        self._last_stream_t = 0.0
        self._segment_started = 0.0
        self._segment_files = deque()
        self._last_record_write = 0.0

        self._t_capture = deque(maxlen=60)
        self._t_detect = deque(maxlen=60)
        self._t_output = deque(maxlen=60)
        self._detect_ms = 0.0
        self._camera_error = ''

        # ---------- Publisher ----------
        self.pub_buoys = self.create_publisher(BuoyArray, 'vision/buoys', 10)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Asıl akış: JPEG. Ham Image yalnızca istenirse.
        self.pub_compressed = self.create_publisher(
            CompressedImage, self._annotated_topic + '/compressed', sensor_qos)
        self.pub_annotated = self.create_publisher(Image, self._annotated_topic, sensor_qos)
        # Arayüzün FPS/kayıt durumunu gösterebilmesi için.
        self.pub_stats = self.create_publisher(String, 'vision/stats', 10)

        if self._video_source == 'ros':
            self.create_subscription(Image, self._camera_topic, self._cb_image, sensor_qos)
            self.get_logger().info(f'Kamera kaynağı: ROS topic — {self._camera_topic}')
        else:
            self._open_camera()

        self.create_subscription(String, 'telemetry/user_params', self._cb_user_params, LATCHED_QOS)
        # Web panelinden doğrudan model seçimi — SCR_USER4 ile AYNI switching
        # mantığını (bkz. _select_model_by_index) kullanır, sahada Jetson'a
        # network erişimi panel üzerinden zaten var (hotspot).
        self.create_subscription(Int32, 'cmd/select_model', self._cb_select_model, 10)

        self._build_clahe()
        if self._use_yolo:
            self._load_model()

        self.add_on_set_parameters_callback(self._on_param_change)

        # ---------- Thread'ler ve timer'lar ----------
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        self._detect_thread = threading.Thread(target=self._detect_loop, daemon=True)
        self._detect_thread.start()
        # Çıkış da AYRI THREAD — ROS timer'ı OLARAK ÇALIŞTIRILAMAZ. Çizim +
        # ölçekleme + Image dönüşümü + encode 1080p'de periyottan uzun sürüyor
        # ve timer executor'ı doyurup AYNI node'daki diğer her şeyi aç
        # bırakıyor: istatistik timer'ı susuyor ve daha kötüsü parametre
        # servisi (/yolo_vision_node/set_parameters) yanıt vermiyor — yani
        # panelden hiçbir ayar değiştirilemiyordu. Executor'da yalnızca hafif
        # işler kalmalı.
        self._output_thread = threading.Thread(target=self._output_loop, daemon=True)
        self._output_thread.start()

        self.create_timer(1.0, self._publish_stats)

        self.get_logger().info(
            f'yolo_vision_node başlatıldı — giriş {self._width}x{self._height}@{self._fps} '
            f'({self._fourcc}), çıkarım genişliği {self._infer_width}, '
            f'kayıt {self._record_width}px @ {self._record_fps:.0f} FPS, '
            f'{self._segment_sec:.0f} sn\'lik segmentler.'
        )

    # ═════════════════════ Kamera ═════════════════════

    def _open_camera(self):
        if self._video_source == 'gstreamer':
            pipeline = (
                f'udpsrc port={self._gst_port} ! application/x-rtp, payload=96 ! '
                'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
                'appsink drop=true sync=false'
            )
            self.get_logger().info(f'Kamera açılıyor (GStreamer, port={self._gst_port})')
            self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            self.get_logger().info(f'Kamera açılıyor (index={self._camera_idx}, V4L2)')
            self._cap = cv2.VideoCapture(self._camera_idx, cv2.CAP_V4L2)
            if self._cap.isOpened():
                # SIRA ÖNEMLİ: fourcc, çözünürlük/FPS'ten ÖNCE. Sürücü kabul
                # edilebilir kip listesini fourcc'e göre belirliyor; sonra
                # ayarlanırsa 1080p50 sessizce 1080p5'e düşer.
                if self._fourcc:
                    self._cap.set(cv2.CAP_PROP_FOURCC,
                                  cv2.VideoWriter_fourcc(*self._fourcc))
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
                self._cap.set(cv2.CAP_PROP_FPS, self._fps)
                # Gecikmenin ana kaynağı sürücü kuyruğu — en son kare okunsun.
                self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if self._cap is None or not self._cap.isOpened():
            self._camera_error = 'Kamera açılamadı'
            self.get_logger().error(
                f'Kamera açılamadı (kaynak={self._video_source}). '
                'Başka bir node /dev/video0\'ı tutuyor olabilir.'
            )
            self._cap = None
            return

        actual_w = self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Kamera açıldı — sürücünün verdiği: {actual_w:.0f}x{actual_h:.0f} @ {actual_fps:.0f} FPS'
        )
        if actual_fps and actual_fps < self._fps * 0.6:
            self.get_logger().warn(
                f'Sürücü {actual_fps:.0f} FPS bildiriyor ({self._fps} istendi) — '
                f'fourcc={self._fourcc} bu çözünürlükte desteklenmiyor olabilir. '
                '`v4l2-ctl --list-formats-ext` ile kipleri kontrol edin.'
            )
        self._camera_error = ''
        self._apply_v4l2_controls()

    def _apply_v4l2_controls(self):
        """UNSET (-1) olanlara dokunulmaz — kameranın kendi varsayılanı kalır."""
        if self._cap is None:
            return
        for name, prop in V4L2_CONTROLS.items():
            value = float(self.get_parameter(name).value)
            if value == UNSET:
                continue
            self._cap.set(prop, value)

    def _cb_image(self, msg: Image):
        frame = self._imgmsg_to_bgr(msg)
        if frame is not None:
            self._ros_frame = frame

    @staticmethod
    def _imgmsg_to_bgr(msg: Image):
        """sensor_msgs/Image → BGR numpy dizisi (cv_bridge bağımlılığı olmadan)."""
        enc = msg.encoding.lower()
        channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(enc)
        if channels is None:
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.step
        if buf.size < expected:
            return None
        img = buf[:expected].reshape(msg.height, msg.step)
        img = img[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        if enc == 'rgb8':
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if enc == 'rgba8':
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if enc == 'bgra8':
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        if enc == 'mono8':
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return img

    # ═════════════════════ Filtre ═════════════════════

    def _build_clahe(self):
        clip = float(self.get_parameter('clahe_clip').value)
        grid = max(1, int(self.get_parameter('clahe_grid').value))
        self._clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(grid, grid))

    def _apply_filter(self, frame):
        """Güneş altında yanmış görüntüyü toparlar. Sıra önemli: önce gamma
        (ton eğrisi), sonra yerel kontrast (CLAHE), sonra doğrusal
        parlaklık/kontrast, en son doygunluk."""
        if not bool(self.get_parameter('filter_enabled').value):
            return frame

        gamma = float(self.get_parameter('gamma').value)
        gain = float(self.get_parameter('brightness_gain').value)
        offset = float(self.get_parameter('brightness_offset').value)
        sat = float(self.get_parameter('saturation_scale').value)
        clahe_on = bool(self.get_parameter('clahe_enabled').value)

        out = frame

        if abs(gamma - 1.0) > 1e-3 and gamma > 0:
            # LUT tek geçişte uygulanır — piksel başına pow()'dan çok daha hızlı.
            inv = 1.0 / gamma
            lut = np.clip(((np.arange(256) / 255.0) ** inv) * 255.0, 0, 255).astype(np.uint8)
            out = cv2.LUT(out, lut)

        if clahe_on:
            # Sadece parlaklık kanalına: RGB'ye tek tek uygulamak renkleri kaydırır.
            lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            l = self._clahe.apply(l)
            out = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)

        if abs(gain - 1.0) > 1e-3 or abs(offset) > 1e-3:
            out = cv2.convertScaleAbs(out, alpha=gain, beta=offset)

        if abs(sat - 1.0) > 1e-3:
            hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[:, :, 1] = np.clip(hsv[:, :, 1] * sat, 0, 255)
            out = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

        return out

    # ═════════════════════ 1. Yakalama thread'i ═════════════════════

    def _capture_loop(self):
        while self._running:
            frame = None
            if self._video_source == 'ros':
                frame = self._ros_frame
                if frame is None:
                    time.sleep(0.01)
                    continue
                time.sleep(1.0 / max(1, self._fps))
            else:
                if self._cap is None:
                    time.sleep(1.0)
                    self._open_camera()
                    continue
                ok, frame = self._cap.read()
                if not ok or frame is None:
                    self._camera_error = 'Kare okunamadı'
                    time.sleep(0.05)
                    continue
                self._camera_error = ''

            try:
                filtered = self._apply_filter(frame)
            except Exception as e:
                self.get_logger().warn(f'Filtre hatası: {e}')
                filtered = frame

            with self._lock:
                self._latest_raw = frame
                self._latest_frame = filtered
                self._frame_seq += 1
            self._t_capture.append(time.monotonic())

    # ═════════════════════ 2. Tespit thread'i ═════════════════════

    def _load_model(self, path=None, name=None):
        """Başlangıçta senkron (__init__'te), model değişiminde ayrı thread'de
        (bkz. _cb_user_params) çağrılır — TensorRT engine yüklemesi birkaç
        saniye sürebilir, bunu executor/callback thread'inde yapmak parametre
        servisini ve istatistik timer'ını tıkar (bkz. modül başındaki not)."""
        path = path or self._model_path
        try:
            from ultralytics import YOLO
            self.get_logger().info(f'YOLO modeli yükleniyor: {path}')
            model = YOLO(path)
            self._model = model
            self._model_path = path
            self._model_display_name = name or os.path.basename(path)
            self.get_logger().info(f'YOLO modeli hazır: {self._model_display_name}')
        except Exception as e:
            self._model = None
            self.get_logger().error(
                f'YOLO modeli yüklenemedi ({path}): {e} — kamera akışı '
                've kayıt devam ediyor, tespit yapılmıyor.'
            )

    def _scan_model_dir(self):
        """model_dir içindeki *.engine/*.pt DOSYALARINI (klasör DEĞİL) isme
        göre alfabetik sıralı [(yol, ad)] olarak döner — ad, uzantısız dosya
        adıdır. Her çağrıda yeniden taranır ki dizine yeni atılan bir dosya
        (scp) yeniden derleme/restart gerektirmeden görünsün.

        isfile() kontrolü ŞART: .pt aslında bir ZIP arşividir, bazı yükleme
        yollarında (ör. bir tarayıcı/araç zip'i otomatik açması) dosya yerine
        AYNI ADDA BİR KLASÖR oluşabiliyor — bu durumda YOLO(...) 'Is a
        directory' hatasıyla çöküyordu. isfile() bunu baştan eler."""
        try:
            files = sorted(
                f for f in os.listdir(self._model_dir)
                if f.lower().endswith(('.engine', '.pt'))
                and os.path.isfile(os.path.join(self._model_dir, f))
            )
        except OSError as e:
            self.get_logger().warn(f'model_dir okunamadı ({self._model_dir}): {e}')
            return []
        return [(os.path.join(self._model_dir, f), os.path.splitext(f)[0]) for f in files]

    def _cb_user_params(self, msg: String):
        """SCR_USER4 — sahada Jetson'a terminal/ağ erişimi olmadığında model
        değiştirmenin yolu. 0=değişiklik yok, 1/2/3... model_dir'in HER
        değişimde yeniden taranmış, alfabetik sıralı 1-tabanlı indeksi."""
        try:
            params = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if self._model_select_param not in params:
            return

        idx = int(round(float(params[self._model_select_param])))
        if idx == self._model_select_index:
            return
        self._select_model_by_index(idx, source=f'{self._model_select_param}=')

    def _cb_select_model(self, msg: Int32):
        """Web panelinden doğrudan model seçimi (bkz. web_ui_node /api/model/select).
        SCR_USER4 ile aynı model_dir indeksini kullanır, ama otopilot
        parametresine dokunmaz — panel ile SCR_USER4 birbirinden bağımsız,
        ikisi de aynı switching mantığına varır."""
        idx = int(msg.data)
        if idx == self._model_select_index:
            return
        self._select_model_by_index(idx, source='panel: ')

    def _select_model_by_index(self, idx: int, source: str = ''):
        """model_dir'deki dosyalardan idx (1-tabanlı, _scan_model_dir sırasına
        göre) ile seçileni yükler. Hem SCR_USER4 hem web panelindeki doğrudan
        seçim bunu kullanır — tek switching mantığı, iki tetikleyici."""
        self._model_select_index = idx
        if idx <= 0:
            return

        available = self._scan_model_dir()
        self._model_available = [(i + 1, name) for i, (_, name) in enumerate(available)]
        if idx > len(available):
            self.get_logger().warn(
                f'{source}model_index={idx} — model_dir\'de yalnızca '
                f'{len(available)} dosya var, yok sayılıyor.'
            )
            return

        path, name = available[idx - 1]
        if path == self._model_path:
            return
        self.get_logger().info(
            f'{source}model_index={idx} → model değiştiriliyor: {name} ({path})'
        )
        threading.Thread(target=self._load_model, args=(path, name), daemon=True).start()

    def _detect_loop(self):
        period = 1.0 / self._detect_hz
        while self._running:
            start = time.monotonic()

            if self._model is None:
                time.sleep(0.2)
                continue

            with self._lock:
                use_filtered = bool(self.get_parameter('filter_before_detect').value)
                frame = self._latest_frame if use_filtered else self._latest_raw
                seq = self._frame_seq
                if frame is not None:
                    frame = frame.copy()

            if frame is None:
                time.sleep(0.05)
                continue

            try:
                boxes = self._infer(frame)
            except Exception as e:
                self.get_logger().warn(f'Tespit hatası: {e}')
                time.sleep(0.1)
                continue

            with self._lock:
                self._boxes = boxes
                self._boxes_seq = seq

            self._publish_buoys(boxes, frame.shape[1], frame.shape[0])
            self._detect_ms = (time.monotonic() - start) * 1000.0
            self._t_detect.append(time.monotonic())

            # detect_hz bir ÜST SINIR; engine daha yavaşsa uyumaya gerek yok.
            slack = period - (time.monotonic() - start)
            if slack > 0:
                time.sleep(slack)

    def _infer(self, frame):
        """Kareyi infer_width'e küçültüp çalıştırır, kutuları ORİJİNAL kare
        koordinatına geri ölçekler. Küçültme ölçeği tek sayı (en-boy korunur),
        bu yüzden geri dönüşüm basit bir çarpma."""
        h, w = frame.shape[:2]
        scale = 1.0
        infer_frame = frame
        if 0 < self._infer_width < w:
            scale = self._infer_width / float(w)
            infer_frame = cv2.resize(
                frame, (self._infer_width, int(round(h * scale))),
                interpolation=cv2.INTER_LINEAR)

        results = self._model(infer_frame, conf=self._conf, verbose=False)

        boxes = []
        for r in results:
            names = r.names
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                if scale != 1.0:
                    x1, y1, x2, y2 = x1 / scale, y1 / scale, x2 / scale, y2 / scale
                cls_id = int(box.cls[0])
                boxes.append((
                    int(x1), int(y1), int(x2), int(y2),
                    names.get(cls_id, str(cls_id)),
                    float(box.conf[0]),
                ))
        return boxes

    def _publish_buoys(self, boxes, width, height):
        msg = BuoyArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.frame_width = int(width)
        msg.frame_height = int(height)
        for x1, y1, x2, y2, label, conf in boxes:
            b = Buoy()
            b.x_min, b.y_min, b.x_max, b.y_max = x1, y1, x2, y2
            b.center_x = (x1 + x2) / 2.0
            b.center_y = (y1 + y2) / 2.0
            b.confidence = conf
            b.label = label
            msg.buoys.append(b)
        self.pub_buoys.publish(msg)

    # ═════════════════════ 3. Çıkış (çizim + yayın + kayıt) ═════════════════════

    def _output_loop(self):
        """Kendi hızında döner; çıkış işi executor'a ASLA konulmaz (bkz.
        __init__'teki açıklama). Yakalamadan hızlı dönmesinin anlamı yok —
        aynı kareyi iki kez çizip yayınlamak boşa CPU."""
        period = 1.0 / self._output_hz
        last_seq = -1
        while self._running:
            start = time.monotonic()
            with self._lock:
                seq = self._frame_seq
            if seq != last_seq:
                last_seq = seq
                try:
                    self._output_tick()
                except Exception as e:
                    self.get_logger().warn(f'Çıkış hatası: {e}')
            slack = period - (time.monotonic() - start)
            time.sleep(slack if slack > 0 else 0.001)

    def _output_tick(self):
        with self._lock:
            frame = self._latest_frame
            boxes = list(self._boxes)
            if frame is not None:
                frame = frame.copy()
        if frame is None:
            return

        self._draw(frame, boxes)

        if self._publish_annotated:
            now = time.monotonic()
            if now - self._last_stream_t >= 1.0 / self._stream_hz:
                self._last_stream_t = now
                small = self._downscale(frame, self._stream_max_width)
                ok, buf = cv2.imencode(
                    '.jpg', small, [int(cv2.IMWRITE_JPEG_QUALITY), self._stream_quality])
                if ok:
                    cmsg = CompressedImage()
                    cmsg.header.stamp = self.get_clock().now().to_msg()
                    cmsg.header.frame_id = 'camera'
                    cmsg.format = 'jpeg'
                    cmsg.data = buf.tobytes()
                    self.pub_compressed.publish(cmsg)
                if self._publish_raw:
                    self.pub_annotated.publish(self._bgr_to_imgmsg(small))

        if self._record_enabled:
            self._record(frame)

        self._t_output.append(time.monotonic())

    def _draw(self, frame, boxes):
        for x1, y1, x2, y2, label, conf in boxes:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{label} {conf:.2f}', (x1, max(12, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # Şartname: kaydedilen her karede zaman etiketi görünür olacak.
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        cv2.putText(frame, stamp, (8, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    @staticmethod
    def _downscale(frame, max_width):
        h, w = frame.shape[:2]
        if max_width <= 0 or w <= max_width:
            return frame
        scale = max_width / float(w)
        return cv2.resize(frame, (max_width, int(h * scale)), interpolation=cv2.INTER_AREA)

    def _bgr_to_imgmsg(self, frame) -> Image:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(frame).tobytes()
        return msg

    # ═════════════════════ Kayıt (segmentli) ═════════════════════

    def _record(self, frame):
        now = time.monotonic()
        # Kayıt hızını çıkış hızından bağımsız tut — 50 FPS'te 1080p encode
        # Jetson'ı boğuyor ve asıl iş olan tespiti yavaşlatıyor.
        if now - self._last_record_write < 1.0 / self._record_fps:
            return
        self._last_record_write = now

        rec = self._downscale(frame, self._record_width)

        if self._writer is not None and (now - self._segment_started) >= self._segment_sec:
            self._close_segment()
        if self._writer is None:
            self._open_segment(rec)
        if self._writer is not None:
            self._writer.write(rec)

    def _open_segment(self, frame):
        try:
            os.makedirs(self._record_dir, exist_ok=True)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = os.path.join(self._record_dir, f'detect_{ts}.mp4')
            h, w = frame.shape[:2]
            self._writer = cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*'mp4v'), self._record_fps, (w, h))
            if not self._writer.isOpened():
                self.get_logger().error(f'Kayıt dosyası açılamadı: {path}')
                self._writer = None
                return
            self._writer_path = path
            self._segment_started = time.monotonic()
            self._segment_files.append(path)
            self._prune_segments()
            self.get_logger().info(f'Kayıt segmenti: {os.path.basename(path)} ({w}x{h})')
        except Exception as e:
            self.get_logger().error(f'Kayıt başlatılamadı: {e}')
            self._writer = None

    def _close_segment(self):
        if self._writer is None:
            return
        try:
            self._writer.release()   # moov atom burada yazılır — dosya oynatılabilir olur
        except Exception as e:
            self.get_logger().warn(f'Kayıt kapatma hatası: {e}')
        self._writer = None
        self._writer_path = None
        self._last_stream_t = 0.0

    def _prune_segments(self):
        """Kart dolup Jetson'ı kilitlemesin diye en eski segmentleri siler."""
        if self._max_segments <= 0:
            return
        while len(self._segment_files) > self._max_segments:
            old = self._segment_files.popleft()
            try:
                os.remove(old)
                self.get_logger().info(f'Eski segment silindi: {os.path.basename(old)}')
            except OSError:
                pass

    # ═════════════════════ İstatistik ═════════════════════

    @staticmethod
    def _hz(samples):
        if len(samples) < 2:
            return 0.0
        span = samples[-1] - samples[0]
        return (len(samples) - 1) / span if span > 0 else 0.0

    def _publish_stats(self):
        # 1 Hz'de os.listdir — ucuz, executor'ı bloklamaz (YOLO yükleme gibi
        # ağır değil). SCR_USER4'e dokunmadan da panelde hangi indeksin
        # hangi dosyaya karşılık geldiğini görebilsin diye HER tikte taranır.
        available = self._scan_model_dir()
        self._model_available = [(i + 1, name) for i, (_, name) in enumerate(available)]
        stats = {
            'capture_fps': round(self._hz(self._t_capture), 1),
            'detect_fps': round(self._hz(self._t_detect), 1),
            'output_fps': round(self._hz(self._t_output), 1),
            'detect_ms': round(self._detect_ms, 1),
            'boxes': len(self._boxes),
            'model': self._model_display_name if self._model is not None else 'YOK',
            'model_index': self._model_select_index,
            'model_available': [{'index': i, 'name': n} for i, n in self._model_available],
            'recording': os.path.basename(self._writer_path) if self._writer_path else '',
            'segments': len(self._segment_files),
            'camera_error': self._camera_error,
            'infer_width': self._infer_width,
        }
        msg = String()
        msg.data = json.dumps(stats)
        self.pub_stats.publish(msg)

    # ═════════════════════ Canlı Parametre Değişikliği ═════════════════════

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult

        touched_clahe = False
        for prm in params:
            n, v = prm.name, prm.value

            if n in V4L2_CONTROLS:
                if self._cap is not None and float(v) != UNSET:
                    self._cap.set(V4L2_CONTROLS[n], float(v))
                    self.get_logger().info(f'Kamera {n} = {v}')
            elif n in ('clahe_clip', 'clahe_grid'):
                touched_clahe = True
            elif n == 'confidence_threshold':
                self._conf = float(v)
            elif n == 'infer_width':
                self._infer_width = int(v)
                self.get_logger().info(f'Çıkarım genişliği = {v}')
            elif n == 'record_enabled':
                self._record_enabled = bool(v)
                if not v:
                    self._close_segment()
            elif n == 'record_segment_sec':
                self._segment_sec = max(5.0, float(v))
            elif n == 'record_fps':
                self._record_fps = max(1.0, float(v))
            elif n == 'record_width':
                self._record_width = int(v)
                self._close_segment()   # yeni boyutla yeni segment açılsın
            elif n == 'stream_max_width':
                self._stream_max_width = int(v)
            elif n == 'stream_jpeg_quality':
                self._stream_quality = int(v)
            elif n == 'stream_hz':
                self._stream_hz = max(1.0, float(v))

        if touched_clahe:
            # Parametre henüz yazılmadığı için yeni değeri elle ver.
            clip = float(self.get_parameter('clahe_clip').value)
            grid = max(1, int(self.get_parameter('clahe_grid').value))
            for prm in params:
                if prm.name == 'clahe_clip':
                    clip = float(prm.value)
                elif prm.name == 'clahe_grid':
                    grid = max(1, int(prm.value))
            self._clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(grid, grid))

        return SetParametersResult(successful=True)

    # ═════════════════════ Temizlik ═════════════════════

    def destroy_node(self):
        self._running = False
        for t in (getattr(self, '_capture_thread', None),
                  getattr(self, '_detect_thread', None),
                  getattr(self, '_output_thread', None)):
            if t is not None:
                t.join(timeout=2.0)
        self._close_segment()
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        super().destroy_node()


def main(args=None):
    # mp4 kaydının bozulmaması için SIGTERM'de de temiz kapanış.
    import signal

    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _on_sigterm)

    rclpy.init(args=args)
    node = YoloVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGINT'te rclpy kendi handler'ıyla context'i zaten kapatmış olabilir;
        # ikinci kez çağırmak RCLError fırlatır.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
