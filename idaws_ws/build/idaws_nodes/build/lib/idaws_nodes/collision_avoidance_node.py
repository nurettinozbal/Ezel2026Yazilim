"""
collision_avoidance_node
────────────────────────
LiDAR (LaserScan) tabanlı reaktif kaçış — bölge tabanlı (sol/orta/sağ) karar
ağacı, kullanıcının donanımda doğrulanmış referans script'iyle aynı: engel
merkezdeyse boş tarafa dön, tek taraftaysa ters tarafa dön, her yön kapalıysa
geri git. YOLO (BuoyArray) tespitleri aynı bölgelere füzyonlanır.

Bölgeleme iki modda çalışır (bkz. corridor_mode_enabled):
  Açı tabanlı (eski)  — sektörler açıya göre (center_half_deg/side_max_deg),
                        menzil sınırsız. Dar bir parkurda (ör. 8 m'lik sınır
                        dubaları arası) engel merkezdeyken "en boş taraf"
                        parkur DIŞINDAKİ açık alan olabilir — araç sınır
                        dubalarının ötesine çıkabilir.
  Koridor (varsayılan) — LiDAR/YOLO noktaları araç-eksenli Kartezyene (x=ileri,
                        y=yanal) çevrilir; |y| > corridor_half_width olan
                        noktalar YOK SAYILIR (parkur sınırı dışı hiç görülmez),
                        kalanlar merkez/sol/sağ şeridine (corridor_center_
                        half_width) ayrılır, mesafe olarak x (ileri derinlik)
                        kullanılır. Ortadaki 3. duba merkez şeritte derinlik
                        düşüşü olarak görülür; araç yapısal olarak koridor
                        dışına çıkamadığı için dubaların arasından geçmeye
                        zorlanır.

Görev fazına (mission/phase, SCR_USER1'den gelir) göre İKİ farklı motor komut
yolu vardır — aynı sensör verisi, farklı aktarım:

  drive_phases      (DRIVETEST) — araç MANUAL'de, node SÜREKLİ RC override ile
                    sürer: yol temizken ileri, engelde kaçış. "Test drive
                    escape" senaryosu budur.
  auto_watch_phases (WAYPOINT)  — otopilot AUTO'da yüklü waypoint görevini
                    sürüyor. RC override AUTO'da yol bulmaya ETKİ ETMEZ, o
                    yüzden burada sürekli komut üretilmez; bir bekçi çalışır:
                    engel yaklaşınca GEÇİCİ MANUAL'e geçip RC override ile
                    kaçar, temizlenince AUTO'ya döner ve görev kaldığı yerden
                    devam eder (bkz. _auto_watchdog_loop).

Diğer fazlarda (IDLE, KAMIKAZE) node hiçbir motor komutu ÜRETMEZ. IDLE'da bu
şart: aksi hâlde araç ARM edilir edilmez, görev başlamamışken ileri gider ve
manuel kumanda otonomi komutlarıyla çakışır. KAMIKAZE fazını ise kamikaze_node
yürütür — LiDAR kaçışı o fazda KASITLI olarak devre dışıdır (görev tam da
hedefe çarpmak), bu yüzden burada sessiz kalınır ve cmd/velocity ona bırakılır.

Kaçışın sahada aç/kapa anahtarı otopilot parametresi SCR_USER3'tür (>=0.5
ETKİN). Yer istasyonu Jetson'a ağdan erişemediği için tek kontrol yolu budur ve
HER fazda geçerlidir — DRIVETEST'te kapalıysa node hiç sürmez (kaçışsız
körlemesine ileri sürmek çarpmak demektir), WAYPOINT'te kapalıysa bekçi
tetiklenmez ve otopilot görevini kesintisiz sürdürür.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from idaws_msgs.msg import BuoyArray


# Otopilot mod isteği (AUTO/MANUAL) FC tarafından SESSİZCE reddedilebilir
# (set_mode_send için COMMAND_ACK gelmeyebilir — bkz. pymavlink_controller_
# node.set_mode). Bu yüzden _set_fc_mode, telemetry/mode ile ONAYLANANA
# kadar bu periyotla TEKRAR dener; hiç onaylanmazsa şu süre sonunda
# STATUSTEXT ile uyarır.
MODE_RETRY_PERIOD_SEC = 1.0
MODE_FAIL_WARN_SEC = 5.0
# _fc_mode == hedef olduğu an SONSUZA DEK doğru sayılırsa, açılışta MAVLink
# bağlantısı tam kurulmadan gelen ilk heartbeat'lerde mode_mapping() henüz
# eksik olabilir; _decode_mode bu durumda YANLIŞ bir mod adına düşerse (ör.
# gerçekte MANUAL iken bir an için 'AUTO' okunursa) _set_fc_mode bunu "zaten
# başarılı" sanıp GERÇEK komutu hiç göndermeden sonsuza dek yanlış modda
# kalabilir — sahada "kod AUTO vermeden hep MANUAL'de kalıyor, SCR_USER1'i
# 0'a alıp tekrar 10 yapınca düzeliyor" şikayetinin sebebi buydu. Bu yüzden
# "başarılı" durumda bile bu periyotla SESSİZCE yeniden onaylanır — tek
# seferlik yanlış okumaya sonsuza dek güvenilmez, birkaç saniye içinde
# kendiliğinden düzelir.
MODE_REASSERT_PERIOD_SEC = 5.0

# mission_manager_node'un SCR_USER1'den ürettiği faz adları.
DEFAULT_DRIVE_PHASES = ['DRIVETEST']
#   WAYPOINT_KAMIKAZE: WAYPOINT ile birebir aynı sürüş/kaçış davranışı —
#   pymavlink_controller_node son waypoint'e ulaşınca kendisi KAMIKAZE'ye
#   geçirene kadar bu node farkını göremez/gözetmez, aynı bekçi çalışır.
DEFAULT_AUTO_WATCH_PHASES = ['WAYPOINT', 'WAYPOINT_KAMIKAZE']

# SCR_USER3 → kaçış modu seçici. Sahada Jetson'a terminal/ağ erişimi
# olmadığı için (yalnızca yer istasyonundan otopilot parametresi yazılabilir)
# bu, hem kaçışı aç/kapa hem İKİ ALGORİTMA arasında seçim yapmanın TEK yolu:
#   0 = OFF       kaçış tamamen devre dışı
#   1 = ANGLE     eski açı tabanlı bölgeleme (menzil sınırsız)
#   2 = CORRIDOR  sanal koridor kısıtlamalı bölgeleme (bkz. _classify_point_xy)
#   3 = FUSION    CORRIDOR + LiDAR/kamera renk füzyonu (bkz. _lookup_fusion_color)
LIDAR_ESCAPE_MODE_STEP = 1.0
LIDAR_ESCAPE_MODES = ['OFF', 'ANGLE', 'CORRIDOR', 'FUSION']

# Faz ve SCR_USER3 durumu latched (TRANSIENT_LOCAL) yayınlanıyor — değer
# değişmediyse tekrar gelmiyor. Bu node çöküp yeniden başlarsa mevcut durumu
# kaçırmamak için abonelikler de aynı QoS ile kurulur.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

# scan_filter_node / sllidar BEST_EFFORT yayınlıyor; burada RELIABLE kullanmak
# QoS uyuşmazlığı yaratıp hiç veri almadan sessizce boşta kalmaya yol açıyordu
# (bu projede tekrarlayan hata sınıfı) — histogram hep sıfır, min_range hep inf.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class CollisionAvoidanceNode(Node):

    def __init__(self):
        super().__init__('collision_avoidance_node')

        # ---------- Parametreler ----------
        self.declare_parameter('safe_distance', 3.0)            # metre — bu mesafenin altı "engel"
        # Geri gitme (çıkmaz sokak) eşiği. safe_distance 3 m'ye çıkarıldığında
        # tek eşik bırakmak, dar bir kanalda üç bölgenin de sürekli "kapalı"
        # görünmesine ve aracın durmadan geri gitmesine yol açıyor. Dönüş 3
        # m'de, geri gitme yalnızca gerçekten yakın engelde tetiklensin diye
        # ayrıldı. safe_distance'a eşitlenirse eski tek eşikli davranış birebir
        # geri gelir.
        self.declare_parameter('reverse_distance', 1.5)
        self.declare_parameter('forward_speed', 0.4)            # yol temizken ileri itki [-1,1]
        self.declare_parameter('reverse_speed', 0.4)            # çıkmaz sokakta geri itki büyüklüğü
        self.declare_parameter('turn_speed', 0.8)               # kaçış dönüş komutu büyüklüğü [-1,1]
        self.declare_parameter('center_half_deg', 15.0)         # merkez bölge: ±bu derece
        self.declare_parameter('side_max_deg', 75.0)            # yan bölgeler: center_half_deg..bu derece
        self.declare_parameter('min_valid_range', 0.1)          # altı gövde/su yansıması
        self.declare_parameter('max_valid_range', 12.0)         # üstü menzil dışı / geçersiz
        self.declare_parameter('camera_fov_deg', 60.0)          # kamera görüş açısı (buoy açı tahmini)
        self.declare_parameter('decision_hz', 20.0)             # karar döngüsü — kaçış tepki süresi
        self.declare_parameter('drive_phases', DEFAULT_DRIVE_PHASES)
        self.declare_parameter('auto_watch_phases', DEFAULT_AUTO_WATCH_PHASES)
        # En yakın engeli yer istasyonu arayüzüne STATUSTEXT olarak bildir.
        self.declare_parameter('obstacle_report_max_range', 10.0)   # metre
        self.declare_parameter('obstacle_report_hz', 2.0)
        self.declare_parameter('obstacle_report_color', 'red')
        # AUTO-mod kaçınma bekçisi zaman histerezisi (bkz. _auto_watchdog_loop).
        self.declare_parameter('auto_avoid_min_manual_sec', 0.5)       # MANUAL'de kalınan asgari süre
        self.declare_parameter('auto_avoid_clear_hold_sec', 0.4)       # AUTO'ya dönmeden önce temiz kalma
        self.declare_parameter('auto_avoid_resume_cooldown_sec', 1.0)  # AUTO'ya dönünce yeni tetik beklemesi
        self.declare_parameter('auto_avoid_max_duration_sec', 20.0)    # aşılırsa sıkışmış say, dur
        # LiDAR kaçış manevrasının sahadaki tek kontrolü: Pixhawk parametresi
        # SCR_USER3 (bkz. LIDAR_ESCAPE_MODES: 0=OFF 1=ANGLE 2=CORRIDOR).
        # Aşağıdaki düğüm parametresi yalnızca otopilottan HENÜZ hiç okuma
        # gelmediği ilk anlar için başlangıç değeridir — SCR_USER3 okununca
        # geçersiz kılınır.
        self.declare_parameter('lidar_escape_enabled_default', True)
        self.declare_parameter('lidar_escape_user_param', 'SCR_USER3')
        # Sanal koridor kısıtlaması: dar (ör. 8 m) bir parkurda, sınır dubalarının
        # DIŞINDAKİ açık alan bölge-min hesabına hiç girmez. Açı tabanlı bölgeleme
        # (aşağıdaki center_half_deg/side_max_deg) menzili sınırlamadığı için engel
        # merkezdeyken "en boş taraf" olarak parkur dışını seçip aracı sınır
        # dubalarının ötesine çıkarabilirdi — bu mod bunu yapısal olarak engeller.
        self.declare_parameter('corridor_mode_enabled', True)
        # Koridu yarı genişliği (metre) — bunun ötesindeki nokta YOK SAYILIR,
        # yani araç iki sınır dubası arasının dışını hiç göremez. 8 m'lik bir
        # parkur için gerçek sınır 4.0 m'dir; güvenlik payıyla varsayılan 3.5.
        self.declare_parameter('corridor_half_width', 3.5)
        # Merkez şerit yarı genişliği (metre) — bunun içindeki nokta merkez,
        # dışı (corridor_half_width'e kadar) sol/sağ şerit sayılır.
        self.declare_parameter('corridor_center_half_width', 1.0)
        # ---------- LiDAR + Kamera Füzyonu (SCR_USER3=3 FUSION) ----------
        # Kamera güvenilmez (parlama/su sıçraması/az ışık) olabileceği için
        # LiDAR HER ZAMAN "gerçeklik" kabul edilir; kamera yalnızca RENK
        # etiketler. Kural: turuncu = SINIR (o açıdaki LiDAR noktası merkez
        # kaçış tetiklemez, ama sol/sağ şeritte hâlâ tam engel gibi sayılır —
        # araç sınırın ÖTESİNE kaçamaz). Sarı VEYA kamera hiç etiket
        # veremiyorsa (bounding box titriyor/kayıp) = varsayılan ENGEL —
        # güvenlik önceliği: sınıflandırılamayan kütle engel sayılır.
        self.declare_parameter('fusion_boundary_color', 'ORANGE')
        # Bir YOLO etiketinin bu renklerden hangisine ait olduğunu anlamak
        # için alt-dize araması (kamikaze_node'daki color_aliases ile aynı
        # mantık). Yalnızca fusion_boundary_color eşleşmesi davranışı
        # değiştirir; diğer renkler (sarı dahil) "engel" varsayılanına düşer.
        self.declare_parameter('fusion_color_aliases_json', json.dumps({
            'ORANGE': ['orange', 'turuncu'],
            'YELLOW': ['yellow', 'sari', 'sarı'],
        }))
        # Kamera açısını LiDAR açısıyla eşleştirme toleransı — kamera açı
        # tahmini (bounding box merkezi → FOV oranı) LiDAR kadar hassas değil.
        self.declare_parameter('fusion_match_angle_deg', 8.0)
        # Histerezis: bir açı son kaç saniyedir turuncu etiketliyse, kamera o
        # frame'de nesneyi kaybetse/karıştırsa bile o süre boyunca hâlâ
        # SINIR sayılmaya devam eder — titremenin merkez kaçışını yanlışlıkla
        # tetiklemesini (ya da tam tersini) önler.
        self.declare_parameter('fusion_hysteresis_hold_sec', 1.0)
        self.declare_parameter('fusion_angle_bin_deg', 3.0)

        self._safe_distance = float(self.get_parameter('safe_distance').value)
        self._reverse_distance = float(self.get_parameter('reverse_distance').value)
        self._forward_speed = float(self.get_parameter('forward_speed').value)
        self._reverse_speed = float(self.get_parameter('reverse_speed').value)
        self._turn_speed = float(self.get_parameter('turn_speed').value)
        self._center_half_rad = math.radians(float(self.get_parameter('center_half_deg').value))
        self._side_max_rad = math.radians(float(self.get_parameter('side_max_deg').value))
        self._min_valid_range = float(self.get_parameter('min_valid_range').value)
        self._max_valid_range = float(self.get_parameter('max_valid_range').value)
        self._cam_fov = math.radians(float(self.get_parameter('camera_fov_deg').value))
        decision_hz = max(1.0, float(self.get_parameter('decision_hz').value))
        self._drive_phases = set(self.get_parameter('drive_phases').value)
        self._auto_watch_phases = set(self.get_parameter('auto_watch_phases').value)
        self._obstacle_report_max_range = float(self.get_parameter('obstacle_report_max_range').value)
        self._obstacle_report_color = self.get_parameter('obstacle_report_color').value
        obstacle_report_hz = max(0.1, float(self.get_parameter('obstacle_report_hz').value))
        self._auto_avoid_min_manual = float(self.get_parameter('auto_avoid_min_manual_sec').value)
        self._auto_avoid_clear_hold = float(self.get_parameter('auto_avoid_clear_hold_sec').value)
        self._auto_avoid_cooldown = float(self.get_parameter('auto_avoid_resume_cooldown_sec').value)
        self._auto_avoid_max_duration = float(self.get_parameter('auto_avoid_max_duration_sec').value)
        self._lidar_escape_enabled = bool(self.get_parameter('lidar_escape_enabled_default').value)
        self._lidar_escape_param = self.get_parameter('lidar_escape_user_param').value
        self._corridor_mode = bool(self.get_parameter('corridor_mode_enabled').value)
        self._corridor_half_width = float(self.get_parameter('corridor_half_width').value)
        self._corridor_center_half_width = float(self.get_parameter('corridor_center_half_width').value)
        self._fusion_mode = False   # yalnızca SCR_USER3=3 (FUSION) iken True
        self._fusion_boundary_color = str(self.get_parameter('fusion_boundary_color').value).upper()
        try:
            self._fusion_color_aliases = {
                k.upper(): [s.lower() for s in v] for k, v in
                json.loads(self.get_parameter('fusion_color_aliases_json').value).items()
            }
        except (json.JSONDecodeError, AttributeError):
            self._fusion_color_aliases = {'ORANGE': ['orange', 'turuncu'], 'YELLOW': ['yellow', 'sari', 'sarı']}
        self._fusion_match_angle_rad = math.radians(float(self.get_parameter('fusion_match_angle_deg').value))
        self._fusion_hysteresis_hold = float(self.get_parameter('fusion_hysteresis_hold_sec').value)
        self._fusion_angle_bin_rad = math.radians(float(self.get_parameter('fusion_angle_bin_deg').value))

        # ---------- Durum ----------
        self._buoy_obstacles = []  # (açı_rad, tahmini_mesafe, label)
        # açı_bin → (renk, son_görülme_t) — bkz. _lookup_fusion_color
        self._fusion_color_cache = {}
        self._phase = 'IDLE'
        self._last_phase = None

        # Yerel LiDAR (/scan) bölge minimumları.
        self._scan_min_left = math.inf
        self._scan_min_center = math.inf
        self._scan_min_right = math.inf
        self._min_range = math.inf   # 360° genel en yakın — yer istasyonu raporu
        self._min_angle = 0.0
        self._fusion_boundary_depth = math.inf   # merkezdeki en yakın SINIR (yalnızca rapor)
        self._fusion_boundary_angle = 0.0

        self._boxed_in_warned = False    # çıkmaz sokak log spamını önler
        self._escape_off_warned = False  # DRIVETEST'te "kaçış kapalı" log spamını önler

        # ---------- AUTO-mod kaçınma bekçisi ----------
        self._fc_mode = 'UNKNOWN'
        self._avoiding = False
        self._avoid_entered_t = 0.0
        self._clear_since_t = None
        self._cooldown_until_t = 0.0
        self._last_mode_cmd = None
        self._mode_cmd_first_t = 0.0
        self._mode_cmd_last_sent_t = 0.0
        self._mode_fail_warned = False
        self._stuck_warned = False

        # ---------- Subscriber'lar ----------
        self.create_subscription(LaserScan, '/scan', self._cb_lidar, SENSOR_QOS)
        self.create_subscription(BuoyArray, 'vision/buoys', self._cb_buoys, 10)
        self.create_subscription(String, 'mission/phase', self._cb_phase, LATCHED_QOS)
        self.create_subscription(String, 'telemetry/mode', self._cb_fc_mode, 10)
        self.create_subscription(
            String, 'telemetry/user_params', self._cb_user_params, LATCHED_QOS)

        # ---------- Publisher'lar ----------
        # TEK motor komutu yolu: RC_CHANNELS_OVERRIDE (pymavlink_controller_node
        # bunu MAVLink'e çevirir). Sürüş fazı VE AUTO kaçış manevrası aynı
        # topic'i kullanır — ikisi asla aynı anda aktif olmaz (drive_phases ile
        # auto_watch_phases ayrık kümelerdir).
        self.pub_cmd = self.create_publisher(Twist, 'cmd/velocity', 10)
        # Mod komutları: faz girişinde temel mod (MANUAL/AUTO), AUTO bekçisinde
        # geçici MANUAL ↔ AUTO. cmd/mode'un TEK sahibi bu node'dur.
        self.pub_mode_cmd = self.create_publisher(String, 'cmd/mode', 10)
        # Yer istasyonuna en yakın engel bildirimi ve durum metni
        # (pymavlink_controller_node bunları MAVLink STATUSTEXT'e çevirir).
        self.pub_obstacle_report = self.create_publisher(String, 'telemetry/obstacle_report', 10)
        self.pub_statustext = self.create_publisher(String, 'telemetry/statustext', 10)

        # ---------- Timer'lar ----------
        self.create_timer(1.0 / decision_hz, self._decision_loop)
        self.create_timer(1.0 / obstacle_report_hz, self._report_obstacle)
        self.create_timer(1.0 / obstacle_report_hz, self._report_fusion)

        if self._corridor_mode:
            zone_desc = (
                f'sanal koridor ±{self._corridor_half_width:.1f} m '
                f'(merkez ±{self._corridor_center_half_width:.1f} m)'
            )
        else:
            zone_desc = (
                f'merkez ±{math.degrees(self._center_half_rad):.0f}°, '
                f'yanlar {math.degrees(self._center_half_rad):.0f}°–{math.degrees(self._side_max_rad):.0f}°'
            )
        self.get_logger().info(
            f'collision_avoidance_node başlatıldı — safe_distance={self._safe_distance:.1f} m '
            f'(geri gitme {self._reverse_distance:.1f} m), karar {decision_hz:.0f} Hz, {zone_desc}. '
            f'Sürüş fazları: {sorted(self._drive_phases)}, AUTO bekçisi: {sorted(self._auto_watch_phases)}.'
        )

    # ───────────────────── Bölge Sınıflandırma ─────────────────────

    @staticmethod
    def _wrap_pi(angle_rad: float) -> float:
        return ((angle_rad + math.pi) % (2 * math.pi)) - math.pi

    def _classify_zone(self, angle_rad: float):
        """0=merkez, 1=sol (+açı), -1=sağ (-açı), None=iki yan bölgenin de
        dışında (arka sektör, hiçbir kararda kullanılmaz)."""
        wrapped = self._wrap_pi(angle_rad)
        a = abs(wrapped)
        if a <= self._center_half_rad:
            return 0
        if a <= self._side_max_rad:
            return 1 if wrapped > 0 else -1
        return None

    def _classify_point_xy(self, x: float, y: float):
        """Sanal koridor: x=ileri (araç ekseni), y=yanal (sol +). |y| >
        corridor_half_width olan nokta parkur sınırının DIŞINDA sayılır ve
        YOK SAYILIR — araç iki sınır dubası arasının ötesini hiç görmez,
        dolayısıyla "en boş taraf" olarak parkur dışına kaçamaz. x<=0 (araç
        gerisi) da yok sayılır, yalnızca ileri koridor değerlendirilir.
        0=merkez, 1=sol, -1=sağ, None=koridor dışı/gerisi."""
        if x <= 0.0 or abs(y) > self._corridor_half_width:
            return None
        if abs(y) <= self._corridor_center_half_width:
            return 0
        return 1 if y > 0 else -1

    # ───────────────────── LiDAR + Kamera Renk Füzyonu ─────────────────────

    def _classify_buoy_color(self, label: str):
        """YOLO etiketini fusion_color_aliases'a göre ORANGE/YELLOW/None'a
        çevirir (alt-dize araması, kamikaze_node ile aynı mantık)."""
        low = (label or '').lower()
        for color, needles in self._fusion_color_aliases.items():
            if any(n in low for n in needles):
                return color
        return None

    def _remember_fusion_color(self, angle_rad: float, color: str, now: float):
        if color is None:
            return
        bin_key = round(angle_rad / self._fusion_angle_bin_rad)
        self._fusion_color_cache[bin_key] = (color, now)

    def _lookup_fusion_color(self, angle_rad: float, now: float):
        """Bu açıya en yakın, hâlâ taze (fusion_hysteresis_hold_sec içinde)
        renk etiketini döner — kamera o anki frame'de nesneyi kaybetse/
        karıştırsa bile histerezis süresi boyunca son bilinen renk geçerli
        sayılır. Eşleşme yoksa None (varsayılan: sınıflandırılamayan engel)."""
        if not self._fusion_color_cache:
            return None
        tol_bins = max(1, round(self._fusion_match_angle_rad / self._fusion_angle_bin_rad))
        center_bin = round(angle_rad / self._fusion_angle_bin_rad)
        best = None
        for offset in range(-tol_bins, tol_bins + 1):
            entry = self._fusion_color_cache.get(center_bin + offset)
            if entry is None:
                continue
            color, seen_t = entry
            if now - seen_t > self._fusion_hysteresis_hold:
                continue
            if best is None or abs(offset) < best[0]:
                best = (abs(offset), color)
        return best[1] if best else None

    def _local_zone_mins(self):
        """Yerel LiDAR (/scan) + YOLO füzyonu — hem sürüş fazı hem AUTO bekçisi
        BUNU kullanır; ikisi de aynı sensör verisine bakar, tek fark motor
        komutunun nasıl gönderildiği."""
        ml, mc, mr = self._scan_min_left, self._scan_min_center, self._scan_min_right
        now = time.monotonic()
        for angle, dist, _label in self._buoy_obstacles:
            if not (self._min_valid_range < dist < self._max_valid_range):
                continue
            if self._corridor_mode:
                x = dist * math.cos(angle)
                y = dist * math.sin(angle)
                zone = self._classify_point_xy(x, y)
                depth = x
            else:
                zone = self._classify_zone(angle)
                depth = dist
            if (self._fusion_mode and zone == 0
                    and self._lookup_fusion_color(angle, now) == self._fusion_boundary_color):
                # SINIR (ör. turuncu) — merkez kaçış tetiklemez, ama sol/sağ
                # şeritteyse normal engel gibi sayılmaya devam eder (aşağıda).
                continue
            if zone == 0:
                mc = min(mc, depth)
            elif zone == 1:
                ml = min(ml, depth)
            elif zone == -1:
                mr = min(mr, depth)
        return ml, mc, mr

    # ───────────────────── LiDAR Callback ─────────────────────

    def _cb_lidar(self, msg: LaserScan):
        """LaserScan → sol/orta/sağ bölge minimumları + 360° genel en yakın
        (yer istasyonu raporu için, bölge sınırlamasından bağımsız). FUSION
        modunda ayrıca merkezdeki en yakın SINIR (turuncu) noktası da ayrı
        izlenir — kaçış kararını etkilemez, yalnızca _report_fusion için."""
        min_left = min_center = min_right = math.inf
        min_range = math.inf
        min_angle = 0.0
        boundary_depth = math.inf
        boundary_angle = 0.0

        now = time.monotonic()
        angle = msg.angle_min
        for r in msg.ranges:
            if (msg.range_min < r < msg.range_max
                    and self._min_valid_range < r < self._max_valid_range):
                if r < min_range:
                    min_range = r
                    min_angle = angle
                if self._corridor_mode:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    zone = self._classify_point_xy(x, y)
                    depth = x
                else:
                    zone = self._classify_zone(angle)
                    depth = r
                if (self._fusion_mode and zone == 0
                        and self._lookup_fusion_color(angle, now) == self._fusion_boundary_color):
                    # SINIR — bu nokta merkez engel sayılmaz (bkz. _local_zone_mins),
                    # yalnızca yer istasyonu raporu için ayrı izlenir.
                    if depth < boundary_depth:
                        boundary_depth = depth
                        boundary_angle = angle
                elif zone == 0 and depth < min_center:
                    min_center = depth
                elif zone == 1 and depth < min_left:
                    min_left = depth
                elif zone == -1 and depth < min_right:
                    min_right = depth
            angle += msg.angle_increment

        self._scan_min_left = min_left
        self._scan_min_center = min_center
        self._scan_min_right = min_right
        self._min_range = min_range
        self._min_angle = min_angle
        self._fusion_boundary_depth = boundary_depth
        self._fusion_boundary_angle = boundary_angle

    # ───────────────────── YOLO (BuoyArray) Callback ─────────────────────

    def _cb_buoys(self, msg: BuoyArray):
        """Bounding box → kamera frame koordinatlarından açı tahmini; LiDAR ile
        birleştirmek için açı bilgisi kullanılır. FUSION modunda ayrıca her
        tespitin rengi _fusion_color_cache'e (açı → renk, histerezis için son
        görülme zamanıyla) yazılır — kamera SADECE renk etiketler, mesafe/
        varlık kararını her zaman LiDAR verir (bkz. _lookup_fusion_color)."""
        obstacles = []
        if msg.frame_width == 0:
            self._buoy_obstacles = obstacles
            return

        now = time.monotonic()
        for buoy in msg.buoys:
            norm_x = (buoy.center_x / msg.frame_width) - 0.5   # → [-0.5, 0.5]
            angle = norm_x * self._cam_fov

            # Bounding box büyüklüğünden yaklaşık mesafe tahmini.
            box_height = buoy.y_max - buoy.y_min
            ratio = box_height / msg.frame_height
            estimated_dist = max(0.5, self._safe_distance * (1.0 - ratio))

            obstacles.append((angle, estimated_dist, buoy.label))

            if self._fusion_mode:
                self._remember_fusion_color(angle, self._classify_buoy_color(buoy.label), now)

        self._buoy_obstacles = obstacles

    # ───────────────────── Görev Fazı / Otopilot Modu ─────────────────────

    def _cb_phase(self, msg: String):
        self._phase = msg.data

    def _cb_fc_mode(self, msg: String):
        self._fc_mode = msg.data

    def _cb_user_params(self, msg: String):
        """SCR_USER3 (varsayılan) — sahada Jetson'a terminal/ağ erişimi
        olmadığı için yer istasyonundan yazılabilen bu otopilot parametresi
        hem kaçışı aç/kapa hem ÜÇ ALGORİTMA arasında seçim yapmanın TEK yolu
        (bkz. LIDAR_ESCAPE_MODES): 0=OFF 1=ANGLE 2=CORRIDOR 3=FUSION."""
        try:
            params = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if self._lidar_escape_param not in params:
            return

        raw = float(params[self._lidar_escape_param])
        idx = int(round(raw / LIDAR_ESCAPE_MODE_STEP))
        idx = max(0, min(idx, len(LIDAR_ESCAPE_MODES) - 1))
        mode = LIDAR_ESCAPE_MODES[idx]
        enabled = mode != 'OFF'
        # FUSION, CORRIDOR'un Kartezyen/koridor geometrisini temel alır —
        # üzerine renk füzyonu ekler (bkz. _fusion_mode kullanan yerler).
        corridor = mode in ('CORRIDOR', 'FUSION')
        fusion = (mode == 'FUSION')

        if (enabled == self._lidar_escape_enabled and corridor == self._corridor_mode
                and fusion == self._fusion_mode):
            return
        self._lidar_escape_enabled = enabled
        self._corridor_mode = corridor
        self._fusion_mode = fusion
        if not fusion:
            self._fusion_color_cache.clear()
        self._escape_off_warned = False
        self.get_logger().info(
            f'{self._lidar_escape_param}={raw:g} → LiDAR kaçış modu {mode}.'
        )
        # Terminal olmadığı için anahtarın gerçekten uygulandığını operatörün
        # görebildiği tek yol yer istasyonundaki STATUSTEXT'tir.
        self._statustext(f'IDAWS ESCAPE {mode}')

        if not enabled and self._avoiding:
            self.get_logger().warn(
                "Kaçış manevrası sürerken devre dışı bırakıldı — güvenlik için "
                "hemen AUTO'ya dönülüyor."
            )
            self._abort_avoid_to_auto()

    # ───────────────────── Yer İstasyonu Bildirimleri ─────────────────────

    def _statustext(self, text: str):
        """STATUSTEXT 50 karakterle sınırlı — uzun metin otopilotta kırpılır."""
        msg = String()
        msg.data = text[:50]
        self.pub_statustext.publish(msg)

    def _report_obstacle(self):
        """En yakın LiDAR engelini "OBS,1,mesafe,açı,renk" formatında yayınlar.
        pymavlink_controller_node bunu STATUSTEXT olarak Pixhawk'a, oradan da
        yer istasyonu arayüzüne iletir. Faz/otonomi durumundan bağımsızdır —
        araç hangi modda olursa olsun operatör engeli görmeli."""
        if not math.isfinite(self._min_range) or self._min_range >= self._obstacle_report_max_range:
            return

        angle_deg = math.degrees(self._min_angle) % 360.0
        report = String()
        report.data = (
            f'OBS,1,{self._min_range:.2f},{angle_deg:.0f},{self._obstacle_report_color}'
        )
        self.pub_obstacle_report.publish(report)

    def _report_fusion(self):
        """FUSION modunda merkez şeritteki en yakın nesnenin sınıflandırmasını
        aynı kanaldan (STATUSTEXT) yer istasyonuna bildirir — kamera/panel
        görmeden turuncu (SINIR, yok sayıldı) / gerçek engel ayrımı sahada
        doğrulanabilsin. Diğer modlarda (ANGLE/CORRIDOR/OFF) sessiz kalır."""
        if not self._fusion_mode:
            return
        center = self._scan_min_center
        boundary = self._fusion_boundary_depth
        if not math.isfinite(center) and not math.isfinite(boundary):
            text = 'IDAWS FUS C:CLEAR'
        elif boundary < center:
            text = f'IDAWS FUS C:BOUND {boundary:.1f}m'
        else:
            text = f'IDAWS FUS C:OBST {center:.1f}m'
        report = String()
        report.data = text
        self.pub_obstacle_report.publish(report)

    # ───────────────────── Bölge Tabanlı Kaçınma Kararı ─────────────────────

    def _zone_cmd(self, min_left: float, min_center: float, min_right: float) -> Twist:
        """Referans script'le aynı karar ağacı: engel merkezdeyse boş tarafa
        dön, tek taraftaysa ters tarafa dön, her yön yakın mesafede kapalıysa
        (çıkmaz sokak) geri git."""
        twist = Twist()
        min_overall = min(min_left, min_center, min_right)

        if min_overall >= self._safe_distance:
            self._boxed_in_warned = False
            twist.linear.x = self._forward_speed
            twist.angular.z = 0.0
            return twist

        # ÇIKMAZ SOKAK — her yön GERİ GİTME eşiğinin içinde. Ayrı (daha yakın)
        # eşik kullanılır, yoksa 3 m'lik safe_distance dar bir kanalda sürekli
        # geri gitmeye yol açar.
        if (min_center < self._reverse_distance
                and min_left < self._reverse_distance
                and min_right < self._reverse_distance):
            if not self._boxed_in_warned:
                self._boxed_in_warned = True
                self.get_logger().warn('ÇIKMAZ SOKAK! Her yer kapalı -> GERİ GİDİLİYOR!')
                self._statustext('IDAWS BOXED IN - REVERSING')
            twist.linear.x = -self._reverse_speed
            twist.angular.z = -self._turn_speed
            return twist

        self._boxed_in_warned = False
        twist.linear.x = 0.0
        if min_center < self._safe_distance:
            # Merkez kapalı — daha boş olan tarafa dön.
            twist.angular.z = self._turn_speed if min_left > min_right else -self._turn_speed
        elif min_left < self._safe_distance:
            twist.angular.z = -self._turn_speed      # engel solda -> sağa kaç
        elif min_right < self._safe_distance:
            twist.angular.z = self._turn_speed       # engel sağda -> sola kaç
        return twist

    def _compute_cmd(self) -> Twist:
        ml, mc, mr = self._local_zone_mins()
        return self._zone_cmd(ml, mc, mr)

    def _obstacle_close(self) -> bool:
        ml, mc, mr = self._local_zone_mins()
        return min(ml, mc, mr) < self._safe_distance

    # ───────────────────── Otopilot Modu ─────────────────────

    def _set_fc_mode(self, mode: str):
        """Mod isteğini gönderir ve FC bunu telemetry/mode üzerinden ONAYLAYANA
        kadar periyodik olarak TEKRAR dener. set_mode_send için COMMAND_ACK
        gelmeyebilir (bkz. pymavlink_controller_node.set_mode) — otopilot
        ARMING_CHECK/GPS/EKF hazır değilken ya da görev boşken (AUTO için)
        SESSİZCE reddedebilir.

        ESKİDEN _last_mode_cmd == mode olduğu an BİR DAHA HİÇ tekrar
        denenmiyordu: istek reddedilirse araç o moda GERÇEKTEN hiç geçmeden
        "istendi" sayılıp sonsuza dek beklemede kalıyordu. Sahada "WAYPOINT
        fazında bazen AUTO'ya geçiyor bazen geçmiyor" şikayetinin sebebi
        tam olarak buydu — otopilot tek seferlik denemeyi sessizce reddettiği
        an kod bir daha asla denemiyordu."""
        now = time.monotonic()
        achieved = self._fc_mode == mode

        if achieved:
            self._mode_fail_warned = False
            if self._last_mode_cmd == mode and (now - self._mode_cmd_last_sent_t) < MODE_REASSERT_PERIOD_SEC:
                return
            # Zaten doğru moddayız ama uzun zamandır yeniden onaylanmadı —
            # açılıştaki olası yanlış okumaya karşı sessizce tekrar gönder.
        else:
            if self._last_mode_cmd == mode and (now - self._mode_cmd_last_sent_t) < MODE_RETRY_PERIOD_SEC:
                return  # az önce zaten istendi, FC henüz onaylamadı — bekle
            if self._last_mode_cmd != mode:
                self._mode_cmd_first_t = now
                self._mode_fail_warned = False

        self._last_mode_cmd = mode
        self._mode_cmd_last_sent_t = now
        m = String()
        m.data = mode
        self.pub_mode_cmd.publish(m)

        if not achieved:
            self.get_logger().info(
                f'Otopilot modu {mode} olarak isteniyor (FC şu an {self._fc_mode}).')
            if not self._mode_fail_warned and (now - self._mode_cmd_first_t) > MODE_FAIL_WARN_SEC:
                self._mode_fail_warned = True
                self.get_logger().error(
                    f'{mode} modu {MODE_FAIL_WARN_SEC:.0f} sn içinde ONAYLANMADI — '
                    'otopilot reddediyor olabilir (GPS/EKF hazır değil, görev boş, vs).'
                )
                self._statustext(f'IDAWS MODE FAIL {mode}')

    def _abort_avoid_to_auto(self):
        """Kaçış manevrasını hemen kesip AUTO'ya döner (ör. SCR_USER3 ile
        manevra ortasında devre dışı bırakıldığında güvenlik amaçlı)."""
        self._avoiding = False
        self.pub_cmd.publish(Twist())
        self._set_fc_mode('AUTO')
        self._cooldown_until_t = time.monotonic() + self._auto_avoid_cooldown

    # ───────────────────── Faz Geçişi ─────────────────────

    def _on_phase_change(self, phase: str):
        """Faz değişiminde kaçış durumu sıfırlanır ve fazın TEMEL otopilot modu
        istenir. Sıfırlama şart: WAYPOINT'te kaçış manevrasının ortasında faz
        değişirse `_avoiding` True kalır ve bir sonraki fazda bekçi yanlış
        durumdan devam eder."""
        self._avoiding = False
        self._clear_since_t = None
        self._cooldown_until_t = 0.0
        self._stuck_warned = False
        self._boxed_in_warned = False
        self._escape_off_warned = False

        # Faz değişiminde motorlar her zaman sıfırlanır — araç bir önceki fazın
        # son komutunda (ör. dönüş) takılı kalmasın.
        self.pub_cmd.publish(Twist())

        if phase in self._drive_phases:
            # RC override yalnızca RC girişini okuyan modlarda (MANUAL/ACRO)
            # işlenir; AUTO/GUIDED'de göz ardı edilir.
            self._set_fc_mode('MANUAL')
            self.get_logger().info(
                f'Faz {phase} — MANUAL\'de sürekli RC override sürüşü + LiDAR kaçışı devrede.')
        elif phase in self._auto_watch_phases:
            self._set_fc_mode('AUTO')
            self.get_logger().info(
                f'Faz {phase} — otopilot AUTO görevini sürüyor, LiDAR kaçış bekçisi devrede.')
        else:
            # IDLE / KAMIKAZE: mod komutu GÖNDERİLMEZ. KAMIKAZE'de modu ve
            # motor komutunu kamikaze_node üstlenir. IDLE'da operatörü zorla
            # bir moda almak sürpriz olur; kumandanın/yer istasyonunun seçtiği
            # mod korunur. Motor komutu üretilmediği için pymavlink_controller_
            # node'daki RC failsafe override'ı zaten ~0.5 sn içinde bırakır.
            self._last_mode_cmd = None
            self.get_logger().info(
                f'Faz {phase} — otonomi beklemede, motor komutu üretilmiyor.')

    # ───────────────────── Sürüş Fazı (DRIVETEST) ─────────────────────

    def _drive_loop(self):
        """Araç MANUAL'de; node sürekli RC override ile sürer ve engelde kaçar."""
        self._set_fc_mode('MANUAL')  # onaylanana kadar sessizce tekrar dener
        if not self._lidar_escape_enabled:
            # Kaçış kapalıyken körlemesine ileri sürmek doğrudan çarpmak demek.
            # Bu fazın bütün amacı kaçış testi olduğu için araç durdurulur.
            if not self._escape_off_warned:
                self._escape_off_warned = True
                self.get_logger().warn(
                    f'{self._lidar_escape_param} DEVRE DIŞI — sürüş fazında kaçış '
                    'yapılamayacağı için motorlar durduruldu. Sürmek için '
                    f'{self._lidar_escape_param}=1 yazın.'
                )
                self._statustext('IDAWS DRIVE HELD - ESCAPE OFF')
            self.pub_cmd.publish(Twist())
            return

        self.pub_cmd.publish(self._compute_cmd())

    # ───────────────────── AUTO-mod Kaçınma Bekçisi (WAYPOINT) ─────────────────────

    def _auto_watchdog_loop(self):
        """Otopilot AUTO'da kendi yüklü görevini (waypoint listesi) sürerken bu
        node motor komutu ÜRETMEZ — RC override AUTO'da yol bulmaya etki etmez.
        Engel yaklaşınca GEÇİCİ olarak MANUAL'e geçip RC override ile bölge
        tabanlı kaçış uygular, temizlenince AUTO'ya döner ki görev kaldığı
        yerden devam etsin.

        Zaman tabanlı histerezis (tek mesafe eşiği olduğu için AUTO↔MANUAL
        salınımını bu önler):
          * AUTO'ya dönmeden önce en az `auto_avoid_clear_hold_sec` temiz
            kalınmalı VE en az `auto_avoid_min_manual_sec` MANUAL'de kalınmış
            olmalı.
          * AUTO'ya döndükten sonra `auto_avoid_resume_cooldown_sec` boyunca
            yeni tetiklemeye izin verilmez.
          * `auto_avoid_max_duration_sec` aşılırsa sıkışmış sayılır, motorlar
            durdurulur ve tekrar denenmez — müdahale gerekir.
        """
        now = time.monotonic()

        if not self._avoiding:
            # Onaylanana kadar sessizce tekrar dener — FC bir önceki AUTO
            # isteğini sessizce reddetmiş olsa bile burada sıkışıp kalmaz.
            self._set_fc_mode('AUTO')
            if (self._obstacle_close() and self._fc_mode == 'AUTO'
                    and self._lidar_escape_enabled and now >= self._cooldown_until_t):
                self._avoiding = True
                self._avoid_entered_t = now
                self._clear_since_t = None
                self._stuck_warned = False
                self._set_fc_mode('MANUAL')
                self.get_logger().warn(
                    "AUTO sırasında engel tespit edildi — MANUAL'e geçilip "
                    "RC override kaçış manevrası başlatılıyor."
                )
                self._statustext('IDAWS ESCAPE START')
            return

        self._set_fc_mode('MANUAL')
        elapsed = now - self._avoid_entered_t
        if elapsed > self._auto_avoid_max_duration:
            self.pub_cmd.publish(Twist())
            if not self._stuck_warned:
                self._stuck_warned = True
                self.get_logger().error(
                    f'{elapsed:.0f} sn içinde kaçamadı — sıkışmış olabilir, motorlar '
                    'durduruldu, müdahale gerekiyor (AUTO\'ya otomatik dönülmeyecek).'
                )
                self._statustext('IDAWS STUCK - MOTORS STOPPED')
            return

        if self._obstacle_close():
            self._clear_since_t = None
        else:
            if self._clear_since_t is None:
                self._clear_since_t = now
            if (now - self._clear_since_t >= self._auto_avoid_clear_hold
                    and elapsed >= self._auto_avoid_min_manual):
                self._avoiding = False
                self.pub_cmd.publish(Twist())
                self._set_fc_mode('AUTO')
                self._cooldown_until_t = now + self._auto_avoid_cooldown
                self.get_logger().info(
                    "Engelden temiz — AUTO'ya dönülüyor, görev kaldığı yerden sürüyor."
                )
                self._statustext('IDAWS ESCAPE CLEAR - AUTO')
                return

        self.pub_cmd.publish(self._compute_cmd())

    # ───────────────────── Karar Döngüsü ─────────────────────

    def _decision_loop(self):
        phase = self._phase
        if phase != self._last_phase:
            self._last_phase = phase
            self._on_phase_change(phase)

        if phase in self._auto_watch_phases:
            self._auto_watchdog_loop()
        elif phase in self._drive_phases:
            self._drive_loop()
        # IDLE / KAMIKAZE / bilinmeyen faz: sessiz kalınır. _on_phase_change
        # geçişte bir kez sıfır hız bastı; sonrası manuel kumandaya bırakılır.


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
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
