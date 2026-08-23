"""
mission_manager_node
────────────────────
Görev modu seçicisi. TEK giriş yolu otopilot parametresi SCR_USER1'dir —
sahada Jetson'da terminal ya da ağ erişimi yok, yer istasyonu (Mission Planner
/ QGC) yalnızca otopilot parametresi yazabiliyor. Eskiden burada bir UART JSON
dinleyicisi, /tmp dosya fallback'i ve web arayüzünün mission/start-stop
topic'leri vardı; hiçbiri sahada kullanılmıyordu ve mission/parkur_complete'i
zaten hiçbir node yayınlamıyordu (faz makinesi PARKUR_1'de sonsuza kadar
takılı kalıyordu). Tek gerçek yol olan SCR_USER1 bırakıldı.

SCR_USER1 — 5'er aralıklı. Gelen değer en yakın 5'in katına yuvarlanır: float
parametre 10.0 yerine 9.9999 olarak geri okunduğunda da doğru moda düşsün diye
değerler arasında pay bırakılmıştır.

    0  = IDLE       otonomi hiçbir motor komutu üretmez, araç kumandada kalır
    5  = DRIVETEST  sürüş + LiDAR kaçış testi: collision_avoidance_node MANUAL'de
                    sürekli RC override ile ileri sürer, engel görünce kaçar
    10 = WAYPOINT   otopilot AUTO'da yer istasyonundan yüklenmiş waypoint
                    görevini sürer; LiDAR engel görünce geçici MANUAL'e geçilip
                    kaçılır, temizlenince AUTO'ya dönülür ve görev kaldığı
                    yerden devam eder
    15 = KAMIKAZE   kamera (YOLO) ile SCR_USER2'deki renkteki dubayı bulup
                    üzerine sürer. Bu fazı kamikaze_node yürütür; LiDAR kaçışı
                    KASITLI olarak devre dışıdır (görev zaten çarpmak).
    20 = WAYPOINT_KAMIKAZE  WAYPOINT ile BİREBİR aynı davranır (görevi bu node
                    da bu şekilde işler); TEK fark, pymavlink_controller_node
                    otopilottan SON waypoint'e ulaşıldığını (MISSION_ITEM_
                    REACHED / MISSION_CURRENT) görünce SCR_USER1'i kendisi
                    15'e (KAMIKAZE) yazar — operatör elle müdahale etmez.

SCR_USER2 = KAMIKAZE hedef renk kodu → mission/target_color
SCR_USER3 = LiDAR kaçış manevrası aç/kapa → collision_avoidance_node okur

Faz değişimleri ayrıca MAVLink STATUSTEXT olarak yer istasyonuna bildirilir
(telemetry/statustext → pymavlink_controller_node). Terminal olmadığı için
operatörün SCR_USER1'in gerçekten uygulandığını görebildiği tek yol budur.
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


# SCR_USER1 değeri → faz adı. Liste indeksi değeri 5'e bölünce elde edilir.
MODE_STEP = 5.0
# WAYPOINT_KAMIKAZE (20): WAYPOINT ile birebir aynı davranır (AUTO görev +
# LiDAR kaçış bekçisi) — TEK fark, pymavlink_controller_node otopilottan
# gelen MISSION_ITEM_REACHED/MISSION_CURRENT mesajlarıyla SON waypoint'e
# ulaşıldığını görünce SCR_USER1'i kendisi 15'e (KAMIKAZE) yazar; operatör
# elle müdahale etmez. Faz makinesi bunu manuel SCR_USER1=15 yazımından
# AYIRT EDEMEZ/ETMEZ — kamikaze_node'da hiçbir değişiklik gerekmiyor.
MODE_NAMES = ['IDLE', 'DRIVETEST', 'WAYPOINT', 'KAMIKAZE', 'WAYPOINT_KAMIKAZE']

# SCR_USER2 renk kodu → ad. Tamsayı kod (0,1,2,...) — renk için doğal kodlama.
DEFAULT_COLOR_NAMES = ['NONE', 'RED', 'GREEN', 'BLACK', 'YELLOW', 'ORANGE']

# Açılış kilidi hatırlatmasının yer istasyonuna tekrar yazılma aralığı.
LOCKED_WARN_PERIOD_SEC = 5.0

# Durum topic'leri OLAY değil GÜNCEL DURUM taşır: sonradan başlayan ya da
# çöküp yeniden başlayan aboneler (collision_avoidance_node) mevcut fazı
# kaçırmamalı. Latched (TRANSIENT_LOCAL) olmazsa yeniden başlayan node bir
# sonraki SCR_USER1 DEĞİŞİKLİĞİNE kadar fazı hiç öğrenemez.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


class MissionManagerNode(Node):

    def __init__(self):
        super().__init__('mission_manager_node')

        # ---------- Parametreler ----------
        self.declare_parameter('mode_param_name', 'SCR_USER1')
        self.declare_parameter('color_param_name', 'SCR_USER2')
        self.declare_parameter('color_names', DEFAULT_COLOR_NAMES)
        self.declare_parameter('phase_publish_hz', 1.0)
        # Açılış kilidi: stack başladığında SCR_USER1 bir önceki testten kalma
        # değerdeyse (ör. 5=DRIVETEST) araç ARM edilir edilmez kendiliğinden
        # sürmeye başlardı. Kilit açıkken görev, parametre önce 0'a (IDLE)
        # çekilmeden seçilemez — operatörden açık bir onay istenir.
        self.declare_parameter('require_idle_at_start', True)

        self._mode_param = self.get_parameter('mode_param_name').value
        self._color_param = self.get_parameter('color_param_name').value
        self._color_names = list(self.get_parameter('color_names').value)
        phase_hz = max(0.1, float(self.get_parameter('phase_publish_hz').value))

        # ---------- Durum ----------
        self._phase = 'IDLE'
        self._last_mode_value = None
        self._last_color_value = None
        # False iken görev seçimi kilitli; SCR_USER1=0 görülünce açılır.
        self._selector_unlocked = not bool(self.get_parameter('require_idle_at_start').value)
        self._locked_warn_t = -LOCKED_WARN_PERIOD_SEC  # ilk uyarı hemen çıksın

        # ---------- Publisher'lar ----------
        self.pub_phase = self.create_publisher(String, 'mission/phase', LATCHED_QOS)
        self.pub_target_color = self.create_publisher(String, 'mission/target_color', LATCHED_QOS)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd/velocity', 10)
        # Yer istasyonuna serbest metin bildirim (pymavlink_controller_node bunu
        # MAVLink STATUSTEXT'e çevirir). Terminal yokken tek geri bildirim yolu.
        self.pub_statustext = self.create_publisher(String, 'telemetry/statustext', 10)

        # ---------- Subscriber ----------
        # pymavlink_controller_node değer DEĞİŞMEDİYSE tekrar yayınlamıyor; bu
        # node sonradan başladığında mevcut SCR_USER durumunu kaçırmamak için
        # aynı latched QoS ile abone olunur.
        self.create_subscription(
            String, 'telemetry/user_params', self._cb_user_params, LATCHED_QOS)

        # ---------- Timer ----------
        # Faz latched yayınlanıyor ama düşük hızlı bir tekrar, geç gelen ya da
        # latched örneği kaçıran abonelerin de fazı görmesini garantiler.
        self.create_timer(1.0 / phase_hz, self._publish_phase)

        self._publish_phase()
        self.get_logger().info(
            f'mission_manager_node başlatıldı — faz IDLE, mod seçici {self._mode_param} '
            f'({", ".join(f"{i * int(MODE_STEP)}={n}" for i, n in enumerate(MODE_NAMES))}).'
        )

    # ───────────────────── Yayınlar ─────────────────────

    def _publish_phase(self):
        msg = String()
        msg.data = self._phase
        self.pub_phase.publish(msg)

    def _statustext(self, text: str):
        """STATUSTEXT 50 karakterle sınırlı — uzun metin otopilotta kırpılır."""
        msg = String()
        msg.data = text[:50]
        self.pub_statustext.publish(msg)

    def _stop_motors(self):
        self.pub_cmd_vel.publish(Twist())  # tüm alanlar 0.0

    # ───────────────────── Otopilot Parametreleri (SCR_USER) ─────────────────────

    def _cb_user_params(self, msg: String):
        """pymavlink_controller_node'un otopilottan okuduğu kullanıcı
        parametreleri. Sadece DEĞİŞEN değerler işlenir — parametreler 1 Hz
        yoklanıyor, aksi hâlde her yoklamada aynı faz yeniden tetiklenirdi."""
        try:
            params = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'user_params çözülemedi: {msg.data!r}')
            return

        if self._color_param in params:
            self._apply_color(params[self._color_param])
        if self._mode_param in params:
            self._apply_mode(params[self._mode_param])

    def _apply_mode(self, raw):
        try:
            value = float(raw)
        except (TypeError, ValueError):
            self.get_logger().warn(f'{self._mode_param} sayısal değil: {raw!r}')
            return

        idx = int(round(value / MODE_STEP))
        valid = 0 <= idx < len(MODE_NAMES)
        name = MODE_NAMES[idx] if valid else 'IDLE'

        # ---------- Açılış kilidi (bkz. require_idle_at_start) ----------
        # Kilit kontrolü "değer değişti mi" testinden ÖNCE gelir: parametre
        # sabit kaldığı sürece hiç mesaj işlenmezse, operatör tek bir açılış
        # uyarısını kaçırdığında sistemin neden hiçbir şey yapmadığını
        # anlamasının yolu kalmaz — sahada terminal yok. Bu yüzden kilitliyken
        # hatırlatma periyodik olarak TEKRARLANIR.
        if not self._selector_unlocked:
            if valid and name == 'IDLE':
                self._selector_unlocked = True
                self._last_mode_value = value
                self.get_logger().info(
                    f'{self._mode_param}=0 görüldü — görev seçimi açıldı.')
                self._statustext('IDAWS READY')
                return
            now = time.monotonic()
            if now - self._locked_warn_t >= LOCKED_WARN_PERIOD_SEC:
                self._locked_warn_t = now
                self.get_logger().warn(
                    f'{self._mode_param}={value} ({name}) yok sayılıyor: stack '
                    f'başlarken parametre zaten görev seçiyordu, araç ARM edilir '
                    f'edilmez kendiliğinden hareket ederdi. Önce {self._mode_param}=0 '
                    f'yazın, sonra istediğiniz modu seçin.'
                )
                self._statustext(f'IDAWS LOCKED-SET {self._mode_param}=0')
            return

        if value == self._last_mode_value:
            return
        self._last_mode_value = value

        if not valid:
            self.get_logger().warn(
                f'{self._mode_param}={value} geçerli bir mod değil '
                f'(0–{int((len(MODE_NAMES) - 1) * MODE_STEP)} arası, {int(MODE_STEP)}\'er) '
                f'— IDLE\'a düşülüyor.'
            )

        self._set_phase(name, value)

    def _apply_color(self, raw):
        try:
            value = float(raw)
        except (TypeError, ValueError):
            return
        if value == self._last_color_value:
            return
        self._last_color_value = value

        code = int(round(value))
        name = self._color_names[code] if 0 <= code < len(self._color_names) else 'UNKNOWN'
        out = String()
        out.data = json.dumps({'code': code, 'name': name})
        self.pub_target_color.publish(out)
        self.get_logger().info(f'{self._color_param}={value} → hedef renk {name}.')

    # ───────────────────── Faz Geçişi ─────────────────────

    def _set_phase(self, phase: str, param_value: float):
        if phase == self._phase:
            return

        previous = self._phase
        self._phase = phase

        # Modlar arasında motorlar her zaman sıfırlanır: bir sonraki modun
        # kendi komutu gelene kadar araç son komutta (ör. tam gaz) takılı
        # kalmasın. collision_avoidance_node da faz değişiminde sıfır basar —
        # ikisi de sıfır bastığı için çakışma zararsız, bu ilk fren.
        self._stop_motors()
        self._publish_phase()

        self.get_logger().info(
            f'{self._mode_param}={param_value} → faz {previous} → {phase}.')
        self._statustext(f'IDAWS {phase}')

        if phase == 'KAMIKAZE':
            self.get_logger().warn(
                'KAMIKAZE — kamikaze_node devralıyor, LiDAR kaçışı bu fazda '
                'devre dışı. Hedef renk SCR_USER2 ile seçilir.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
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
