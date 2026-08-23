"""
kamikaze_node
─────────────
KAMIKAZE fazı (SCR_USER1 = 15): kamerayla (YOLO) hedef renkteki dubayı bulup
üzerine sürer.

NEDEN AYRI NODE
  collision_avoidance_node KAMIKAZE fazında hiçbir şey yayınlamaz — LiDAR
  kaçışı bu fazda KASITLI olarak devre dışıdır, çünkü görev tam da engele
  ÇARPMAK. Dolayısıyla cmd/velocity ve cmd/mode bu fazda boştadır ve bu node
  onları tek başına kullanır; iki node aynı anda motor komutu üretmez.

DURUM MAKİNESİ
  BEKLEME → faz KAMIKAZE değil; hiçbir şey yayınlanmaz.
  ARAMA   → faz aktif ama hedef görünmüyor; yerinde yavaşça dönerek tarar.
  TAKIP   → hedef görünüyor; görüntüdeki yatay hataya göre dümen, sabit itki.
  DALIŞ   → hedef yeterince büyüdü (yakın); tam gaz.
  BİTTİ   → çarpma sayıldı ya da süre doldu; motorlar durur, faz IDLE'a
            alınana kadar bir daha komut üretilmez.

HEDEF SEÇİMİ
  SCR_USER2 rengi mission_manager_node tarafından mission/target_color olarak
  yayınlanır. Model etiketleri serbest metin olduğu için eşleştirme, etiketin
  İÇİNDE renk adının geçmesine bakar (küçük/büyük harf duyarsız, Türkçe
  karşılıkları da dahil — bkz. color_aliases). Birden çok aday varsa EN BÜYÜK
  kutu seçilir: en yakın hedef odur.

MESAFE
  Ayrı bir mesafe sensörü yok; kutunun yüksekliğinin kare yüksekliğine oranı
  yakınlık ölçüsü olarak kullanılır. Oran ram_ratio'yu geçince tam gaz,
  hit_ratio'yu geçince çarpma sayılır.

⚠ DÜMEN İŞARETİ — SAHADA DOĞRULA
  Kodda iki çelişkili kayıt var: collision_avoidance_node standart ROS
  gelenegini kullanıyor (angular.z > 0 = SOLA), pymavlink_controller_node'daki
  yorum ise bu araçta angular.z > 0'ın SAĞA döndürdüğünü söylüyor. Hangisinin
  doğru olduğu araca/RC ters çevirme ayarlarına bağlı, koddan çözülemez.
  Bu yüzden işaret `steer_sign` PARAMETRESİDİR: tekne hedeften UZAKLAŞARAK
  dönüyorsa panelden/parametreden -1.0 yap, kod değiştirmeye gerek yok.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from idaws_msgs.msg import BuoyArray


LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

# Renk adı → model etiketinde aranacak parçalar. Model sınıf adları serbest
# metin ("red_buoy", "kirmizi", "RedBall"...), bu yüzden tam eşitlik yerine
# alt dize araması yapılır.
DEFAULT_COLOR_ALIASES = {
    'RED':    ['red', 'kirmizi', 'kırmızı'],
    'GREEN':  ['green', 'yesil', 'yeşil'],
    'BLACK':  ['black', 'siyah'],
    'YELLOW': ['yellow', 'sari', 'sarı'],
    'ORANGE': ['orange', 'turuncu'],
}

WAITING, SEARCHING, TRACKING, DIVING, DONE = 'BEKLEME', 'ARAMA', 'TAKIP', 'DALIS', 'BITTI'


class KamikazeNode(Node):

    def __init__(self):
        super().__init__('kamikaze_node')

        # ---------- Faz ----------
        self.declare_parameter('active_phase', 'KAMIKAZE')

        # ---------- Hedef seçimi ----------
        # Kilit ALMAK için (ARAMA/BEKLEME'den TAKİP'e geçiş) yüksek bar —
        # yanlış pozitiflerle rastgele bir şeyi kovalamayı önler.
        self.declare_parameter('min_confidence', 0.6)
        # Kilit zaten VARKEN (TAKİP/DALIŞ) tolere edilen çok daha düşük bar.
        # NEDEN AYRI: hedef yaklaştıkça kutu kareyi doldurur, kısmen kadraj
        # dışına taşar, hareket bulanıklığı artar — YOLO güven skoru doğal
        # olarak DÜŞER tam da en çok emin olmamız gereken anda (temasa
        # yakınken). Önceden zaten kilitlenmiş bir hedefi salt bu yüzden
        # kaybedip ARAMA'ya dönmek (ve dalıştan vazgeçmek) istenmiyor.
        self.declare_parameter('min_confidence_locked', 0.2)
        # Renk seçilmemişse (SCR_USER2 = 0 / NONE) saldırılsın mı? Varsayılan
        # HAYIR: rastgele bir dubaya dalmak sahada kabul edilebilir değil.
        # Renksiz test etmek için true yapılabilir.
        self.declare_parameter('require_target_color', True)
        self.declare_parameter('color_aliases_json', json.dumps(DEFAULT_COLOR_ALIASES))

        # ---------- Sürüş ----------
        self.declare_parameter('approach_speed', 0.5)     # takipte ileri itki [-1,1]
        self.declare_parameter('ram_speed', 0.9)          # dalışta tam gaz
        self.declare_parameter('search_turn_speed', 0.35) # ararken yerinde dönüş
        self.declare_parameter('steer_gain', 2.0)         # yatay hata → dümen kazancı
        self.declare_parameter('max_steer', 0.9)
        # +1.0 = ROS geleneği (angular.z > 0 SOLA döndürür).
        # Tekne hedeften UZAKLAŞIYORSA -1.0 yap. Bkz. modül başlığındaki uyarı.
        self.declare_parameter('steer_sign', 1.0)
        # Sert dönüşte gazı kıs, yoksa hedefin yanından geçip gider.
        self.declare_parameter('turn_throttle_cut', 0.5)

        # ---------- Mesafe (kutu oranı) ----------
        self.declare_parameter('ram_ratio', 0.35)   # kutu yüksekliği / kare yüksekliği
        self.declare_parameter('hit_ratio', 0.75)   # bunun üstü "çarptı" sayılır

        # ---------- Zamanlamalar ----------
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('target_lost_sec', 1.0)    # bu süre görünmezse ARAMA'ya dön
        # DALIŞ'a girmiş (ram_ratio aşılmış = zaten çok yakın ve emindik)
        # bir taahhüdün, kamera hedefi TAMAMEN kaybetse bile son bilinen
        # yönde kör biçimde ne kadar sürdürüleceği. target_lost_sec'ten
        # SONRA devreye girer (o süre zaten normal "fresh" sayılıyor).
        self.declare_parameter('dive_commit_hold_sec', 2.5)
        self.declare_parameter('hit_hold_sec', 1.5)       # çarpma sonrası itkiyi sürdürme
        self.declare_parameter('max_duration_sec', 120.0) # bu süre sonunda dur
        self.declare_parameter('command_manual_mode', True)

        p = self.get_parameter
        self._active_phase = p('active_phase').value
        self._min_conf = float(p('min_confidence').value)
        self._min_conf_locked = float(p('min_confidence_locked').value)
        self._require_color = bool(p('require_target_color').value)
        try:
            self._aliases = {k.upper(): [s.lower() for s in v] for k, v in
                             json.loads(p('color_aliases_json').value).items()}
        except (json.JSONDecodeError, AttributeError):
            self.get_logger().warn('color_aliases_json çözülemedi — varsayılan kullanılıyor.')
            self._aliases = DEFAULT_COLOR_ALIASES

        self._approach = float(p('approach_speed').value)
        self._ram_speed = float(p('ram_speed').value)
        self._search_turn = float(p('search_turn_speed').value)
        self._steer_gain = float(p('steer_gain').value)
        self._max_steer = float(p('max_steer').value)
        self._steer_sign = float(p('steer_sign').value)
        self._turn_cut = float(p('turn_throttle_cut').value)
        self._ram_ratio = float(p('ram_ratio').value)
        self._hit_ratio = float(p('hit_ratio').value)
        control_hz = max(1.0, float(p('control_hz').value))
        self._lost_sec = float(p('target_lost_sec').value)
        self._dive_commit_hold = float(p('dive_commit_hold_sec').value)
        self._hit_hold = float(p('hit_hold_sec').value)
        self._max_duration = float(p('max_duration_sec').value)
        self._command_manual = bool(p('command_manual_mode').value)

        # ---------- Durum ----------
        self._phase = 'IDLE'
        self._state = WAITING
        self._target_color = ''
        self._target = None          # (err_x, ratio, label, conf)
        self._last_seen_t = 0.0
        self._entered_t = 0.0
        self._hit_t = 0.0
        self._mode_sent = False
        self._committed = False        # ram_ratio bir kez aşıldı mı (DALIŞ taahhüdü)
        self._committed_steer = 0.0    # taahhüt anındaki son bilinen dümen
        self._dive_lost_since = None   # taahhütteyken hedef ne zaman kayboldu

        # ---------- Publisher ----------
        self.pub_cmd = self.create_publisher(Twist, 'cmd/velocity', 10)
        self.pub_mode = self.create_publisher(String, 'cmd/mode', 10)
        self.pub_status = self.create_publisher(String, 'kamikaze/status', 10)
        self.pub_statustext = self.create_publisher(String, 'telemetry/statustext', 10)

        # ---------- Subscriber ----------
        self.create_subscription(String, 'mission/phase', self._cb_phase, LATCHED_QOS)
        self.create_subscription(String, 'mission/target_color', self._cb_color, LATCHED_QOS)
        self.create_subscription(BuoyArray, 'vision/buoys', self._cb_buoys, 10)

        self.create_timer(1.0 / control_hz, self._control_loop)

        self.get_logger().info(
            f'kamikaze_node başlatıldı — faz "{self._active_phase}", '
            f'yaklaşma {self._approach:.2f}, dalış {self._ram_speed:.2f}, '
            f'dalış oranı {self._ram_ratio:.2f}, çarpma oranı {self._hit_ratio:.2f}, '
            f'dümen işareti {self._steer_sign:+.0f}.'
        )

    # ───────────────────── Girdiler ─────────────────────

    def _cb_phase(self, msg: String):
        if msg.data == self._phase:
            return
        previous, self._phase = self._phase, msg.data
        if self._phase == self._active_phase:
            self._enter()
        elif previous == self._active_phase:
            self._exit()

    def _cb_color(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._target_color = str(data.get('name', '')).upper()
        except (json.JSONDecodeError, AttributeError):
            self._target_color = ''

    def _cb_buoys(self, msg: BuoyArray):
        """En uygun hedefi seçer: renk eşleşen adaylar arasında EN BÜYÜK kutu
        (yani en yakın olan)."""
        if msg.frame_width == 0 or msg.frame_height == 0:
            return

        # Zaten kilitliyken (TAKİP/DALIŞ) çok daha düşük bar tolere edilir —
        # bkz. min_confidence_locked açıklaması. Kilit ALIRKEN (ARAMA/BEKLEME)
        # yüksek bar (min_confidence) geçerli.
        conf_floor = self._min_conf_locked if self._state in (TRACKING, DIVING) else self._min_conf

        best = None
        best_area = 0.0
        for b in msg.buoys:
            if b.confidence < conf_floor:
                continue
            if not self._color_matches(b.label):
                continue
            area = float(max(0, b.x_max - b.x_min) * max(0, b.y_max - b.y_min))
            if area > best_area:
                best_area = area
                err_x = (b.center_x / float(msg.frame_width)) - 0.5
                ratio = (b.y_max - b.y_min) / float(msg.frame_height)
                best = (err_x, ratio, b.label, b.confidence)

        if best is not None:
            self._target = best
            self._last_seen_t = time.monotonic()

    def _color_matches(self, label: str) -> bool:
        """Renk seçilmemişse: require_target_color=False iken her duba geçerli."""
        if not self._target_color or self._target_color == 'NONE':
            return not self._require_color
        needles = self._aliases.get(self._target_color)
        if not needles:
            return False
        low = (label or '').lower()
        return any(n in low for n in needles)

    # ───────────────────── Faz giriş/çıkış ─────────────────────

    def _enter(self):
        self._entered_t = time.monotonic()
        self._target = None
        self._last_seen_t = 0.0
        self._hit_t = 0.0
        self._mode_sent = False
        self._committed = False
        self._committed_steer = 0.0
        self._dive_lost_since = None

        if self._require_color and (not self._target_color or self._target_color == 'NONE'):
            self._state = DONE
            self.get_logger().error(
                'KAMIKAZE istendi ama hedef renk seçilmemiş (SCR_USER2 = 0). '
                'Rastgele bir dubaya dalmamak için görev başlatılmadı — '
                'SCR_USER2 ile rengi seçip fazı yeniden verin.'
            )
            self._statustext('IDAWS KAMIKAZE-NO COLOR SET')
            return

        self._state = SEARCHING
        if self._command_manual:
            # RC override yalnızca RC girişi okuyan modlarda (MANUAL/ACRO) işlenir.
            m = String()
            m.data = 'MANUAL'
            self.pub_mode.publish(m)
            self._mode_sent = True
        self.get_logger().warn(
            f'KAMIKAZE BAŞLADI — hedef renk {self._target_color or "(herhangi)"}. '
            'LiDAR kaçışı bu fazda KASITLI olarak devre dışı.'
        )
        self._statustext(f'IDAWS KAMIKAZE {self._target_color or "ANY"}')

    def _exit(self):
        self._state = WAITING
        self._committed = False
        self._dive_lost_since = None
        self.pub_cmd.publish(Twist())
        self.get_logger().info('KAMIKAZE fazından çıkıldı — motorlar durduruldu.')

    # ───────────────────── Kontrol döngüsü ─────────────────────

    def _control_loop(self):
        if self._phase != self._active_phase:
            if self._state != WAITING:
                self._exit()
            return
        if self._state == DONE:
            self.pub_cmd.publish(Twist())
            self._publish_status()
            return

        now = time.monotonic()

        if now - self._entered_t > self._max_duration:
            self._finish(f'{self._max_duration:.0f} sn doldu — hedefe ulaşılamadı.',
                         'IDAWS KAMIKAZE TIMEOUT')
            return

        # Çarpma zaten ONAYLANDIYSA — temas anında kutunun kadraj dışına
        # taşıp "hedef kayboldu" görünmesi DOĞAL; bu durumda dahi itkiyi
        # hit_hold_sec boyunca sürdür (görünürlükten bağımsız), sonra bitir.
        if self._hit_t != 0.0:
            if now - self._hit_t >= self._hit_hold:
                self._finish('Çarpma tamamlandı.', 'IDAWS KAMIKAZE DONE')
                return
            twist = Twist()
            twist.linear.x = self._ram_speed
            twist.angular.z = 0.0
            self.pub_cmd.publish(twist)
            self._publish_status()
            return

        fresh = self._target is not None and (now - self._last_seen_t) <= self._lost_sec
        twist = Twist()

        if not fresh:
            if self._committed:
                # DALIŞ'a girmişti (ram_ratio aşılmış = zaten çok yakın ve
                # emindik) — kamera hedefi TAMAMEN kaybetse bile son bilinen
                # yönde bir süre kör olarak dalışa devam et, hemen ARAMA'ya
                # dönüp taahhütten vazgeçme.
                if self._dive_lost_since is None:
                    self._dive_lost_since = now
                if now - self._dive_lost_since <= self._dive_commit_hold:
                    self._state = DIVING
                    twist.linear.x = self._ram_speed
                    twist.angular.z = self._committed_steer
                    self.pub_cmd.publish(twist)
                    self._publish_status()
                    return
            # Taahhüt yoktu ya da kör devam süresi de doldu: yerinde tarayarak
            # ara. İleri gitmiyoruz — hiç görmediğimiz bir yöne tam gaz gitmek
            # kör dalış olurdu (bu, yukarıdaki taahhütlü kör devamdan farklı —
            # o zaten bir hedefe kilitliyken son bilinen yöne devam ediyordu).
            self._committed = False
            self._dive_lost_since = None
            self._state = SEARCHING
            twist.linear.x = 0.0
            twist.angular.z = self._search_turn * self._steer_sign
            self.pub_cmd.publish(twist)
            self._publish_status()
            return

        self._dive_lost_since = None
        err_x, ratio, label, conf = self._target

        # Çarpma: kutu kareyi dolduruyor.
        if ratio >= self._hit_ratio:
            self._hit_t = now
            self.get_logger().warn(f'HEDEFE ÇARPILDI ({label}, oran {ratio:.2f}).')
            self._statustext('IDAWS KAMIKAZE HIT')
            twist.linear.x = self._ram_speed
            twist.angular.z = 0.0
            self.pub_cmd.publish(twist)
            self._publish_status()
            return

        # Dümen: görüntüde sağdaysa (err_x > 0) sağa dönmeliyiz. ROS geleneğinde
        # sağa dönüş NEGATİF angular.z'dir, bu yüzden eksi işaret.
        steer = -self._steer_gain * err_x * 2.0
        steer = max(-self._max_steer, min(self._max_steer, steer)) * self._steer_sign

        if ratio >= self._ram_ratio:
            self._state = DIVING
            self._committed = True
            self._committed_steer = steer
            throttle = self._ram_speed
        else:
            self._state = TRACKING
            # Sert dönüşte gazı kıs — yoksa hedefin yanından geçip gider.
            throttle = self._approach * (1.0 - self._turn_cut * min(1.0, abs(steer)))

        twist.linear.x = float(throttle)
        twist.angular.z = float(steer)
        self.pub_cmd.publish(twist)
        self._publish_status()

    def _finish(self, log_msg: str, status: str):
        self._state = DONE
        self.pub_cmd.publish(Twist())
        self.get_logger().warn(f'KAMIKAZE bitti — {log_msg}')
        self._statustext(status)
        self._publish_status()

    # ───────────────────── Bildirim ─────────────────────

    def _statustext(self, text: str):
        msg = String()
        msg.data = text[:50]
        self.pub_statustext.publish(msg)

    def _publish_status(self):
        err, ratio, label, conf = self._target if self._target else (0.0, 0.0, '', 0.0)
        out = String()
        out.data = json.dumps({
            'state': self._state,
            'color': self._target_color,
            'label': label,
            'conf': round(float(conf), 2),
            'err_x': round(float(err), 3),
            'ratio': round(float(ratio), 3),
            'elapsed': round(time.monotonic() - self._entered_t, 1) if self._entered_t else 0.0,
        })
        self.pub_status.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = KamikazeNode()
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
