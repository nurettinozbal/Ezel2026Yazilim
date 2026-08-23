"""
scan_filter_node
────────────────
Ham LiDAR taramasını (sllidar_ros2 → /scan_raw) araç gövde çerçevesine
hizalar, kör sektörleri maskeler ve /scan olarak yayınlar.

Neden gerekli:
  * LiDAR gövdeye asla tam pruva hizasında monte edilemez. Otonomi yığını
    "+x ileri" varsayar; birkaç derecelik montaj hatası kaçınma manevrasını
    yanlış tarafa çevirir.
  * Tarayıcı 360° döner ve kendi teknesini görür (direk, anten, kamera
    kulesi, GPS mastı). Bu sabit dönüşler collision_avoidance için 0.3 m'de
    duran kalıcı bir engeldir — araç hiç hareket etmez.
  * Su yüzeyinden gelen yansımalar range_min civarında sahte nokta üretir.

Bu node yalnızca LaserScan'i yeniden düzenler; aşağı akıştaki hiçbir node
değişmez. collision_avoidance_node ve lidar_logger_node yine /scan dinler.

Yayın:
  /scan : sensor_msgs/LaserScan  (frame_id = base_link, +x ileri, +y sol)
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanFilterNode(Node):

    def __init__(self):
        super().__init__('scan_filter_node')

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('output_frame', 'base_link')

        # LiDAR'ın 0° işareti pruvaya göre kaç derece dönük (CCW pozitif).
        # Ölçmek için: tekneyi boşlukta bırak, pruvanın tam önüne 2 m'ye bir
        # duba koy, `idaws lidar` çıktısındaki tepe açısını oku, işaretini
        # ters çevirip buraya yaz.
        self.declare_parameter('yaw_offset_deg', 0.0)

        # Tarayıcı ters monte edildiyse (kapak altına baş aşağı) açı yönü
        # tersine döner; bu, sllidar'ın `inverted` parametresinden bağımsızdır.
        self.declare_parameter('mirror', False)

        # Kör sektörler: gövdenin kendisini gördüğü açı aralıkları.
        # "başlangıç:bitiş" derece çiftleri, araç çerçevesinde, CCW pozitif.
        # Örn. kıç taraftaki anten direği için ['170:190'].
        self.declare_parameter('blind_sectors_deg', [''])

        # Menzil kırpma. range_min altındaki dönüşler gövde/su yansımasıdır.
        self.declare_parameter('range_min', 0.25)
        self.declare_parameter('range_max', 12.0)

        # Bu süre boyunca tarama gelmezse uyar (USB düştü, motor durdu).
        self.declare_parameter('timeout_sec', 2.0)

        self._in_topic = self.get_parameter('input_topic').value
        self._out_topic = self.get_parameter('output_topic').value
        self._out_frame = self.get_parameter('output_frame').value
        self._yaw = math.radians(float(self.get_parameter('yaw_offset_deg').value))
        self._mirror = bool(self.get_parameter('mirror').value)
        self._range_min = float(self.get_parameter('range_min').value)
        self._range_max = float(self.get_parameter('range_max').value)
        self._timeout = float(self.get_parameter('timeout_sec').value)

        self._blind = self._parse_sectors(self.get_parameter('blind_sectors_deg').value)

        # Açı ızgarası tarama başına sabit; her mesajda yeniden hesaplamamak
        # için önbelleğe alınır (Jetson'da 10 Hz × 1440 ışın hatırı sayılır).
        self._cached_key = None
        self._cached_angles = None
        self._cached_blind_mask = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._pub = self.create_publisher(LaserScan, self._out_topic, sensor_qos)
        self.create_subscription(LaserScan, self._in_topic, self._cb_scan, sensor_qos)

        self._last_scan_t = None
        self._warned = False
        self.create_timer(1.0, self._watchdog)

        self.get_logger().info(
            f'scan_filter_node başlatıldı — {self._in_topic} → {self._out_topic} '
            f'(yaw {math.degrees(self._yaw):.1f}°, ayna={self._mirror}, '
            f'kör sektör={len(self._blind)}, menzil {self._range_min}–{self._range_max} m)'
        )

    # ───────────────────── Parametre yardımcıları ─────────────────────

    def _parse_sectors(self, raw):
        """['170:190', '-10:10'] → [(rad, rad), ...]"""
        sectors = []
        for item in raw or []:
            item = str(item).strip()
            if not item:
                continue
            try:
                lo, hi = item.split(':')
                sectors.append((math.radians(float(lo)), math.radians(float(hi))))
            except ValueError:
                self.get_logger().warn(
                    f"Kör sektör ayrıştırılamadı: '{item}' — 'baslangic:bitis' bekleniyor"
                )
        return sectors

    # ───────────────────── Tarama işleme ─────────────────────

    def _cb_scan(self, msg: LaserScan):
        self._last_scan_t = self.get_clock().now()
        self._warned = False

        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return

        ranges = np.asarray(msg.ranges, dtype=np.float32)
        inten = np.asarray(msg.intensities, dtype=np.float32) if msg.intensities else None

        perm = self._permutation(n, msg.angle_min, msg.angle_increment)
        ranges = ranges[perm]
        if inten is not None and inten.size == n:
            inten = inten[perm]
        else:
            inten = None

        angles, blind_mask = self._angle_grid(n, msg.angle_min, msg.angle_increment)

        # Geçersiz dönüşleri inf yap. Hem collision_avoidance hem lidar_logger
        # `range_min < r < range_max` süzgecinden geçiriyor, dolayısıyla inf
        # "burada engel yok" değil "bu ışın okunamadı" anlamına geliyor.
        lo = max(self._range_min, float(msg.range_min))
        hi = min(self._range_max, float(msg.range_max))
        bad = ~np.isfinite(ranges) | (ranges < lo) | (ranges > hi)
        if blind_mask is not None:
            bad |= blind_mask
        ranges = np.where(bad, np.inf, ranges).astype(np.float32)

        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._out_frame
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = lo
        out.range_max = hi
        out.ranges = ranges.tolist()
        out.intensities = inten.tolist() if inten is not None else []
        self._pub.publish(out)

    def _permutation(self, n, angle_min, inc):
        """
        Aynalama + yaw kaydırmasını tek bir indeks dizisine indirger.

        Işın i, LiDAR çerçevesinde a_i = angle_min + i·inc açısına bakar.
        Çıkışta aynı ızgarayı koruyup değerleri kaydırıyoruz, böylece
        angle_min / angle_increment değişmiyor ve tüketiciler açıyı her
        zamanki gibi hesaplıyor.

        İstenen dönüşüm — önce aynala, sonra döndür:

            a_çıkış = −a_giriş + ψ        (ayna açıkken)
            a_çıkış =  a_giriş + ψ        (ayna kapalıyken)

        İndislere çevirince, k = round(ψ/inc) ve m = round(−2·angle_min/inc):

            ayna açık :  out[j] = in[m − j + k]
            ayna kapalı: out[j] = in[j − k]

        İKİ ADIMI AYRI AYRI UYGULAMA. `idx = (m − idx) % n` sonrası
        `idx = (idx − k) % n` yazmak m − j − k verir: aynalama açı eksenini
        ters çevirdiği için ikinci kaydırma da ters yöne gider ve tekne
        yaw_offset'in iki katı kadar yanlış tarafa kaçar. Bunun sahadaki
        belirtisi "kaçınma çalışmıyor" değil, engelin yanlış tarafından
        dönmektir — kağıt üstünde fark edilmesi zor, suda pahalı.

        Kaydırma en yakın ışına yuvarlanır; A1'de ışın adımı ~0.9° olduğu için
        azami hizalama hatası ~0.45°.
        """
        j = np.arange(n)
        k = int(round(self._yaw / inc)) if self._yaw != 0.0 else 0
        if self._mirror:
            m = int(round(-2.0 * angle_min / inc))
            return (m - j + k) % n
        return (j - k) % n

    def _angle_grid(self, n, angle_min, inc):
        key = (n, round(angle_min, 6), round(inc, 9))
        if key != self._cached_key:
            angles = angle_min + np.arange(n, dtype=np.float64) * inc
            mask = None
            if self._blind:
                # Açıları (−π, π] aralığına indirge; kör sektörler bu ölçekte
                # tanımlı. Aralığı sarmalayanlar (ör. 170:−170) da çalışır.
                wrapped = (angles + math.pi) % (2 * math.pi) - math.pi
                mask = np.zeros(n, dtype=bool)
                for lo, hi in self._blind:
                    lo_w = (lo + math.pi) % (2 * math.pi) - math.pi
                    hi_w = (hi + math.pi) % (2 * math.pi) - math.pi
                    if lo_w <= hi_w:
                        mask |= (wrapped >= lo_w) & (wrapped <= hi_w)
                    else:
                        mask |= (wrapped >= lo_w) | (wrapped <= hi_w)
            self._cached_key = key
            self._cached_angles = angles
            self._cached_blind_mask = mask
        return self._cached_angles, self._cached_blind_mask

    # ───────────────────── Sağlık denetimi ─────────────────────

    def _watchdog(self):
        if self._last_scan_t is None:
            if not self._warned:
                self.get_logger().warn(
                    f'{self._in_topic} üzerinden henüz tarama gelmedi — '
                    'sllidar_node çalışıyor mu, /dev/idaws_lidar bağlı mı?'
                )
                self._warned = True
            return

        dt = (self.get_clock().now() - self._last_scan_t).nanoseconds / 1e9
        if dt > self._timeout and not self._warned:
            self.get_logger().error(
                f'LiDAR {dt:.1f} saniyedir sessiz — otonomi engel göremiyor, '
                'aracı MANUEL moda al.'
            )
            self._warned = True


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
