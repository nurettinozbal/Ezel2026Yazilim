"""
pymavlink_controller_node
─────────────────────────
Doğrudan pymavlink ile Pixhawk'a seri port üzerinden bağlanır.
Thread-1 : Telemetri okuma  → ROS 2 topic'lerine publish
Thread-2 : ROS 2 komutları  → MAVLink mesajlarına çevirip otopilota yazma
"""

import os
# OBSTACLE_DISTANCE (id 330) MAVLink2-only bir mesajdır — bu olmadan pymavlink
# v1 dialect'ini yükler ve mesaj tipi tanımsız kalır (recv_match sessizce hiç
# eşleşmez). Donanımda test edilen referans script'le birebir aynı, pymavlink
# import edilmeden ÖNCE ayarlanmalı.
os.environ["MAVLINK20"] = "1"

import json
import threading
import time
import math
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64, String, Bool
from geometry_msgs.msg import Twist, Vector3Stamped
from sensor_msgs.msg import NavSatFix, Imu, LaserScan

from pymavlink import mavutil

# telemetry/user_params bir OLAY akışı değil, GÜNCEL DURUMDUR (SCR_USER1..3'ün
# son bilinen değerleri) — collision_avoidance_node / mission_manager_node gibi
# sonradan başlayan ya da yeniden başlayan abonelerin "değer değişmediyse hiç
# mesaj gelmez" yüzünden başlangıç durumunu hiç öğrenememesini önlemek için
# TRANSIENT_LOCAL (latched) kullanılıyor — abone olur olmaz son değeri alır.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


# Görev seçimi otopilot üzerindeki kullanıcı parametreleri ile yapılır — sahada
# Jetson'da terminal/ağ yok, yer istasyonu (Mission Planner/QGC) yalnızca
# otopilot parametresi yazabiliyor:
#   SCR_USER1 → görev modu seçici (bkz. mission_manager_node):
#               0=IDLE  5=DRIVETEST  10=WAYPOINT  15=KAMIKAZE
#   SCR_USER2 → KAMIKAZE için hedef renk kodu
#   SCR_USER3 → LiDAR kaçış modu (collision_avoidance_node):
#               0=OFF 1=ANGLE(eski açı tabanlı) 2=CORRIDOR(sanal koridor)
#               3=FUSION(koridor + LiDAR/kamera renk füzyonu)
#   SCR_USER4 → YOLO model seçimi (yolo_vision_node): 0=değişiklik yok,
#               1/2/3... model_dir'in (jetson_params.yaml) alfabetik
#               taranmış 1-tabanlı indeksi
# Yer istasyonundan bu parametreler değiştirildiğinde Jetson değeri okuyup
# ilgili faz geçişini uygular; sonuç STATUSTEXT olarak geri bildirilir.
DEFAULT_USER_PARAMS = ['SCR_USER1', 'SCR_USER2', 'SCR_USER3', 'SCR_USER4']

# COMMAND_ACK yalnizca BU komutlar icin raporlanir — hat paylasimli, digerleri
# baska bir GCS'e ait (bkz. _handle_command_ack).
OUR_COMMANDS = {
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
}

# RC_CHANNELS_OVERRIDE ile motor kontrolü: SET_ATTITUDE_TARGET (GUIDED'e mecbur
# bırakıyordu, ayrıca GUIDED'de sıfır-yaw-hatası motoru sessizce durduran bir
# firmware tuhaflığı vardı) donanımda test edilen referans script lehine terk
# edildi. RC override chan1/chan2'ye YAZAR — Pixhawk bunu MANUAL/ACRO gibi RC
# girişini okuyan modlarda gerçek RC kumandası gibi işler; GUIDED/AUTO'da göz
# ardı edilir (bkz. collision_avoidance_node'daki AUTO→MANUAL→AUTO geçişi).
# DİKKAT: "release" (kontrolü kumandaya bırakma) MUTLAKA 0 ile yapılmalı — 1500
# gibi gerçek bir PWM değeri göndermek kanalı override'da nötr konumda KİLİTLER,
# kumandaya kontrolü GERİ VERMEZ.
RC_CHAN_STEERING = 1
RC_CHAN_THROTTLE = 2
RC_PWM_CENTER = 1500
RC_STEERING_SPAN = 200
# Referans script 250 kullanıyordu (PWM 1750'de kırpılıyordu, RC_PWM_MAX=1700
# ile fiilen 1700'e sınırlanıyordu) — sahada ölçülen RPM yetersizdi (590),
# daha fazla itki için hem span hem tavan artırıldı.
RC_THROTTLE_SPAN = 400
RC_PWM_MIN = 1300
RC_PWM_MAX = 1900
RC_FAILSAFE_TIMEOUT_SEC = 0.5

# Yerel /scan'i MAVLink OBSTACLE_DISTANCE'a çevirip Pixhawk'a GÖNDERİR (referans
# script'le birebir aynı) — bu olmadan yer istasyonu (QGC vb.) hiçbir LiDAR
# verisi görmez, çünkü Pixhawk'ın kendi telemetri radyosu sadece kendisine
# GELEN mesajları GCS'ye iletir. Sadece GÖNDERİR — collision_avoidance_node
# kaçış kararı için doğrudan yerel /scan'i kullanır (bkz. collision_avoidance_
# node._local_zone_mins), bu yüzden burada ayrıca bir OBSTACLE_DISTANCE alım/
# filtreleme yolu YOKTUR.
LIDAR_MIN_CM = 10
LIDAR_MAX_CM = 1200
LIDAR_INCREMENT_DEG = 5
LIDAR_BIN_COUNT = 72


class PymavlinkControllerNode(Node):

    def __init__(self):
        super().__init__('pymavlink_controller_node')

        # ---------- Parametreler ----------
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('system_id', 1)
        self.declare_parameter('component_id', 1)
        self.declare_parameter('user_param_names', DEFAULT_USER_PARAMS)
        self.declare_parameter('user_param_poll_sec', 1.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self._sys_id = self.get_parameter('system_id').value
        self._comp_id = self.get_parameter('component_id').value
        self._user_param_names = list(self.get_parameter('user_param_names').value)
        self._user_param_poll = float(self.get_parameter('user_param_poll_sec').value)

        # ---------- MAVLink bağlantısı ----------
        self._conn = self._open_connection(port, baud)
        self._wait_for_vehicle_heartbeat(timeout=30.0)
        self.get_logger().info(
            f'Heartbeat alındı — system {self._conn.target_system}, '
            f'component {self._conn.target_component}'
        )

        # ---------- Publisher'lar ----------
        self.pub_gps = self.create_publisher(NavSatFix, 'telemetry/gps', 10)
        self.pub_imu = self.create_publisher(Imu, 'telemetry/imu', 10)
        self.pub_heading = self.create_publisher(Float64, 'telemetry/heading', 10)
        self.pub_groundspeed = self.create_publisher(Float64, 'telemetry/groundspeed', 10)
        self.pub_altitude = self.create_publisher(Float64, 'telemetry/altitude', 10)
        self.pub_heartbeat_status = self.create_publisher(String, 'telemetry/heartbeat_status', 10)
        self.pub_user_params = self.create_publisher(String, 'telemetry/user_params', LATCHED_QOS)
        # Sade mod adı — collision_avoidance_node'un AUTO/GUIDED izleyicisi
        # heartbeat_status'un serbest metnini ayrıştırmak zorunda kalmasın.
        self.pub_mode = self.create_publisher(String, 'telemetry/mode', 10)
        # ARM/DISARM ve mod değişikliği komutlarının GERÇEKTEN kabul edilip
        # edilmediği — bkz. _handle_command_ack. Olmadan "komut gönderildi"
        # logu, otopilot komutu reddetse bile (ör. ARMING_CHECK/GPS/EKF)
        # başarılı görünüyordu.
        self.pub_command_ack = self.create_publisher(String, 'telemetry/command_ack', 10)
        # Otopilotun KENDİ gönderdiği STATUSTEXT (ör. "AUTO: mission empty",
        # "EKF variance" gibi mod reddi sebepleri) — bizim yer istasyonuna
        # gönderdiğimiz telemetry/statustext ile KARIŞTIRILMASIN, o TERSİ
        # yöndür (bizden→GCS). Bu topic sahada "neden AUTO'ya geçmiyor"
        # sorusunu terminalden görebilmek için eklendi — eskiden STATUSTEXT
        # recv_match filtresinde HİÇ YOKTU, otopilotun ret sebebi sessizce
        # atılıyordu.
        self.pub_fc_statustext = self.create_publisher(String, 'telemetry/fc_statustext', 10)
        # Şartname Dosya 2 (araç telemetri verisi) "hız set pointi" ve "yön
        # set pointi" istiyor — bunlar GERÇEKLEŞEN hız/yönden (telemetry/
        # groundspeed, telemetry/heading) FARKLI, otopilotun HEDEFLEDİĞİ
        # değerler. Yön hedefi NAV_CONTROLLER_OUTPUT.nav_bearing'den (AUTO/
        # GUIDED'de otopilotun kendi navigasyon hedefi), hız hedefi ise zaten
        # izlenen CRUISE_SPEED kullanıcı parametresinden alınır. Sadece
        # loglama amaçlı — hiçbir kontrol mantığı bundan ETKİLENMEZ.
        self.pub_nav_setpoint = self.create_publisher(String, 'telemetry/nav_setpoint', 10)
        # Motor/tekerlek çıkışının GERÇEKTEN hareket ettiğini doğrulamak için
        # ham PWM — ROS tarafında "araç hareket ediyor mu" için başka doğrudan
        # gözlenebilir bir sinyal yok.
        self.pub_servo_output = self.create_publisher(String, 'telemetry/servo_output', 10)

        # ---------- Subscriber'lar ----------
        self.create_subscription(Twist, 'cmd/velocity', self._cb_velocity, 10)
        self.create_subscription(Vector3Stamped, 'cmd/setpoint_ned', self._cb_setpoint_ned, 10)
        self.create_subscription(Bool, 'cmd/arm', self._cb_arm, 10)
        self.create_subscription(String, 'cmd/mode', self._cb_mode, 10)
        self.create_subscription(String, 'cmd/set_user_param', self._cb_set_user_param, 10)
        # WAYPOINT_KAMIKAZE (SCR_USER1=20) — son waypoint'e ulaşıldığını
        # görüp otomatik KAMIKAZE geçişi yapabilmek için hangi fazda
        # olunduğunu bilmek gerekiyor (bkz. _cb_phase / _handle_mission_*).
        self.create_subscription(String, 'mission/phase', self._cb_phase, LATCHED_QOS)
        # collision_avoidance_node'un en yakın engel bildirimi — yer istasyonu
        # arayüzüne STATUSTEXT olarak iletilir.
        self.create_subscription(String, 'telemetry/obstacle_report', self._cb_obstacle_report, 10)
        # Serbest metin durum bildirimi (faz değişimi, SCR_USER3 anahtarı,
        # kaçış başladı/bitti). Sahada Jetson'da terminal yok; operatörün
        # SCR_USER parametrelerinin GERÇEKTEN uygulandığını görebildiği tek yol
        # yer istasyonundaki STATUSTEXT satırlarıdır.
        self.create_subscription(String, 'telemetry/statustext', self._cb_statustext, 10)
        # Yerel /scan'i OBSTACLE_DISTANCE'a çevirip Pixhawk'a göndermek için —
        # bkz. LIDAR_* sabitleri / _cb_scan / _send_obstacle_distance. scan_filter_node
        # BEST_EFFORT yayınlıyor; RELIABLE burada da QoS uyuşmazlığıyla veri
        # akmadan sessizce boşta kalmaya yol açar (bu projede tekrarlayan hata sınıfı).
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(LaserScan, '/scan', self._cb_scan, scan_qos)
        # fake_gps_node (SADECE bench/kapalı alan testi) — GPS_INPUT olarak
        # Pixhawk'a iletir. GPS_TYPE=14 (MAV) yer istasyonundan önceden
        # ayarlanmış olmalı, aksi hâlde otopilot bu mesajları sessizce yok
        # sayar (bkz. fake_gps_node docstring).
        self.create_subscription(NavSatFix, 'cmd/fake_gps', self._cb_fake_gps, 10)

        # ---------- Durum ----------
        self._lock = threading.Lock()
        self._running = True
        self._user_params = {}       # {'SCR_USER1': 5.0, ...}
        # ---------- WAYPOINT_KAMIKAZE (bkz. _cb_phase / _handle_mission_*) ----------
        self._current_phase = 'IDLE'
        self._mission_total = 0          # MISSION_COUNT'tan — 0 = henüz bilinmiyor
        self._kamikaze_auto_triggered = False
        # RC override failsafe: cmd/velocity RC_FAILSAFE_TIMEOUT_SEC'den uzun
        # süre gelmezse kanallar release edilir (kumandaya geri bırakılır).
        self._rc_override_active = False
        self._last_velocity_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self._rc_failsafe_callback)

        # OBSTACLE_DISTANCE gönderim durumu (referans script'le birebir aynı).
        self._latest_obstacle_distances = [LIDAR_MAX_CM + 1] * LIDAR_BIN_COUNT
        self._last_scan_time = 0.0
        self.create_timer(0.2, self._send_obstacle_distance)  # 5 Hz

        # ---------- SCR_USER parametre yoklama ----------
        if self._user_param_names:
            self.create_timer(self._user_param_poll, self._poll_user_params)
            self.get_logger().info(
                f'Kullanıcı parametreleri {self._user_param_poll:.1f} sn\'de bir '
                f'okunuyor: {", ".join(self._user_param_names)}'
            )

        # ---------- Thread'ler ----------
        self._telem_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._telem_thread.start()

        self.get_logger().info('pymavlink_controller_node başlatıldı.')

    # ───────────────────── Bağlantı Kurulumu ─────────────────────

    def _open_connection(self, port: str, baud: int):
        """CubeOrange USB üzerinden İKİ CDC-ACM arayüzü çıkarır (ttyACM0/
        ttyACM1); hangisinin gerçek MAVLink hattı olduğu USB enumerasyon
        sırasına göre değişebilir (udev kuralı normalde bunu sabitler, ama
        elle/doğrudan `serial_port` verilen durumlarda — ör. `ros2 run` ile
        launch dosyası olmadan test — yanlış port seçilmiş olabilir). serial_
        port ttyACM0/ttyACM1 ise ve açılamıyorsa ya da dosya yoksa, diğerini
        dener."""
        fallback = None
        if port == '/dev/ttyACM0':
            fallback = '/dev/ttyACM1'
        elif port == '/dev/ttyACM1':
            fallback = '/dev/ttyACM0'

        if fallback and not os.path.exists(port):
            self.get_logger().warn(f'{port} yok — {fallback} deneniyor.')
            self.get_logger().info(f'MAVLink bağlantısı açılıyor: {fallback}@{baud}')
            return mavutil.mavlink_connection(fallback, baud=baud)

        self.get_logger().info(f'MAVLink bağlantısı açılıyor: {port}@{baud}')
        try:
            return mavutil.mavlink_connection(port, baud=baud)
        except Exception as e:
            if not fallback:
                raise
            self.get_logger().warn(f'{port} açılamadı ({e}) — {fallback} deneniyor.')
            self.get_logger().info(f'MAVLink bağlantısı açılıyor: {fallback}@{baud}')
            return mavutil.mavlink_connection(fallback, baud=baud)

    def _wait_for_vehicle_heartbeat(self, timeout: float):
        """pymavlink'in wait_heartbeat()'i HEARTBEAT tipindeki İLK mesajı döner —
        hangisi olduğuna bakmaz. Bu araçta USB hattında Pixhawk'ın kendi
        heartbeat'i (sysid=1, compid=1, tip=SURFACE_BOAT) dışında İKİNCİ bir
        heartbeat daha akıyor (sysid=1, compid=0, tip=ADSB / autopilot=INVALID —
        muhtemelen bağlı bir ADS-B alıcısının pass-through'u); pymavlink bunu
        "araç" saymadığı için target_system'i güncellemiyor (self.sysid=0'da
        kalıyor), AMA target_component'i pymavlink hiçbir zaman otomatik
        ayarlamıyor — hangi heartbeat gelirse gelsin 0 (broadcast) kalıyordu.
        Sonuç: tüm komutlar (arm, mod, SET_ATTITUDE_TARGET, param okuma/yazma)
        component_id=0 ile gidiyordu; bazı ArduPilot sürümlerinde bu sessizce
        yok sayılabiliyor. Bu yüzden gerçek araç heartbeat'i gelene kadar
        döngüde bekleyip system/component'i BİZ elle sabitliyoruz.

        Süre dolana kadar hiç "gerçek" heartbeat gelmezse (ör. yanlış porta
        bağlanıldıysa) düğümü tamamen çökertmek yerine — bu, tüm telemetri/
        komut köprüsünü öldürüp operatörü hiçbir geri bildirim olmadan bırakır,
        eski (sabitleme olmadan önceki) davranıştan daha kötüdür — son görülen
        heartbeat'e (varsa) düşülür, aksi hâlde bağlantı hiç kurulamamış demektir
        ve o zaman gerçekten hata verilir."""
        deadline = time.time() + timeout
        last_seen = None
        while time.time() < deadline:
            hb = self._conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5.0)
            if hb is None:
                continue
            last_seen = hb
            if (hb.type == mavutil.mavlink.MAV_TYPE_ADSB
                    or hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                self.get_logger().warn(
                    f'Araç dışı heartbeat yok sayıldı (sysid={hb.get_srcSystem()}, '
                    f'compid={hb.get_srcComponent()}, tip={hb.type}, autopilot={hb.autopilot}).'
                )
                continue
            self._conn.target_system = hb.get_srcSystem()
            self._conn.target_component = hb.get_srcComponent()
            return

        if last_seen is not None:
            self.get_logger().error(
                f'{timeout:.0f} sn içinde araç heartbeat\'i doğrulanamadı — son görülen '
                f'heartbeat\'e (sysid={last_seen.get_srcSystem()}, '
                f'compid={last_seen.get_srcComponent()}) düşülüyor. Doğru porta bağlı '
                f'olduğunuzu kontrol edin (bkz. udev/idaws_pixhawk).'
            )
            self._conn.target_system = last_seen.get_srcSystem()
            self._conn.target_component = last_seen.get_srcComponent()
            return

        raise RuntimeError(f'{timeout:.0f} sn içinde HİÇBİR heartbeat alınamadı — port/bağlantı kontrol edin.')

    # ───────────────────── Telemetri Okuma Thread'i ─────────────────────

    def _telemetry_loop(self):
        """Kesintisiz olarak MAVLink mesajlarını okuyup ROS 2'ye publish eder."""
        while self._running:
            try:
                msg = self._conn.recv_match(
                    type=[
                        'HEARTBEAT',
                        'GLOBAL_POSITION_INT',
                        'ATTITUDE',
                        'VFR_HUD',
                        'PARAM_VALUE',
                        'COMMAND_ACK',
                        'SERVO_OUTPUT_RAW',
                        'MISSION_ITEM_REACHED',
                        'MISSION_CURRENT',
                        'MISSION_COUNT',
                        'STATUSTEXT',
                        'NAV_CONTROLLER_OUTPUT',
                    ],
                    blocking=True,
                    timeout=1.0,
                )
                if msg is None:
                    continue

                msg_type = msg.get_type()

                if msg_type == 'HEARTBEAT':
                    self._handle_heartbeat(msg)
                elif msg_type == 'GLOBAL_POSITION_INT':
                    self._handle_global_position(msg)
                elif msg_type == 'ATTITUDE':
                    self._handle_attitude(msg)
                elif msg_type == 'VFR_HUD':
                    self._handle_vfr_hud(msg)
                elif msg_type == 'PARAM_VALUE':
                    self._handle_param_value(msg)
                elif msg_type == 'COMMAND_ACK':
                    self._handle_command_ack(msg)
                elif msg_type == 'SERVO_OUTPUT_RAW':
                    self._handle_servo_output(msg)
                elif msg_type == 'MISSION_ITEM_REACHED':
                    self._handle_mission_item_reached(msg)
                elif msg_type == 'MISSION_CURRENT':
                    self._handle_mission_current(msg)
                elif msg_type == 'MISSION_COUNT':
                    self._handle_mission_count(msg)
                elif msg_type == 'STATUSTEXT':
                    self._handle_fc_statustext(msg)
                elif msg_type == 'NAV_CONTROLLER_OUTPUT':
                    self._handle_nav_setpoint(msg)

            except Exception as e:
                self.get_logger().warn(f'Telemetri okuma hatası: {e}')
                time.sleep(0.1)

    def _decode_mode(self, msg):
        """mavutil.mode_string_v10() base_mode'da CUSTOM_MODE_ENABLED biti
        eksikse (ArduPilot'ta bazı heartbeat'lerde görülen bilinen bir
        tuhaflık) "Mode(0x00000004)" gibi ham hex'e düşer — telemetry/mode'a
        bu yazılırsa collision_avoidance_node'un _fc_mode karşılaştırması hiç
        eşleşmez, araç MANUAL/AUTO'da olsa bile "onaylanmadı" sanılıp sonsuza
        dek yeniden dener. custom_mode ALAN her zaman doğru gelir — set_mode()
        zaten aynı mode_mapping()'i AD→numara için kullanıyor, burada tersini
        (numara→ad) yapıp öncelikli kullanıyoruz."""
        if not hasattr(msg, 'custom_mode'):
            return 'UNKNOWN'
        try:
            mapping = self._conn.mode_mapping()
        except Exception:
            mapping = None
        if mapping:
            name = next((k for k, v in mapping.items() if v == msg.custom_mode), None)
            if name:
                return name
        return mavutil.mode_string_v10(msg)

    def _handle_heartbeat(self, msg):
        mode = self._decode_mode(msg)
        status = String()
        status.data = f'mode={mode} armed={msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED != 0}'
        self.pub_heartbeat_status.publish(status)

        mode_msg = String()
        mode_msg.data = mode
        self.pub_mode.publish(mode_msg)

    def _handle_command_ack(self, msg):
        """ARM/DISARM ve mod değişikliği gibi komutların otopilot tarafından
        GERÇEKTEN kabul edilip edilmediğini gösterir. arm()/disarm()/set_mode()
        komutu gönderir göndermez "gönderildi" diye loglar — bu, otopilot
        ARMING_CHECK/GPS/EKF gibi bir sebeple REDDETSE bile başarılı görünmesine
        yol açıyordu. Reddedilirse burada net biçimde görünür.

        SADECE BİZİM gönderdiğimiz komutlar raporlanır. MAVLink hattı
        paylaşımlıdır: MAVProxy / Mission Planner / QGC de komut gönderir ve
        onların ACK'leri de bize ulaşır. Filtrelenmezse başkasının komutu bizim
        hatamız gibi görünüyordu — sim'de MAVProxy'nin GET_HOME_POSITION (410)
        isteği, GPS/EKF daha hazır değilken FAILED dönüp durmadan ERROR
        basıyordu. Daha kötüsü, bu gürültü GERÇEK arm/mod hatalarını gizler."""
        if msg.command not in OUR_COMMANDS:
            self.get_logger().debug(
                f'Başka bir GCS\'in komut sonucu yok sayıldı (komut {msg.command}).')
            return

        try:
            result_name = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
        except (KeyError, AttributeError):
            result_name = str(msg.result)

        command_name = {
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM: 'ARM/DISARM',
            mavutil.mavlink.MAV_CMD_DO_SET_MODE: 'MOD DEĞİŞTİRME',
        }.get(msg.command, f'komut {msg.command}')

        ack = String()
        ack.data = f'{command_name}: {result_name}'
        self.pub_command_ack.publish(ack)

        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.get_logger().info(f'{command_name} KABUL EDİLDİ.')
        else:
            self.get_logger().error(
                f'{command_name} REDDEDİLDİ: {result_name} — ARMING_CHECK/GPS/EKF/mod '
                f'geçiş şartlarını kontrol edin.'
            )

    def _handle_fc_statustext(self, msg):
        """Otopilotun kendi STATUSTEXT'i — mod reddi/failsafe/pre-arm gibi
        sebepleri GENELLİKLE burada açıklar (ör. "AUTO: mission empty",
        "PreArm: ...", "EKF variance"). Terminalde/log'da doğrudan görünsün
        diye WARN seviyesinde basılır — Mission Planner/QGC olmadan da
        "neden mod değişmiyor" sorusuna cevap versin."""
        text = getattr(msg, 'text', '')
        if isinstance(text, bytes):
            text = text.decode('utf-8', errors='replace')
        text = text.rstrip('\x00').strip()
        if not text:
            return
        self.get_logger().warn(f'[OTOPİLOT] {text}')
        out = String()
        out.data = text[:200]
        self.pub_fc_statustext.publish(out)

    def _handle_nav_setpoint(self, msg):
        """NAV_CONTROLLER_OUTPUT.nav_bearing — otopilotun AUTO/GUIDED'de
        HEDEFLEDİĞİ yön (derece). MANUAL/RC override fazlarında otopilotun
        kendi navigasyon hedefi yoktur, bu durumda ArduPilot yine de bu
        mesajı gönderir ama değer anlamsız/eski kalabilir — mission_logger_
        node bunu yorumlarken faz bilgisiyle (mission/phase) birlikte
        değerlendirmeli, burada filtrelenmiyor (ham veri, yorum yapılmıyor)."""
        out = String()
        out.data = json.dumps({
            'heading_setpoint_deg': float(msg.nav_bearing),
            'speed_setpoint_mps': float(self._user_params.get('CRUISE_SPEED', 0.0)),
        })
        self.pub_nav_setpoint.publish(out)

    def _handle_global_position(self, msg):
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps'
        fix.latitude = msg.lat / 1e7
        fix.longitude = msg.lon / 1e7
        fix.altitude = msg.alt / 1e3
        self.pub_gps.publish(fix)

        alt_msg = Float64()
        alt_msg.data = msg.relative_alt / 1e3
        self.pub_altitude.publish(alt_msg)

    def _handle_attitude(self, msg):
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'
        # Quaternion dönüşümü (roll, pitch, yaw → quaternion)
        cr = math.cos(msg.roll * 0.5)
        sr = math.sin(msg.roll * 0.5)
        cp = math.cos(msg.pitch * 0.5)
        sp = math.sin(msg.pitch * 0.5)
        cy = math.cos(msg.yaw * 0.5)
        sy = math.sin(msg.yaw * 0.5)
        imu.orientation.w = cr * cp * cy + sr * sp * sy
        imu.orientation.x = sr * cp * cy - cr * sp * sy
        imu.orientation.y = cr * sp * cy + sr * cp * sy
        imu.orientation.z = cr * cp * sy - sr * sp * cy
        imu.angular_velocity.x = msg.rollspeed
        imu.angular_velocity.y = msg.pitchspeed
        imu.angular_velocity.z = msg.yawspeed
        self.pub_imu.publish(imu)

    def _handle_vfr_hud(self, msg):
        heading = Float64()
        heading.data = float(msg.heading)
        self.pub_heading.publish(heading)

        gs = Float64()
        gs.data = float(msg.groundspeed)
        self.pub_groundspeed.publish(gs)

    def _handle_servo_output(self, msg):
        """Hangi kanalın direksiyon/itki olduğu frame/parametre ayarına göre
        değiştiği için tüm kanallar ham PWM (µs) olarak yayınlanır — yorumlama
        test script'ine/operatöre bırakılır."""
        out = String()
        out.data = json.dumps({
            f'servo{i}_raw': getattr(msg, f'servo{i}_raw')
            for i in range(1, 9)
            if hasattr(msg, f'servo{i}_raw')
        })
        self.pub_servo_output.publish(out)

    # ───────────────────── WAYPOINT_KAMIKAZE: Otomatik Görev Bitişi ─────────────────────

    def _cb_phase(self, msg: String):
        """mission_manager_node'un yayınladığı güncel faz. WAYPOINT_KAMIKAZE'ye
        her GİRİŞTE tetik bayrağı sıfırlanır ve mission_total yeniden istenir
        — operatör her seferinde yeni/farklı bir görev yüklemiş olabilir."""
        phase = msg.data
        if phase != self._current_phase and phase == 'WAYPOINT_KAMIKAZE':
            self._kamikaze_auto_triggered = False
            self._mission_total = 0
            try:
                with self._lock:
                    self._conn.mav.mission_request_list_send(
                        self._conn.target_system, self._conn.target_component)
            except Exception as e:
                self.get_logger().warn(f'mission_request_list gönderilemedi: {e}')
        self._current_phase = phase

    def _handle_mission_count(self, msg):
        """MISSION_COUNT — mission_request_list_send'e yanıt. count, ArduPilot'ta
        0. sırayı (ev/home) da sayar; SON gerçek waypoint indeksi count-1'dir."""
        self._mission_total = int(msg.count)
        self.get_logger().info(f'Görev öğesi sayısı: {self._mission_total}')

    def _handle_mission_item_reached(self, msg):
        """Her waypoint'e ulaşıldığında ArduPilot'tan gelir. WAYPOINT_KAMIKAZE
        fazındaysak ve ulaşılan sıra SON öğeyse (mission_total-1) otomatik
        KAMIKAZE geçişini tetikler."""
        if (self._current_phase == 'WAYPOINT_KAMIKAZE' and not self._kamikaze_auto_triggered
                and self._mission_total > 0 and msg.seq >= self._mission_total - 1):
            self._trigger_kamikaze_handoff(f'MISSION_ITEM_REACHED seq={msg.seq}')

    def _handle_mission_current(self, msg):
        """MISSION_CURRENT — bazı ArduPilot sürümlerinde mission_state alanı
        var (MISSION_STATE_COMPLETE=5): MISSION_ITEM_REACHED'e ek/yedek bir
        'görev bitti' sinyali. Alan yoksa/desteklenmiyorsa sessizce yok sayılır."""
        state = getattr(msg, 'mission_state', 0)
        if (state == 5 and self._current_phase == 'WAYPOINT_KAMIKAZE'
                and not self._kamikaze_auto_triggered):
            self._trigger_kamikaze_handoff('MISSION_CURRENT mission_state=COMPLETE')

    def _trigger_kamikaze_handoff(self, reason: str):
        self._kamikaze_auto_triggered = True
        self.get_logger().warn(
            f'WAYPOINT_KAMIKAZE: son waypoint\'e ulaşıldı ({reason}) — '
            'SCR_USER1=15 (KAMIKAZE) yazılıyor.'
        )
        self._send_statustext('IDAWS AUTO->KAMIKAZE')
        self._write_user_param('SCR_USER1', 15.0)

    # ───────────────────── LiDAR → OBSTACLE_DISTANCE Gönderimi ─────────────────────

    def _cb_scan(self, msg: LaserScan):
        """LaserScan → 72 açısal dilim (referans script'le birebir aynı
        dönüşüm) — bkz. LIDAR_* sabitleri."""
        bins = [LIDAR_MAX_CM + 1] * LIDAR_BIN_COUNT  # her taramada "engel yok"tan başla

        for i, distance_m in enumerate(msg.ranges):
            if not math.isfinite(distance_m):
                continue
            if distance_m < max(msg.range_min, LIDAR_MIN_CM / 100.0):
                continue
            if distance_m > min(msg.range_max, LIDAR_MAX_CM / 100.0):
                continue

            ros_angle_rad = msg.angle_min + (i * msg.angle_increment)
            ros_angle_deg = math.degrees(ros_angle_rad)
            # MAV_FRAME_BODY_FRD: 0=ileri, pozitif=sağ/saat yönü.
            mav_angle_deg = (-ros_angle_deg) % 360.0
            bin_index = int(mav_angle_deg // LIDAR_INCREMENT_DEG) % LIDAR_BIN_COUNT

            distance_cm = int(round(distance_m * 100.0))
            distance_cm = max(LIDAR_MIN_CM, min(LIDAR_MAX_CM, distance_cm))

            # Aynı dilimde birden fazla ölçüm varsa en yakın engeli sakla.
            if bins[bin_index] > LIDAR_MAX_CM or distance_cm < bins[bin_index]:
                bins[bin_index] = distance_cm

        self._latest_obstacle_distances = bins
        self._last_scan_time = time.monotonic()

    def _send_obstacle_distance(self):
        # Son 1 saniyede /scan gelmediyse eski LiDAR verisini gönderme.
        if self._last_scan_time == 0.0 or (time.monotonic() - self._last_scan_time) > 1.0:
            return

        try:
            with self._lock:
                self._conn.mav.obstacle_distance_send(
                    int(time.time() * 1_000_000),
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                    self._latest_obstacle_distances,
                    LIDAR_INCREMENT_DEG,
                    LIDAR_MIN_CM,
                    LIDAR_MAX_CM,
                    float(LIDAR_INCREMENT_DEG),
                    0.0,  # angle_offset
                    mavutil.mavlink.MAV_FRAME_BODY_FRD,
                )
        except Exception as e:
            self.get_logger().error(f'OBSTACLE_DISTANCE gönderme hatası: {e}')

    # ───────────────────── SCR_USER Parametreleri ─────────────────────

    def _handle_param_value(self, msg):
        """Otopilottan gelen PARAM_VALUE — sadece izlediğimiz parametreler yayınlanır."""
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode('utf-8', errors='ignore')
        name = name.strip('\x00')
        if name not in self._user_param_names:
            return

        value = float(msg.param_value)
        if self._user_params.get(name) == value:
            return  # değişmediyse tekrar yayınlama

        previous = self._user_params.get(name)
        self._user_params[name] = value
        self.get_logger().info(f'{name}: {previous} → {value}')

        out = String()
        out.data = json.dumps(self._user_params)
        self.pub_user_params.publish(out)

    def _poll_user_params(self):
        """İzlenen parametreleri otopilottan periyodik olarak ister."""
        with self._lock:
            for name in self._user_param_names:
                try:
                    self._conn.mav.param_request_read_send(
                        self._conn.target_system,
                        self._conn.target_component,
                        name.encode('utf-8'),
                        -1,
                    )
                except Exception as e:
                    self.get_logger().warn(f'{name} okunamadı: {e}')

    def _cb_set_user_param(self, msg: String):
        """
        Web arayüzünden parametre yazma: {"name": "SCR_USER1", "value": 10}
        Gerçek akışla aynı yolu kullanır — değer otopiluta yazılır, sonra
        PARAM_VALUE olarak geri okunup görev fazına dönüşür.
        """
        try:
            data = json.loads(msg.data)
            name = str(data['name'])
            value = float(data['value'])
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f'Geçersiz set_user_param isteği: {msg.data!r} ({e})')
            return
        self._write_user_param(name, value)

    def _write_user_param(self, name: str, value: float):
        """Otopilota PARAM_SET yazar — web arayüzü ve WAYPOINT_KAMIKAZE'nin
        otomatik KAMIKAZE geçişi (bkz. _handle_mission_item_reached) AYNI
        yolu kullanır: hangi kaynaktan geldiği fark etmeksizin, sonuç
        PARAM_VALUE olarak geri okunup normal görev fazı akışına girer."""
        with self._lock:
            self._conn.mav.param_set_send(
                self._conn.target_system,
                self._conn.target_component,
                name.encode('utf-8'),
                value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
        self.get_logger().info(f'{name} = {value} yazıldı.')

    # ───────────────────── Arm / Mod Komutları ─────────────────────

    def _cb_arm(self, msg: Bool):
        self.arm() if msg.data else self.disarm()

    def _cb_obstacle_report(self, msg: String):
        self._send_statustext(msg.data)

    def _cb_statustext(self, msg: String):
        self._send_statustext(msg.data)

    def _send_statustext(self, text: str):
        """MAVLink STATUSTEXT'in metin alanı 50 BAYT — daha uzunu pymavlink'te
        paketleme hatası verir (ve mesaj hiç gitmez). ASCII'ye indirgenir:
        Türkçe karakterler yer istasyonlarında bozuk görünüyor."""
        payload = text.encode('ascii', errors='replace')[:50]
        try:
            with self._lock:
                self._conn.mav.statustext_send(
                    mavutil.mavlink.MAV_SEVERITY_INFO,
                    payload,
                )
        except Exception as e:
            self.get_logger().warn(f'STATUSTEXT gönderilemedi: {e}')

    def _cb_mode(self, msg: String):
        self.set_mode(msg.data)

    # ───────────────────── Komut Gönderme (ROS → MAVLink) ─────────────────────

    def _cb_velocity(self, twist: Twist):
        """RC_CHANNELS_OVERRIDE ile motor kontrolü — referans script'le birebir
        aynı dönüşüm. linear.x [-1,1] → throttle PWM, angular.z [-1,1] →
        steering PWM (RC_CHAN_* sabitleri). İşaret sahada DOĞRULANDI
        (angular.z>0 → tekne sağa döner, referansla aynı — bir önceki "tersi
        doğru" denemesi yanlış çıktı, bu orijinal işarettir)."""
        throttle_norm = self._clamp(twist.linear.x, -1.0, 1.0)
        steer_norm = self._clamp(twist.angular.z, -1.0, 1.0)

        throttle_pwm = int(RC_PWM_CENTER + throttle_norm * RC_THROTTLE_SPAN)
        steering_pwm = int(RC_PWM_CENTER - steer_norm * RC_STEERING_SPAN)
        throttle_pwm = self._clamp(throttle_pwm, RC_PWM_MIN, RC_PWM_MAX)
        steering_pwm = self._clamp(steering_pwm, RC_PWM_MIN, RC_PWM_MAX)

        self._last_velocity_cmd_time = self.get_clock().now()
        self._set_rc(throttle_pwm, steering_pwm)

    def _set_rc(self, throttle_pwm: int, steering_pwm: int):
        rc_values = [65535] * 8
        rc_values[RC_CHAN_STEERING - 1] = steering_pwm
        rc_values[RC_CHAN_THROTTLE - 1] = throttle_pwm
        with self._lock:
            self._conn.mav.rc_channels_override_send(
                self._conn.target_system,
                self._conn.target_component,
                *rc_values,
            )
        self._rc_override_active = True

    def _release_rc(self):
        """chan1/chan2'yi 0 (release) gönderip kumandaya geri bırakır. DİKKAT:
        buraya 1500 gibi gerçek bir PWM değeri göndermek kanalı override'da
        NÖTR konumda KİLİTLER, kumandaya kontrolü GERİ VERMEZ — değer MUTLAKA
        0 olmalı."""
        rc_values = [65535] * 8
        rc_values[RC_CHAN_STEERING - 1] = 0
        rc_values[RC_CHAN_THROTTLE - 1] = 0
        with self._lock:
            self._conn.mav.rc_channels_override_send(
                self._conn.target_system,
                self._conn.target_component,
                *rc_values,
            )
        if self._rc_override_active:
            self.get_logger().warn('RC OVERRIDE BIRAKILDI — kontrol kumandaya döndü.')
        self._rc_override_active = False

    def _rc_failsafe_callback(self):
        """cmd/velocity RC_FAILSAFE_TIMEOUT_SEC'den uzun süre gelmezse override
        release edilir — aksi hâlde son komutta (ör. tam gaz) takılı kalır."""
        if not self._rc_override_active:
            return
        elapsed = (self.get_clock().now() - self._last_velocity_cmd_time).nanoseconds
        if elapsed > RC_FAILSAFE_TIMEOUT_SEC * 1e9:
            self._release_rc()

    def _cb_setpoint_ned(self, msg: Vector3Stamped):
        """SET_POSITION_TARGET_LOCAL_NED ile pozisyon komutu gönderir."""
        with self._lock:
            self._conn.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,  # position only
                msg.vector.x, msg.vector.y, msg.vector.z,
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                0, 0,     # yaw, yaw_rate
            )

    def _cb_fake_gps(self, msg: NavSatFix):
        """fake_gps_node'dan gelen sabit konumu MAVLink GPS_INPUT'e çevirip
        Pixhawk'a gönderir — SADECE bench/kapalı alan testi (bkz. fake_gps_node
        docstring). Hız/doğruluk alanlarımız yok, bu yüzden ignore_flags ile
        bunlar EKF'ye "yok say" olarak işaretlenir; sadece konum (lat/lon/alt)
        ve fix_type/uydu sayısı gerçek kabul edilir.
        ÖN KOŞUL: otopilotta GPS_TYPE=14 (MAV) elle ayarlanmış olmalı, aksi
        hâlde bu mesajlar sessizce yok sayılır (biz bunu doğrulayamayız)."""
        now = datetime.now(timezone.utc)
        gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
        delta = now - gps_epoch
        time_week = int(delta.days / 7)
        seconds_into_week = delta.total_seconds() - time_week * 7 * 86400
        time_week_ms = int(seconds_into_week * 1000) % (7 * 86400 * 1000)

        IGNORE_VEL_HORIZ = 8
        IGNORE_VEL_VERT = 16
        IGNORE_SPEED_ACCURACY = 32
        IGNORE_HORIZ_ACCURACY = 64
        IGNORE_VERT_ACCURACY = 128
        ignore_flags = (IGNORE_VEL_HORIZ | IGNORE_VEL_VERT | IGNORE_SPEED_ACCURACY
                        | IGNORE_HORIZ_ACCURACY | IGNORE_VERT_ACCURACY)

        with self._lock:
            self._conn.mav.gps_input_send(
                int(time.time() * 1e6),   # time_usec
                0,                         # gps_id
                ignore_flags,
                time_week_ms,
                time_week,
                3,                         # fix_type: 3 = 3D fix
                int(msg.latitude * 1e7),
                int(msg.longitude * 1e7),
                float(msg.altitude),
                1.0, 1.0,                  # hdop, vdop (ignore edilmiyor ama EKF için makul sabit)
                0.0, 0.0, 0.0,             # vn, ve, vd (ignore_flags ile yok sayılıyor)
                0.0, 0.0, 0.0,             # speed/horiz/vert accuracy (ignore ediliyor)
                10,                        # satellites_visible
            )

    # ───────────────────── Arm / Disarm / Mode ─────────────────────

    def arm(self):
        # arducopter_arm() da aynı MAV_CMD_COMPONENT_ARM_DISARM COMMAND_LONG'unu
        # gönderiyor (pymavlink kaynağında doğrulandı) — donanımda test edilen
        # referans script'le birebir aynı çağrı.
        with self._lock:
            self._conn.arducopter_arm()
        # "Gönderildi" ≠ "kabul edildi" — otopilot ARMING_CHECK/GPS/EKF
        # sebebiyle reddedebilir. Gerçek sonuç _handle_command_ack'te loglanır.
        self.get_logger().info('ARM komutu gönderildi (arducopter_arm, sonuç için COMMAND_ACK bekleniyor).')

    def disarm(self):
        with self._lock:
            self._conn.arducopter_disarm()
        self.get_logger().info('DISARM komutu gönderildi (arducopter_disarm, sonuç için COMMAND_ACK bekleniyor).')

    def set_mode(self, mode_name: str):
        """Ham SET_MODE mesajı (donanımda test edilen referans script'le
        birebir aynı çağrı — DOKUNULMADI) + EK olarak MAV_CMD_DO_SET_MODE
        (COMMAND_LONG). İkincisi COMMAND_ACK üretir — _handle_command_ack bu
        komut için zaten hazır (OUR_COMMANDS'ta vardı ama hiç tetiklenmiyordu,
        çünkü hiçbir yerde gönderilmiyordu). AUTO reddi "Flight mode change
        failed" gibi genel bir STATUSTEXT dışında sebep vermiyordu; bu ikinci
        çağrı MAV_RESULT sebebini COMMAND_ACK üzerinden görünür kılmak için
        eklendi. set_mode_send zaten çalışan yolu BOZMAZ, bu sadece ek bir
        teşhis/deneme kanalı."""
        mode_id = self._conn.mode_mapping().get(mode_name.upper())
        if mode_id is None:
            self.get_logger().error(f'Bilinmeyen mod: {mode_name}')
            return
        with self._lock:
            self._conn.mav.set_mode_send(
                self._conn.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self._conn.mav.command_long_send(
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,  # confirmation
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
                0, 0, 0, 0, 0,
            )
        self.get_logger().info(
            f'{mode_name} için mod değişikliği gönderildi (set_mode_send + DO_SET_MODE).')

    # ───────────────────── Yardımcılar ─────────────────────

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))

    def destroy_node(self):
        self._running = False
        try:
            self._release_rc()
        except Exception:
            pass
        self._telem_thread.join(timeout=2.0)
        self._conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PymavlinkControllerNode()
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
