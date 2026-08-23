import asyncio
from contextlib import contextmanager
import json
import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .bridge_health import BridgeHealth, strict_json, telemetry_payload
from .mission_contract import ROVER_SPEED_TURN_RATE_MASK
from .yki_status_contract import autonomy_fields, logging_fields, perception_count


class MavsdkBridgeNode(Node):
    """Bridge limited ROS2 body velocity commands to MAVSDK.

    The node defaults to dry_run=True so it is safe on developer laptops and in
    simulation. On the vehicle, set dry_run:=False after bench testing.
    """

    def __init__(self) -> None:
        super().__init__("ida_mavsdk_bridge")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("system_address", "udp://:14540")
        self.declare_parameter("command_timeout_s", 0.6)
        self.declare_parameter("send_hz", 10.0)
        # MAVSDK's armed stream is normally ~1 Hz; a 0.5 s composite timeout
        # makes an otherwise healthy bridge oscillate disconnected between
        # samples.  Keep the timeout above one source period while retaining a
        # bounded fail-closed health window.
        self.declare_parameter("health_source_timeout_s", 1.5)
        # pymavlink komut yolu (ArduRover'da MAVSDK offboard desteklenmez):
        # GUIDED modda SET_POSITION_TARGET_LOCAL_NED ile hız komutu gönderilir.
        # Port, MAVSDK (system_address) ile AYNI OLMAMALI (çift istemci çakışması).
        self.declare_parameter("pymavlink_address", "udp://:14540")
        self.declare_parameter("pymavlink_baud", 115200)
        self.declare_parameter("guided_retry_s", 1.0)
        self.declare_parameter("guided_timeout_s", 5.0)
        self.declare_parameter("command_retry_s", 0.5)
        # YKİ -> Pixhawk -> USB akışı: canonical alanlar takım stack'inin
        # SCR_USER1/2/3 faz-renk-kaçış kontratıyla çakışmaz. Jetson bu parametreyi
        # MAVSDK param API ile USB üzerinden okur. Görev noktaları mission_raw ile
        # Pixhawk'tan indirilir. (mavlink_passthrough YOK — eski NAMED_VALUE_INT yolu
        # RFD'den gelen TARGET_COLOR'u Jetson'a ulaştırmıyordu, kökten çözüldü.)
        self.declare_parameter("target_color_param", "SCR_USER4")
        self.declare_parameter("target_color_poll_hz", 1.0)
        self.declare_parameter("mission_download_hz", 0.2)
        self.declare_parameter("mission_raw_enabled", True)
        self.declare_parameter("mission_p1_count_param", "SCR_USER5")
        self.declare_parameter("mission_p2_count_param", "SCR_USER6")
        # Çift MAVLink istemcisi (YKİ TELEM1 + MAVSDK USB) çakışması koruması.
        # Pixhawk ACK'ları diğer GCS portlarına da forward eder (ArduPilot routing);
        # USB'ye düşen ek ACK'lar MAVSDK param poll'unu bozuyor (param timeout +
        # "Received ack for not-existing command: 512" spam'i). Poll: per-çağrı
        # timeout + (opak sıra bozulmasına karşı) iki bağımsız deneme, yanıt gelmezse
        # sessizce bir sonraki poll'a geç (offboard döngüsü bu poll'dan bağımsızdır).
        self.declare_parameter("param_call_timeout_s", 2.0)
        self.declare_parameter("param_max_retries", 2)
        self.declare_parameter("param_consecutive_fail_limit", 30)
        # One physical UART owner is provided by mavlink-router.  This companion
        # endpoint uses the vehicle SYS_ID and a distinct component ID so its
        # IDA->YKİ status packets pass the YKİ SYS_ID filter without colliding
        # with GCS SYS_ID 255.
        self.declare_parameter("companion_system_id", 1)
        self.declare_parameter("companion_component_id", 191)
        self.declare_parameter("yki_status_heartbeat_s", 2.0)
        # Canlı bağlantı tek başına araç konfigürasyonu veya motor çıkışı açmaz.
        # Takımın mevcut Jetson süreci yanında önce salt telemetri alternatifi
        # olarak denenebilmesi için iki ayrı, açık opt-in kapısı kullanılır.
        self.declare_parameter("vehicle_setup_enabled", False)
        self.declare_parameter("guided_mode_enabled", False)
        self.declare_parameter("motor_command_enabled", False)
        # CubeOrange gerçek araç kablolaması: AUX1/AUX3 = SERVO9/SERVO11.
        # Sim launch SERVO1/SERVO3 değerlerini açıkça override eder.
        self.declare_parameter("left_motor_servo_channel", 9)
        self.declare_parameter("right_motor_servo_channel", 11)
        self.declare_parameter("vehicle_frame_class", 2)
        # Real Rover accepts privileged GCS actuation from SYSID_MYGCS (255).
        # Keep companion_system_id=1 for vehicle/YKI status routing and change
        # the packet source only around actuation operations.
        self.declare_parameter("actuation_source_system_id", 255)
        # Production backend GUIDED velocity'dir. bench_rc_override yalnızca
        # GPS/EKF bulunmayan sabit bench'te fiziksel ROS->Pixhawk->ESC zincirini
        # doğrulamak için vardır; explicit ack olmadan motor etkinleşmez.
        self.declare_parameter("actuation_backend", "guided_velocity")
        self.declare_parameter("bench_rc_safety_ack", "")
        self.declare_parameter("bench_rc_steering_channel", 1)
        self.declare_parameter("bench_rc_throttle_channel", 2)
        self.declare_parameter("bench_rc_neutral_pwm", 1500)
        self.declare_parameter("bench_rc_max_delta_pwm", 100)
        self.declare_parameter("bench_rc_max_forward_mps", 0.30)
        # ArduPilot accepts MAVLink RC override only from SYSID_MYGCS on the
        # tested Rover configuration.  Keep the normal companion connection at
        # vehicle SYS_ID for YKI status routing and change the packet source
        # only while emitting the bench RC message.
        self.declare_parameter("bench_rc_source_system_id", 255)

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.system_address = str(self.get_parameter("system_address").value)
        self.command_timeout = float(self.get_parameter("command_timeout_s").value)
        self.send_hz = float(self.get_parameter("send_hz").value)
        self.health_source_timeout = float(
            self.get_parameter("health_source_timeout_s").value
        )
        if (
            not math.isfinite(self.health_source_timeout)
            or not 1.0 <= self.health_source_timeout <= 10.0
        ):
            raise ValueError("health_source_timeout_s must be finite in [1, 10]")
        self.pymavlink_address = str(self.get_parameter("pymavlink_address").value)
        self.pymavlink_baud = int(self.get_parameter("pymavlink_baud").value)
        self.guided_retry = float(self.get_parameter("guided_retry_s").value)
        self.guided_timeout = float(self.get_parameter("guided_timeout_s").value)
        self.command_retry = float(self.get_parameter("command_retry_s").value)
        self.target_color_param = str(self.get_parameter("target_color_param").value)
        self.target_color_poll_hz = float(self.get_parameter("target_color_poll_hz").value)
        self.mission_download_hz = float(self.get_parameter("mission_download_hz").value)
        self.mission_raw_enabled = bool(self.get_parameter("mission_raw_enabled").value)
        self.mission_p1_count_param = str(
            self.get_parameter("mission_p1_count_param").value
        )
        self.mission_p2_count_param = str(
            self.get_parameter("mission_p2_count_param").value
        )
        self.param_call_timeout = float(self.get_parameter("param_call_timeout_s").value)
        self.param_max_retries = int(self.get_parameter("param_max_retries").value)
        self.param_consecutive_fail_limit = int(self.get_parameter("param_consecutive_fail_limit").value)
        self.companion_system_id = int(self.get_parameter("companion_system_id").value)
        self.companion_component_id = int(self.get_parameter("companion_component_id").value)
        self.yki_status_heartbeat = float(self.get_parameter("yki_status_heartbeat_s").value)
        self.vehicle_setup_enabled = bool(
            self.get_parameter("vehicle_setup_enabled").value
        )
        self.guided_mode_enabled = bool(
            self.get_parameter("guided_mode_enabled").value
        )
        self.motor_command_enabled = bool(
            self.get_parameter("motor_command_enabled").value
        )
        self.left_motor_servo_channel = self._exact_int_parameter(
            "left_motor_servo_channel", 1, 16
        )
        self.right_motor_servo_channel = self._exact_int_parameter(
            "right_motor_servo_channel", 1, 16
        )
        self.vehicle_frame_class = self._exact_int_parameter(
            "vehicle_frame_class", 1, 10
        )
        self.actuation_source_system_id = self._exact_int_parameter(
            "actuation_source_system_id", 1, 255
        )
        self.actuation_backend = str(
            self.get_parameter("actuation_backend").value
        ).strip()
        self.bench_rc_safety_ack = str(
            self.get_parameter("bench_rc_safety_ack").value
        )
        self.bench_rc_steering_channel = self._exact_int_parameter(
            "bench_rc_steering_channel", 1, 8
        )
        self.bench_rc_throttle_channel = self._exact_int_parameter(
            "bench_rc_throttle_channel", 1, 8
        )
        self.bench_rc_neutral_pwm = self._exact_int_parameter(
            "bench_rc_neutral_pwm", 1400, 1600
        )
        self.bench_rc_max_delta_pwm = self._exact_int_parameter(
            "bench_rc_max_delta_pwm", 20, 150
        )
        self.bench_rc_max_forward_mps = float(
            self.get_parameter("bench_rc_max_forward_mps").value
        )
        self.bench_rc_source_system_id = self._exact_int_parameter(
            "bench_rc_source_system_id", 1, 255
        )
        if self.left_motor_servo_channel == self.right_motor_servo_channel:
            raise ValueError("left/right motor servo channels must be different")
        if self.actuation_backend not in {"guided_velocity", "bench_rc_override"}:
            raise ValueError("actuation_backend must be guided_velocity or bench_rc_override")
        if self.bench_rc_steering_channel == self.bench_rc_throttle_channel:
            raise ValueError("bench RC steering/throttle channels must be different")
        if (
            not math.isfinite(self.bench_rc_max_forward_mps)
            or not 0.05 <= self.bench_rc_max_forward_mps <= 0.5
        ):
            raise ValueError("bench_rc_max_forward_mps must be finite in [0.05,0.5]")
        if self.actuation_backend == "bench_rc_override" and self.motor_command_enabled:
            if self.bench_rc_safety_ack != "PROPELLER_AREA_CLEAR":
                raise ValueError("bench RC motor enable requires PROPELLER_AREA_CLEAR ack")
            if self.guided_mode_enabled:
                raise ValueError("bench RC backend and GUIDED mode cannot be enabled together")
            if self.vehicle_setup_enabled:
                raise ValueError("bench RC backend forbids vehicle parameter setup")
        if not 1 <= self.companion_system_id <= 255:
            raise ValueError("companion_system_id must be in [1,255]")
        if not 1 <= self.companion_component_id <= 255:
            raise ValueError("companion_component_id must be in [1,255]")
        if not math.isfinite(self.yki_status_heartbeat) or not 1.0 <= self.yki_status_heartbeat <= 10.0:
            raise ValueError("yki_status_heartbeat_s must be finite in [1,10]")
        self._param_consecutive_failures = 0
        self._mavsdk_param_lock = asyncio.Lock()

        self.latest_cmd = Twist()
        self.latest_cmd_time = self.get_clock().now()
        self.status_pub = self.create_publisher(String, "/control/mavsdk_status", 10)
        # Gerçek telemetri yayını: Pixhawk -> /telemetry/state (README kontratı).
        # MAVSDK async görevleri state dict'ine yazar, rclpy timer yayınlar (thread-safe lock).
        self.telemetry_pub = self.create_publisher(String, "/telemetry/state", 20)
        self._telemetry_lock = threading.Lock()
        self._telemetry_state: dict = {
            "lat": 0.0, "lon": 0.0, "heading_deg": 0.0, "ground_speed": 0.0,
            "roll_deg": 0.0, "pitch_deg": 0.0, "mode": "DISARMED",
        }
        self._health = BridgeHealth(
            self.dry_run, timeout_s=self.health_source_timeout
        )
        self._pymavlink_lock = threading.RLock()
        self._pymavlink_conn = None
        self._yki_values: dict[str, int] = {}
        self._yki_last_sent: dict[str, int] = {}
        self._yki_last_heartbeat = 0.0
        self.create_timer(1.0 / 10.0, self.publish_telemetry)  # 10Hz yayın
        self.create_timer(1.0 / 2.0, self.publish_status)
        self.create_timer(1.0, self.flush_yki_status)

        # YKİ'den gelen hedef rengi / görev noktaları (Pixhawk'ı DEPO olarak kullan).
        # YKİ PARAM_SET ile SCR_USER4'e hedef rengi kodunu yazar; Jetson bu parametreyi
        # periyodik poll ile okur. Görev noktaları mission_raw.download_mission() ile
        # Pixhawk'tan indirilir (YKİ upload zaten Pixhawk'a ulaşıyor). mavlink_passthrough
        # KULLANILMAZ: MAVSDK'ta yok, NAMED_VALUE_INT RFD'den forward edilmiyor (kök neden).
        self.target_pub = self.create_publisher(String, "/mission/target_color", 20)
        self.waypoint_pub = self.create_publisher(String, "/mission/waypoints", 20)
        self.create_subscription(String, "/autonomy/state", self.on_autonomy_state, 20)
        self.create_subscription(String, "/perception/buoys", self.on_buoys, 20)
        self.create_subscription(String, "/perception/obstacles", self.on_obstacles, 20)
        self.create_subscription(String, "/logging/status", self.on_logging_status, 10)
        self._last_target_color: Optional[str] = None
        self._last_published_waypoints: list = []

        self.create_subscription(Twist, "/control/cmd_vel_body", self.on_cmd, 20)

        if self.dry_run:
            self.get_logger().warn("MAVSDK bridge running in dry_run mode")
        else:
            # MAVSDK System() kendi asyncio loop'unu kurar; run_coroutine_threadsafe
            # farklı loop'ta coroutine çalıştırınca connect() asılı kalır (sessiz).
            # Çözüm: kendi thread'inde asyncio.run() — minimal testle aynı pattern.
            self._mavsdk_thread = threading.Thread(target=self._run_mavsdk, daemon=True)
            self._mavsdk_thread.start()
            self.get_logger().info(f"MAVSDK bridge connecting to {self.system_address}")

    def _run_mavsdk(self) -> None:
        """MAVSDK'yı kendi asyncio loop'unda çalıştırır (thread).

        asyncio.run() KULLANILMAZ: mavsdk_task telemetri/poll/komut task'larını
        create_task ile başlatıp DÖNER; asyncio.run sonra loop'u kapatırken o
        task'ları iptal eder (_cancel_all_tasks) — telemetri ölür. Bu yüzden loop
        elle çalıştırılır ve asla kapatılmaz; task'lar process ömrü boyunca yaşar.
        """
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.mavsdk_task())
            # mavsdk_task tamamlandı (komut yolu başlatıldı veya bağlantı hatası);
            # arka plan task'ları loop ile yaşar. Asla loop.close() YAPILMAZ.
            loop.run_forever()
        except Exception as exc:
            self._health.mark_disconnected()
            self.get_logger().error(f"MAVSDK task hatası: {exc}")

    def on_cmd(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def destroy_node(self):
        # Bench RC override, process kapanırken de explicit neutral ile biter.
        # ArduPilot RC override timeout'u ikinci emniyet katmanıdır.
        with self._pymavlink_lock:
            self._neutralize_bench_rc(self._pymavlink_conn)
        return super().destroy_node()

    def publish_dry_status(self) -> None:
        """Backward-compatible entry point; payload is now structured JSON."""
        self.publish_status()

    def publish_status(self) -> None:
        now_mono = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds / 1e9
        payload = {
            "stamp": ros_now,
            **self._health.snapshot(now_mono, ros_now),
        }
        msg = String()
        msg.data = strict_json(payload)
        self.status_pub.publish(msg)

    def publish_telemetry(self) -> None:
        """En güncel telemetriyi /telemetry/state'e yayınlar (rclpy timer, 10Hz)."""
        with self._telemetry_lock:
            state = dict(self._telemetry_state)
        ros_now = self.get_clock().now().nanoseconds / 1e9
        health = self._health.snapshot(time.monotonic(), ros_now)
        # Before all real telemetry streams have produced an acquisition, emit
        # nothing. A timer timestamp must never make default zeros look fresh.
        if not health["connected"] or health["acquisition_stamp"] is None:
            return
        msg = String()
        msg.data = strict_json(telemetry_payload(state, health["acquisition_stamp"]))
        self.telemetry_pub.publish(msg)

    def _update_telemetry(self, source: str, **kwargs) -> None:
        """Async telemetri görevlerinden çağrılır (thread-safe)."""
        with self._telemetry_lock:
            candidate = {**self._telemetry_state, **kwargs}
        acquisition_stamp = self.get_clock().now().nanoseconds / 1e9
        try:
            telemetry_payload(candidate, acquisition_stamp)
        except (TypeError, ValueError, OverflowError) as exc:
            self._health.invalidate_telemetry()
            self.get_logger().warn(f"Geçersiz telemetri reddedildi: {exc}")
            return
        with self._telemetry_lock:
            self._telemetry_state = candidate
        self._health.mark_telemetry(source, acquisition_stamp, time.monotonic())

    def timed_command(self) -> Twist:
        age = (self.get_clock().now() - self.latest_cmd_time).nanoseconds / 1e9
        if age <= self.command_timeout:
            return self.latest_cmd
        return Twist()

    async def mavsdk_task(self) -> None:
        from mavsdk import System

        drone = System()
        try:
            await drone.connect(system_address=self.system_address)
        except Exception as exc:
            self.get_logger().error(f"MAVSDK bağlantı hatası: {exc}")
            return
        connection_states = drone.core.connection_state()
        async for state in connection_states:
            if state.is_connected:
                self._health.mark_connection()
                self.get_logger().info("MAVSDK bağlandı (Pixhawk)")
                break
        # Keep consuming the same stream so a later disconnect cannot leave the
        # health contract latched connected forever.
        asyncio.create_task(self._monitor_connection_state(connection_states))

        # Telemetri: position/attitude/velocity'i ayrı task'larda tüket (sonsuz
        # döngüler — gather ile beklenmez, create_task ile arka planda çalışır).
        asyncio.create_task(self._telemetry_position(drone))
        asyncio.create_task(self._telemetry_attitude(drone))
        asyncio.create_task(self._telemetry_velocity(drone))
        asyncio.create_task(self._telemetry_armed(drone))
        # YKİ'den gelen hedef rengini configured canonical parametreden poll et.
        asyncio.create_task(self._poll_target_color(drone))
        # Görev noktaları Pixhawk'ta (YKİ upload); mission_raw ile indir ve yayınla.
        if self.mission_raw_enabled:
            asyncio.create_task(self._poll_mission(drone))
        else:
            self.get_logger().info("mission_raw devre dışı; /mission/waypoints yayınlanmayacak")

        # Komut yolu: MAVSDK offboard DEĞİL — ArduRover'da desteklenmez
        # (OffboardError -> tüm MAVSDK task'ları iptal olurdu). ArduRover'ın
        # native GUIDED + SET_POSITION_TARGET_LOCAL_NED yolu pymavlink ile.
        # Telemetri/poll task'ları pymavlink bağlantısından ÖNCE başlatılır:
        # bağlantı hatası (port yok, pymavlink eksik) yalnız komut yolunu öldürür,
        # telemetri/poll task'ları yaşar. to_thread: wait_heartbeat BLOCKING'dir
        # (guided_timeout_s kadar) — asyncio loop'unda direkt çağrı telemetri
        # task'larını dondurabilir, bu yüzden executor thread'inde koşar.
        try:
            conn = await asyncio.to_thread(self._connect_pymavlink)
        except Exception as exc:
            self.get_logger().error(f"pymavlink bağlantı hatası: {exc}; telemetri devam ediyor")
            return
        self._set_pymavlink_conn(conn)
        if not self.motor_command_enabled:
            self.get_logger().warn(
                "Motor komut yolu kapalı; pymavlink yalnız YKİ durum telemetrisi gönderiyor"
            )
            return
        if self.vehicle_setup_enabled:
            self._configure_vehicle(conn)
        if self.actuation_backend == "guided_velocity":
            if not self.guided_mode_enabled:
                self.get_logger().warn(
                    "GUIDED mod izni kapalı; motor komut döngüsü başlatılmadı"
                )
                return
            self._set_guided_mode(conn)
        else:
            self.get_logger().warn(
                "BENCH RC OVERRIDE etkin: yalnızca sınırlı ileri pulse, "
                "GPS/EKF bench testi; production navigasyonda kullanılmaz"
            )
        asyncio.create_task(self._command_loop_pymavlink(conn))

    async def _monitor_connection_state(self, connection_states) -> None:
        async for state in connection_states:
            if state.is_connected:
                self._health.mark_connection()
            else:
                self._health.mark_disconnected()
                self.get_logger().warn("MAVSDK bağlantısı kesildi")

    def _connect_pymavlink(self):
        """pymavlink bağlantısını kurar (MAVSDK'tan AYRI port).

        Import patlarsa veya heartbeat gelmezse telemetriyi öldürme — komut ve
        YKİ durum yolu düşer, MAVSDK telemetrisi devam eder.
        mavsdk_task telemetri/poll task'larını ÖNCE başlatır; bu fonksiyon başarısız
        olursa (port yok, pymavlink eksik) yalnız komut yolu düşer, telemetri yaşar.

        Dönüş: bağlantı nesnesi. Hata durumunda istisna yükseltir (çağıran
        mavsdk_task yakalar; telemetri task'ları etkilenmez).
        """
        from pymavlink import mavutil

        conn = mavutil.mavlink_connection(
            self.pymavlink_address,
            baud=self.pymavlink_baud,
            source_system=self.companion_system_id,
            source_component=self.companion_component_id,
            autoreconnect=True,
        )
        conn.wait_heartbeat(timeout=self.guided_timeout)
        # MAVProxy/mavlink-router aynı UDP ucuna kendi GCS heartbeat'ini de
        # yollar. Bu heartbeat autopilot değildir ve target_component=0
        # bırakabilir. Komut yolunu ancak gerçek vehicle/autopilot heartbeat'i
        # ile aç; aksi halde hedef 0'a GUIDED/hız komutu gönderilebilir.
        if not conn.target_system or not conn.target_component:
            deadline = time.monotonic() + self.guided_timeout
            while time.monotonic() < deadline:
                remaining = max(0.0, deadline - time.monotonic())
                heartbeat = conn.recv_match(
                    type="HEARTBEAT",
                    blocking=True,
                    timeout=min(1.0, remaining),
                )
                if heartbeat is None:
                    continue
                source_system = int(heartbeat.get_srcSystem())
                source_component = int(heartbeat.get_srcComponent())
                autopilot = int(getattr(heartbeat, "autopilot", 8))
                if source_system > 0 and source_component > 0 and autopilot != 8:
                    conn.target_system = source_system
                    conn.target_component = source_component
                    break
        if not conn.target_system or not conn.target_component:
            raise ConnectionError("pymavlink vehicle heartbeat alınamadı")
        self.get_logger().info(f"pymavlink bağlandı ({self.pymavlink_address})")
        return conn

    def _configure_vehicle(self, conn) -> None:
        """Explicit opt-in servo/frame setup for the selected hardware outputs."""
        from pymavlink import mavutil

        values = [
            (f"SERVO{self.left_motor_servo_channel}_FUNCTION", 73.0),
            (f"SERVO{self.right_motor_servo_channel}_FUNCTION", 74.0),
            ("FRAME_CLASS", float(self.vehicle_frame_class)),
        ]
        for name, value in values:
            conn.mav.param_set_send(
                conn.target_system,
                conn.target_component,
                name.encode(),
                value,
                mavutil.mavlink.MAV_PARAM_TYPE_INT8,
            )
            time.sleep(0.2)
        self.get_logger().info(
            "Araç kurulumu yazıldı "
            f"(SERVO{self.left_motor_servo_channel}=73, "
            f"SERVO{self.right_motor_servo_channel}=74, "
            f"FRAME_CLASS={self.vehicle_frame_class})"
        )

    def _set_guided_mode(self, conn) -> None:
        """ArduRover'ı GUIDED (15) moduna geçirir (velocity komutu için).

        Komut yolu SET_POSITION_TARGET_LOCAL_NED (velocity + yaw_rate): ArduRover
        kendi hız kontrolcüsüyle vx/yaw'ı uygular. Servo/frame parametreleri
        bu fonksiyonda yazılmaz; yalnız ayrı vehicle_setup_enabled kapısına aittir.
        GUIDED = 15 (Rover). Geçiş set_mode_send(custom_mode=15) ile; yedekte
        MAV_CMD_DO_SET_MODE (param2=15).
        """
        from pymavlink import mavutil

        try:
            with self._mav_source_system(conn, self.actuation_source_system_id):
                conn.mav.set_mode_send(
                    conn.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    15,  # GUIDED (Rover)
                )
            self.get_logger().info("GUIDED (15) moduna geçildi (set_mode_send)")
            return
        except Exception as exc:
            self.get_logger().warn(f"set_mode_send GUIDED başarısız ({exc}); MAV_CMD_DO_SET_MODE denenecek")
        # Yedek: MAV_CMD_DO_SET_MODE (param1=1 custom enabled, param2=15 GUIDED).
        # B2: eski kod param2=0 (MANUAL) gönderiyordu — velocity komutları yutulur,
        # araç tepkisiz kalır. GUIDED (15) hedeflenir; log da gerçek değeri söyler.
        try:
            with self._mav_source_system(conn, self.actuation_source_system_id):
                conn.mav.command_long_send(1, 1, 176, 0, 1, 15, 0, 0, 0, 0, 0)
            self.get_logger().info("GUIDED (15) moduna geçildi (MAV_CMD_DO_SET_MODE yedek)")
        except Exception as exc:
            self.get_logger().error(f"GUIDED mod geçişi başarısız: {exc}")

    @contextmanager
    def _mav_source_system(self, conn, source_system: int):
        """Temporarily emit one MAVLink operation from an explicit SYS_ID."""
        previous_source_system = getattr(conn.mav, "srcSystem", None)
        try:
            conn.mav.srcSystem = source_system
            yield
        finally:
            if previous_source_system is not None:
                conn.mav.srcSystem = previous_source_system

    def _bench_rc_pwm(self, cmd: Twist) -> tuple[int, int]:
        """Return fail-closed steering/throttle PWM for stationary bench use.

        Bench backend is intentionally forward-only and straight-only. Any
        malformed, reverse or yaw command becomes neutral instead of being
        clamped into an unexpected physical movement.
        """
        forward = float(cmd.linear.x)
        yaw_rate = float(cmd.angular.z)
        if (
            not math.isfinite(forward)
            or not math.isfinite(yaw_rate)
            or forward < 0.0
            or abs(yaw_rate) > 1e-6
        ):
            return self.bench_rc_neutral_pwm, self.bench_rc_neutral_pwm
        ratio = min(forward, self.bench_rc_max_forward_mps) / self.bench_rc_max_forward_mps
        throttle = self.bench_rc_neutral_pwm + int(round(ratio * self.bench_rc_max_delta_pwm))
        return self.bench_rc_neutral_pwm, throttle

    def _send_bench_rc_override(self, conn, cmd: Twist) -> None:
        steering_pwm, throttle_pwm = self._bench_rc_pwm(cmd)
        channels = [65535] * 8  # UINT16_MAX: untouched channel
        channels[self.bench_rc_steering_channel - 1] = steering_pwm
        channels[self.bench_rc_throttle_channel - 1] = throttle_pwm
        # SYSID_MYGCS is 255 on the bench Pixhawk.  The connection itself must
        # remain companion_system_id=1 because its status packets are routed to
        # the YKI as vehicle telemetry.  Change only this packet's MAVLink
        # source and restore it even if serialization/send fails.
        with self._mav_source_system(conn, self.bench_rc_source_system_id):
            conn.mav.rc_channels_override_send(
                conn.target_system,
                conn.target_component,
                *channels,
            )

    def _neutralize_bench_rc(self, conn) -> None:
        if self.actuation_backend != "bench_rc_override" or conn is None:
            return
        try:
            self._send_bench_rc_override(conn, Twist())
        except Exception as exc:
            self.get_logger().error(f"Bench RC nötr gönderilemedi: {exc}")

    async def _command_loop_pymavlink(self, conn, max_iterations: Optional[int] = None) -> None:
        """GUIDED modda SET_POSITION_TARGET_LOCAL_NED ile hız komutu gönderir.

        ArduRover kendi hız kontrolcüsüyle vx/yaw_rate'i uygular (RC override PWM
        kaba hız veriyordu — 0.3 m/s komutu 4.47 m/s olabiliyordu). Skid
        yapılandırması (gerçek araçta SERVO9/11) + motorboat-skid frame ile GUIDED velocity
        çalışır. ArduRover'ın 0x05E7 speed + turn-rate dalında
        cmd.linear.x -> signed forward speed, cmd.angular.z -> yaw_rate olarak
        gönderilir. Bu dal vx'i NED north bileşeni olarak yorumlamaz.

        Bağlantı dayanıklılığı: gönderim hatası (seri hat kopması, port çakışması)
        task'ı sessizce öldürmez — hata loglanır, ``command_retry_s`` kadar beklenir,
        sonra bağlantı yeniden kurulur (``_connect_pymavlink`` + ``_set_guided_mode``).
        Yeniden bağlanma başarısızsa hata loglanır ve bir sonraki turda tekrar dener
        (döngü canlı kalır; bu süre boyunca araç ArduPilot kendi komut timeout'uyla
        durur — güvenlik katmanı komut kaybında devrededir).

        ``max_iterations`` yalnız testler içindir (sonlu döngü); prod'da None
        (rclpy.ok() ile sonsuz).
        """

        period = 1.0 / self.send_hz
        iteration = 0
        while rclpy.ok():
            if max_iterations is not None and iteration >= max_iterations:
                break
            iteration += 1
            cmd = self.timed_command()
            try:
                # GUIDED + SET_POSITION_TARGET_LOCAL_NED (velocity + yaw_rate):
                # ArduRover kendi hız kontrolcüsüyle vx/yaw'ı uygular (RC override
                # PWM kaba hız veriyordu — 0.3 komutu 4.47 m/s olabiliyordu).
                # cmd.linear.x (ileri m/s) -> signed speed, cmd.angular.z
                # (yaw rad/s) -> turn rate.
                # KRİTİK (araştırma): ArduPilot Rover GUIDED mask bitleri —
                # bit10=YAW_IGNORE(0x400), bit11=YAW_RATE_IGNORE(0x800).
                #   0x3C7 -> yaw/yaw_rate etkin ama handler'ın hiçbir daliyla eşleşmez (komut yutulur).
                #   0xFC7 -> velocity-only (yaw_rate ignore) -> çalışır ama yaw_rate kaybolur.
                #   0x05E7 (1511) -> Velocity + Yaw Rate -> handler Dal 2 (turn_rate+speed).
                # Velocity + yaw_rate birlikte için 0x05E7 kullanılır (dokümantasyon).
                # LOCAL_NED frame ArduRover'da çalışır; BODY_NED daha önce
                # güvenilir bulunmadı. Kritik ArduRover ayrıntısı: 0x05E7
                # maskesinin speed + turn-rate dalı vx'i NED north bileşeni değil,
                # signed forward speed olarak kullanır; vy dikkate alınmaz. Bu nedenle
                # vx'i heading ile döndürmek, cos(heading)<0 iken pozitif ileri komutu
                # negatif hıza çevirip aracı geri sürer. Lateral komut diferansiyel
                # tekne kontratında zaten limiter tarafından sıfırlanır.
                from pymavlink import mavutil as _mavutil

                with self._pymavlink_lock:
                    if self.actuation_backend == "bench_rc_override":
                        self._send_bench_rc_override(conn, cmd)
                    else:
                        forward_speed = float(cmd.linear.x)
                        with self._mav_source_system(
                            conn, self.actuation_source_system_id
                        ):
                            conn.mav.set_position_target_local_ned_send(
                                0,
                                conn.target_system,
                                conn.target_component,
                                _mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                ROVER_SPEED_TURN_RATE_MASK,
                                0, 0, 0,  # konum ignore
                                forward_speed, 0.0, 0.0,  # vx=signed forward speed; vy/vz unused
                                0, 0, 0,  # ivme ignore
                                0, cmd.angular.z,  # yaw ignore, yaw_rate
                            )
            except Exception as exc:
                self.get_logger().error(f"SET_POSITION_TARGET gönderim hatası: {exc}")
                with self._pymavlink_lock:
                    if self._pymavlink_conn is conn:
                        self._pymavlink_conn = None
                # command_retry_s kadar bekle, sonra bağlantıyı yeniden kur.
                await asyncio.sleep(max(0.0, self.command_retry))
                try:
                    conn = await asyncio.to_thread(self._connect_pymavlink)
                    if self.vehicle_setup_enabled:
                        self._configure_vehicle(conn)
                    if (
                        self.actuation_backend == "guided_velocity"
                        and self.guided_mode_enabled
                    ):
                        self._set_guided_mode(conn)
                    self._set_pymavlink_conn(conn)
                    self.get_logger().info("pymavlink bağlantısı yeniden kuruldu")
                except Exception as exc2:
                    self.get_logger().error(f"pymavlink yeniden bağlanma hatası: {exc2}")
                await asyncio.sleep(period)
                continue
            await asyncio.sleep(period)

    # --- Telemetri async görevleri (Pixhawk -> /telemetry/state) ---

    async def _telemetry_position(self, drone) -> None:
        async for p in drone.telemetry.position():
            self._update_telemetry("position", lat=p.latitude_deg, lon=p.longitude_deg)

    async def _telemetry_attitude(self, drone) -> None:
        async for a in drone.telemetry.attitude_euler():
            self._update_telemetry(
                "attitude",
                heading_deg=float(a.yaw_deg),
                roll_deg=float(a.roll_deg),
                pitch_deg=float(a.pitch_deg),
            )

    async def _telemetry_velocity(self, drone) -> None:
        async for v in drone.telemetry.velocity_ned():
            ground_speed = math.sqrt(v.north_m_s ** 2 + v.east_m_s ** 2)
            self._update_telemetry("velocity", ground_speed=ground_speed)

    async def _telemetry_armed(self, drone) -> None:
        async for armed in drone.telemetry.armed():
            self._update_telemetry("armed", mode="ARMED" if armed else "DISARMED")

    # --- YKİ'den gelen hedef rengi: Pixhawk SCR_USER4 parametresi (poll) ---------

    async def _poll_target_color(self, drone) -> None:
        """Configured target-color parametresini periyodik okuyup ROS'a yayınlar.

        YKİ send_target_info -> PARAM_SET SCR_USER4 (FLOAT, değer: 1/2/4). ArduPilot
        parametre tablosu USB dahil tüm portlardan okunur; RFD forward'ı gerekmez.
        Değer değişince yayınlanır (spam önleme). Param okunamazsa (bağlantı kopması,
        parametre yoksa sessizce yok sayılır; bir sonraki poll yeniden dener.
        """
        while rclpy.ok():
            value = await self._read_mavsdk_param(drone, self.target_color_param)
            if value is not None:
                self._handle_target_color(int(round(value)))
                self._param_consecutive_failures = 0
            else:
                self._param_consecutive_failures += 1
                if self._param_consecutive_failures % max(1, self.param_consecutive_fail_limit) == 0:
                    self.get_logger().warn(
                        f"{self.target_color_param} {self._param_consecutive_failures} ardışık okunamadı"
                    )
            await asyncio.sleep(1.0 / max(0.1, self.target_color_poll_hz))

    async def _read_mavsdk_param(self, drone, name: str) -> float | None:
        """Read one finite parameter with bounded retries and serialized calls."""
        async with self._mavsdk_param_lock:
            for _attempt in range(max(1, self.param_max_retries)):
                try:
                    value = await asyncio.wait_for(
                        drone.param.get_param_float(name), timeout=self.param_call_timeout
                    )
                except Exception:
                    try:
                        value = await asyncio.wait_for(
                            drone.param.get_param_int(name), timeout=self.param_call_timeout
                        )
                    except Exception:
                        continue
                try:
                    numeric = float(value)
                except (TypeError, ValueError, OverflowError):
                    continue
                if math.isfinite(numeric):
                    return numeric
        return None

    def _handle_target_color(self, value: int) -> None:
        """Param/MAVLink renk kodunu kontrat rengine çevir (YKİ: 1=KIRMIZI,2=YEŞİL,4=SİYAH)."""
        from ida_control.mission_contract import map_target_color

        color = map_target_color(int(value))
        if color is None or color == self._last_target_color:
            return
        self._last_target_color = color
        msg = String()
        msg.data = (
            '{"stamp": %.3f, "target_color": "%s", "source": "mavsdk_bridge"}'
            % (self.get_clock().now().nanoseconds / 1e9, color)
        )
        self.target_pub.publish(msg)
        self._update_yki_values({"TGT_ACK": int(value)})
        self.get_logger().info(
            f"Hedef rengi alındı (Pixhawk {self.target_color_param}): {color} (mav={value})"
        )

    # --- İDA -> YKİ compact status (gateway-free, same pymavlink endpoint) --

    def _set_pymavlink_conn(self, conn) -> None:
        with self._pymavlink_lock:
            self._pymavlink_conn = conn
            self._yki_last_sent.clear()
            self._yki_last_heartbeat = 0.0

    def _update_yki_values(self, values: dict[str, int]) -> None:
        with self._pymavlink_lock:
            for name, value in values.items():
                if 1 <= len(name) <= 10:
                    self._yki_values[name] = int(value)
        self.flush_yki_status()

    def flush_yki_status(self) -> None:
        now = time.monotonic()
        with self._pymavlink_lock:
            conn = self._pymavlink_conn
            if conn is None:
                return
            heartbeat_due = now - self._yki_last_heartbeat >= self.yki_status_heartbeat
            for name in sorted(self._yki_values):
                value = self._yki_values[name]
                if not heartbeat_due and self._yki_last_sent.get(name) == value:
                    continue
                try:
                    conn.mav.named_value_int_send(
                        int(now * 1000) & 0xFFFFFFFF,
                        name.encode("ascii"),
                        int(value),
                    )
                    self._yki_last_sent[name] = value
                except Exception as exc:
                    self.get_logger().warn(f"YKİ durum alanı gönderilemedi ({name}): {exc}")
                    return
            if heartbeat_due:
                self._yki_last_heartbeat = now

    @staticmethod
    def _json_object(msg: String) -> dict:
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return {}
        return value if isinstance(value, dict) else {}

    def on_autonomy_state(self, msg: String) -> None:
        self._update_yki_values(autonomy_fields(self._json_object(msg)))

    def on_buoys(self, msg: String) -> None:
        self._update_yki_values({"PERC_DET": perception_count(self._json_object(msg), "detections")})

    def on_obstacles(self, msg: String) -> None:
        self._update_yki_values({"PERC_OBS": perception_count(self._json_object(msg), "obstacles")})

    def on_logging_status(self, msg: String) -> None:
        self._update_yki_values(logging_fields(self._json_object(msg)))

    # --- Görev noktaları: Pixhawk'tan mission_raw download ----------------------

    async def _poll_mission(self, drone) -> None:
        """Görev noktalarını Pixhawk'tan indirip /mission/waypoints'e yayınlar.

        YKİ mission upload'u zaten Pixhawk'a gider (mission_item_int, çalışıyor).
        Jetson MAVSDK mission_raw.download_mission() ile USB üzerinden indirir.
        YKİ değişmeden waypoint'ler aynıysa yayın tekrarlanmaz (spam önleme).
        """
        while rclpy.ok():
            try:
                items = await drone.mission_raw.download_mission()
                p1_raw = await self._read_mavsdk_param(
                    drone, self.mission_p1_count_param
                )
                p2_raw = await self._read_mavsdk_param(
                    drone, self.mission_p2_count_param
                )
                p1_count = self._exact_nonnegative_count(p1_raw, require_positive=True)
                p2_count = self._exact_nonnegative_count(p2_raw, require_positive=False)
                if p1_count is None or p2_count is None:
                    self.get_logger().warn(
                        f"Mission parkur metadata ({self.mission_p1_count_param}/"
                        f"{self.mission_p2_count_param}) geçersiz; görev yayınlanmadı"
                    )
                    await asyncio.sleep(1.0 / max(0.1, self.mission_download_hz))
                    continue
                waypoints = self._mission_items_to_waypoints(
                    items, p1_count=p1_count, p2_count=p2_count
                )
                if waypoints and waypoints != self._last_published_waypoints:
                    msg = String()
                    msg.data = self._serialize_waypoints(
                        waypoints,
                        self.get_clock().now().nanoseconds / 1e9,
                    )
                    self._last_published_waypoints = waypoints
                    self.waypoint_pub.publish(msg)
                    self.get_logger().info(f"Görev Pixhawk'tan indirildi ({len(waypoints)} waypoint)")
            except Exception as exc:
                self.get_logger().debug(f"mission download başarısız: {exc}")
            await asyncio.sleep(1.0 / max(0.1, self.mission_download_hz))

    @staticmethod
    def _serialize_waypoints(waypoints: list, stamp: float) -> str:
        """Return the mission topic payload as strict JSON.

        Python's list/dict representation uses single quotes and is not JSON;
        consumers call ``json.loads`` on this topic, so serialization must be
        explicit and standards compliant.
        """
        return json.dumps(
            {"stamp": float(stamp), "waypoints": waypoints},
            allow_nan=False,
            separators=(",", ":"),
        )

    @staticmethod
    def _mission_items_to_waypoints(
        items, p1_count: int | None = None, p2_count: int | None = None
    ) -> list:
        """MAVSDK MissionItem listesini README waypoint kontratına çevirir.

        Saf dönüşüm mission_contract.mission_items_to_waypoints'ta (rclpy'siz
        test edilebilir). MissionItem.x = lat * 1e7, .y = lon * 1e7 (int32),
        tıpkı MISSION_ITEM_INT.
        """
        from ida_control.mission_contract import mission_items_to_waypoints

        return mission_items_to_waypoints(
            items, p1_count=p1_count, p2_count=p2_count
        )

    @staticmethod
    def _exact_nonnegative_count(
        value: float | None, *, require_positive: bool
    ) -> int | None:
        if value is None or isinstance(value, bool) or not math.isfinite(value):
            return None
        rounded = int(round(value))
        minimum = 1 if require_positive else 0
        if abs(value - rounded) > 1e-6 or not minimum <= rounded <= 1000:
            return None
        return rounded

    def _exact_int_parameter(self, name: str, minimum: int, maximum: int) -> int:
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{name} must be an exact integer")
        if not minimum <= value <= maximum:
            raise ValueError(f"{name} must be in [{minimum},{maximum}]")
        return value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MavsdkBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
