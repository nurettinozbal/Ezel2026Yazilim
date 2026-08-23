"""MAVLink -> ROS2 köprü düğümü (ida_ws_gateway).

MAVLink NAMED_VALUE_INT (hedef rengi) ve MISSION_ITEM_INT (görev noktaları)
mesajlarını README kontratlarındaki /mission/target_color ve /mission/waypoints
std_msgs/String JSON topic'lerine çevirir. Yayınlar transient_local QoS ile
yapılır: hedef rengi hareket öncesi bir kez gelir, geç bağlanan subscriber'lar
bu mesajı kaçırmasın.

HABERLEŞME ŞEMASI (SAHA_TEST_PLANI.md): İDA'da RFD → Pixhawk'a takılı; YKİ
arayüzü RFD ile Pixhawk'a komut gönderir (ARM, mod, mission, hedef rengi).
Bu düğüm Pixhawk'ın Jetson'a giden USB/seri portunu dinler (``serial_port``
parametresi, fallback /dev/pixhawk) — YKİ'den gelen TARGET_COLOR ve
MISSION_ITEM_INT mesajlarını ROS2 topic'lerine çevirir. Aynı hattı MAVSDK
köprüsü (ida_control) GUIDED komutları için kullanır.

pymavlink opsiyonel bir bağımlılıktır; kurulu değilse düğüm dry_run modunda
çalışır. Varsayılan parametrelerle herhangi bir dev makinesinde rclpy olduğu
sürece başlatılabilir.
"""

import json
import math
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from ida_planning.contracts import dumps, loads
from ida_ws_gateway.dry_run_link import DryRunLink, parse_demo_waypoints
from ida_ws_gateway.mavlink_parser import (
    DEFAULT_COLOR_INT_MAP,
    PERC_OBS_OFFSET,
    STATE_CODE_MAP,
    encode_action,
    encode_log_count,
    encode_parkur_from_state,
    encode_perception_count,
    extract_named_and_mission,
    is_target_color_field,
)

# Uzak yayıncının geç bağlanan subscriber'a mesaj iletmesi için transient_local.
_TRANSIENT_LOCAL_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class WsGatewayNode(Node):
    """MAVLink köprüsü: hedef rengi ve görev noktalarını ROS2'ye aktarır."""

    def __init__(self) -> None:
        super().__init__("ida_ws_gateway")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("serial_port", "")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("target_color_param_id", "TARGET_COLOR")
        self.declare_parameter("waypoint_publish_hz", 1.0)
        self.declare_parameter("demo_target_color", "green")
        self.declare_parameter("demo_waypoints", "")
        self.declare_parameter("heartbeat_timeout_s", 5.0)
        self.declare_parameter("color_int_map", '{"1": "red", "2": "green", "3": "orange", "4": "black", "5": "yellow"}')
        # Dry-run demo: İDA otonomi durumu adım adım (saha testi için).
        self.declare_parameter("demo_autonomy_script", "")
        self.declare_parameter("demo_step_hz", 1.0)

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.target_color_param_id = str(self.get_parameter("target_color_param_id").value)
        self.waypoint_publish_hz = float(self.get_parameter("waypoint_publish_hz").value)
        self.demo_target_color = str(self.get_parameter("demo_target_color").value).lower()
        self.heartbeat_timeout = float(self.get_parameter("heartbeat_timeout_s").value)
        self.color_int_map = self._parse_color_int_map(str(self.get_parameter("color_int_map").value))
        self.demo_autonomy_script = self._parse_demo_autonomy_script(
            str(self.get_parameter("demo_autonomy_script").value)
        )
        self.demo_step_hz = float(self.get_parameter("demo_step_hz").value)
        self._demo_step_index = 0

        self.target_pub = self.create_publisher(
            String, "/mission/target_color", qos_profile=_TRANSIENT_LOCAL_QOS
        )
        self.waypoint_pub = self.create_publisher(
            String, "/mission/waypoints", qos_profile=_TRANSIENT_LOCAL_QOS
        )

        # İDA otonomi durumu -> YKİ NAMED_VALUE_INT akışı (yalnız değişimde yazılır).
        self.create_subscription(
            String, "/autonomy/state", self.on_autonomy_state, qos_profile=_TRANSIENT_LOCAL_QOS
        )
        self.create_subscription(String, "/perception/buoys", self.on_buoys, 20)
        self.create_subscription(String, "/perception/obstacles", self.on_obstacles, 20)
        self.create_subscription(
            String, "/logging/status", self.on_logging_status, qos_profile=_TRANSIENT_LOCAL_QOS
        )

        # Değişim tespiti (spam önleme): ilk değer None -> her alan ilk mesajda gönderilir.
        self._last_reported_state: Optional[int] = None
        self._last_reported_parkur: Optional[int] = None
        self._last_reported_action: Optional[int] = None
        self._last_reported_det: Optional[int] = None
        self._last_reported_obs: Optional[int] = None
        self._last_reported_log_active: Optional[int] = None
        self._last_reported_log_count: Optional[int] = None

        # Gerçek MAVLink bağlantısı (dry_run'da None kalır). _write_named_value ve
        # _recv_real_messages bunu kontrol eder; başlatılmazsa AttributeError verir.
        self._conn: Optional[Any] = None
        self._mavutil: Optional[Any] = None

        self.last_heartbeat = self.get_clock().now()
        self._link: Optional[DryRunLink] = None
        self._mavutil = None
        self._setup_link()
        self.create_timer(1.0 / max(0.1, self.waypoint_publish_hz), self.tick)
        self.create_timer(1.0, self.check_heartbeat)
        if self.demo_autonomy_script:
            self.create_timer(1.0 / max(0.1, self.demo_step_hz), self.demo_step)
        self.get_logger().info(
            f"WS gateway started (dry_run={self.dry_run}, "
            f"target_param={self.target_color_param_id})"
        )

    @staticmethod
    def _parse_color_int_map(raw: str) -> Dict[str, str]:
        """JSON dizeyi {int_str: color} sözlüğüne çevirir; hatalı giriş boş olur."""
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            return {}
        if not isinstance(parsed, dict):
            return {}
        return {str(k): str(v).lower() for k, v in parsed.items()}

    @staticmethod
    def _parse_demo_autonomy_script(raw: str) -> List[Dict[str, Any]]:
        """demo_autonomy_script JSON liste -> adım listesi; geçersiz giriş boş."""
        if not raw or not raw.strip():
            return []
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            return []
        if not isinstance(parsed, list):
            return []
        steps = []
        for item in parsed:
            if isinstance(item, dict):
                steps.append(item)
        return steps

    def _setup_link(self) -> None:
        """Gerçek (pymavlink) veya simüle (dry_run) bağlantıyı kurar."""
        if self.dry_run:
            self._link = DryRunLink(
                target_color_param_id=self.target_color_param_id,
                demo_target_color=self.demo_target_color,
                demo_waypoints=parse_demo_waypoints(str(self.get_parameter("demo_waypoints").value)),
                color_int_map=self.color_int_map,
            )
            self._link.connect()
            self.get_logger().warn("WS gateway running in dry_run mode (simulated MAVLink)")
            return
        # Gerçek pymavlink: import patlarsa dry_run'a düş, düğüm çalışmaya devam etsin.
        try:
            from pymavlink import mavutil  # type: ignore

            self._mavutil = mavutil
            # İDA tarafında RFD → Pixhawk'a takılı (YKİ arayüzü RFD üzerinden Pixhawk'a
            # komut gönderir). Jetson, Pixhawk'ın USB/seri portunu dinler; YKİ'den RFD ile
            # gelen TARGET_COLOR + MISSION_ITEM_INT mesajları Pixhawk'a ulaşır, Jetson bu
            # porttan okur. Saha öncesi udev ile sabitlenmeli (örn. /dev/pixhawk).
            port = self.serial_port or "/dev/pixhawk"
            self.get_logger().info(f"Connecting MAVLink (İDA Pixhawk) on {port} @ {self.baudrate}")
            self._conn = mavutil.mavlink_connection(port, baud=self.baudrate)
            self._conn.wait_heartbeat(timeout=self.heartbeat_timeout)
            self.last_heartbeat = self.get_clock().now()
            self.get_logger().info("MAVLink heartbeat received")
        except Exception as exc:
            self.get_logger().error(f"pymavlink unavailable ({exc}); falling back to dry_run")
            self.dry_run = True
            self._link = DryRunLink(
                target_color_param_id=self.target_color_param_id,
                demo_target_color=self.demo_target_color,
                demo_waypoints=parse_demo_waypoints(str(self.get_parameter("demo_waypoints").value)),
                color_int_map=self.color_int_map,
            )
            self._link.connect()

    def tick(self) -> None:
        """Yeni MAVLink mesajlarını okuyup ROS2 topic'lerine yayınlar."""
        if self._link is not None:
            raw_messages = self._link.recv_batch()
        else:
            raw_messages = self._recv_real_messages()

        target_color, waypoints = extract_named_and_mission(raw_messages)
        # param_id normalize edilmiş halde gelir; is_target_color_field hem
        # "TARGET_COLOR" hem de char[10] kırpılmış "TARGET_COL" kabul eder.
        if target_color is not None and is_target_color_field(target_color["param_id"]):
            self.publish_target_color(int(target_color["value"]))
        if waypoints:
            self.publish_waypoints(waypoints)
        self._flush_ida_autonomy_status()

    def _write_named_value(self, name: str, value: int) -> None:
        """NAMED_VALUE_INT'i Pixhawk hattına yazar (YKİ _handle_named_value_int okur).

        Yön: Jetson -> Pixhawk -> RFD -> YKİ. Gerçek modda pymavlink deseni
        (mavlink_vehicle.send_target_info ile aynı):
            master.mav.named_value_int_send(time_usec, name, value)
        Tek port-sahibi (K5): bu düğüm portu hem okur hem yazar (full-duplex seri),
        yarışsız — ayrı bir yazıcı düğüm gerekmez.
        """
        if self._conn is not None and self._mavutil is not None:
            try:
                self._conn.mav.named_value_int_send(
                    (self.get_clock().now().nanoseconds // 1000) & 0xFFFFFFFF,
                    name.encode("ascii"),
                    int(value),
                )
                return
            except Exception as exc:
                self.get_logger().warn(f"NAMED_VALUE_INT yazılamadı ({name}): {exc}")
                return
        if self._link is not None and hasattr(self._link, "put_named_value"):
            # Dry-run: sahte hatta yaz, bir sonraki tick'te okunur (hedef rengi akışıyla aynı yol).
            self._link.put_named_value(name, value)
            return
        self.get_logger().debug(f"NAMED_VALUE_INT dry-run (yazıcı yok): {name}={value}")

    def _flush_ida_autonomy_status(self) -> None:
        """İDA otonomi durumunu MAVLink'e yazar (yalnızca değişen alanlar).

        Her tick'te çağrılır; abonelik callback'leri `_last_reported_*`'i
        yalnızca değer değiştiğinde güncellediği için burada yazılan alanlar
        zaten "değişti" demektir. İlk değer kuralı: `_last_reported_*` None
        iken her alan gönderilir — YKİ sonradan bağlandığında tam durumu ilk
        tick'te alır (K4).
        """
        pairs = [
            ("AUTO_ST", self._last_reported_state),
            ("AUTO_AC", self._last_reported_action),
            ("PARKUR", self._last_reported_parkur),
            ("PERC_DET", self._last_reported_det),
            ("PERC_OBS", self._last_reported_obs),
            ("LOG_ACT", self._last_reported_log_active),
            ("LOG_CNT", self._last_reported_log_count),
        ]
        for name, value in pairs:
            if value is not None:
                self._write_named_value(name, value)

    # --- İDA otonomi durumu abonelikleri (yalnızca değişimde yazılır) ----------

    def _on_autonomy_payload(self, payload: Dict[str, Any]) -> None:
        state = str(payload.get("state", "") or "").upper()
        state_code = STATE_CODE_MAP.get(state, 0)
        if state_code != self._last_reported_state:
            self._last_reported_state = state_code
            self._write_named_value("AUTO_ST", state_code)

        parkur = encode_parkur_from_state(state)
        if parkur != self._last_reported_parkur:
            self._last_reported_parkur = parkur
            self._write_named_value("PARKUR", parkur)

        action = encode_action(str(payload.get("action", "") or ""))
        if action != self._last_reported_action:
            self._last_reported_action = action
            self._write_named_value("AUTO_AC", action)

    def on_autonomy_state(self, msg: String) -> None:
        payload = loads(msg.data, None)
        if isinstance(payload, dict):
            self._on_autonomy_payload(payload)

    def on_buoys(self, msg: String) -> None:
        payload = loads(msg.data, {})
        detections = payload.get("detections", []) if isinstance(payload, dict) else []
        count = len(detections) if isinstance(detections, list) else 0
        encoded = encode_perception_count(count)
        if encoded != self._last_reported_det:
            self._last_reported_det = encoded
            self._write_named_value("PERC_DET", encoded)

    def on_obstacles(self, msg: String) -> None:
        payload = loads(msg.data, {})
        obstacles = payload.get("obstacles", []) if isinstance(payload, dict) else []
        count = len(obstacles) if isinstance(obstacles, list) else 0
        encoded = encode_perception_count(count, offset=PERC_OBS_OFFSET)
        if encoded != self._last_reported_obs:
            self._last_reported_obs = encoded
            self._write_named_value("PERC_OBS", encoded)

    def on_logging_status(self, msg: String) -> None:
        payload = loads(msg.data, None)
        if not isinstance(payload, dict):
            return
        active = 1 if payload.get("active", False) else 0
        if active != self._last_reported_log_active:
            self._last_reported_log_active = active
            self._write_named_value("LOG_ACT", active)
        logger_count = 0
        try:
            logger_count = int(payload.get("logger_count", 0))
        except (TypeError, ValueError):
            logger_count = 0
        encoded = encode_log_count(logger_count)
        if encoded != self._last_reported_log_count:
            self._last_reported_log_count = encoded
            self._write_named_value("LOG_CNT", encoded)

    # --- Dry-run demo (saha testi: YKİ paneli tüm durumları canlı görür) -------

    def demo_step(self) -> None:
        if not self.demo_autonomy_script:
            return
        if self._demo_step_index >= len(self.demo_autonomy_script):
            self._demo_step_index = 0  # döngü: panel sürekli canlı veri görür
        step = self.demo_autonomy_script[self._demo_step_index]
        self._demo_step_index += 1
        self._on_autonomy_payload(step)
        if "det" in step:
            encoded = encode_perception_count(step["det"])
            if encoded != self._last_reported_det:
                self._last_reported_det = encoded
                self._write_named_value("PERC_DET", encoded)
        if "obs" in step:
            encoded = encode_perception_count(step["obs"], offset=PERC_OBS_OFFSET)
            if encoded != self._last_reported_obs:
                self._last_reported_obs = encoded
                self._write_named_value("PERC_OBS", encoded)

    def _recv_real_messages(self) -> List[Dict[str, Any]]:
        """Gerçek pymavlink kuyruğundan ham mesaj dict'leri toplar."""
        if self._mavutil is None:
            return []
        messages: List[Dict[str, Any]] = []
        try:
            while True:
                msg = self._conn.recv_match(blocking=False)
                if msg is None:
                    break
                raw = msg.to_dict()
                if "mavpackettype" not in raw:
                    raw["mavpackettype"] = msg.get_type()
                messages.append(raw)
                self.last_heartbeat = self.get_clock().now()
        except Exception:
            # Bağlantı hatası anlık; sonraki tick yeniden dener.
            pass
        return messages

    def publish_target_color(self, int_value: int) -> None:
        """int değeri renk string'ine eşle ve /mission/target_color yayınla.

        Eşleme parametresi verilmezse gerçek YKİ kontratındaki varsayılan tablo
        kullanılır (1=red, 2=green, 3=orange, 4=black, 5=yellow).
        """
        color = self.color_int_map.get(str(int_value)) or DEFAULT_COLOR_INT_MAP.get(str(int_value))
        msg = String()
        msg.data = dumps(
            {
                "stamp": self.now_seconds(),
                "target_color": color,
                "source": "ws_gateway",
                "mav_value": int_value,
            }
        )
        self.target_pub.publish(msg)
        self.get_logger().info(f"Published target color: {color} (mav={int_value})")

    def publish_waypoints(self, waypoints: List[Dict[str, Any]]) -> None:
        """Waypoint listesini /mission/waypoints kontratıyla yayınlar."""
        msg = String()
        msg.data = dumps({"stamp": self.now_seconds(), "waypoints": waypoints})
        self.waypoint_pub.publish(msg)
        self.get_logger().info(f"Published {len(waypoints)} mission waypoints")

    def check_heartbeat(self) -> None:
        """Gerçek bağlantıda heartbeat zaman aşımını izler (dry_run'da pasif)."""
        if self.dry_run or self._mavutil is None:
            return
        age = (self.get_clock().now() - self.last_heartbeat).nanoseconds / 1e9
        if age > self.heartbeat_timeout:
            self.get_logger().error("MAVLink heartbeat timeout")

    def now_seconds(self) -> float:
        """Deterministik zaman damgası: simülasyon/oynatma geri alınabilirliği."""
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WsGatewayNode()
    try:
        rclpy.spin(node)
    finally:
        if node._link is not None:
            node._link.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
