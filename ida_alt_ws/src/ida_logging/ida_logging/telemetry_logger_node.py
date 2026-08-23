"""Telemetri loglayıcı düğümü (ida_telemetry_logger).

/telemetry/state (JSON), /autonomy/state ve sınırlayıcı sonrası
/control/cmd_vel_body (Twist) mesajlarını dinler;
timer ile en az ``log_hz`` (varsayılan 1 Hz) hızında mevcut en son telemetriyi
CSV'ye yazar. Telemetri hiç gelmemişse satır yazmaz (şartname: en az 1 Hz,
telemetri yoksa satır yok).

CSV; konum/yönelim, parkur/waypoint, sınırlanmış hareket setpoint'i ve
varsa SERVO_OUTPUT_RAW motor PWM kanıtını sabit sütunlarla kaydeder.
"""

from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from ida_logging.csv_writer import CsvWriter
from ida_logging.judge_contract import TELEMETRY_CSV_HEADER, build_telemetry_row
from ida_logging.run_dir import make_run_dir, make_run_dir_same
from ida_planning.contracts import loads

class TelemetryLoggerNode(Node):
    """Telemetri + komut setpoint'lerini CSV'ye kaydeder."""

    def __init__(self) -> None:
        super().__init__("ida_telemetry_logger")
        self.declare_parameter("log_dir", "./logs")
        self.declare_parameter("run_name", "")
        self.declare_parameter("log_hz", 1.0)
        self.declare_parameter("flush_every_lines", 100)

        self.log_dir = str(self.get_parameter("log_dir").value)
        self.run_name = str(self.get_parameter("run_name").value).strip()
        self.log_hz = float(self.get_parameter("log_hz").value)
        self.flush_every_lines = int(self.get_parameter("flush_every_lines").value)

        self.telemetry: Optional[Dict[str, Any]] = None
        self.autonomy: Dict[str, Any] = {}
        self.cmd: Optional[Twist] = None

        self.create_subscription(String, "/telemetry/state", self.on_telemetry, 50)
        self.create_subscription(String, "/autonomy/state", self.on_autonomy, 50)
        # Hakem kaydında planlayıcının ham adayı değil, command limiter
        # sonrası gerçek araç setpoint'i tutulur.
        self.create_subscription(Twist, "/control/cmd_vel_body", self.on_cmd, 50)

        run_dir = (
            make_run_dir_same(self.log_dir, self.run_name)
            if self.run_name
            else make_run_dir(self.log_dir)
        )
        self.csv_path = run_dir / "telemetry.csv"
        self._writer = CsvWriter(self.csv_path, TELEMETRY_CSV_HEADER, self.flush_every_lines)

        self.create_timer(1.0 / max(0.1, self.log_hz), self.tick)
        self.get_clock().now()
        self.get_logger().info(f"Telemetry logger started -> {self.csv_path}")
        self._on_shutdown_registered = False

    def on_telemetry(self, msg: String) -> None:
        payload = loads(msg.data, {})
        if isinstance(payload, dict):
            self.telemetry = payload

    def on_cmd(self, msg: Twist) -> None:
        self.cmd = msg

    def on_autonomy(self, msg: String) -> None:
        payload = loads(msg.data, {})
        if isinstance(payload, dict):
            self.autonomy = payload

    def tick(self) -> None:
        """Mevcut son telemetriyi CSV'ye yazar (yoksa boş geçer)."""
        if self.telemetry is None:
            return
        now = self.now_seconds()
        command = {
            "vx": float(self.cmd.linear.x) if self.cmd else 0.0,
            "vy": float(self.cmd.linear.y) if self.cmd else 0.0,
            "yaw_rate": float(self.cmd.angular.z) if self.cmd else 0.0,
        }
        row = build_telemetry_row(self.telemetry, self.autonomy, command, now)
        self._writer.add_row(row)

    @staticmethod
    def _stamp_value(payload: Dict[str, Any], fallback: float) -> Any:
        """Mesajdaki stamp'ı kullan; yoksa (fallback) timer zamanını kullan."""
        stamp = payload.get("stamp")
        if stamp is None:
            return fallback
        try:
            return float(stamp)
        except (TypeError, ValueError):
            return fallback

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_shutdown(self) -> None:
        """rclpy shutdown'da son flush + close (finally ile de güvence altında)."""
        if not self._on_shutdown_registered:
            self._on_shutdown_registered = True
            self._writer.flush()
            self._writer.close()
            self.get_logger().info("Telemetry logger closed")

    def destroy_node(self) -> None:
        self._writer.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryLoggerNode()
    # Timer kazalarında bile dosya kapanır: spin bittikten sonra destroy edilir.
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
