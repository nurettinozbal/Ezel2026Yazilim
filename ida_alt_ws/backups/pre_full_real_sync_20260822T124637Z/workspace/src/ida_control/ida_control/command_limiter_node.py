import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class CommandLimiterNode(Node):
    def __init__(self) -> None:
        super().__init__("ida_command_limiter")
        # B13: default otonomi max_speed 0.6 ile hizalandı (YAML yüklenemezse güvenli tavan).
        self.declare_parameter("max_vx_mps", 0.6)
        self.declare_parameter("max_vy_mps", 0.0)
        self.declare_parameter("max_yaw_rate_rad_s", 0.785398163)
        # Disabled by default. The operator CLI opens this gate only for the
        # duration of one restrained bench pulse and closes it afterwards.
        self.declare_parameter("bench_pulse_enabled", False)
        self.declare_parameter("bench_pulse_max_speed_mps", 0.30)
        self.declare_parameter("bench_pulse_max_duration_s", 6.0)

        self.max_vx = float(self.get_parameter("max_vx_mps").value)
        self.max_vy = float(self.get_parameter("max_vy_mps").value)
        self.max_yaw = float(self.get_parameter("max_yaw_rate_rad_s").value)
        self.bench_pulse_enabled = bool(
            self.get_parameter("bench_pulse_enabled").value
        )
        self.bench_pulse_max_speed = float(
            self.get_parameter("bench_pulse_max_speed_mps").value
        )
        self.bench_pulse_max_duration = float(
            self.get_parameter("bench_pulse_max_duration_s").value
        )
        if not 0.05 <= self.bench_pulse_max_speed <= 0.30:
            raise ValueError("bench_pulse_max_speed_mps must be in [0.05, 0.30]")
        if not 0.5 <= self.bench_pulse_max_duration <= 6.0:
            raise ValueError("bench_pulse_max_duration_s must be in [0.5, 6.0]")

        self._bench_deadline = 0.0
        self._bench_command = Twist()

        self.sub = self.create_subscription(Twist, "/autonomy/cmd_vel_body", self.on_cmd, 20)
        self.bench_sub = self.create_subscription(
            String, "/control/bench_pulse_request", self.on_bench_pulse, 10
        )
        self.pub = self.create_publisher(Twist, "/control/cmd_vel_body", 20)
        self.bench_timer = self.create_timer(0.05, self.on_bench_timer)
        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.get_logger().info("Command limiter started")

    def on_cmd(self, msg: Twist) -> None:
        if time.monotonic() < self._bench_deadline:
            return
        out = Twist()
        out.linear.x = clamp(msg.linear.x, -self.max_vx, self.max_vx)
        out.linear.y = clamp(msg.linear.y, -self.max_vy, self.max_vy) if self.max_vy > 0 else 0.0
        out.angular.z = clamp(msg.angular.z, -self.max_yaw, self.max_yaw)
        self.pub.publish(out)

    def on_set_parameters(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "bench_pulse_enabled":
                if type(parameter.value) is not bool:
                    return SetParametersResult(
                        successful=False, reason="bench_pulse_enabled must be bool"
                    )
                self.bench_pulse_enabled = parameter.value
                if not parameter.value:
                    self._bench_deadline = 0.0
                    self._bench_command = Twist()
                    self.pub.publish(Twist())
        return SetParametersResult(successful=True)

    def on_bench_pulse(self, msg: String) -> None:
        if not self.bench_pulse_enabled:
            self.get_logger().warning("Bench pulse rejected: gate disabled")
            return
        try:
            payload = json.loads(msg.data)
            if set(payload) != {"safety_ack", "speed_mps", "duration_s"}:
                raise ValueError("unexpected request fields")
            if payload["safety_ack"] != "PROPELLER_AREA_CLEAR":
                raise ValueError("invalid safety acknowledgement")
            speed = payload["speed_mps"]
            duration = payload["duration_s"]
            if isinstance(speed, bool) or isinstance(duration, bool):
                raise ValueError("numeric values cannot be bool")
            speed = float(speed)
            duration = float(duration)
            if not math.isfinite(speed) or not math.isfinite(duration):
                raise ValueError("values must be finite")
            if not 0.0 < speed <= self.bench_pulse_max_speed:
                raise ValueError("speed outside bench bound")
            if not 0.5 <= duration <= self.bench_pulse_max_duration:
                raise ValueError("duration outside bench bound")
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Bench pulse rejected: {exc}")
            return

        command = Twist()
        command.linear.x = speed
        self._bench_command = command
        self._bench_deadline = time.monotonic() + duration
        self.get_logger().warning(
            f"RESTRAINED BENCH pulse active: vx={speed:.3f} m/s, duration={duration:.2f}s"
        )

    def on_bench_timer(self) -> None:
        if self._bench_deadline <= 0.0:
            return
        if time.monotonic() < self._bench_deadline:
            self.pub.publish(self._bench_command)
            return
        self._bench_deadline = 0.0
        self._bench_command = Twist()
        self.pub.publish(Twist())
        self.get_logger().warning("RESTRAINED BENCH pulse complete; zero command sent")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandLimiterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
