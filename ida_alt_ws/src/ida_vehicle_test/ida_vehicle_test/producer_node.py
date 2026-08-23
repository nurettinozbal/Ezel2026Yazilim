"""Passive Jetson producer for the YKI websocket debug channel.

ROS callbacks only update bounded memory or enqueue output.  This node exposes
no vehicle actuation API and never transports raw images or video.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .contracts import bounded_json_loads, compact_json
from .producer_contract import DebugSnapshotComposer, vehicle_test_ros_envelope
from .producer_runtime import (
    ProducerRuntime,
    connection_loop,
    deliver_pending_commands,
    supported_websockets_version,
)


CHANNEL_TOPICS = {
    "camera_p1p2": "/perception/camera/p1p2/raw",
    "camera_p3": "/perception/camera/p3/raw",
    "lidar": "/perception/lidar/raw_obstacles",
    "fusion_status": "/perception/fusion/status",
    "canonical_buoys": "/perception/buoys",
    "canonical_obstacles": "/perception/obstacles",
    "shadow_buoys": "/perception/fusion/shadow/buoys",
    "shadow_obstacles": "/perception/fusion/shadow/obstacles",
    "autonomy_state": "/autonomy/state",
    "autonomy_debug": "/autonomy/debug",
}
VEHICLE_TEST_TOPICS = ("/vehicle_test/status", "/vehicle_test/events", "/vehicle_test/result")
_RETAINED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def load_websockets() -> tuple[Any | None, str]:
    """Lazy import keeps package/node discovery usable without the optional wheel."""
    try:
        import websockets  # type: ignore
    except ImportError:
        return None, "python websockets dependency is not installed"
    version = getattr(websockets, "__version__", None)
    if not supported_websockets_version(version):
        return None, (
            "unsupported websockets version "
            f"{version!r}; required >=9.1,<14"
        )
    return websockets, ""


class VehicleTestProducerNode(Node):
    def __init__(self) -> None:
        super().__init__("ida_vehicle_test_producer")
        # Portable default: the YKI advertises itself on the local network.
        # A fixed ws/wss URL remains an explicit deployment override.
        self.declare_parameter("websocket_url", "auto://yki")
        self.declare_parameter("snapshot_hz", 3.0)
        self.declare_parameter("freshness_s", 1.0)
        self.declare_parameter("max_lidar_points", 360)
        self.declare_parameter("reconnect_initial_s", 0.5)
        self.declare_parameter("reconnect_max_s", 10.0)
        self.declare_parameter("outbox_limit", 128)
        self.declare_parameter("inbox_limit", 32)

        url = str(self.get_parameter("websocket_url").value)
        # The producer credential must not be exposed through ROS parameter
        # services or launch arguments. It is inherited from the process env.
        token = os.environ.get("EZEL_JETSON_DEBUG_TOKEN", "").strip()
        snapshot_hz = self._finite_float("snapshot_hz", 2.0, 5.0)
        freshness_s = self._finite_float("freshness_s", 0.1, 10.0)
        initial = self._finite_float("reconnect_initial_s", 0.1, 30.0)
        maximum = self._finite_float("reconnect_max_s", initial, 120.0)
        max_points = self._integer("max_lidar_points", 1, 720)
        outbox_limit = self._integer("outbox_limit", 8, 1024)
        inbox_limit = self._integer("inbox_limit", 1, 256)
        if url != "auto://yki" and (
            not url.startswith(("ws://", "wss://")) or len(url) > 512
        ):
            raise ValueError("websocket_url must be auto://yki or a bounded ws/wss URL")
        if len(token) > 4096 or "\r" in token or "\n" in token:
            raise ValueError("token is invalid")

        composer = DebugSnapshotComposer(freshness_s, max_points)
        self.runtime = ProducerRuntime(composer, outbox_limit, inbox_limit)
        self.request_pub = self.create_publisher(String, "/vehicle_test/request", 10)
        self.health_pub = self.create_publisher(String, "/vehicle_test/producer_status", 5)
        self._input_subscriptions = []
        for channel, topic in CHANNEL_TOPICS.items():
            self._input_subscriptions.append(self.create_subscription(
                String, topic, lambda msg, selected=channel: self.on_snapshot(selected, msg), 10
            ))
        for topic in VEHICLE_TEST_TOPICS:
            qos = 20 if topic == "/vehicle_test/events" else _RETAINED_QOS
            self._input_subscriptions.append(self.create_subscription(
                String, topic, lambda msg, selected=topic: self.on_vehicle_test(selected, msg), qos
            ))
        self.create_timer(0.05, self.flush_commands)
        self.create_timer(1.0, self.publish_health)
        self._network_loop: asyncio.AbstractEventLoop | None = None
        self._network_task: asyncio.Task | None = None
        self._network_thread: threading.Thread | None = None

        dependency, dependency_error = load_websockets()
        if dependency is None:
            self.runtime.set_dependency_missing(dependency_error)
            self.get_logger().error(f"YKI producer disabled: {dependency_error}")
        elif not token:
            self.runtime.set_connection(False, "token is empty")
            self.get_logger().error("YKI producer disabled: token parameter is empty")
        else:
            self._network_thread = threading.Thread(
                target=self._network_main,
                args=(dependency.connect, url, token, snapshot_hz, initial, maximum),
                name="ida-yki-websocket",
                daemon=True,
            )
            self._network_thread.start()

    def _finite_float(self, name: str, low: float, high: float) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{name} must be finite in [{low}, {high}]")
        return value

    def _integer(self, name: str, low: int, high: int) -> int:
        raw = self.get_parameter(name).value
        if isinstance(raw, bool) or not isinstance(raw, int) or not low <= raw <= high:
            raise ValueError(f"{name} must be an integer in [{low}, {high}]")
        return raw

    def _network_main(self, connect: Any, url: str, token: str, hz: float, initial: float, maximum: float) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._network_loop = loop
        self._network_task = loop.create_task(
            connection_loop(connect, url, token, self.runtime, hz, initial, maximum)
        )
        try:
            loop.run_until_complete(self._network_task)
        except asyncio.CancelledError:
            pass
        finally:
            loop.close()

    def on_snapshot(self, channel: str, msg: String) -> None:
        self.runtime.composer.ingest_json(channel, msg.data, time.monotonic())

    def on_vehicle_test(self, topic: str, msg: String) -> None:
        try:
            payload = bounded_json_loads(msg.data, 2097152, 80000, 12)
            envelope = vehicle_test_ros_envelope(topic, payload)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning(f"Dropped malformed passive result from {topic}")
            return
        if not self.runtime.outbox.put(envelope):
            self.get_logger().error(f"YKI producer outbox full; dropped {topic}")

    def flush_commands(self) -> None:
        def publish(request: dict[str, Any]) -> None:
            msg = String()
            msg.data = compact_json(request)
            self.request_pub.publish(msg)
        subscriber_count = self.count_subscribers("/vehicle_test/request")
        delivered = deliver_pending_commands(
            self.runtime, subscriber_count, publish, 4
        )
        if delivered:
            self.get_logger().info(
                f"Published {delivered} passive test request(s); subscribers={subscriber_count}"
            )

    def publish_health(self) -> None:
        msg = String()
        msg.data = compact_json(self.runtime.health(time.monotonic()))
        self.health_pub.publish(msg)

    def destroy_node(self) -> bool:
        if self._network_loop is not None and self._network_task is not None:
            self._network_loop.call_soon_threadsafe(self._network_task.cancel)
        if self._network_thread is not None:
            self._network_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleTestProducerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
