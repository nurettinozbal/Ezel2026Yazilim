"""ROS 2 passive vehicle-test monitor.

Safety invariant: this node publishes only `/vehicle_test/*` JSON observability
topics.  It has no Twist publisher and no arm, mode or motor service/client.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .contracts import RequestError, bounded_json_loads, compact_json, parse_request
from .core import CHANNEL_RULES, DEFAULT_PUBLISHER_NODE_ALLOWLIST, PassiveTestCore
from .storage import atomic_persist, probe_log_files


_STATUS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class VehicleTestMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("ida_vehicle_test_monitor")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("max_input_bytes", 2097152)
        self.declare_parameter("max_json_items", 80000)
        self.declare_parameter("log_root", "./logs")
        self.declare_parameter("evidence_root", "./vehicle_test_evidence")
        self.declare_parameter("allow_sim_sources", False)
        self.declare_parameter(
            "publisher_node_allowlist_json",
            json.dumps({key: sorted(value) for key, value in DEFAULT_PUBLISHER_NODE_ALLOWLIST.items()}),
        )
        publish_hz = float(self.get_parameter("publish_hz").value)
        if not math.isfinite(publish_hz) or not 0.5 <= publish_hz <= 20.0:
            raise ValueError("publish_hz must be finite and between 0.5 and 20")

        self.max_input_bytes = self._bounded_int("max_input_bytes", 1024, 8388608)
        self.max_json_items = self._bounded_int("max_json_items", 100, 200000)
        self.log_root = Path(str(self.get_parameter("log_root").value)).resolve()
        self.evidence_root = Path(str(self.get_parameter("evidence_root").value)).resolve()
        self.evidence_root.mkdir(parents=True, exist_ok=True)
        allowlist = self._publisher_allowlist()
        allow_sim_sources = self.get_parameter("allow_sim_sources").value
        if not isinstance(allow_sim_sources, bool):
            raise ValueError("allow_sim_sources must be boolean")
        if allow_sim_sources:
            # Explicit lab/simulation opt-in only. Field defaults retain the
            # production YOLO/S2 publisher identity requirements.
            for channel in ("camera_p1p2", "camera_p3", "lidar"):
                allowlist[channel].append("ida_perception_sim")
        self.core = PassiveTestCore(allowlist)
        self._logging_sizes: Dict[str, int] = {}
        self._logging_grown: set[str] = set()
        self._logging_run_key: tuple[str | None, int | None] | None = None
        self._logging_pending: tuple[Dict[str, Any], float, float] | None = None
        self.status_pub = self.create_publisher(String, "/vehicle_test/status", _STATUS_QOS)
        self.metrics_pub = self.create_publisher(String, "/vehicle_test/metrics", 10)
        self.events_pub = self.create_publisher(String, "/vehicle_test/events", 20)
        self.result_pub = self.create_publisher(String, "/vehicle_test/result", _STATUS_QOS)
        self._request_subscription = self.create_subscription(
            String, "/vehicle_test/request", self.on_request, 10
        )

        # Never reuse rclpy.Node's private ``_subscriptions`` container.  Doing
        # so removes already-created entities from the executor even though a
        # stale DDS endpoint can remain visible in graph introspection.
        self._input_subscriptions = []
        for channel, rule in CHANNEL_RULES.items():
            subscription = self.create_subscription(
                String,
                rule.topic,
                lambda msg, selected=channel: self.on_observation(selected, msg),
                20,
            )
            self._input_subscriptions.append(subscription)
        self.create_timer(1.0 / publish_hz, self.tick)
        self.get_logger().info("Passive vehicle-test monitor started; actuation is unavailable")

    def _bounded_int(self, name: str, low: int, high: int) -> int:
        raw = self.get_parameter(name).value
        if isinstance(raw, bool) or not isinstance(raw, int) or not low <= raw <= high:
            raise ValueError(f"{name} must be an integer in [{low}, {high}]")
        return raw

    def _publisher_allowlist(self) -> Dict[str, list[str]]:
        try:
            raw = bounded_json_loads(
                str(self.get_parameter("publisher_node_allowlist_json").value), 16384, 512, 5
            )
        except ValueError as exc:
            raise ValueError("publisher_node_allowlist_json is invalid") from exc
        if not isinstance(raw, dict) or set(raw) != set(CHANNEL_RULES):
            raise ValueError("publisher node allow-list must cover every channel")
        result: Dict[str, list[str]] = {}
        for channel, nodes in raw.items():
            if (
                not isinstance(nodes, list)
                or not 1 <= len(nodes) <= 8
                or any(not isinstance(node, str) or not node or len(node) > 128 for node in nodes)
            ):
                raise ValueError(f"publisher allow-list for {channel} is invalid")
            result[channel] = list(nodes)
        return result

    @staticmethod
    def monotonic_seconds() -> float:
        return time.monotonic()

    def ros_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_request(self, msg: String) -> None:
        now = self.monotonic_seconds()
        try:
            request = parse_request(msg.data)
            self.core.request(request, now, self.ros_seconds())
            self.get_logger().info(
                f"Accepted passive request action={request.action} "
                f"test={request.test} run_id={request.run_id} seq={request.seq}"
            )
            run_key = (self.core.run_id, self.core.seq)
            if self.core.active and self.core.test == "logging" and run_key != self._logging_run_key:
                self._logging_sizes.clear()
                self._logging_grown.clear()
                self._logging_run_key = run_key
        except (RequestError, ValueError) as exc:
            self.get_logger().warning(f"Rejected passive request: {exc}")
            self._publish(
                self.events_pub,
                {
                    "schema_version": 1,
                    "event": "REQUEST_REJECTED",
                    "monotonic_s": now,
                    "reason": str(exc),
                    "actuation_enabled": False,
                },
            )
        self._flush_events()
        self._persist_terminal(now)
        self._flush_result(now)

    def on_observation(self, channel: str, msg: String) -> None:
        try:
            payload: Any = bounded_json_loads(
                msg.data, self.max_input_bytes, self.max_json_items, 12
            )
        except (TypeError, ValueError, json.JSONDecodeError):
            payload = None
        receive_mono = self.monotonic_seconds()
        ros_now = self.ros_seconds()
        if channel == "logging" and isinstance(payload, dict):
            # Filesystem access is kept out of the subscription callback.
            self._logging_pending = (payload, receive_mono, ros_now)
            return
        self.core.observe(channel, payload, receive_mono, ros_now)
        self._flush_events()
        self._persist_terminal(receive_mono)
        self._flush_result()

    def _with_logging_growth(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Add file-growth evidence without exposing file sizes on output topics."""
        enriched = dict(payload)
        files = payload.get("files")
        current, grown, valid = probe_log_files(
            self.log_root, files, self._logging_sizes, self._logging_grown
        )
        self._logging_sizes = current
        self._logging_grown = grown
        enriched["growing_file_count"] = len(grown)
        enriched["path_policy_valid"] = valid
        return enriched

    def tick(self) -> None:
        now = self.monotonic_seconds()
        if self.core.active and self.core.test == "logging" and self._logging_pending is not None:
            payload, received, ros_received = self._logging_pending
            self._logging_pending = None
            self.core.observe(
                "logging", self._with_logging_growth(payload), received, ros_received
            )
        if self.core.active:
            # Threat boundary: ROS 2 graph metadata is not authenticated here.
            # Node/type allow-lists prevent accidental wiring, but the vehicle
            # DDS domain must remain trusted and access-controlled; a malicious
            # participant could spoof graph identity and String payloads.
            graph: Dict[str, Any] = {}
            for channel in self.core.evidence:
                try:
                    # Resolve launch remappings before graph inspection. This
                    # keeps the default field contract unchanged while allowing
                    # an isolated rosbag-replay domain to use /ida_alt topics.
                    resolved_topic = self.resolve_topic_name(
                        CHANNEL_RULES[channel].topic
                    )
                    infos = self.get_publishers_info_by_topic(resolved_topic)
                    graph[channel] = {
                        "count": len(infos),
                        "types": sorted({info.topic_type for info in infos}),
                        "nodes": sorted(info.node_name for info in infos),
                    }
                except Exception:
                    graph[channel] = {"error": "introspection_failed"}
            self.core.observe_graph(graph, now)
        self.core.tick(now)
        if self.core.pass_ready:
            try:
                artifact = self._persist_evidence(self.core.pass_candidate(now))
                self.core.commit_pass(artifact)
            except (OSError, TypeError, ValueError):
                self.core.fail_persistence()
        self._persist_terminal(now)
        status = self.core.status(now)
        status["stamp"] = self.ros_seconds()
        metrics = {
            "schema_version": 1,
            "stamp": status["stamp"],
            "run_id": self.core.run_id,
            "seq": self.core.seq,
            "test": self.core.test,
            **self.core.metrics(now),
        }
        self._publish(self.status_pub, status)
        self._publish(self.metrics_pub, metrics)
        self._flush_events()
        self._flush_result(now)

    def _persist_evidence(self, candidate: Dict[str, Any]) -> Dict[str, str]:
        filename = f"{self.core.run_id}-{self.core.seq}.json"
        return atomic_persist(self.evidence_root, filename, candidate)

    def _persist_terminal(self, now: float) -> None:
        if not self.core.terminal_needs_persistence:
            return
        try:
            artifact = self._persist_evidence(self.core.terminal_candidate(now))
            self.core.attach_terminal_artifact(artifact)
        except (OSError, TypeError, ValueError):
            self.core.mark_terminal_persistence_failed()

    def _flush_events(self) -> None:
        for event in self.core.pop_events():
            event["stamp"] = self.ros_seconds()
            event["actuation_enabled"] = False
            self._publish(self.events_pub, event)

    def _flush_result(self, now: float | None = None) -> None:
        if now is None:
            now = self.monotonic_seconds()
        result = self.core.pop_result(now)
        if result is not None:
            result["stamp"] = self.ros_seconds()
            self._publish(self.result_pub, result)

    @staticmethod
    def _publish(publisher: Any, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = compact_json(payload)
        publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleTestMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
