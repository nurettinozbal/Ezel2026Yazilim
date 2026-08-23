"""Pure contracts and bounded latest-only composer for the YKI debug producer."""

from __future__ import annotations

import json
import math
import threading
from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, Mapping

from .contracts import bounded_json_loads, compact_json, parse_request


INPUT_CHANNELS = frozenset(
    {
        "camera_p1p2", "camera_p3", "lidar", "fusion_status",
        "canonical_buoys", "canonical_obstacles", "shadow_buoys",
        "shadow_obstacles", "autonomy_state", "autonomy_debug",
    }
)
_VALID_COLORS = {"unknown", "orange", "red", "green", "blue", "black", "yellow"}


@dataclass(frozen=True)
class Sample:
    payload: Any
    received_mono: float


def _finite(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _stamp(payload: Any) -> float | None:
    if not isinstance(payload, dict):
        return None
    return _finite(payload.get("acquisition_stamp", payload.get("stamp")))


def decode_vehicle_test_command(raw: str | Mapping[str, Any]) -> tuple[Dict[str, Any], Dict[str, Any]]:
    envelope = bounded_json_loads(raw, 16384, 256, 8) if isinstance(raw, str) else dict(raw)
    if not isinstance(envelope, dict) or set(envelope) != {"type", "data"}:
        raise ValueError("server envelope must contain exactly type and data")
    if envelope["type"] != "vehicle_test_request" or not isinstance(envelope["data"], dict):
        raise ValueError("unsupported server producer message")
    request = parse_request(envelope["data"])
    canonical = dict(envelope["data"])
    ack = {
        "type": "vehicle_test_ack",
        "data": {
            "action": request.action,
            "run_id": request.run_id,
            "seq": request.seq,
            "accepted": True,
        },
    }
    return canonical, ack


def vehicle_test_ros_envelope(topic: str, payload: Any) -> Dict[str, Any]:
    if topic not in {"/vehicle_test/status", "/vehicle_test/events", "/vehicle_test/result"}:
        raise ValueError("unsupported vehicle_test ROS topic")
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise ValueError("invalid vehicle_test ROS payload")
    return {"type": "vehicle_test_ros", "data": {"topic": topic, "payload": payload}}


class BoundedOutbox:
    """Thread-safe bounded queue; terminal results are never silently evicted."""

    def __init__(self, limit: int = 128) -> None:
        if not 8 <= limit <= 1024:
            raise ValueError("outbox limit out of range")
        self.limit = limit
        self._lock = threading.Lock()
        self._critical: deque[Dict[str, Any]] = deque()
        self._queue: deque[Dict[str, Any]] = deque()
        self._latest_status: Dict[str, Any] | None = None
        self._sent_terminal_keys: set[tuple[Any, ...]] = set()

    @staticmethod
    def _critical_key(envelope: Dict[str, Any]) -> tuple[Any, ...] | None:
        if envelope.get("type") == "vehicle_test_ack":
            data = envelope.get("data", {})
            return ("ack", data.get("action"), data.get("run_id"), data.get("seq"), data.get("accepted"))
        data = envelope.get("data", {})
        if data.get("topic") == "/vehicle_test/result":
            payload = data.get("payload", {})
            return ("result", payload.get("run_id"), payload.get("seq"), payload.get("state"))
        return None

    def put(self, envelope: Dict[str, Any]) -> bool:
        topic = envelope.get("data", {}).get("topic") if isinstance(envelope, dict) else None
        with self._lock:
            if topic == "/vehicle_test/status":
                self._latest_status = envelope
                return True
            critical_key = self._critical_key(envelope)
            if critical_key is not None:
                if any(self._critical_key(item) == critical_key for item in self._critical):
                    return True
                if len(self._critical) + len(self._queue) >= self.limit:
                    if self._queue:
                        self._queue.popleft()
                    else:
                        return False
                self._critical.append(envelope)
                return True
            if len(self._critical) + len(self._queue) >= self.limit:
                return False
            self._queue.append(envelope)
            return True

    def peek(self) -> Dict[str, Any] | None:
        with self._lock:
            for item in self._critical:
                key = self._critical_key(item)
                if key is None or key[0] != "result" or key not in self._sent_terminal_keys:
                    return item
            if self._queue:
                return self._queue[0]
            return self._latest_status

    def mark_sent(self, envelope: Dict[str, Any]) -> None:
        with self._lock:
            key = self._critical_key(envelope)
            if key is not None and key[0] == "result" and any(item is envelope for item in self._critical):
                self._sent_terminal_keys.add(key)
            elif any(item is envelope for item in self._critical):
                self._critical.remove(envelope)
            elif any(item is envelope for item in self._queue):
                self._queue.remove(envelope)
            elif self._latest_status is envelope:
                self._latest_status = None

    def replay_unacked_terminals(self) -> None:
        with self._lock:
            self._sent_terminal_keys.clear()

    def acknowledge_terminal(self, run_id: str, seq: int) -> bool:
        with self._lock:
            for item in list(self._critical):
                key = self._critical_key(item)
                if key is not None and key[0] == "result" and key[1:3] == (run_id, seq):
                    self._critical.remove(item)
                    self._sent_terminal_keys.discard(key)
                    return True
        return False

    def __len__(self) -> int:
        with self._lock:
            return len(self._critical) + len(self._queue) + (1 if self._latest_status is not None else 0)


class DebugSnapshotComposer:
    def __init__(self, freshness_s: float = 1.0, max_lidar_points: int = 360) -> None:
        if not math.isfinite(freshness_s) or not 0.1 <= freshness_s <= 10.0:
            raise ValueError("freshness_s out of range")
        if not isinstance(max_lidar_points, int) or not 1 <= max_lidar_points <= 720:
            raise ValueError("max_lidar_points out of range")
        self.freshness_s = freshness_s
        self.max_lidar_points = max_lidar_points
        self._lock = threading.Lock()
        self._latest: Dict[str, Sample] = {}

    def ingest_json(self, channel: str, raw: str, received_mono: float) -> bool:
        if channel not in INPUT_CHANNELS or not math.isfinite(received_mono):
            return False
        try:
            payload = bounded_json_loads(raw, 2097152, 80000, 12)
        except ValueError:
            payload = None
        with self._lock:
            self._latest[channel] = Sample(payload, received_mono)
        return payload is not None

    def _copy(self) -> Dict[str, Sample]:
        with self._lock:
            return dict(self._latest)

    def compose(self, now_epoch: float, now_mono: float) -> Dict[str, Any]:
        if not math.isfinite(now_epoch) or not math.isfinite(now_mono) or now_epoch < 1.0:
            raise ValueError("composer clocks invalid")
        samples = self._copy()
        source_stamps = [
            stamp for sample in samples.values()
            if (stamp := _stamp(sample.payload)) is not None and 1.0 <= stamp <= now_epoch + 5.0
        ]
        source_stamp = max(source_stamps, default=now_epoch)
        lidar = self._lidar(samples.get("lidar"), now_mono, now_epoch)
        camera = self._camera(samples, now_mono, now_epoch)
        fusion = self._fusion(samples, now_mono, now_epoch)
        autonomy, command = self._autonomy(samples, now_mono, now_epoch)
        return {
            "schema_version": 1,
            "source_stamp": source_stamp,
            "frame_id": "base_link",
            "lidar": lidar,
            "camera": camera,
            "fusion": fusion,
            "autonomy": autonomy,
            "chosen_command": command,
        }

    def _status(self, sample: Sample | None, now_mono: float, now_epoch: float) -> str:
        if sample is None:
            return "unknown"
        if sample.payload is None or not isinstance(sample.payload, dict):
            return "error"
        stamp = _stamp(sample.payload)
        if stamp is None or stamp > now_epoch + 0.25:
            return "error"
        if (
            bool(sample.payload.get("stale", False))
            or now_mono - sample.received_mono > self.freshness_s
            or now_epoch - stamp > self.freshness_s
        ):
            return "stale"
        if now_mono < sample.received_mono:
            return "error"
        return "ok"

    @staticmethod
    def _downsample(items: list, limit: int) -> list:
        if len(items) <= limit:
            return items
        step = len(items) / limit
        return [items[min(len(items) - 1, int(index * step))] for index in range(limit)]

    def _lidar(self, sample: Sample | None, now: float, epoch: float) -> Dict[str, Any]:
        status = self._status(sample, now, epoch)
        clusters_by_id: Dict[str, Dict[str, Any]] = {}
        duplicate_ids: set[str] = set()
        if status in {"ok", "stale"} and not isinstance(sample.payload.get("clusters"), list):
            status = "error"
        elif status in {"ok", "stale"}:
            for index, raw in enumerate(sample.payload["clusters"][:1000]):
                if not isinstance(raw, dict):
                    status = "error"
                    continue
                forward = _finite(raw.get("forward_m"))
                left = _finite(raw.get("lateral_left_m"))
                if forward is None or left is None or not -100 <= forward <= 100 or not -100 <= left <= 100:
                    status = "error"
                    continue
                identity = raw.get("id")
                if not isinstance(identity, str):
                    status = "error"
                    continue
                if not identity or len(identity) > 64 or any(ch.isspace() for ch in identity):
                    status = "error"
                    continue
                if identity in clusters_by_id:
                    duplicate_ids.add(identity)
                else:
                    clusters_by_id[identity] = {"id": identity, "forward_m": forward, "lateral_right_m": -left}
        for identity in duplicate_ids:
            clusters_by_id.pop(identity, None)
        if duplicate_ids:
            status = "error"
        clusters = self._downsample(list(clusters_by_id.values()), 100)
        raw_points = sample.payload.get("points") if status in {"ok", "stale"} else None
        points: list[Dict[str, float]] = []
        if raw_points is not None:
            if not isinstance(raw_points, list) or len(raw_points) > 5000:
                status = "error"
            else:
                for raw in raw_points:
                    if not isinstance(raw, dict):
                        status = "error"
                        continue
                    forward = _finite(raw.get("forward_m"))
                    left = _finite(raw.get("lateral_left_m"))
                    if forward is None or left is None or not -100 <= forward <= 100 or not -100 <= left <= 100:
                        status = "error"
                        continue
                    points.append({"forward_m": forward, "lateral_right_m": -left})
                points = self._downsample(points, self.max_lidar_points)
        else:
            # Backward compatibility with older S2 bridge payloads.
            points = self._downsample(
                [{"forward_m": item["forward_m"], "lateral_right_m": item["lateral_right_m"]} for item in clusters],
                self.max_lidar_points,
            )
        return {"status": status, "points": points, "clusters": clusters}

    def _camera(self, samples: Mapping[str, Sample], now: float, epoch: float) -> Dict[str, Any]:
        candidates = []
        statuses = []
        for channel in ("camera_p1p2", "camera_p3"):
            sample = samples.get(channel)
            status = self._status(sample, now, epoch)
            detections = sample.payload.get("detections") if status in {"ok", "stale"} else None
            if status in {"ok", "stale"} and not isinstance(detections, list):
                status = "error"
            elif status == "ok":
                seen_ids: set[str] = set()
                for detection in sample.payload["detections"][:256]:
                    if not isinstance(detection, dict):
                        status = "error"
                        continue
                    bearing = _finite(detection.get("bearing_deg"))
                    confidence = _finite(detection.get("confidence"))
                    identity = detection.get("id")
                    color = detection.get("color")
                    if (
                        bearing is None or confidence is None
                        or not -180 <= bearing <= 180 or not 0 <= confidence <= 1
                        or not isinstance(identity, str) or not identity or len(identity) > 64
                        or any(ch.isspace() for ch in identity) or identity in seen_ids
                        or not isinstance(color, str) or color.lower() not in _VALID_COLORS - {"unknown", "blue"}
                    ):
                        status = "error"
                        continue
                    seen_ids.add(identity)
                    candidates.append((confidence, bearing))
            statuses.append(status)
        status = "error" if "error" in statuses else ("ok" if "ok" in statuses else ("stale" if "stale" in statuses else "unknown"))
        bearing_rad = math.radians(max(candidates)[1]) if candidates else None
        if bearing_rad is not None and not -math.pi <= bearing_rad <= math.pi:
            status, bearing_rad = "error", None
        return {"status": status, "bearing_rad": bearing_rad}

    def _fusion(self, samples: Mapping[str, Sample], now: float, epoch: float) -> Dict[str, Any]:
        status_sample = samples.get("fusion_status")
        fusion_status = self._status(status_sample, now, epoch)
        if fusion_status != "ok":
            return {"status": fusion_status, "objects": []}
        meta = status_sample.payload
        required = ("accepted", "source_health", "camera_fresh", "lidar_fresh", "shadow_mode")
        if (
            any(key not in meta for key in required)
            or not isinstance(meta.get("accepted"), bool)
            or not isinstance(meta.get("source_health"), str)
            or not isinstance(meta.get("camera_fresh"), bool)
            or not isinstance(meta.get("lidar_fresh"), bool)
            or not isinstance(meta.get("shadow_mode"), bool)
        ):
            return {"status": "error", "objects": []}
        if not meta["accepted"] or meta["source_health"] != "ok":
            degraded = "stale" if "stale" in meta["source_health"] else "error"
            return {"status": degraded, "objects": []}
        if not meta["camera_fresh"] or not meta["lidar_fresh"]:
            return {"status": "stale", "objects": []}
        pairs = ([
            (samples.get("shadow_buoys"), samples.get("shadow_obstacles"), "shadow"),
        ] if meta["shadow_mode"] else [
            (samples.get("canonical_buoys"), samples.get("canonical_obstacles"), "canonical"),
        ])
        selected = next((pair for pair in pairs if self._status(pair[0], now, epoch) == "ok" and self._status(pair[1], now, epoch) == "ok"), None)
        if selected is None:
            observed = [self._status(item, now, epoch) for pair in pairs for item in pair[:2]]
            status = "error" if "error" in observed else ("stale" if "stale" in observed else "unknown")
            return {"status": status, "objects": []}
        buoys, obstacles, source_name = selected
        objects: Dict[str, Dict[str, Any]] = {}
        duplicate_ids: set[str] = set()
        malformed = False
        for payload, field in ((obstacles.payload, "obstacles"), (buoys.payload, "detections")):
            raw_items = payload.get(field, [])
            if not isinstance(raw_items, list):
                return {"status": "error", "objects": []}
            field_ids: set[str] = set()
            for index, raw in enumerate(raw_items[:200]):
                if not isinstance(raw, dict):
                    malformed = True
                    continue
                forward = _finite(raw.get("forward_m"))
                right = _finite(raw.get("lateral_m"))
                if forward is None or right is None or not -100 <= forward <= 100 or not -100 <= right <= 100:
                    malformed = True
                    continue
                identity = raw.get("lidar_obstacle_id", raw.get("id"))
                color = str(raw.get("color", "unknown")).lower()
                if not isinstance(identity, str) or color not in _VALID_COLORS or not identity or len(identity) > 64 or any(ch.isspace() for ch in identity):
                    malformed = True
                    continue
                if identity in field_ids:
                    duplicate_ids.add(identity)
                field_ids.add(identity)
                objects[identity] = {
                    "id": identity, "forward_m": forward, "lateral_right_m": right,
                    "color": color, "source": source_name, "status": "ok",
                }
        for identity in duplicate_ids:
            objects.pop(identity, None)
        return {
            "status": "error" if malformed or duplicate_ids else "ok",
            "objects": self._downsample(list(objects.values()), 100),
        }

    def _autonomy(self, samples: Mapping[str, Sample], now: float, epoch: float) -> tuple[Dict[str, Any], Dict[str, Any]]:
        state_sample, debug_sample = samples.get("autonomy_state"), samples.get("autonomy_debug")
        state_status, debug_status = self._status(state_sample, now, epoch), self._status(debug_sample, now, epoch)
        status = "ok" if state_status == debug_status == "ok" else ("error" if "error" in {state_status, debug_status} else ("stale" if "stale" in {state_status, debug_status} else "unknown"))
        state = state_sample.payload if state_status in {"ok", "stale"} else {}
        debug = debug_sample.payload if debug_status in {"ok", "stale"} else {}
        command = debug.get("command", {}) if isinstance(debug.get("command", {}), dict) else {}
        forward = _finite(command.get("vx"))
        yaw = _finite(command.get("yaw_rate"))
        current = state.get("current_waypoint", 0)
        if "current_waypoint" not in state or isinstance(current, bool) or not isinstance(current, int) or not 0 <= current <= 10000:
            current, status = 0, "error"
        state_name = state.get("state")
        action = command.get("action", state.get("action"))
        if not isinstance(state_name, str) or not state_name or len(state_name) > 64:
            state_name, status = "", "error"
        if not isinstance(action, str) or len(action) > 160:
            action, status = "", "error"
        if not isinstance(debug.get("command"), dict):
            status = "error"
        failsafe = state.get("failsafe_reason", debug.get("failsafe_reason", ""))
        if not isinstance(failsafe, str) or len(failsafe) > 160:
            failsafe, status = "", "error"
        command_status = debug_status
        if (
            not isinstance(debug.get("command"), dict)
            or not isinstance(action, str) or not action
            or forward is None or yaw is None
            or not -5 <= forward <= 5 or not -5 <= yaw <= 5
        ):
            command_status = "error"
        return (
            {
                "status": status,
                "state": state_name,
                "action": action,
                "current_waypoint": current,
                "failsafe_reason": failsafe,
            },
            {
                "status": command_status,
                "forward_mps": forward if forward is not None and -5 <= forward <= 5 else 0.0,
                "yaw_rate_rps": yaw if yaw is not None and -5 <= yaw <= 5 else 0.0,
            },
        )


def encode_envelope(payload: Mapping[str, Any]) -> str:
    return compact_json(payload)
