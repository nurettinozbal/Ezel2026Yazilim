"""Strict lab-only snapshot and vehicle-test relay contracts."""

from __future__ import annotations

import copy
import json
import math
import re
import secrets
import threading
import time
import uuid
from collections import deque
from pathlib import Path
from typing import Any

CONTRACT_PATH = Path(__file__).resolve().parents[2] / "contracts" / "ida_vehicle_test.v1.json"
CONTRACT = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
SCHEMA_VERSION = int(CONTRACT["schema_version"])
TEST_IDS = {item["id"] for item in CONTRACT["tests"]}
RESULT_STATUSES = set(CONTRACT["result_statuses"])
MAX_LIDAR_POINTS = 720
MAX_CLUSTERS = 100
MAX_OBJECTS = 100
VALID_STATUSES = {"unknown", "ok", "stale", "error"}
VALID_COLORS = {"unknown", "orange", "red", "green", "blue", "black", "yellow"}
ID_PATTERN = re.compile(r"^[A-Za-z0-9_.:-]{1,64}$")


def is_lab_authorized(command_authorized: bool, auth_required: bool, token: str) -> bool:
    return bool(command_authorized and auth_required and token)


def producer_token_valid(provided: Any, configured: str, enabled: bool) -> bool:
    return bool(enabled and configured and secrets.compare_digest(str(provided or ""), configured))


def validate_lab_config(enabled: bool, auth_required: bool, browser_token: str, producer_token: str) -> None:
    if not enabled:
        return
    if not auth_required or not browser_token or not producer_token:
        raise RuntimeError("LAB debug requires auth plus non-empty browser and producer tokens")
    if secrets.compare_digest(browser_token, producer_token):
        raise RuntimeError("LAB browser and producer tokens must be different")


def raw_lidar_left_to_canonical(point: Any) -> dict[str, float]:
    item = _exact_dict(point, {"forward_m", "lateral_left_m"}, "raw lidar point")
    return {
        "forward_m": _number(item["forward_m"], "raw_lidar.forward_m", -100.0, 100.0),
        "lateral_right_m": -_number(item["lateral_left_m"], "raw_lidar.lateral_left_m", -100.0, 100.0),
    }


def _exact_dict(value: Any, keys: set[str], field: str) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != keys:
        raise ValueError(f"{field} keys must be exactly {sorted(keys)}")
    return value


def _number(value: Any, field: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ValueError(f"{field} is nonfinite/out of range")
    return result


def _integer(value: Any, field: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(f"{field} must be an in-range integer")
    return value


def _text(value: Any, field: str, maximum: int) -> str:
    if not isinstance(value, str) or len(value) > maximum:
        raise ValueError(f"{field} must be a bounded string")
    return value


def _status(value: Any, field: str) -> str:
    if not isinstance(value, str) or value not in VALID_STATUSES:
        raise ValueError(f"{field} has invalid status")
    return value


def _identifier(value: Any, field: str) -> str:
    if not isinstance(value, str) or ID_PATTERN.fullmatch(value) is None:
        raise ValueError(f"{field} has invalid id")
    return value


def _coordinate(value: Any, field: str, *, with_id: bool = False) -> dict[str, Any]:
    keys = {"forward_m", "lateral_right_m", "id"} if with_id else {"forward_m", "lateral_right_m"}
    item = _exact_dict(value, keys, field)
    result = {
        "forward_m": _number(item["forward_m"], f"{field}.forward_m", -100.0, 100.0),
        "lateral_right_m": _number(item["lateral_right_m"], f"{field}.lateral_right_m", -100.0, 100.0),
    }
    if with_id:
        result["id"] = _identifier(item["id"], f"{field}.id")
    return result


def normalize_snapshot(payload: Any, *, now_epoch: float | None = None, now_monotonic: float | None = None) -> dict[str, Any]:
    """Strictly validate producer input; malformed data is rejected, never coerced."""
    top = _exact_dict(payload, {"schema_version", "source_stamp", "frame_id", "lidar", "camera", "fusion", "autonomy", "chosen_command"}, "snapshot")
    if top["schema_version"] != SCHEMA_VERSION:
        raise ValueError("unsupported debug_snapshot schema_version")
    epoch = time.time() if now_epoch is None else now_epoch
    monotonic = time.monotonic() if now_monotonic is None else now_monotonic
    source_stamp = _number(top["source_stamp"], "source_stamp", 1.0, epoch + 5.0)
    if top["frame_id"] != "base_link":
        raise ValueError("frame_id must be base_link")

    lidar = _exact_dict(top["lidar"], {"status", "points", "clusters"}, "lidar")
    if not isinstance(lidar["points"], list) or len(lidar["points"]) > MAX_LIDAR_POINTS:
        raise ValueError("lidar.points exceeds bound or is not a list")
    if not isinstance(lidar["clusters"], list) or len(lidar["clusters"]) > MAX_CLUSTERS:
        raise ValueError("lidar.clusters exceeds bound or is not a list")
    points = [_coordinate(item, f"lidar.points[{index}]") for index, item in enumerate(lidar["points"])]
    clusters = [_coordinate(item, f"lidar.clusters[{index}]", with_id=True) for index, item in enumerate(lidar["clusters"])]
    if len({item["id"] for item in clusters}) != len(clusters):
        raise ValueError("lidar cluster ids must be unique")

    camera = _exact_dict(top["camera"], {"status", "bearing_rad"}, "camera")
    bearing = None if camera["bearing_rad"] is None else _number(camera["bearing_rad"], "camera.bearing_rad", -math.pi, math.pi)
    fusion = _exact_dict(top["fusion"], {"status", "objects"}, "fusion")
    if not isinstance(fusion["objects"], list) or len(fusion["objects"]) > MAX_OBJECTS:
        raise ValueError("fusion.objects exceeds bound or is not a list")
    objects = []
    for index, raw in enumerate(fusion["objects"]):
        item = _exact_dict(raw, {"id", "forward_m", "lateral_right_m", "color", "source", "status"}, f"fusion.objects[{index}]")
        color = item["color"]
        if not isinstance(color, str) or color not in VALID_COLORS:
            raise ValueError("fusion object has invalid color")
        objects.append({
            "id": _identifier(item["id"], "fusion.id"),
            "forward_m": _number(item["forward_m"], "fusion.forward_m", -100.0, 100.0),
            "lateral_right_m": _number(item["lateral_right_m"], "fusion.lateral_right_m", -100.0, 100.0),
            "color": color, "source": _text(item["source"], "fusion.source", 32),
            "status": _status(item["status"], "fusion.status"),
        })
    if len({item["id"] for item in objects}) != len(objects):
        raise ValueError("fusion object ids must be unique")

    autonomy = _exact_dict(top["autonomy"], {"status", "state", "action", "current_waypoint", "failsafe_reason"}, "autonomy")
    command = _exact_dict(top["chosen_command"], {"status", "forward_mps", "yaw_rate_rps"}, "chosen_command")
    return {
        "schema_version": SCHEMA_VERSION, "source_stamp": source_stamp,
        "server_received_monotonic": monotonic, "server_received_epoch": epoch,
        "frame_id": "base_link",
        "lidar": {"status": _status(lidar["status"], "lidar.status"), "points": points, "clusters": clusters},
        "camera": {"status": _status(camera["status"], "camera.status"), "bearing_rad": bearing},
        "fusion": {"status": _status(fusion["status"], "fusion.status"), "objects": objects},
        "autonomy": {
            "status": _status(autonomy["status"], "autonomy.status"),
            "state": _text(autonomy["state"], "autonomy.state", 64),
            "action": _text(autonomy["action"], "autonomy.action", 160),
            "current_waypoint": _integer(autonomy["current_waypoint"], "autonomy.current_waypoint", 0, 10000),
            "failsafe_reason": _text(autonomy["failsafe_reason"], "autonomy.failsafe_reason", 160),
        },
        "chosen_command": {
            "status": _status(command["status"], "chosen_command.status"),
            "forward_mps": _number(command["forward_mps"], "chosen_command.forward_mps", -5.0, 5.0),
            "yaw_rate_rps": _number(command["yaw_rate_rps"], "chosen_command.yaw_rate_rps", -5.0, 5.0),
        },
    }


class InMemoryDebugAdapter:
    """Latest-only producer store with server/source freshness metadata."""
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: dict[str, Any] | None = None

    def ingest(self, payload: Any, **clock: Any) -> dict[str, Any]:
        snapshot = normalize_snapshot(payload, **clock)
        with self._lock:
            self._latest = snapshot
        return self.display_snapshot(**clock)

    def display_snapshot(self, *, now_epoch: float | None = None, now_monotonic: float | None = None) -> dict[str, Any] | None:
        epoch = time.time() if now_epoch is None else now_epoch
        monotonic = time.monotonic() if now_monotonic is None else now_monotonic
        with self._lock:
            if self._latest is None:
                return None
            result = copy.deepcopy(self._latest)
        receive_age = monotonic - result["server_received_monotonic"]
        source_age = epoch - result["source_stamp"]
        # Clock rollback is invalid evidence. Clamping negative age to zero
        # would make an old snapshot appear freshly acquired.
        result["clock_valid"] = receive_age >= 0.0 and source_age >= 0.0
        result["server_receive_age_s"] = receive_age
        result["source_age_s"] = source_age
        result["display_stamp"] = epoch
        return result


class VehicleTestRelay:
    """One-active-run, ACKed-delivery relay for the real ROS request contract."""
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._active: dict[str, Any] | None = None
        self._history: deque[dict[str, Any]] = deque(maxlen=int(CONTRACT["terminal_history_limit"]))
        # ROS monitor replay protection requires request sequences to increase
        # across runs, not merely within a single run.
        self._request_seq = 0

    @staticmethod
    def manifest() -> dict[str, Any]:
        return {"schema_version": 1, "event": "manifest", "contract": CONTRACT["contract"], "tests": copy.deepcopy(CONTRACT["tests"])}

    @staticmethod
    def _expectations(test: dict[str, Any], raw: Any) -> dict[str, Any] | None:
        if not test["requires_expectations"]:
            if raw is not None:
                raise ValueError("basic test must not include expectations")
            return None
        keys = {"profile", "case", "expected_color", "expected_range_m", "expected_bearing_deg", "range_tolerance_m", "bearing_tolerance_deg", "expected_id"}
        value = _exact_dict(raw, keys, "expectations")
        if value["profile"] != test["default_profile"] or value["case"] not in test["allowed_cases"]:
            raise ValueError("fixture profile/case mismatch")
        color = value["expected_color"]
        if color is not None and color not in test["allowed_colors"]:
            raise ValueError("fixture color mismatch")
        if value["case"] == "positive":
            if value["expected_range_m"] is None or value["expected_bearing_deg"] is None:
                raise ValueError("positive fixture requires range and bearing")
            if test["allowed_colors"] and color is None:
                raise ValueError("positive color fixture requires color")
        result = {
            "profile": value["profile"], "case": value["case"], "expected_color": color,
            "expected_range_m": None if value["expected_range_m"] is None else _number(value["expected_range_m"], "expected_range_m", 0.05, 100.0),
            "expected_bearing_deg": None if value["expected_bearing_deg"] is None else _number(value["expected_bearing_deg"], "expected_bearing_deg", -180.0, 180.0),
            "range_tolerance_m": _number(value["range_tolerance_m"], "range_tolerance_m", 0.01, 5.0),
            "bearing_tolerance_deg": _number(value["bearing_tolerance_deg"], "bearing_tolerance_deg", 0.1, 30.0),
            "expected_id": None if value["expected_id"] is None else _identifier(value["expected_id"], "expected_id"),
        }
        return result

    def start(self, test_id: str, timeout_s: Any, expectations: Any) -> dict[str, Any]:
        test = next((item for item in CONTRACT["tests"] if item["id"] == test_id), None)
        if test is None:
            raise ValueError("test is not in canonical manifest")
        timeout = _number(timeout_s, "timeout_s", 0.5, 300.0)
        parsed_expectations = self._expectations(test, expectations)
        with self._lock:
            if self._active is not None:
                raise ValueError("one vehicle test is already active")
            self._request_seq += 1
            run_id = str(uuid.uuid4())
            request = {
                "action": "start", "run_id": run_id,
                "seq": self._request_seq, "test": test_id, "timeout_s": timeout,
            }
            if parsed_expectations is not None:
                request["expectations"] = parsed_expectations
            self._active = {"run_id": run_id, "test": test_id, "lifecycle_seq": 0, "state": "pending", "start": request, "start_sent": False, "start_inflight": False, "start_acked": False, "start_ack_event": None, "cancel": None, "cancel_sent": False, "cancel_inflight": False, "cancel_acked": False, "cancel_ack_event": None}
        return {"schema_version": 1, "event": "pending", "status": "pending", "run_id": run_id, "test_id": test_id, "seq": 0}

    def pending_requests(self) -> list[dict[str, Any]]:
        with self._lock:
            if self._active is None:
                return []
            requests = []
            if not self._active["start_acked"] and not self._active["start_inflight"]:
                requests.append(copy.deepcopy(self._active["start"]))
            if self._active["cancel"] is not None and not self._active["cancel_acked"] and not self._active["cancel_inflight"]:
                requests.append(copy.deepcopy(self._active["cancel"]))
            return requests

    def mark_sent(self, request: dict[str, Any]) -> None:
        """Reserve before network await so cancel cannot lose a maybe-sent start."""
        with self._lock:
            if self._active and request["run_id"] == self._active["run_id"]:
                self._active[f"{request['action']}_sent"] = True
                self._active[f"{request['action']}_inflight"] = True

    def release_inflight(self) -> None:
        """Allow idempotent redelivery after producer disconnect/send failure."""
        with self._lock:
            if self._active is not None:
                self._active["start_inflight"] = False
                self._active["cancel_inflight"] = False

    def acknowledge(self, payload: Any) -> dict[str, Any]:
        ack = _exact_dict(payload, {"action", "run_id", "seq", "accepted"}, "producer ack")
        if ack["action"] not in {"start", "cancel"} or not isinstance(ack["accepted"], bool):
            raise ValueError("invalid producer ack")
        with self._lock:
            run = self._active
            if run is None or ack["run_id"] != run["run_id"]:
                raise ValueError("ack is not bound to active run")
            expected = run[ack["action"]]
            if expected is None or ack["seq"] != expected["seq"]:
                raise ValueError("ack action/sequence mismatch")
            if not run[f"{ack['action']}_sent"]:
                raise ValueError("ack arrived before command delivery")
            if run[f"{ack['action']}_acked"]:
                if ack["accepted"] is not True:
                    raise ValueError("conflicting producer ack replay")
                return copy.deepcopy(run[f"{ack['action']}_ack_event"])
            if not ack["accepted"]:
                run[f"{ack['action']}_inflight"] = False
                return {"schema_version": 1, "event": "rejected", "status": "delivery_rejected", "run_id": run["run_id"], "test_id": run["test"], "seq": run["lifecycle_seq"]}
            run[f"{ack['action']}_inflight"] = False
            run[f"{ack['action']}_acked"] = True
            run["lifecycle_seq"] += 1
            event = "forwarded" if ack["action"] == "start" else "cancel_requested"
            run["state"] = event
            response = {"schema_version": 1, "event": event, "status": event, "run_id": run["run_id"], "test_id": run["test"], "seq": run["lifecycle_seq"]}
            run[f"{ack['action']}_ack_event"] = copy.deepcopy(response)
            return response

    def cancel(self, run_id: str) -> dict[str, Any]:
        with self._lock:
            run = self._active
            if run is None or run_id != run["run_id"]:
                raise ValueError("unknown run_id")
            if not run["start_sent"]:
                event = {"schema_version": 1, "event": "cancelled", "status": "CANCELLED", "run_id": run_id, "test_id": run["test"], "seq": run["lifecycle_seq"] + 1, "message": "cancelled before producer delivery", "artifact": None}
                self._history.append(copy.deepcopy(event)); self._active = None
                return event
            if run["cancel"] is not None:
                raise ValueError("cancel already requested")
            self._request_seq += 1
            run["cancel"] = {
                "action": "cancel", "run_id": run_id, "seq": self._request_seq,
            }
            run["lifecycle_seq"] += 1; run["state"] = "cancel_requested"
            return {"schema_version": 1, "event": "cancel_requested", "status": "cancel_requested", "run_id": run_id, "test_id": run["test"], "seq": run["lifecycle_seq"]}

    def accept_ros(self, topic: str, payload: Any) -> dict[str, Any]:
        if topic not in {"/vehicle_test/status", "/vehicle_test/events", "/vehicle_test/result"}:
            raise ValueError("unsupported ROS vehicle_test topic")
        if not isinstance(payload, dict) or payload.get("schema_version") != 1:
            raise ValueError("invalid ROS vehicle_test payload")
        with self._lock:
            run = self._active
            if run is None and topic == "/vehicle_test/result":
                # Producer retains terminal evidence until this process sends an
                # application ACK. A socket drop after acceptance therefore
                # causes an exact replay, which must be idempotent.
                for terminal in reversed(self._history):
                    if terminal.get("event") == "result" and terminal.get("raw") == payload:
                        return copy.deepcopy(terminal)
            if run is None or payload.get("run_id") != run["run_id"] or payload.get("test") != run["test"] or payload.get("seq") != run["start"]["seq"]:
                raise ValueError("ROS payload is not bound to active request")
            if not run["start_acked"]:
                raise ValueError("ROS payload arrived before delivery ACK")
            if topic == "/vehicle_test/events":
                _exact_dict(payload, {"schema_version", "run_id", "seq", "test", "event", "monotonic_s", "stamp", "actuation_enabled"}, "ROS event")
                if payload["event"] not in {"STARTED", "PASS", "FAIL", "CANCELLED"}:
                    raise ValueError("invalid ROS event")
                if payload["actuation_enabled"] is not False:
                    raise ValueError("ROS event attempted actuation")
                _number(payload["monotonic_s"], "event.monotonic_s", 0.0, 1e12)
                _number(payload["stamp"], "event.stamp", 0.0, 1e12)
                return {"schema_version": 1, "event": "monitor_event", "status": payload["event"], "run_id": run["run_id"], "test_id": run["test"], "seq": run["lifecycle_seq"], "raw": copy.deepcopy(payload)}
            status_keys = {"schema_version", "run_id", "seq", "test", "state", "actuation_enabled", "remaining_s", "reasons", "expectations", "stamp"}
            if payload.get("state") not in {"RUNNING", "PASS", "FAIL", "CANCELLED"} or payload.get("actuation_enabled") is not False:
                raise ValueError("invalid ROS passive status")
            if payload.get("remaining_s") is not None:
                _number(payload["remaining_s"], "status.remaining_s", 0.0, 300.0)
            _number(payload.get("stamp"), "status.stamp", 0.0, 1e12)
            reasons = payload.get("reasons")
            if not isinstance(reasons, list) or len(reasons) > 100 or any(not isinstance(item, str) or len(item) > 256 for item in reasons):
                raise ValueError("invalid ROS status reasons")
            expected_expectations = run["start"].get("expectations")
            if payload.get("expectations") != expected_expectations:
                raise ValueError("ROS status expectations do not match request")
            if topic == "/vehicle_test/status":
                _exact_dict(payload, status_keys, "ROS status")
                return {"schema_version": 1, "event": "monitor_status", "status": payload["state"], "run_id": run["run_id"], "test_id": run["test"], "seq": run["lifecycle_seq"], "raw": copy.deepcopy(payload)}
            result_keys = status_keys | {"pass", "artifact", "evidence_manifest", "bundle_correlations", "channels"}
            _exact_dict(payload, result_keys, "ROS result")
            if payload["state"] not in RESULT_STATUSES or not isinstance(payload["pass"], bool) or payload["pass"] != (payload["state"] == "PASS"):
                raise ValueError("invalid ROS terminal result")
            if not isinstance(payload["evidence_manifest"], dict) or not isinstance(payload["bundle_correlations"], list) or not isinstance(payload["channels"], dict):
                raise ValueError("invalid ROS evidence payload")
            try:
                raw_size = len(json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8"))
            except (TypeError, ValueError) as exc:
                raise ValueError("invalid ROS result JSON") from exc
            if raw_size > 524288:
                raise ValueError("ROS result exceeds bound")
            artifact = payload["artifact"]
            if artifact is not None:
                artifact = _exact_dict(artifact, {"path", "sha256"}, "artifact")
                if not isinstance(artifact["path"], str) or not 1 <= len(artifact["path"]) <= 4096 or not re.fullmatch(r"[0-9a-f]{64}", str(artifact["sha256"])):
                    raise ValueError("invalid artifact/hash")
            event = {"schema_version": 1, "event": "result", "status": payload["state"], "run_id": run["run_id"], "test_id": run["test"], "seq": run["lifecycle_seq"] + 1, "pass": payload["pass"], "artifact": copy.deepcopy(artifact), "raw": copy.deepcopy(payload)}
            self._history.append(copy.deepcopy(event)); self._active = None
            return event

    def terminal_history(self) -> list[dict[str, Any]]:
        with self._lock:
            return copy.deepcopy(list(self._history))


async def handle_producer_message(message: Any, adapter: InMemoryDebugAdapter, relay: VehicleTestRelay) -> tuple[str, dict[str, Any]]:
    """Pure async producer handler used by the WebSocket endpoint and contract tests."""
    envelope = _exact_dict(message, {"type", "data"}, "producer envelope")
    if envelope["type"] == "debug_snapshot":
        return "debug_snapshot", adapter.ingest(envelope["data"])
    if envelope["type"] == "vehicle_test_ack":
        return "vehicle_test", relay.acknowledge(envelope["data"])
    if envelope["type"] == "vehicle_test_ros":
        ros = _exact_dict(envelope["data"], {"topic", "payload"}, "ROS envelope")
        return "vehicle_test", relay.accept_ros(ros["topic"], ros["payload"])
    raise ValueError("producer accepts only debug_snapshot, vehicle_test_ack or vehicle_test_ros")
