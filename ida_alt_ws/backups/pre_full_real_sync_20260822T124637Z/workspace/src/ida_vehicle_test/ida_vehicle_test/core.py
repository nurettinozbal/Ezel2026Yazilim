"""Pure passive test state machine and evidence evaluators."""

from __future__ import annotations

import math
import hashlib
import json
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple

from .contracts import FixtureExpectations, TestRequest


@dataclass(frozen=True)
class ChannelRule:
    topic: str
    min_samples: int
    min_hz: float
    max_receive_age_s: float
    max_source_age_s: float
    publisher_count: int = 1


@dataclass(frozen=True)
class TestRule:
    channels: Tuple[str, ...]


CHANNEL_RULES: Dict[str, ChannelRule] = {
    # Composite identity advances with the slowest required source (~1 Hz).
    # Allow scheduler jitter while still requiring three unique acquisitions.
    "bridge_status": ChannelRule("/control/mavsdk_status", 3, 0.9, 1.0, 1.5),
    # Current real bridge has no structured health topic.  Comms therefore uses
    # the existing fresh telemetry stream plus graph identity of its publisher.
    # The bridge republishes at 10 Hz, but the composite acquisition stamp is
    # intentionally the oldest required field (normally the ~1 Hz armed
    # stream). Require five genuinely distinct composite acquisitions and
    # tolerate scheduler jitter; duplicate republishing is never evidence.
    "comms_telemetry": ChannelRule("/telemetry/state", 5, 0.9, 1.5, 1.5),
    "telemetry": ChannelRule("/telemetry/state", 5, 0.9, 1.5, 1.5),
    "camera_p1p2": ChannelRule("/perception/camera/p1p2/raw", 3, 2.0, 0.5, 0.5),
    "camera_p3": ChannelRule("/perception/camera/p3/raw", 3, 2.0, 0.5, 0.5),
    "lidar": ChannelRule("/perception/lidar/raw_obstacles", 3, 5.0, 0.5, 0.5),
    "fusion_status": ChannelRule("/perception/fusion/status", 3, 2.0, 0.5, 0.5),
    "fusion_buoys": ChannelRule("/perception/fusion/shadow/buoys", 3, 2.0, 0.5, 0.5),
    "fusion_obstacles": ChannelRule("/perception/fusion/shadow/obstacles", 3, 2.0, 0.5, 0.5),
    "autonomy_state": ChannelRule("/autonomy/state", 3, 2.0, 0.5, 0.5),
    "autonomy_debug": ChannelRule("/autonomy/debug", 3, 2.0, 0.5, 0.5),
    "costmap": ChannelRule("/planning/costmap", 3, 2.0, 0.5, 0.5),
    "logging": ChannelRule("/logging/status", 2, 0.5, 2.0, 2.0),
}

TEST_RULES: Dict[str, TestRule] = {
    "comms": TestRule(("bridge_status", "comms_telemetry")),
    "telemetry": TestRule(("bridge_status", "telemetry")),
    "camera_p1p2": TestRule(("camera_p1p2",)),
    "camera_p3": TestRule(("camera_p3",)),
    "lidar": TestRule(("lidar",)),
    "fusion_shadow": TestRule(("fusion_status", "fusion_buoys", "fusion_obstacles")),
    "autonomy_shadow": TestRule(("autonomy_state", "autonomy_debug", "costmap")),
    "logging": TestRule(("logging",)),
}

DEFAULT_PUBLISHER_NODE_ALLOWLIST: Dict[str, set[str]] = {
    "bridge_status": {"ida_mavsdk_bridge"},
    "comms_telemetry": {"ida_mavsdk_bridge"},
    "telemetry": {"ida_mavsdk_bridge"},
    "camera_p1p2": {"ida_yolo_camera"},
    "camera_p3": {"ida_yolo_camera_p3"},
    "lidar": {"ida_sllidar_bridge"},
    "fusion_status": {"ida_sensor_fusion"},
    "fusion_buoys": {"ida_sensor_fusion"},
    "fusion_obstacles": {"ida_sensor_fusion"},
    "autonomy_state": {"ida_autonomy"},
    "autonomy_debug": {"ida_autonomy"},
    "costmap": {"ida_autonomy"},
    "logging": {"ida_logging_status"},
}


@dataclass
class Evidence:
    receives: List[float] = field(default_factory=list)
    source_stamps: List[float] = field(default_factory=list)
    last_source_stamp: float | None = None
    last_metrics: Dict[str, Any] = field(default_factory=dict)
    sample_metrics: List[Dict[str, Any]] = field(default_factory=list)

    def add(self, receive_mono: float, source_stamp: float, metrics: Mapping[str, Any]) -> bool:
        # Retransmission of one acquisition cannot manufacture sample rate or
        # persistence evidence.  Keep the original receive time unchanged.
        if source_stamp in self.source_stamps:
            return False
        self.receives.append(receive_mono)
        self.source_stamps.append(source_stamp)
        if len(self.receives) > 100:
            del self.receives[:-100]
            del self.source_stamps[:-100]
        self.last_source_stamp = source_stamp
        self.last_metrics = dict(metrics)
        self.sample_metrics.append(dict(metrics))
        if len(self.sample_metrics) > 100:
            del self.sample_metrics[:-100]
        return True


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name}_invalid")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name}_invalid") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name}_nonfinite")
    return number


def _nonnegative_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name}_invalid")
    return value


def _source_stamp(payload: Mapping[str, Any]) -> float:
    return _finite_number(payload.get("acquisition_stamp", payload.get("stamp")), "stamp")


def _list(payload: Mapping[str, Any], name: str) -> list:
    value = payload.get(name)
    if not isinstance(value, list):
        raise ValueError(f"{name}_invalid")
    if len(value) > 20000:
        raise ValueError(f"{name}_item_limit")
    return value


def _not_stale(payload: Mapping[str, Any]) -> None:
    if payload.get("stale") is True:
        raise ValueError("source_stale")


def _validate_detection(det: Any, colors: set[str]) -> None:
    if not isinstance(det, dict) or str(det.get("color", "")).lower() not in colors:
        raise ValueError("detection_invalid")
    _finite_number(det.get("bearing_deg"), "bearing_deg")
    confidence = _finite_number(det.get("confidence"), "confidence")
    if not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence_out_of_range")


def _detection_matches(det: Mapping[str, Any], expected: FixtureExpectations) -> bool:
    if expected.expected_color is not None and str(det.get("color", "")).lower() != expected.expected_color:
        return False
    if expected.expected_id is not None and str(det.get("id", "")) != expected.expected_id:
        return False
    if expected.expected_bearing_deg is not None:
        bearing = _finite_number(det.get("bearing_deg"), "bearing_deg")
        if _angle_error(bearing, expected.expected_bearing_deg) > expected.bearing_tolerance_deg:
            return False
    if expected.expected_range_m is not None:
        distance = _finite_number(det.get("distance"), "distance")
        if abs(distance - expected.expected_range_m) > expected.range_tolerance_m:
            return False
    return True


def _cluster_geometry(cluster: Mapping[str, Any]) -> Tuple[float, float]:
    forward = _finite_number(cluster.get("forward_m"), "forward_m")
    lateral_left = _finite_number(cluster.get("lateral_left_m"), "lateral_left_m")
    return math.hypot(forward, lateral_left), math.degrees(math.atan2(-lateral_left, forward))


def _angle_error(a_deg: float, b_deg: float) -> float:
    return abs((a_deg - b_deg + 180.0) % 360.0 - 180.0)


def _cluster_matches(cluster: Mapping[str, Any], expected: FixtureExpectations) -> bool:
    distance, bearing = _cluster_geometry(cluster)
    if expected.expected_id is not None and str(cluster.get("id", "")) != expected.expected_id:
        return False
    if expected.expected_range_m is not None and abs(distance - expected.expected_range_m) > expected.range_tolerance_m:
        return False
    if expected.expected_bearing_deg is not None and _angle_error(bearing, expected.expected_bearing_deg) > expected.bearing_tolerance_deg:
        return False
    return True


def _validate_cluster(cluster: Any) -> None:
    if not isinstance(cluster, dict):
        raise ValueError("cluster_invalid")
    forward = _finite_number(cluster.get("forward_m"), "forward_m")
    _finite_number(cluster.get("lateral_left_m"), "lateral_left_m")
    if forward <= 0.0:
        raise ValueError("cluster_not_forward")


def validate_payload(
    channel: str, payload: Any, expected: FixtureExpectations | None = None
) -> Tuple[float | None, Dict[str, Any]]:
    if not isinstance(payload, dict):
        raise ValueError("payload_not_object")
    special_fusion = expected is not None and expected.case in {"ambiguity", "lidar_only", "camera_only"}
    if not special_fusion:
        _not_stale(payload)
    # Costmap has no source stamp in the existing contract.  It is therefore
    # freshness-checked exclusively with the monitor's monotonic receive time.
    if channel == "costmap":
        cells = _list(payload, "cells")
        cell_m = _finite_number(payload.get("cell_m"), "cell_m")
        if cell_m <= 0.0:
            raise ValueError("cell_m_out_of_range")
        for cell in cells:
            if not isinstance(cell, list) or len(cell) != 4:
                raise ValueError("costmap_cell_invalid")
            for value in cell[:3]:
                _finite_number(value, "costmap_cell")
            if not isinstance(cell[3], str):
                raise ValueError("costmap_tag_invalid")
        digest = hashlib.sha256(
            json.dumps(payload, ensure_ascii=True, sort_keys=True, allow_nan=False, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
        return None, {"cell_count": len(cells), "_sample_identity": f"costmap:{digest}"}
    stamp = _source_stamp(payload)

    if channel == "bridge_status":
        if (
            payload.get("dry_run") is not False
            or payload.get("connected") is not True
            or payload.get("heartbeat_fresh") is not True
            or payload.get("telemetry_ready") is not True
        ):
            raise ValueError("bridge_health_not_verified")
        receive_age = _finite_number(payload.get("last_receive_age_s"), "last_receive_age_s")
        source_age = _finite_number(payload.get("source_age_s"), "source_age_s")
        if not 0.0 <= receive_age <= 1.5 or not -0.25 <= source_age <= 1.5:
            raise ValueError("bridge_health_stale")
        return stamp, {
            "connected": True,
            "heartbeat_fresh": True,
            "dry_run": False,
            "last_receive_age_s": receive_age,
            "source_age_s": source_age,
        }
    if channel in {"comms_telemetry", "telemetry"}:
        for name in ("lat", "lon", "heading_deg", "ground_speed", "roll_deg", "pitch_deg"):
            _finite_number(payload.get(name), name)
        if not isinstance(payload.get("mode"), str) or not payload["mode"]:
            raise ValueError("mode_invalid")
        if not -90.0 <= float(payload["lat"]) <= 90.0 or not -180.0 <= float(payload["lon"]) <= 180.0:
            raise ValueError("geographic_position_out_of_range")
        if float(payload["ground_speed"]) < 0.0:
            raise ValueError("ground_speed_out_of_range")
        return stamp, {
            "mode": payload["mode"],
            "ground_speed": float(payload["ground_speed"]),
            "bridge_stream_present": channel == "comms_telemetry",
        }
    if channel in {"camera_p1p2", "camera_p3"}:
        detections = _list(payload, "detections")
        if expected is None:
            raise ValueError("fixture_expectations_missing")
        if expected.case == "negative":
            if detections:
                raise ValueError("unexpected_detection")
            return stamp, {"detection_count": 0, "fixture_match": True}
        if not detections:
            raise ValueError("no_detection_evidence")
        colors = {"orange", "yellow"} if channel == "camera_p1p2" else {"red", "green", "black"}
        for detection in detections:
            _validate_detection(detection, colors)
        matches = [item for item in detections if _detection_matches(item, expected)]
        if len(matches) != 1:
            raise ValueError("fixture_detection_mismatch_or_ambiguous")
        matched_item = matches[0]
        return stamp, {
            "detection_count": len(detections), "fixture_match": True, "matched": matched_item,
            "measurement": {
                "id": matched_item.get("id"), "color": matched_item.get("color"),
                "bearing_deg": matched_item.get("bearing_deg"),
                "range_m": matched_item.get("distance"), "lidar_id": None,
            },
        }
    if channel == "lidar":
        clusters = _list(payload, "clusters")
        if expected is None:
            raise ValueError("fixture_expectations_missing")
        if expected.case == "negative":
            if clusters:
                raise ValueError("unexpected_cluster")
            return stamp, {"cluster_count": 0, "fixture_match": True}
        if not clusters:
            raise ValueError("no_cluster_evidence")
        for cluster in clusters:
            _validate_cluster(cluster)
        matches = [item for item in clusters if _cluster_matches(item, expected)]
        if len(matches) != 1:
            raise ValueError("fixture_cluster_mismatch_or_ambiguous")
        distance, bearing = _cluster_geometry(matches[0])
        return stamp, {
            "cluster_count": len(clusters), "fixture_match": True,
            "matched": dict(matches[0]), "range_m": distance, "bearing_deg": bearing,
            "measurement": {
                "id": matches[0].get("id"), "color": None,
                "bearing_deg": bearing, "range_m": distance, "lidar_id": matches[0].get("id"),
            },
        }
    if channel == "fusion_status":
        if expected is None:
            raise ValueError("fixture_expectations_missing")
        if payload.get("shadow_mode") is not True:
            raise ValueError("fusion_not_shadow")
        matched = _nonnegative_int(payload.get("matched_count", 0), "matched_count")
        if expected.case == "positive":
            if payload.get("camera_fresh") is not True or payload.get("lidar_fresh") is not True:
                raise ValueError("fusion_source_stale")
            if payload.get("accepted") is not True or payload.get("source_health") != "ok" or matched < 1:
                raise ValueError("fusion_not_accepted")
            dt_ms = _finite_number(payload.get("dt_ms"), "dt_ms")
            if abs(dt_ms) > 100.0:
                raise ValueError("fusion_time_not_correlated")
        elif expected.case == "ambiguity":
            ambiguous = _nonnegative_int(payload.get("ambiguous_camera_count", 0), "ambiguous_camera_count")
            if (
                payload.get("camera_fresh") is not True
                or payload.get("lidar_fresh") is not True
                or payload.get("accepted") is not True
                or payload.get("source_health") != "ok"
                or ambiguous < 1
                or matched != 0
            ):
                raise ValueError("fusion_ambiguity_not_observed")
            dt_ms = _finite_number(payload.get("dt_ms"), "dt_ms")
        elif expected.case == "lidar_only":
            if (
                payload.get("camera_fresh") is not False
                or payload.get("lidar_fresh") is not True
                or payload.get("accepted") is not False
                or payload.get("source_health") != "camera_stale"
                or matched != 0
            ):
                raise ValueError("fusion_lidar_only_not_observed")
            dt_ms = _finite_number(payload.get("dt_ms"), "dt_ms")
        elif expected.case == "camera_only":
            if (
                payload.get("camera_fresh") is not True
                or payload.get("lidar_fresh") is not False
                or payload.get("accepted") is not False
                or payload.get("source_health") != "lidar_stale"
                or payload.get("dt_ms") is not None
                or matched != 0
            ):
                raise ValueError("fusion_camera_only_not_observed")
            dt_ms = None
        elif expected.case == "negative":
            if (
                payload.get("camera_fresh") is not True
                or payload.get("lidar_fresh") is not True
                or payload.get("accepted") is not True
                or payload.get("source_health") != "ok"
                or matched != 0
            ):
                raise ValueError("fusion_negative_not_observed")
            dt_ms = _finite_number(payload.get("dt_ms"), "dt_ms")
        else:
            raise ValueError("fusion_case_invalid")
        return stamp, {
            "matched_count": matched, "dt_ms": dt_ms, "fixture_case": expected.case,
            "measurement": {"dt_ms": dt_ms},
        }
    if channel == "fusion_buoys":
        detections = _list(payload, "detections")
        if expected is None:
            raise ValueError("fixture_expectations_missing")
        if expected.case != "positive":
            if detections:
                raise ValueError("unsafe_fusion_color_output")
            expected_stale = expected.case in {"lidar_only", "camera_only"}
            if bool(payload.get("stale", False)) != expected_stale:
                raise ValueError("fusion_buoy_stale_contract_mismatch")
            return stamp, {"detection_count": 0, "detections": []}
        if not detections:
            raise ValueError("fusion_no_buoy")
        for detection in detections:
            _validate_detection(detection, {"orange", "yellow", "red", "green", "black"})
        matches = [item for item in detections if _detection_matches(item, expected)]
        if len(matches) != 1:
            raise ValueError("fusion_buoy_fixture_mismatch")
        matched_item = matches[0]
        return stamp, {
            "detection_count": len(detections), "detections": detections, "matched": matched_item,
            "measurement": {
                "id": matched_item.get("id"), "color": matched_item.get("color"),
                "bearing_deg": matched_item.get("bearing_deg"),
                "range_m": matched_item.get("distance"),
                "lidar_id": matched_item.get("lidar_obstacle_id"),
            },
        }
    if channel == "fusion_obstacles":
        obstacles = _list(payload, "obstacles")
        if expected is None:
            raise ValueError("fixture_expectations_missing")
        if expected.case in {"camera_only", "negative"}:
            if obstacles:
                raise ValueError("empty_fixture_created_metric_obstacle")
            if bool(payload.get("stale", False)) != (expected.case == "camera_only"):
                raise ValueError("fusion_obstacle_stale_contract_mismatch")
            return stamp, {"obstacle_count": 0, "obstacles": []}
        if not obstacles:
            raise ValueError("fusion_no_obstacle")
        for obstacle in obstacles:
            if not isinstance(obstacle, dict) or obstacle.get("hard_obstacle") is not True:
                raise ValueError("fusion_obstacle_not_hard")
            _finite_number(obstacle.get("forward_m"), "forward_m")
            _finite_number(obstacle.get("lateral_m"), "lateral_m")
            if expected.case in {"ambiguity", "lidar_only"} and obstacle.get("color") != "unknown":
                raise ValueError("unmatched_lidar_not_unknown")
        if payload.get("stale") is True:
            raise ValueError("metric_lidar_obstacle_marked_stale")
        return stamp, {"obstacle_count": len(obstacles), "obstacles": obstacles}
    if channel == "autonomy_state":
        if not isinstance(payload.get("state"), str) or not payload["state"]:
            raise ValueError("autonomy_state_invalid")
        if payload.get("failsafe_reason") not in (None, ""):
            raise ValueError("autonomy_failsafe")
        return stamp, {"state": payload["state"]}
    if channel == "autonomy_debug":
        command = payload.get("command")
        if not isinstance(command, dict):
            raise ValueError("autonomy_command_missing")
        for name in ("vx", "vy", "yaw_rate"):
            _finite_number(command.get(name), name)
        if payload.get("costmap") is not True:
            raise ValueError("autonomy_costmap_inactive")
        return stamp, {"action": command.get("action", "")}
    if channel == "logging":
        if payload.get("path_policy_valid") is not True:
            raise ValueError("logging_path_policy_invalid")
        if payload.get("active") is not True:
            raise ValueError("logging_inactive")
        count = _nonnegative_int(payload.get("logger_count"), "logger_count")
        files = _list(payload, "files")
        expected_names = {"telemetry.csv", "processed_video.mp4", "map.mp4"}
        if (
            count != 3
            or len(files) != 3
            or any(not isinstance(path, str) or not path for path in files)
            or {Path(path).name for path in files} != expected_names
        ):
            raise ValueError("logging_incomplete")
        growing = _nonnegative_int(payload.get("growing_file_count", 0), "growing_file_count")
        return stamp, {"logger_count": count, "growing_file_count": growing}
    raise ValueError("unknown_channel")


class PassiveTestCore:
    """Single-run state machine driven only by observations and monotonic time."""

    def __init__(self, publisher_node_allowlist: Mapping[str, Sequence[str]] | None = None) -> None:
        self.state = "IDLE"
        self.run_id: str | None = None
        self.seq: int | None = None
        self.test: str | None = None
        self.started_mono: float | None = None
        self.started_ros: float | None = None
        self.deadline_mono: float | None = None
        self.last_mono: float | None = None
        self.evidence: Dict[str, Evidence] = {}
        self.graph_counts: Dict[str, int] = {}
        self.graph_checked_mono: float | None = None
        self.reasons: List[str] = []
        self.events: List[Dict[str, Any]] = []
        self._result_pending = False
        self._last_request_seq = -1
        self._request_history: Dict[int, TestRequest] = {}
        self._history_order: deque[int] = deque()
        self._seen_run_ids: set[str] = set()
        self._run_order: deque[str] = deque()
        configured = publisher_node_allowlist or DEFAULT_PUBLISHER_NODE_ALLOWLIST
        self.publisher_node_allowlist = {name: set(nodes) for name, nodes in configured.items()}
        if set(self.publisher_node_allowlist) != set(CHANNEL_RULES):
            raise ValueError("publisher node allow-list must cover every channel")
        self._pass_ready = False
        self._artifact: Dict[str, Any] | None = None
        self._bundle_correlations: List[Dict[str, Any]] = []
        self._persistence_attempted = False

    @property
    def active(self) -> bool:
        return self.state == "RUNNING"

    def _check_mono(self, now: float) -> bool:
        if not math.isfinite(now):
            self._finish("FAIL", "monotonic_time_invalid")
            return False
        if self.last_mono is not None and now < self.last_mono:
            self._finish("FAIL", "monotonic_clock_rollback")
            return False
        self.last_mono = now
        return True

    def request(self, request: TestRequest, now_mono: float, ros_now: float) -> None:
        if not self._check_mono(now_mono):
            return
        ros_now = _finite_number(ros_now, "ros_now")
        previous = self._request_history.get(request.seq)
        if previous is not None:
            if previous == request:
                return  # exact transport replay is idempotent
            raise ValueError("request seq was already used by a different request")
        if request.seq <= self._last_request_seq:
            raise ValueError("request seq must increase monotonically")
        if request.action == "cancel":
            if not self.active or request.run_id != self.run_id:
                raise ValueError("cancel does not match the active run")
            self._record_request(request)
            self._last_request_seq = request.seq
            self._finish("CANCELLED", "operator_cancelled")
            return
        if self.active:
            raise ValueError("a test run is already active")
        if self._result_pending:
            raise ValueError("previous terminal result has not been consumed")
        if request.run_id in self._seen_run_ids:
            raise ValueError("run_id was already used")
        assert request.test is not None and request.timeout_s is not None
        self.state = "RUNNING"
        self.run_id = request.run_id
        self.seq = request.seq
        self.test = request.test
        self.started_mono = now_mono
        self.started_ros = ros_now
        self.deadline_mono = now_mono + request.timeout_s
        self.evidence = {name: Evidence() for name in TEST_RULES[request.test].channels}
        self.graph_counts = {}
        self.graph_checked_mono = None
        self.reasons = []
        self._result_pending = False
        self._pass_ready = False
        self._artifact = None
        self._bundle_correlations = []
        self._persistence_attempted = False
        self._record_request(request)
        self._seen_run_ids.add(request.run_id)
        self._run_order.append(request.run_id)
        while len(self._run_order) > 256:
            self._seen_run_ids.discard(self._run_order.popleft())
        self._last_request_seq = request.seq
        self.events.append(self._event("STARTED", now_mono))

    def _record_request(self, request: TestRequest) -> None:
        self._request_history[request.seq] = request
        self._history_order.append(request.seq)
        while len(self._history_order) > 256:
            expired = self._history_order.popleft()
            self._request_history.pop(expired, None)

    def observe(self, channel: str, payload: Any, receive_mono: float, ros_now: float) -> None:
        if not self.active or channel not in self.evidence:
            return
        if not self._check_mono(receive_mono):
            return
        try:
            ros_now = _finite_number(ros_now, "ros_now")
            stamp, metrics = validate_payload(channel, payload, self._expectations())
            if stamp is not None:
                assert self.started_ros is not None
                age = ros_now - stamp
                max_age = CHANNEL_RULES[channel].max_source_age_s
                if age < -0.25:
                    raise ValueError("source_stamp_stale_or_future")
                if age > max_age:
                    # A bounded DDS queue may still contain older samples when
                    # START arrives.  Old data cannot refresh evidence; ignore
                    # it and let a fresh sample arrive or the test time out.
                    return
                if stamp < self.started_ros:
                    # A producer or DDS queue may deliver one acquisition made
                    # just before START after the request callback.  It must
                    # never count toward PASS, but failing the entire fixture
                    # would make healthy live streams race-dependent.  Ignore
                    # it; repeated stale/pre-start frames eventually timeout.
                    return
                previous_stamp = self.evidence[channel].last_source_stamp
                if previous_stamp is not None and stamp < previous_stamp:
                    raise ValueError("source_stamp_out_of_order")
        except (TypeError, ValueError, OverflowError) as exc:
            self._finish("FAIL", f"{channel}:{exc}")
            return
        identity = metrics.pop("_sample_identity", ros_now if stamp is None else stamp)
        self.evidence[channel].add(receive_mono, identity, metrics)

    def _expectations(self) -> FixtureExpectations | None:
        if self.test is None:
            return None
        request = self._request_history.get(self.seq if self.seq is not None else -1)
        return request.expectations if request is not None else None

    def observe_graph(self, counts: Mapping[str, Any], now_mono: float) -> None:
        if not self.active or not self._check_mono(now_mono):
            return
        for channel in self.evidence:
            expected = CHANNEL_RULES[channel].publisher_count
            raw = counts.get(channel)
            if not isinstance(raw, dict) or raw.get("error"):
                self._finish("FAIL", f"{channel}:graph_introspection_failed")
                return
            count = raw.get("count")
            types = raw.get("types")
            nodes = raw.get("nodes")
            if isinstance(count, bool) or not isinstance(count, int) or count != expected:
                self._finish("FAIL", f"{channel}:publisher_count_expected_{expected}_got_{count}")
                return
            if types != ["std_msgs/msg/String"]:
                self._finish("FAIL", f"{channel}:publisher_type_mismatch")
                return
            if not isinstance(nodes, list) or len(nodes) != 1 or nodes[0] not in self.publisher_node_allowlist[channel]:
                self._finish("FAIL", f"{channel}:publisher_node_not_allowed")
                return
            self.graph_counts[channel] = count
        self.graph_checked_mono = now_mono

    def tick(self, now_mono: float) -> None:
        if not self.active or not self._check_mono(now_mono):
            return
        assert self.deadline_mono is not None
        if now_mono >= self.deadline_mono:
            self._finish("FAIL", "test_timeout")
            return
        # PASS requires a graph snapshot from this exact evaluation cycle; an
        # old single-writer observation is not sufficient.
        if set(self.graph_counts) != set(self.evidence) or self.graph_checked_mono != now_mono:
            return
        for channel, evidence in self.evidence.items():
            rule = CHANNEL_RULES[channel]
            if not evidence.receives:
                return
            age = now_mono - evidence.receives[-1]
            if age < 0.0 or age > rule.max_receive_age_s:
                self._finish("FAIL", f"{channel}:receive_stale")
                return
            if len(evidence.receives) < rule.min_samples:
                return
            span = evidence.receives[-1] - evidence.receives[0]
            if span <= 0.0:
                return
            rate = (len(evidence.receives) - 1) / span
            if rate + 1e-9 < rule.min_hz:
                self._finish("FAIL", f"{channel}:rate_below_{rule.min_hz:g}hz")
                return
            if channel == "logging" and evidence.last_metrics.get("growing_file_count", 0) < 3:
                return
        if not self._cross_channel_fixture_valid():
            self._finish("FAIL", "fixture_cross_correlation_failed")
            return
        self._pass_ready = True

    @property
    def pass_ready(self) -> bool:
        return self.active and self._pass_ready

    def pass_candidate(self, now_mono: float) -> Dict[str, Any]:
        if not self.pass_ready:
            raise ValueError("PASS evidence is not ready")
        return {
            **self.status(now_mono),
            "state": "PASS",
            "pass": True,
            "evidence_manifest": self.evidence_manifest(),
            "bundle_correlations": list(self._bundle_correlations),
            **self.metrics(now_mono),
        }

    def commit_pass(self, artifact: Mapping[str, Any]) -> None:
        if not self.pass_ready:
            raise ValueError("PASS evidence is not ready")
        path = artifact.get("path")
        digest = artifact.get("sha256")
        if not isinstance(path, str) or not path or not isinstance(digest, str) or len(digest) != 64:
            raise ValueError("artifact record is invalid")
        self._artifact = {"path": path, "sha256": digest}
        self._persistence_attempted = True
        self._pass_ready = False
        self._finish("PASS", "observed_evidence_persisted")

    def fail_persistence(self) -> None:
        if self.pass_ready:
            self._pass_ready = False
            self._finish("FAIL", "evidence_persist_failed")
            self._persistence_attempted = True

    @property
    def terminal_needs_persistence(self) -> bool:
        return (
            self.state in {"PASS", "FAIL", "CANCELLED"}
            and self._result_pending
            and self._artifact is None
            and not self._persistence_attempted
        )

    def terminal_candidate(self, now_mono: float) -> Dict[str, Any]:
        if not self.terminal_needs_persistence:
            raise ValueError("terminal evidence is not pending persistence")
        return self._result_payload(now_mono)

    def attach_terminal_artifact(self, artifact: Mapping[str, Any]) -> None:
        if not self.terminal_needs_persistence:
            raise ValueError("terminal evidence is not pending persistence")
        path = artifact.get("path")
        digest = artifact.get("sha256")
        if not isinstance(path, str) or not path or not isinstance(digest, str) or len(digest) != 64:
            raise ValueError("artifact record is invalid")
        self._artifact = {"path": path, "sha256": digest}
        self._persistence_attempted = True

    def mark_terminal_persistence_failed(self) -> None:
        if self.terminal_needs_persistence:
            if "evidence_persist_failed" not in self.reasons:
                self.reasons.append("evidence_persist_failed")
            self._persistence_attempted = True

    def _cross_channel_fixture_valid(self) -> bool:
        expected = self._expectations()
        if self.test != "fusion_shadow" or expected is None:
            return True
        channels = TEST_RULES["fusion_shadow"].channels
        bundle_count = min(len(self.evidence[name].source_stamps) for name in channels)
        required = max(CHANNEL_RULES[name].min_samples for name in channels)
        if bundle_count < required:
            return False
        correlations: List[Dict[str, Any]] = []
        for offset in range(-required, 0):
            stamps = [self.evidence[name].source_stamps[offset] for name in channels]
            if any(not isinstance(stamp, (int, float)) for stamp in stamps):
                return False
            if max(stamps) - min(stamps) > 0.100000001:
                return False
            metrics = {name: self.evidence[name].sample_metrics[offset] for name in channels}
            correlated, summary = self._bundle_valid(expected, metrics, stamps)
            if not correlated:
                return False
            correlations.append(summary)
        self._bundle_correlations = correlations
        return True

    def _bundle_valid(
        self,
        expected: FixtureExpectations,
        metrics: Mapping[str, Mapping[str, Any]],
        stamps: Sequence[float],
    ) -> Tuple[bool, Dict[str, Any]]:
        status_metrics = metrics["fusion_status"]
        if expected.case != "positive":
            return True, {
                "correlated": True,
                "case": expected.case,
                "stamps": list(stamps),
                "dt_ms": status_metrics.get("dt_ms"),
                "matched_count": status_metrics.get("matched_count"),
            }
        buoy = metrics["fusion_buoys"].get("matched")
        obstacles = metrics["fusion_obstacles"].get("obstacles", [])
        if not isinstance(buoy, dict) or not isinstance(obstacles, list):
            return False, {}
        lidar_id = buoy.get("lidar_obstacle_id")
        correlated = [item for item in obstacles if isinstance(item, dict) and item.get("id") == lidar_id]
        if len(correlated) != 1:
            return False, {}
        obstacle = correlated[0]
        try:
            distance = math.hypot(
                _finite_number(obstacle.get("forward_m"), "forward_m"),
                _finite_number(obstacle.get("lateral_m"), "lateral_m"),
            )
            bearing = math.degrees(math.atan2(float(obstacle["lateral_m"]), float(obstacle["forward_m"])))
        except (KeyError, TypeError, ValueError):
            return False, {}
        valid = (
            str(buoy.get("color", "")).lower() == expected.expected_color
            and abs(distance - float(expected.expected_range_m)) <= expected.range_tolerance_m
            and obstacle.get("color") == expected.expected_color
            and _angle_error(bearing, float(expected.expected_bearing_deg)) <= expected.bearing_tolerance_deg
        )
        return valid, {
            "correlated": valid,
            "case": expected.case,
            "stamps": list(stamps),
            "id": buoy.get("id"),
            "color": buoy.get("color"),
            "bearing_deg": bearing,
            "range_m": distance,
            "lidar_id": lidar_id,
            "dt_ms": status_metrics.get("dt_ms"),
        }

    def _finish(self, state: str, reason: str) -> None:
        if self.state != "RUNNING":
            return
        self.state = state
        self.reasons.append(reason)
        self.events.append(self._event(state, self.last_mono))
        self._result_pending = True

    def _event(self, event: str, now: float | None) -> Dict[str, Any]:
        return {
            "schema_version": 1,
            "run_id": self.run_id,
            "seq": self.seq,
            "test": self.test,
            "event": event,
            "monotonic_s": now,
        }

    def metrics(self, now_mono: float) -> Dict[str, Any]:
        channels: Dict[str, Any] = {}
        for name, evidence in self.evidence.items():
            count = len(evidence.receives)
            span = evidence.receives[-1] - evidence.receives[0] if count > 1 else 0.0
            public_metrics = {
                key: value for key, value in evidence.last_metrics.items()
                if key not in {"matched", "detections", "obstacles"}
            }
            channels[name] = {
                "sample_count": count,
                "rate_hz": ((count - 1) / span if span > 0.0 else 0.0),
                "receive_age_s": (now_mono - evidence.receives[-1] if count else None),
                "source_stamp": evidence.last_source_stamp,
                "publisher_count": self.graph_counts.get(name),
                **public_metrics,
            }
        return {"channels": channels}

    def evidence_manifest(self) -> Dict[str, Any]:
        manifest: Dict[str, Any] = {}
        for name, evidence in self.evidence.items():
            rule = CHANNEL_RULES[name]
            span = evidence.receives[-1] - evidence.receives[0] if len(evidence.receives) > 1 else 0.0
            manifest[name] = {
                "topic": rule.topic,
                "unique_sample_count": len(evidence.source_stamps),
                "first_sample_identity": evidence.source_stamps[0] if evidence.source_stamps else None,
                "last_sample_identity": evidence.source_stamps[-1] if evidence.source_stamps else None,
                "freshness_basis": "receive_and_content_hash" if name == "costmap" else "receive_and_source_stamp",
                "observation_span_s": span,
                "required_samples": rule.min_samples,
                "required_rate_hz": rule.min_hz,
                "required_publisher_count": rule.publisher_count,
                "final_publisher_count": self.graph_counts.get(name),
                "measured_samples": [
                    item["measurement"] for item in evidence.sample_metrics
                    if isinstance(item.get("measurement"), dict)
                ],
            }
        return manifest

    def status(self, now_mono: float) -> Dict[str, Any]:
        remaining = None
        if self.active and self.deadline_mono is not None:
            remaining = max(0.0, self.deadline_mono - now_mono)
        expectations = self._expectations()
        return {
            "schema_version": 1,
            "run_id": self.run_id,
            "seq": self.seq,
            "test": self.test,
            "state": self.state,
            "actuation_enabled": False,
            "remaining_s": remaining,
            "reasons": list(self.reasons),
            "expectations": asdict(expectations) if expectations is not None else None,
        }

    def pop_events(self) -> List[Dict[str, Any]]:
        events, self.events = self.events, []
        return events

    def pop_result(self, now_mono: float) -> Dict[str, Any] | None:
        if not self._result_pending or self.terminal_needs_persistence:
            return None
        self._result_pending = False
        return self._result_payload(now_mono)

    def _result_payload(self, now_mono: float) -> Dict[str, Any]:
        return {
            **self.status(now_mono),
            "pass": self.state == "PASS",
            "artifact": self._artifact,
            "evidence_manifest": self.evidence_manifest(),
            "bundle_correlations": list(self._bundle_correlations),
            **self.metrics(now_mono),
        }
