"""ROS-independent bridge telemetry/health contract."""

from __future__ import annotations

import json
import math
import threading
from typing import Any, Dict, Mapping


REQUIRED_TELEMETRY_SOURCES = frozenset({"position", "attitude", "velocity", "armed"})


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be finite")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


class BridgeHealth:
    """Thread-safe proof state; freshness is based on real acquisitions only."""

    def __init__(self, dry_run: bool, timeout_s: float = 0.5) -> None:
        timeout_s = _finite(timeout_s, "timeout_s")
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be > 0")
        self.dry_run = bool(dry_run)
        self.timeout_s = timeout_s
        self._lock = threading.Lock()
        self._connection_seen = False
        self._source_times: Dict[str, tuple[float, float]] = {}
        self._last_receive_mono: float | None = None

    def mark_connection(self) -> None:
        with self._lock:
            self._connection_seen = True

    def mark_disconnected(self) -> None:
        with self._lock:
            self._connection_seen = False

    def invalidate_telemetry(self) -> None:
        with self._lock:
            self._source_times.clear()
            self._last_receive_mono = None

    def mark_telemetry(
        self, source: str, acquisition_stamp: float, receive_mono: float
    ) -> None:
        if source not in REQUIRED_TELEMETRY_SOURCES:
            raise ValueError("unknown telemetry source")
        acquisition_stamp = _finite(acquisition_stamp, "acquisition_stamp")
        receive_mono = _finite(receive_mono, "receive_mono")
        with self._lock:
            if self._last_receive_mono is not None and receive_mono < self._last_receive_mono:
                self._source_times.clear()
            self._source_times[source] = (acquisition_stamp, receive_mono)
            self._last_receive_mono = receive_mono

    def snapshot(self, now_mono: float, ros_now: float) -> Dict[str, Any]:
        now_mono = _finite(now_mono, "now_mono")
        ros_now = _finite(ros_now, "ros_now")
        with self._lock:
            connection_seen = self._connection_seen
            source_times = dict(self._source_times)
        sources = set(source_times)
        per_source: Dict[str, Dict[str, Any]] = {}
        ages_valid = True
        for source in sorted(REQUIRED_TELEMETRY_SOURCES):
            timing = source_times.get(source)
            if timing is None:
                ages_valid = False
                per_source[source] = {
                    "acquisition_stamp": None,
                    "receive_age_s": None,
                    "source_age_s": None,
                    "fresh": False,
                }
                continue
            acquisition, received = timing
            receive_age = now_mono - received
            source_age = ros_now - acquisition
            fresh = (
                0.0 <= receive_age <= self.timeout_s
                and -0.25 <= source_age <= self.timeout_s
            )
            ages_valid = ages_valid and fresh
            per_source[source] = {
                "acquisition_stamp": acquisition,
                "receive_age_s": receive_age,
                "source_age_s": source_age,
                "fresh": fresh,
            }
        telemetry_ready = REQUIRED_TELEMETRY_SOURCES.issubset(sources)
        heartbeat_fresh = bool(not self.dry_run and connection_seen and telemetry_ready and ages_valid)
        required_times = [source_times[name] for name in REQUIRED_TELEMETRY_SOURCES if name in source_times]
        # A composite sample is only as fresh as its oldest required field.
        acquisition = min((item[0] for item in required_times), default=None)
        received = min((item[1] for item in required_times), default=None)
        receive_age = None if received is None else now_mono - received
        source_age = None if acquisition is None else ros_now - acquisition
        return {
            "dry_run": self.dry_run,
            "connected": heartbeat_fresh,
            "heartbeat_fresh": heartbeat_fresh,
            "telemetry_ready": telemetry_ready,
            "acquisition_stamp": acquisition,
            "last_receive_age_s": receive_age,
            "source_age_s": source_age,
            "sources": sorted(sources),
            "source_health": per_source,
        }


def telemetry_payload(state: Mapping[str, Any], acquisition_stamp: float) -> Dict[str, Any]:
    payload = {
        "stamp": _finite(acquisition_stamp, "acquisition_stamp"),
        "acquisition_stamp": _finite(acquisition_stamp, "acquisition_stamp"),
        "lat": _finite(state.get("lat"), "lat"),
        "lon": _finite(state.get("lon"), "lon"),
        "heading_deg": _finite(state.get("heading_deg"), "heading_deg"),
        "ground_speed": _finite(state.get("ground_speed"), "ground_speed"),
        "roll_deg": _finite(state.get("roll_deg"), "roll_deg"),
        "pitch_deg": _finite(state.get("pitch_deg"), "pitch_deg"),
        "mode": state.get("mode"),
    }
    # SERVO_OUTPUT_RAW -> motor çıkışı (karar günlüğü kanıtı). Alanlar yoksa
    # (dry_run, bağlantı yok) yayına dahil edilmez; varsa tamsayı PWM olmalı.
    for key in ("motor_left_pwm", "motor_right_pwm"):
        value = state.get(key)
        if value is None:
            continue
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{key} must be an integer PWM")
        if not 800 <= value <= 2200:
            raise ValueError(f"{key} must be a plausible PWM")
        payload[key] = value
    if not isinstance(payload["mode"], str) or not payload["mode"]:
        raise ValueError("mode must be a non-empty string")
    if not -90.0 <= payload["lat"] <= 90.0 or not -180.0 <= payload["lon"] <= 180.0:
        raise ValueError("geographic position is out of range")
    if payload["ground_speed"] < 0.0:
        raise ValueError("ground_speed must be >= 0")
    return payload


def strict_json(payload: Mapping[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=True, allow_nan=False, separators=(",", ":"))
