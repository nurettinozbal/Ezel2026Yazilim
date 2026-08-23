"""Pure JSON contracts for passive vehicle tests.

This module deliberately contains no ROS imports.  Requests cannot express an
arm, mode or motor operation; the allow-list is the safety boundary.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from typing import Any, Dict, Mapping


ALLOWED_TESTS = frozenset(
    {
        "comms",
        "telemetry",
        "camera_p1p2",
        "camera_p3",
        "lidar",
        "fusion_shadow",
        "autonomy_shadow",
        "logging",
    }
)
_RUN_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
_IDENTITY = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")
FIXTURE_CASES = frozenset({"positive", "negative", "ambiguity", "lidar_only", "camera_only"})
FIXTURE_PROFILES = frozenset(
    {"camera_p1p2_fixture", "camera_p3_fixture", "lidar_bottle", "fusion_buoy"}
)
_PROFILE_FOR_TEST = {
    "camera_p1p2": "camera_p1p2_fixture",
    "camera_p3": "camera_p3_fixture",
    "lidar": "lidar_bottle",
    "fusion_shadow": "fusion_buoy",
}


class RequestError(ValueError):
    """Raised when a request is not exactly within the passive contract."""


@dataclass(frozen=True)
class TestRequest:
    action: str
    run_id: str
    seq: int
    test: str | None = None
    timeout_s: float | None = None
    expectations: "FixtureExpectations | None" = None


@dataclass(frozen=True)
class FixtureExpectations:
    profile: str
    case: str
    expected_color: str | None
    expected_range_m: float | None
    expected_bearing_deg: float | None
    range_tolerance_m: float
    bearing_tolerance_deg: float
    expected_id: str | None


def _exact_nonnegative_int(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise RequestError(f"{field} must be a non-negative integer")
    return value


def parse_request(raw: str | Mapping[str, Any]) -> TestRequest:
    if isinstance(raw, str) and len(raw.encode("utf-8")) > 8192:
        raise RequestError("request exceeds 8192 bytes")
    try:
        value = bounded_json_loads(raw, 8192, 128, 6) if isinstance(raw, str) else dict(raw)
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise RequestError("request must be a JSON object") from exc
    if not isinstance(value, dict):
        raise RequestError("request must be a JSON object")
    action = value.get("action")
    if action not in {"start", "cancel"}:
        raise RequestError("action must be start or cancel")
    run_id = value.get("run_id")
    if not isinstance(run_id, str) or _RUN_ID.fullmatch(run_id) is None:
        raise RequestError("run_id is invalid")
    seq = _exact_nonnegative_int(value.get("seq"), "seq")
    allowed = {"action", "run_id", "seq"}
    if action == "cancel":
        if set(value) != allowed:
            raise RequestError("cancel contains unsupported fields")
        return TestRequest(action, run_id, seq)

    allowed.update({"test", "timeout_s", "expectations"})
    if not {"action", "run_id", "seq", "test", "timeout_s"}.issubset(value) or not set(value) <= allowed:
        raise RequestError("start contains missing or unsupported fields")
    test = value.get("test")
    if test not in ALLOWED_TESTS:
        raise RequestError("test is not in the passive allow-list")
    try:
        timeout_s = float(value.get("timeout_s"))
    except (TypeError, ValueError, OverflowError) as exc:
        raise RequestError("timeout_s must be finite") from exc
    if not math.isfinite(timeout_s) or not 0.5 <= timeout_s <= 300.0:
        raise RequestError("timeout_s must be between 0.5 and 300 seconds")
    expectations = _parse_expectations(test, value.get("expectations"))
    return TestRequest(action, run_id, seq, test, timeout_s, expectations)


def _optional_finite(value: Any, name: str, low: float, high: float) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool):
        raise RequestError(f"{name} must be finite")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RequestError(f"{name} must be finite") from exc
    if not math.isfinite(number) or not low <= number <= high:
        raise RequestError(f"{name} is out of range")
    return number


def _parse_expectations(test: str, raw: Any) -> FixtureExpectations | None:
    needs_fixture = test in _PROFILE_FOR_TEST
    if not needs_fixture:
        if raw is not None:
            raise RequestError("expectations are supported only by fixture tests")
        return None
    if not isinstance(raw, dict):
        raise RequestError("fixture test requires expectations")
    fields = {
        "profile", "case", "expected_color", "expected_range_m", "expected_bearing_deg",
        "range_tolerance_m", "bearing_tolerance_deg", "expected_id",
    }
    if set(raw) - fields:
        raise RequestError("expectations contain unsupported fields")
    profile = raw.get("profile")
    if profile != _PROFILE_FOR_TEST[test] or profile not in FIXTURE_PROFILES:
        raise RequestError("fixture profile does not match test")
    case = raw.get("case")
    if case not in FIXTURE_CASES:
        raise RequestError("fixture case is not allowed")
    if test != "fusion_shadow" and case not in {"positive", "negative"}:
        raise RequestError("special fixture cases are fusion-only")
    color = raw.get("expected_color")
    if color is not None:
        if not isinstance(color, str):
            raise RequestError("expected_color is invalid")
        color = color.lower()
        allowed_colors = {"orange", "yellow"} if test == "camera_p1p2" else {"red", "green", "black"}
        if test == "fusion_shadow":
            allowed_colors = {"orange", "yellow", "red", "green", "black"}
        if color not in allowed_colors:
            raise RequestError("expected_color is outside the test manifest")
    expected_range = _optional_finite(raw.get("expected_range_m"), "expected_range_m", 0.05, 100.0)
    expected_bearing = _optional_finite(raw.get("expected_bearing_deg"), "expected_bearing_deg", -180.0, 180.0)
    range_tolerance = _optional_finite(raw.get("range_tolerance_m", 0.25), "range_tolerance_m", 0.01, 5.0)
    bearing_tolerance = _optional_finite(raw.get("bearing_tolerance_deg", 3.0), "bearing_tolerance_deg", 0.1, 30.0)
    identity = raw.get("expected_id")
    if identity is not None and (not isinstance(identity, str) or _IDENTITY.fullmatch(identity) is None):
        raise RequestError("expected_id is invalid")
    if case == "positive":
        if expected_bearing is None:
            raise RequestError("positive fixture requires expected_bearing_deg")
        if expected_range is None:
            raise RequestError("positive fixture requires expected_range_m")
        if test in {"camera_p1p2", "camera_p3", "fusion_shadow"} and color is None:
            raise RequestError("color fixture requires expected_color")
    return FixtureExpectations(
        profile, case, color, expected_range, expected_bearing,
        float(range_tolerance), float(bearing_tolerance), identity,
    )


def compact_json(payload: Mapping[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=True, allow_nan=False, separators=(",", ":"))


def bounded_json_loads(raw: str, max_bytes: int, max_items: int, max_depth: int = 12) -> Any:
    if not isinstance(raw, str) or len(raw.encode("utf-8")) > max_bytes:
        raise ValueError("json_byte_limit")
    try:
        value = json.loads(raw)
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError("json_invalid") from exc
    stack = [(value, 0)]
    items = 0
    while stack:
        current, depth = stack.pop()
        if depth > max_depth:
            raise ValueError("json_depth_limit")
        items += 1
        if items > max_items:
            raise ValueError("json_item_limit")
        if isinstance(current, dict):
            stack.extend((key, depth + 1) for key in current.keys())
            stack.extend((item, depth + 1) for item in current.values())
        elif isinstance(current, list):
            stack.extend((item, depth + 1) for item in current)
    return value
