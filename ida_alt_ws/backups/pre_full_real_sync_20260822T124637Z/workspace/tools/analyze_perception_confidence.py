#!/usr/bin/env python3
"""Summarize synthetic/real camera confidence and fusion reliance per phase."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


CAMERA_TOPICS = {
    "/perception/camera/p1p2/raw": "camera_p1p2",
    "/perception/camera/p3/raw": "camera_p3",
}


def _loads(message: Any) -> dict[str, Any]:
    try:
        value = json.loads(message.data)
    except (AttributeError, TypeError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _finite(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _quantile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low = int(math.floor(index))
    high = int(math.ceil(index))
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def _distribution(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "p10": None, "p50": None, "p90": None, "mean": None}
    return {
        "count": len(values),
        "p10": round(_quantile(values, 0.10), 4),
        "p50": round(_quantile(values, 0.50), 4),
        "p90": round(_quantile(values, 0.90), 4),
        "mean": round(sum(values) / len(values), 4),
        "ge_0_20_pct": round(100.0 * sum(v >= 0.20 for v in values) / len(values), 2),
        "ge_0_35_pct": round(100.0 * sum(v >= 0.35 for v in values) / len(values), 2),
        "ge_0_50_pct": round(100.0 * sum(v >= 0.50 for v in values) / len(values), 2),
        "ge_0_70_pct": round(100.0 * sum(v >= 0.70 for v in values) / len(values), 2),
    }


def _phase(state: str) -> str:
    if state == "PARKUR_1_NAV":
        return "P1"
    if state == "PARKUR_2_AVOIDANCE":
        return "P2"
    if state.startswith("PARKUR_3"):
        return "P3"
    return state or "UNKNOWN"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    types = {name: get_message(type_name) for name, type_name in topic_types.items()}

    current_state = "UNKNOWN"
    camera: dict[str, dict[str, Any]] = defaultdict(
        lambda: defaultdict(
            lambda: {
                "frames": 0,
                "nonempty_frames": 0,
                "confidences": [],
                "nearest_frame_confidences": [],
                "within_10m": [],
                "within_20m": [],
                "by_color": defaultdict(list),
            }
        )
    )
    fusion: dict[str, dict[str, Any]] = defaultdict(
        lambda: {
            "frames": 0,
            "accepted": 0,
            "ready": 0,
            "healthy": 0,
            "matched": 0,
            "ambiguous": 0,
            "overflow_frames": 0,
            "dt_ms": [],
        }
    )
    obstacles: dict[str, Counter[str]] = defaultdict(Counter)
    buoy_sources: dict[str, Counter[str]] = defaultdict(Counter)
    gate_truth: dict[str, dict[str, int]] = defaultdict(lambda: {"frames": 0, "detections": 0})

    while reader.has_next():
        topic, data, _stamp_ns = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(data, types[topic])
        payload = _loads(msg)
        if topic == "/autonomy/state":
            current_state = str(payload.get("state", "UNKNOWN"))
            continue
        phase = _phase(current_state)

        if topic in CAMERA_TOPICS:
            role = CAMERA_TOPICS[topic]
            row = camera[phase][role]
            row["frames"] += 1
            values = payload.get("detections", [])
            detections = [item for item in values if isinstance(item, dict)] if isinstance(values, list) else []
            if detections:
                row["nonempty_frames"] += 1
            nearest: tuple[float, float] | None = None
            for item in detections:
                confidence = _finite(item.get("confidence"))
                distance = _finite(item.get("distance"))
                if confidence is None or not 0.0 <= confidence <= 1.0:
                    continue
                row["confidences"].append(confidence)
                row["by_color"][str(item.get("color", "unknown"))].append(confidence)
                if distance is not None and distance <= 10.0:
                    row["within_10m"].append(confidence)
                if distance is not None and distance <= 20.0:
                    row["within_20m"].append(confidence)
                if distance is not None and (nearest is None or distance < nearest[0]):
                    nearest = (distance, confidence)
            if nearest is not None:
                row["nearest_frame_confidences"].append(nearest[1])
            continue

        if topic == "/perception/fusion/status":
            row = fusion[phase]
            row["frames"] += 1
            row["accepted"] += bool(payload.get("accepted"))
            row["ready"] += bool(payload.get("ready"))
            row["healthy"] += payload.get("source_health") == "ok"
            row["matched"] += int(payload.get("matched_count", 0) or 0)
            row["ambiguous"] += int(payload.get("ambiguous_camera_count", 0) or 0)
            row["ambiguous"] += int(payload.get("ambiguous_lidar_count", 0) or 0)
            row["overflow_frames"] += bool(payload.get("association_overflow"))
            dt = _finite(payload.get("dt_ms"))
            if dt is not None:
                row["dt_ms"].append(dt)
            continue

        if topic == "/perception/obstacles":
            values = payload.get("obstacles", [])
            if isinstance(values, list):
                for item in values:
                    if not isinstance(item, dict):
                        continue
                    obstacles[phase][str(item.get("source", "unknown"))] += 1
            continue

        if topic == "/perception/buoys":
            values = payload.get("detections", [])
            if isinstance(values, list):
                for item in values:
                    if isinstance(item, dict):
                        buoy_sources[phase][str(item.get("source", "unknown"))] += 1
            continue

        if topic == "/perception_sim/gate_truth":
            gate_truth[phase]["frames"] += 1
            values = payload.get("detections", [])
            if isinstance(values, list):
                gate_truth[phase]["detections"] += sum(isinstance(item, dict) for item in values)

    camera_output = {}
    for phase, roles in camera.items():
        camera_output[phase] = {}
        for role, row in roles.items():
            frames = row["frames"]
            camera_output[phase][role] = {
                "frames": frames,
                "nonempty_frame_pct": round(100.0 * row["nonempty_frames"] / frames, 2) if frames else None,
                "all_detections": _distribution(row["confidences"]),
                "nearest_per_frame": _distribution(row["nearest_frame_confidences"]),
                "within_10m": _distribution(row["within_10m"]),
                "within_20m": _distribution(row["within_20m"]),
                "by_color": {
                    color: _distribution(values)
                    for color, values in sorted(row["by_color"].items())
                },
            }

    fusion_output = {}
    for phase, row in fusion.items():
        frames = row["frames"]
        fusion_output[phase] = {
            "frames": frames,
            "accepted_pct": round(100.0 * row["accepted"] / frames, 2) if frames else None,
            "ready_pct": round(100.0 * row["ready"] / frames, 2) if frames else None,
            "healthy_pct": round(100.0 * row["healthy"] / frames, 2) if frames else None,
            "matched_per_frame": round(row["matched"] / frames, 3) if frames else None,
            "ambiguous_per_frame": round(row["ambiguous"] / frames, 3) if frames else None,
            "overflow_frame_pct": round(100.0 * row["overflow_frames"] / frames, 2) if frames else None,
            "dt_ms_p95": round(_quantile(row["dt_ms"], 0.95), 3) if row["dt_ms"] else None,
        }

    print(json.dumps({
        "bag": args.bag,
        "camera": camera_output,
        "fusion": fusion_output,
        "obstacle_sources": {phase: dict(values) for phase, values in obstacles.items()},
        "buoy_sources": {phase: dict(values) for phase, values in buoy_sources.items()},
        "sim_gate_truth": dict(gate_truth),
        "interpretation_warning": (
            "perception_sim confidence is synthetic; sim_gate_truth is fixed-confidence "
            "oracle evidence and must not be interpreted as YOLO accuracy"
        ),
    }, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
