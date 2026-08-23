#!/usr/bin/env python3
"""Extract P2 reverse, arbitration and approximate collision evidence from a bag."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BOAT_HALF_FORWARD_M = 0.70
BOAT_HALF_LATERAL_M = 0.45
BUOY_RADIUS_M = 0.15


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


def _clearance(obstacle: dict[str, Any]) -> float | None:
    forward = _finite(obstacle.get("forward_m"))
    lateral = _finite(obstacle.get("lateral_m"))
    if forward is None or lateral is None:
        return None
    dx = max(abs(forward) - BOAT_HALF_FORWARD_M, 0.0)
    dy = max(abs(lateral) - BOAT_HALF_LATERAL_M, 0.0)
    return math.hypot(dx, dy) - BUOY_RADIUS_M


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
    first_ns: int | None = None
    state = ""
    obstacles: list[dict[str, Any]] = []
    reverse_rows: list[dict[str, Any]] = []
    overlap_rows: list[dict[str, Any]] = []
    close_three_rows = 0
    exact_actions: Counter[str] = Counter()
    minimum_by_id: dict[str, float] = {}
    p2_debug_count = 0
    previous_action = ""
    reverse_episodes: list[dict[str, Any]] = []
    open_reverse: dict[str, Any] | None = None

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        first_ns = stamp_ns if first_ns is None else first_ns
        if topic not in types:
            continue
        msg = deserialize_message(data, types[topic])
        rel_s = (stamp_ns - first_ns) / 1e9
        if topic == "/autonomy/state":
            state = str(_loads(msg).get("state", ""))
            continue
        if topic == "/perception/obstacles":
            payload = _loads(msg)
            values = payload.get("obstacles", [])
            obstacles = [item for item in values if isinstance(item, dict)] if isinstance(values, list) else []
            continue
        if topic != "/autonomy/debug" or state != "PARKUR_2_AVOIDANCE":
            continue

        payload = _loads(msg)
        command = payload.get("command") if isinstance(payload.get("command"), dict) else {}
        action = str(command.get("action", ""))
        vx = _finite(command.get("vx")) or 0.0
        yaw = _finite(command.get("yaw_rate")) or 0.0
        p2_debug_count += 1
        exact_actions[action] += 1

        clearances = []
        close_forward = []
        for obstacle in obstacles:
            clearance = _clearance(obstacle)
            if clearance is None:
                continue
            obstacle_id = str(obstacle.get("id", obstacle.get("lidar_obstacle_id", "unknown")))
            minimum_by_id[obstacle_id] = min(clearance, minimum_by_id.get(obstacle_id, float("inf")))
            clearances.append((clearance, obstacle))
            forward = _finite(obstacle.get("forward_m"))
            lateral = _finite(obstacle.get("lateral_m"))
            if forward is not None and lateral is not None and -0.2 <= forward <= 3.0 and abs(lateral) <= 2.0:
                close_forward.append(obstacle)
        if len(close_forward) >= 3:
            close_three_rows += 1
        overlaps = [(value, item) for value, item in clearances if value <= 0.0]
        if overlaps and len(overlap_rows) < 80:
            value, item = min(overlaps, key=lambda pair: pair[0])
            overlap_rows.append({
                "t_s": round(rel_s, 3),
                "id": item.get("id", item.get("lidar_obstacle_id")),
                "clearance_m": round(value, 3),
                "forward_m": item.get("forward_m"),
                "lateral_m": item.get("lateral_m"),
                "action": action,
                "vx": round(vx, 3),
                "yaw_rate": round(yaw, 3),
            })

        is_reverse = vx < -1e-6 or action.startswith("near_field_escape_reverse")
        if is_reverse:
            nearest = min(clearances, key=lambda pair: pair[0]) if clearances else (None, {})
            row = {
                "t_s": round(rel_s, 3),
                "vx": round(vx, 3),
                "yaw_rate": round(yaw, 3),
                "action": action,
                "nearest_id": nearest[1].get("id", nearest[1].get("lidar_obstacle_id")),
                "nearest_clearance_m": round(nearest[0], 3) if nearest[0] is not None else None,
                "near_field": payload.get("near_field"),
                "behavior": payload.get("behavior"),
            }
            if len(reverse_rows) < 80:
                reverse_rows.append(row)
            if open_reverse is None:
                open_reverse = dict(row)
                open_reverse["start_s"] = row["t_s"]
                open_reverse["samples"] = 0
            open_reverse["end_s"] = row["t_s"]
            open_reverse["samples"] += 1
        elif open_reverse is not None:
            open_reverse["duration_s"] = round(open_reverse["end_s"] - open_reverse["start_s"] + 0.1, 3)
            reverse_episodes.append(open_reverse)
            open_reverse = None
        previous_action = action

    if open_reverse is not None:
        open_reverse["duration_s"] = round(open_reverse["end_s"] - open_reverse["start_s"] + 0.1, 3)
        reverse_episodes.append(open_reverse)

    output = {
        "bag": args.bag,
        "p2_debug_samples": p2_debug_count,
        "triple_close_duration_s": round(close_three_rows * 0.1, 3),
        "reverse_episode_count": len(reverse_episodes),
        "reverse_total_s": round(sum(item["duration_s"] for item in reverse_episodes), 3),
        "reverse_episodes": reverse_episodes,
        "approximate_collision_sample_count": len(overlap_rows),
        "approximate_collision_samples": overlap_rows,
        "minimum_clearance_by_obstacle_m": {
            key: round(value, 3)
            for key, value in sorted(minimum_by_id.items(), key=lambda pair: pair[1])[:20]
        },
        "top_actions": exact_actions.most_common(30),
    }
    print(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
