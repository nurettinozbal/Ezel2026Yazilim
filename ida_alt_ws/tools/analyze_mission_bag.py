#!/usr/bin/env python3
"""Summarize an IDA full-mission rosbag into deterministic JSON.

Run inside a sourced ROS 2 workspace.  The script is read-only and only needs
the canonical String telemetry/debug/perception topics plus Twist commands.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ORIGIN_LAT = 40.8630501
ORIGIN_LON = 29.2599517
EARTH_RADIUS_M = 6_378_137.0


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)


def rounded(value: Any, digits: int = 3) -> Any:
    return round(float(value), digits) if isinstance(value, (int, float)) and math.isfinite(value) else value


def local_xy(lat: float, lon: float) -> tuple[float, float]:
    x = math.radians(lat - ORIGIN_LAT) * EARTH_RADIUS_M
    y = (
        math.radians(lon - ORIGIN_LON)
        * EARTH_RADIUS_M
        * math.cos(math.radians(ORIGIN_LAT))
    )
    return x, y


def action_category(action: str) -> str:
    value = str(action or "")
    if value.startswith("near_field_stop"):
        return "near_field_stop"
    if "recovery" in value:
        return "recovery"
    if "align_waypoint" in value or "heading_align" in value:
        return "heading_align"
    if value.startswith("near_field_slow"):
        return "near_field_slow"
    if value.startswith("search_target"):
        return "p3_search"
    if "target" in value or "engage" in value or "lock" in value:
        return "p3_target"
    if value.startswith("dwa"):
        return "dwa_navigation"
    if value in {"idle", "mission_ready", "complete"}:
        return value
    if value.startswith("failsafe"):
        return "failsafe"
    return value.split()[0] if value else "unknown"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {name: get_message(type_name) for name, type_name in topic_types.items()}

    first_ns = None
    last_ns = None
    topic_counts: Counter[str] = Counter()
    mission_payloads: list[dict[str, Any]] = []
    state_rows: list[dict[str, Any]] = []
    debug_rows: list[dict[str, Any]] = []
    telemetry_rows: list[dict[str, Any]] = []
    perception_frames: dict[str, list[dict[str, Any]]] = defaultdict(list)
    latest_state: dict[str, Any] = {}
    latest_telemetry: dict[str, Any] = {}

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        first_ns = stamp_ns if first_ns is None else first_ns
        last_ns = stamp_ns
        topic_counts[topic] += 1
        if topic not in message_types:
            continue
        msg = deserialize_message(data, message_types[topic])
        rel_s = (stamp_ns - first_ns) / 1e9
        if topic in {
            "/mission/waypoints",
            "/autonomy/state",
            "/autonomy/debug",
            "/telemetry/state",
            "/perception/buoys",
            "/perception/obstacles",
        }:
            try:
                payload = json.loads(msg.data)
            except (AttributeError, json.JSONDecodeError):
                continue
            payload["_t"] = rel_s
            if topic == "/mission/waypoints":
                mission_payloads.append(payload)
            elif topic == "/autonomy/state":
                latest_state = payload
                state_rows.append(payload)
            elif topic == "/telemetry/state":
                latest_telemetry = payload
                telemetry_rows.append(payload)
            elif topic == "/autonomy/debug":
                row = dict(payload)
                row["_state"] = dict(latest_state)
                row["_telemetry"] = dict(latest_telemetry)
                debug_rows.append(row)
            else:
                perception_frames[topic].append(payload)

    if first_ns is None or last_ns is None:
        raise RuntimeError("bag is empty")

    phase_segments: list[dict[str, Any]] = []
    for row in state_rows:
        state = str(row.get("state", "unknown"))
        if not phase_segments or phase_segments[-1]["state"] != state:
            if phase_segments:
                phase_segments[-1]["end_s"] = row["_t"]
                phase_segments[-1]["duration_s"] = row["_t"] - phase_segments[-1]["start_s"]
            phase_segments.append({"state": state, "start_s": row["_t"]})
    if phase_segments:
        phase_segments[-1]["end_s"] = (last_ns - first_ns) / 1e9
        phase_segments[-1]["duration_s"] = phase_segments[-1]["end_s"] - phase_segments[-1]["start_s"]

    waypoint_events: list[dict[str, Any]] = []
    last_wp = object()
    for row in state_rows:
        wp = row.get("current_waypoint")
        if wp != last_wp:
            waypoint_events.append(
                {
                    "t_s": rounded(row["_t"]),
                    "waypoint": wp,
                    "state": row.get("state"),
                    "action": row.get("action"),
                }
            )
            last_wp = wp

    action_duration: dict[str, Counter[str]] = defaultdict(Counter)
    exact_action_duration: dict[str, Counter[str]] = defaultdict(Counter)
    action_transitions: Counter[str] = Counter()
    last_category_by_state: dict[str, str] = {}
    risky_rows: list[dict[str, Any]] = []
    obstacle_close_forward_rows: list[dict[str, Any]] = []
    course_distance: dict[str, list[float]] = defaultdict(list)
    outside_duration: Counter[str] = Counter()
    command_vx: dict[str, list[float]] = defaultdict(list)
    command_yaw: dict[str, list[float]] = defaultdict(list)
    action_episodes: list[dict[str, Any]] = []
    open_episode: dict[str, Any] | None = None

    for index, row in enumerate(debug_rows):
        state_obj = row.get("_state") or {}
        state = str(state_obj.get("state", "unknown"))
        command = row.get("command") or {}
        category = action_category(command.get("action", ""))
        dt = 0.1
        if index + 1 < len(debug_rows):
            dt = min(0.5, max(0.0, debug_rows[index + 1]["_t"] - row["_t"]))
        action_duration[state][category] += dt
        exact_action_duration[state][str(command.get("action", "unknown"))] += dt
        if (
            open_episode is None
            or open_episode["state"] != state
            or open_episode["category"] != category
        ):
            if open_episode is not None:
                open_episode["end_s"] = row["_t"]
                open_episode["duration_s"] = row["_t"] - open_episode["start_s"]
                action_episodes.append(open_episode)
            open_episode = {
                "state": state,
                "category": category,
                "action": command.get("action"),
                "start_s": row["_t"],
            }
        previous = last_category_by_state.get(state)
        if previous is not None and previous != category:
            action_transitions[f"{state}:{previous}->{category}"] += 1
        last_category_by_state[state] = category

        vx = float(command.get("vx", 0.0) or 0.0)
        yaw = float(command.get("yaw_rate", 0.0) or 0.0)
        command_vx[state].append(vx)
        command_yaw[state].append(yaw)
        distance = state_obj.get("distance_from_course_m")
        if isinstance(distance, (int, float)) and math.isfinite(distance):
            course_distance[state].append(float(distance))
        if state_obj.get("inside_course_geometry") is False:
            outside_duration[state] += dt

        nearest = row.get("nearest_obstacle") or {}
        obstacle_distance = nearest.get("distance")
        sample = {
            "t_s": rounded(row["_t"]),
            "state": state,
            "waypoint": state_obj.get("current_waypoint"),
            "vx": rounded(vx),
            "yaw_rate": rounded(yaw),
            "category": category,
            "action": command.get("action"),
            "obstacle_id": nearest.get("id"),
            "obstacle_distance_m": rounded(obstacle_distance),
            "obstacle_bearing_deg": rounded(nearest.get("bearing_deg")),
        }
        if abs(yaw) >= 0.4 and vx > 0.05:
            risky_rows.append(sample)
        if isinstance(obstacle_distance, (int, float)) and obstacle_distance < 2.0 and vx > 0.05:
            obstacle_close_forward_rows.append(sample)

    if open_episode is not None:
        open_episode["end_s"] = debug_rows[-1]["_t"] if debug_rows else open_episode["start_s"]
        open_episode["duration_s"] = open_episode["end_s"] - open_episode["start_s"]
        action_episodes.append(open_episode)

    longest_episodes: dict[str, dict[str, list[dict[str, Any]]]] = defaultdict(dict)
    episode_groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for episode in action_episodes:
        episode_groups[(episode["state"], episode["category"])].append(episode)
    for (state, category), episodes in episode_groups.items():
        longest_episodes[state][category] = [
            {
                key: rounded(value) if key.endswith("_s") else value
                for key, value in episode.items()
            }
            for episode in sorted(episodes, key=lambda item: item["duration_s"], reverse=True)[:5]
        ]

    telemetry_by_state: dict[str, list[dict[str, Any]]] = defaultdict(list)
    state_idx = 0
    for telem in telemetry_rows:
        while state_idx + 1 < len(state_rows) and state_rows[state_idx + 1]["_t"] <= telem["_t"]:
            state_idx += 1
        state = str(state_rows[state_idx].get("state", "unknown")) if state_rows else "unknown"
        telemetry_by_state[state].append(telem)

    motion_summary = {}
    for state, rows in telemetry_by_state.items():
        path_m = 0.0
        points = []
        speeds = []
        for telem in rows:
            try:
                point = local_xy(float(telem["lat"]), float(telem["lon"]))
            except (KeyError, TypeError, ValueError):
                continue
            if points:
                step = math.hypot(point[0] - points[-1][0], point[1] - points[-1][1])
                if step < 2.0:
                    path_m += step
            points.append(point)
            speed = telem.get("ground_speed")
            if isinstance(speed, (int, float)) and math.isfinite(speed):
                speeds.append(float(speed))
        motion_summary[state] = {
            "samples": len(rows),
            "start_xy_m": [rounded(value) for value in points[0]] if points else None,
            "end_xy_m": [rounded(value) for value in points[-1]] if points else None,
            "path_length_m": rounded(path_m),
            "speed_mps_p50": rounded(percentile(speeds, 0.5)),
            "speed_mps_p95": rounded(percentile(speeds, 0.95)),
        }

    phase_metrics = {}
    for state, durations in action_duration.items():
        distances = course_distance.get(state, [])
        vx_values = command_vx.get(state, [])
        yaw_values = command_yaw.get(state, [])
        phase_metrics[state] = {
            "action_duration_s": {key: rounded(value) for key, value in durations.most_common()},
            "exact_action_duration_s": {
                key: rounded(value)
                for key, value in exact_action_duration[state].most_common(20)
            },
            "action_switches": sum(
                count for key, count in action_transitions.items() if key.startswith(f"{state}:")
            ),
            "outside_course_s": rounded(outside_duration.get(state, 0.0)),
            "course_distance_m_p95": rounded(percentile(distances, 0.95)),
            "course_distance_m_max": rounded(max(distances) if distances else None),
            "vx_p50": rounded(percentile(vx_values, 0.5)),
            "vx_p95": rounded(percentile(vx_values, 0.95)),
            "max_abs_yaw_rate": rounded(max((abs(value) for value in yaw_values), default=0.0)),
            "motion": motion_summary.get(state),
        }

    detection_summary = {}
    for topic, frames in perception_frames.items():
        list_key = "detections" if topic.endswith("buoys") else "obstacles"
        nonempty = 0
        ids = Counter()
        colors = Counter()
        target_frames = 0
        target_min_distance = None
        target_max_confidence = None
        for frame in frames:
            values = frame.get(list_key, [])
            if values:
                nonempty += 1
            for item in values if isinstance(values, list) else []:
                ids[str(item.get("id"))] += 1
                color = str(item.get("color", "unknown"))
                colors[color] += 1
                if color == "green":
                    target_frames += 1
                    distance = item.get("distance")
                    confidence = item.get("confidence")
                    if isinstance(distance, (int, float)):
                        target_min_distance = distance if target_min_distance is None else min(target_min_distance, distance)
                    if isinstance(confidence, (int, float)):
                        target_max_confidence = confidence if target_max_confidence is None else max(target_max_confidence, confidence)
        detection_summary[topic] = {
            "frames": len(frames),
            "nonempty_frames": nonempty,
            "nonempty_ratio": rounded(nonempty / len(frames) if frames else 0.0),
            "colors": dict(colors),
            "most_seen_ids": ids.most_common(15),
            "green_target_frames": target_frames,
            "green_target_min_distance_m": rounded(target_min_distance),
            "green_target_max_confidence": rounded(target_max_confidence),
        }

    perception_by_phase: dict[str, dict[str, Any]] = {}
    state_idx = 0
    buoy_phase_frames: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for frame in perception_frames.get("/perception/buoys", []):
        while state_idx + 1 < len(state_rows) and state_rows[state_idx + 1]["_t"] <= frame["_t"]:
            state_idx += 1
        state = str(state_rows[state_idx].get("state", "unknown")) if state_rows else "unknown"
        buoy_phase_frames[state].append(frame)
    for state, frames in buoy_phase_frames.items():
        colors: Counter[str] = Counter()
        green_frame_count = 0
        green_run_lengths: list[int] = []
        current_green_run = 0
        green_samples: list[dict[str, Any]] = []
        close_wrong_samples: list[dict[str, Any]] = []
        for frame in frames:
            values = frame.get("detections", [])
            greens = []
            for item in values if isinstance(values, list) else []:
                color = str(item.get("color", "unknown"))
                colors[color] += 1
                if color == "green":
                    greens.append(item)
                elif (
                    isinstance(item.get("distance"), (int, float))
                    and float(item["distance"]) < 2.5
                    and len(close_wrong_samples) < 20
                ):
                    close_wrong_samples.append({
                        "t_s": rounded(frame["_t"]),
                        "id": item.get("id"),
                        "color": color,
                        "distance_m": rounded(item.get("distance")),
                        "forward_m": rounded(item.get("forward_m")),
                        "lateral_m": rounded(item.get("lateral_m")),
                        "bearing_deg": rounded(item.get("bearing_deg")),
                    })
            if greens:
                green_frame_count += 1
                current_green_run += 1
                if len(green_samples) < 12:
                    item = min(greens, key=lambda value: float(value.get("distance", 99.0)))
                    green_samples.append({
                        "t_s": rounded(frame["_t"]),
                        "id": item.get("id"),
                        "lidar_obstacle_id": item.get("lidar_obstacle_id"),
                        "distance_m": rounded(item.get("distance")),
                        "confidence": rounded(item.get("confidence")),
                        "bearing_deg": rounded(item.get("bearing_deg")),
                    })
            elif current_green_run:
                green_run_lengths.append(current_green_run)
                current_green_run = 0
        if current_green_run:
            green_run_lengths.append(current_green_run)
        perception_by_phase[state] = {
            "frames": len(frames),
            "colors": dict(colors),
            "green_frames": green_frame_count,
            "green_longest_consecutive_frames": max(green_run_lengths, default=0),
            "green_run_count": len(green_run_lengths),
            "green_samples": green_samples,
            "close_wrong_samples": close_wrong_samples,
        }

    phase_boundary_snapshots = []
    for segment in phase_segments:
        start = segment["start_s"]
        end = segment["end_s"]
        matching_states = [row for row in state_rows if start <= row["_t"] < end]
        matching_telem = [row for row in telemetry_rows if start <= row["_t"] < end]
        snapshot: dict[str, Any] = {
            "state": segment["state"],
            "first_state": matching_states[0] if matching_states else None,
            "last_state": matching_states[-1] if matching_states else None,
        }
        if matching_telem:
            first_xy = local_xy(float(matching_telem[0]["lat"]), float(matching_telem[0]["lon"]))
            last_xy = local_xy(float(matching_telem[-1]["lat"]), float(matching_telem[-1]["lon"]))
            snapshot["start_xy_m"] = [rounded(value) for value in first_xy]
            snapshot["end_xy_m"] = [rounded(value) for value in last_xy]
        phase_boundary_snapshots.append(snapshot)

    p3_rows = telemetry_by_state.get("PARKUR_3_TARGET_LOCK", [])
    p3_total_rotation = 0.0
    p3_net_rotation = 0.0
    if len(p3_rows) >= 2:
        previous = float(p3_rows[0].get("heading_deg", 0.0))
        for row in p3_rows[1:]:
            current = float(row.get("heading_deg", previous))
            delta = (current - previous + 180.0) % 360.0 - 180.0
            p3_total_rotation += abs(delta)
            p3_net_rotation += delta
            previous = current

    mission_waypoints = mission_payloads[-1].get("waypoints", []) if mission_payloads else []
    summary = {
        "bag": str(Path(args.bag).resolve()),
        "duration_s": rounded((last_ns - first_ns) / 1e9),
        "topic_counts": dict(topic_counts),
        "mission_waypoints": mission_waypoints,
        "phase_segments": [
            {key: rounded(value) if key.endswith("_s") else value for key, value in segment.items()}
            for segment in phase_segments
        ],
        "waypoint_events": waypoint_events,
        "phase_metrics": phase_metrics,
        "top_action_transitions": action_transitions.most_common(30),
        "longest_action_episodes": longest_episodes,
        "phase_boundary_snapshots": phase_boundary_snapshots,
        "risk": {
            "high_yaw_with_forward_samples": len(risky_rows),
            "high_yaw_with_forward_duration_s": rounded(len(risky_rows) * 0.1),
            "high_yaw_examples": risky_rows[:20],
            "under_2m_with_forward_samples": len(obstacle_close_forward_rows),
            "under_2m_with_forward_duration_s": rounded(len(obstacle_close_forward_rows) * 0.1),
            "under_2m_examples": obstacle_close_forward_rows[:20],
        },
        "perception": detection_summary,
        "perception_by_phase": perception_by_phase,
        "p3_search": {
            "telemetry_samples": len(p3_rows),
            "total_absolute_heading_change_deg": rounded(p3_total_rotation),
            "net_heading_change_deg": rounded(p3_net_rotation),
            "equivalent_full_rotations": rounded(p3_total_rotation / 360.0),
        },
        "final_state": state_rows[-1] if state_rows else None,
    }

    output = json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False)
    if args.output:
        Path(args.output).write_text(output + "\n", encoding="utf-8")
    else:
        print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
