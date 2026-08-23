#!/usr/bin/env python3
"""Analyze Parkur 2 course deviation from a ROS 2 sqlite3 bag.

Only stdlib is required. The tool decodes ``std_msgs/msg/String`` CDR payloads
directly, so it can run on an offline laptop without ROS 2 installed.
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import statistics
import struct
from pathlib import Path
from typing import Any, Iterable


def resolve_db3(path: Path) -> Path:
    if path.is_file() and path.suffix == ".db3":
        return path
    if path.is_dir():
        files = sorted(path.glob("*.db3"))
        if len(files) == 1:
            return files[0]
        if not files:
            raise ValueError(f"bag directory contains no db3 file: {path}")
        raise ValueError(f"multi-file bags are not supported: {path}")
    raise ValueError(f"bag path is not a db3 file or directory: {path}")


def decode_std_string(raw: bytes) -> str:
    if len(raw) < 9:
        raise ValueError("truncated CDR String")
    # CDR encapsulation kind 0x0001 is little-endian, 0x0000 big-endian.
    kind = int.from_bytes(raw[:2], "big")
    if kind not in (0, 1):
        raise ValueError(f"unsupported CDR encapsulation: {kind}")
    endian = "<" if kind == 1 else ">"
    size = struct.unpack_from(f"{endian}I", raw, 4)[0]
    if size < 1 or 8 + size > len(raw):
        raise ValueError("invalid CDR String length")
    payload = raw[8 : 8 + size]
    if payload[-1] != 0:
        raise ValueError("CDR String is not null terminated")
    return payload[:-1].decode("utf-8")


def read_json_topic(connection: sqlite3.Connection, name: str) -> list[tuple[float, dict[str, Any]]]:
    row = connection.execute("SELECT id, type FROM topics WHERE name = ?", (name,)).fetchone()
    if row is None:
        raise ValueError(f"required topic missing: {name}")
    topic_id, type_name = row
    if type_name != "std_msgs/msg/String":
        raise ValueError(f"unexpected type for {name}: {type_name}")
    result = []
    for timestamp_ns, raw in connection.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,),
    ):
        try:
            payload = json.loads(decode_std_string(raw))
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
            continue
        if isinstance(payload, dict):
            result.append((timestamp_ns / 1e9, payload))
    return result


def read_optional_json_topic(
    connection: sqlite3.Connection, name: str
) -> list[tuple[float, dict[str, Any]]]:
    """Read a JSON topic when it exists; an inactive optional publisher is valid."""
    row = connection.execute("SELECT 1 FROM topics WHERE name = ?", (name,)).fetchone()
    if row is None:
        return []
    return read_json_topic(connection, name)


def nearest(records: list[tuple[float, dict[str, Any]]], stamp: float) -> dict[str, Any] | None:
    if not records:
        return None
    _, payload = min(records, key=lambda item: abs(item[0] - stamp))
    return payload


def outside_intervals(
    records: Iterable[tuple[float, dict[str, Any]]],
    exit_threshold_m: float,
    max_sample_gap_s: float,
) -> list[tuple[float, float]]:
    intervals: list[tuple[float, float]] = []
    start: float | None = None
    previous: float | None = None
    for stamp, payload in records:
        if previous is not None and stamp - previous > max_sample_gap_s:
            if start is not None:
                intervals.append((start, previous))
            start = None
        distance = payload["distance_from_course_m"]
        outside = distance > exit_threshold_m or payload.get("inside_course_geometry") is False
        if outside and start is None:
            start = stamp
        elif not outside and start is not None:
            intervals.append((start, previous if previous is not None else stamp))
            start = None
        previous = stamp
    if start is not None and previous is not None:
        intervals.append((start, previous))
    return intervals


def analyze_bag(
    path: Path,
    exit_threshold_m: float = 5.5,
    max_sample_gap_s: float = 1.0,
) -> dict[str, Any]:
    if not math.isfinite(exit_threshold_m) or exit_threshold_m <= 0:
        raise ValueError("exit threshold must be finite and positive")
    if not math.isfinite(max_sample_gap_s) or max_sample_gap_s <= 0:
        raise ValueError("max sample gap must be finite and positive")
    db3 = resolve_db3(path)
    connection = sqlite3.connect(f"file:{db3.resolve()}?mode=ro", uri=True)
    try:
        states = read_json_topic(connection, "/autonomy/state")
        debug = read_json_topic(connection, "/autonomy/debug")
        telemetry = read_json_topic(connection, "/telemetry/state")
        # Pure perception-sim runs may not start the fusion status publisher.
        # Its absence must not make the otherwise complete rosbag unanalyzable.
        fusion = read_optional_json_topic(connection, "/perception/fusion/status")
    finally:
        connection.close()

    p2 = []
    for stamp, payload in states:
        value = payload.get("distance_from_course_m")
        if payload.get("state") != "PARKUR_2_AVOIDANCE":
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            continue
        if not math.isfinite(float(value)) or float(value) < 0:
            continue
        copied = dict(payload)
        copied["distance_from_course_m"] = float(value)
        p2.append((stamp, copied))
    if not p2:
        raise ValueError("bag contains no finite Parkur 2 course geometry samples")

    distances = [payload["distance_from_course_m"] for _, payload in p2]
    ordered = sorted(distances)
    p95 = ordered[math.ceil(0.95 * len(ordered)) - 1]
    maximum_stamp, maximum_state = max(
        p2, key=lambda item: item[1]["distance_from_course_m"]
    )
    intervals = outside_intervals(p2, exit_threshold_m, max_sample_gap_s)
    longest = max((end - start for start, end in intervals), default=0.0)
    return {
        "schema_version": 1,
        "bag": str(db3),
        "parkur2": {
            "sample_count": len(p2),
            "duration_s": round(p2[-1][0] - p2[0][0], 6),
            "exit_threshold_m": exit_threshold_m,
            "distance_m": {
                "minimum": min(distances),
                "median": statistics.median(distances),
                "p95": p95,
                "maximum": max(distances),
                "final": distances[-1],
            },
            "outside_interval_count": len(intervals),
            "longest_continuous_outside_s": round(longest, 6),
            "outside_intervals_s_from_p2_start": [
                [round(start - p2[0][0], 6), round(end - p2[0][0], 6)]
                for start, end in intervals
            ],
            "maximum_state": maximum_state,
            "nearest_debug": nearest(debug, maximum_stamp),
            "nearest_telemetry": nearest(telemetry, maximum_stamp),
            "nearest_fusion_status": nearest(fusion, maximum_stamp),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag directory or its single db3 file")
    parser.add_argument("--exit-threshold-m", type=float, default=5.5)
    parser.add_argument("--max-sample-gap-s", type=float, default=1.0)
    parser.add_argument(
        "--require-max-distance-m",
        type=float,
        default=None,
        help="return exit 2 when measured maximum exceeds this regression limit",
    )
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    result = analyze_bag(args.bag, args.exit_threshold_m, args.max_sample_gap_s)
    encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    else:
        print(encoded, end="")
    limit = args.require_max_distance_m
    if limit is not None:
        if not math.isfinite(limit) or limit <= 0:
            raise ValueError("required max distance must be finite and positive")
        if result["parkur2"]["distance_m"]["maximum"] > limit:
            return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
