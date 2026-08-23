"""Small, dependency-free contract for the three referee log files.

The referee bundle is intentionally smaller than the internal decision log.
It preserves enough information to replay where the vehicle was, which
course/waypoint it was executing, what final motion was requested and what the
perception/costmap contained.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence


TELEMETRY_CSV_HEADER = [
    "stamp",
    "lat",
    "lon",
    "ground_speed",
    "roll_deg",
    "pitch_deg",
    "heading_deg",
    "mode",
    "parkur",
    "current_waypoint",
    "autonomy_state",
    "autonomy_action",
    "setpoint_vx",
    "setpoint_vy",
    "setpoint_yaw_rate",
    "motor_left_pwm",
    "motor_right_pwm",
]

REQUIRED_FILES = ("telemetry.csv", "processed_video.mp4", "map.mp4")
FRAME_INDEX_FILE = "frames.csv"


def parkur_from_state(state: Any) -> int | None:
    """Return 1/2/3 from the stable autonomy state names, else None."""
    text = str(state or "").upper()
    for parkur in (1, 2, 3):
        if f"PARKUR_{parkur}" in text:
            return parkur
    if text == "ENGAGE":
        return 3
    return None


def build_telemetry_row(
    telemetry: Mapping[str, Any],
    autonomy: Mapping[str, Any],
    command: Mapping[str, Any],
    fallback_stamp: float,
) -> Dict[str, Any]:
    """Build one fixed-schema telemetry row from the latest source samples."""
    stamp = telemetry.get("stamp", fallback_stamp)
    try:
        stamp = float(stamp)
    except (TypeError, ValueError, OverflowError):
        stamp = fallback_stamp
    state = autonomy.get("state", "")
    return {
        "stamp": stamp,
        "lat": telemetry.get("lat"),
        "lon": telemetry.get("lon"),
        "ground_speed": telemetry.get("ground_speed"),
        "roll_deg": telemetry.get("roll_deg"),
        "pitch_deg": telemetry.get("pitch_deg"),
        "heading_deg": telemetry.get("heading_deg"),
        "mode": telemetry.get("mode"),
        "parkur": parkur_from_state(state),
        "current_waypoint": autonomy.get("current_waypoint"),
        "autonomy_state": state,
        "autonomy_action": autonomy.get("action"),
        "setpoint_vx": command.get("vx", 0.0),
        "setpoint_vy": command.get("vy", 0.0),
        "setpoint_yaw_rate": command.get("yaw_rate", 0.0),
        "motor_left_pwm": telemetry.get("motor_left_pwm"),
        "motor_right_pwm": telemetry.get("motor_right_pwm"),
    }


def build_map_row(
    *,
    stamp: float,
    telemetry: Mapping[str, Any],
    autonomy: Mapping[str, Any],
    command: Mapping[str, Any],
    obstacles: Sequence[Mapping[str, Any]],
    buoys: Sequence[Mapping[str, Any]],
    cells: Sequence[Sequence[Any]],
    costmap_meta: Mapping[str, Any],
    score: Mapping[str, Any],
) -> Dict[str, Any]:
    """Build the compact local-map JSON object written once per log tick."""
    state = autonomy.get("state", "")
    return {
        "stamp": stamp,
        "lat": telemetry.get("lat"),
        "lon": telemetry.get("lon"),
        "heading_deg": telemetry.get("heading_deg"),
        "autonomy": {
            "state": state,
            "parkur": parkur_from_state(state),
            "current_waypoint": autonomy.get("current_waypoint"),
            "action": autonomy.get("action"),
        },
        "command": {
            "vx": command.get("vx", 0.0),
            "vy": command.get("vy", 0.0),
            "yaw_rate": command.get("yaw_rate", 0.0),
        },
        "obstacles": list(obstacles),
        "buoys": list(buoys),
        "cells": list(cells),
        "costmap": dict(costmap_meta),
        "score": dict(score),
    }


def _file_size(path: Path) -> int:
    try:
        return path.stat().st_size if path.is_file() else 0
    except OSError:
        return 0


def _has_data_after_header(path: Path) -> bool:
    """Read at most two bounded lines; a header-only CSV is not evidence."""
    try:
        with path.open("r", encoding="utf-8", errors="replace") as stream:
            stream.readline(16_384)
            return bool(stream.readline(16_384).strip())
    except OSError:
        return False


def _has_finalized_video_segment(root: Path, stem: str) -> bool:
    """A journalled non-empty segment is recoverable video evidence."""
    manifest = root / f"{stem}_segments.jsonl"
    try:
        with manifest.open("r", encoding="utf-8", errors="replace") as stream:
            for _ in range(4096):
                line = stream.readline(16_384)
                if not line:
                    break
                try:
                    record = json.loads(line)
                except (TypeError, ValueError):
                    continue
                relative = record.get("file") if isinstance(record, dict) else None
                frame_count = record.get("frame_count") if isinstance(record, dict) else None
                if (
                    not isinstance(relative, str)
                    or not isinstance(frame_count, int)
                    or isinstance(frame_count, bool)
                    or frame_count <= 0
                ):
                    continue
                candidate = root / relative
                if candidate.parent != root / f"{stem}_segments":
                    continue
                if not candidate.name.startswith(f"{stem}_") or candidate.suffix != ".mp4":
                    continue
                if _file_size(candidate) > 0:
                    return True
    except (OSError, json.JSONDecodeError):
        return False
    return False


def evaluate_referee_files(run_dir: str | Path) -> Dict[str, Any]:
    """Fail-closed readiness for telemetry, processed video and local map."""
    root = Path(run_dir)
    telemetry = root / "telemetry.csv"
    video = root / "processed_video.mp4"
    local_map = root / "map.mp4"
    frames = root / FRAME_INDEX_FILE

    telemetry_ready = _has_data_after_header(telemetry)
    delivery_map_ready = _file_size(local_map) > 0
    segment_map_ready = _has_finalized_video_segment(root, "map")
    map_ready = delivery_map_ready or segment_map_ready
    frames_ready = _has_data_after_header(frames)
    delivery_video_ready = _file_size(video) > 0
    segment_video_ready = _has_finalized_video_segment(root, "processed_video")
    video_ready = (delivery_video_ready or segment_video_ready) and frames_ready
    states = {
        "telemetry.csv": {
            "ready": telemetry_ready,
            "bytes": _file_size(telemetry),
        },
        "processed_video.mp4": {
            "ready": video_ready,
            "bytes": _file_size(video),
            "frame_index_ready": frames_ready,
            "delivery_ready": delivery_video_ready,
            "recoverable_segments_ready": segment_video_ready,
        },
        "map.mp4": {
            "ready": map_ready,
            "bytes": _file_size(local_map),
            "delivery_ready": delivery_map_ready,
            "recoverable_segments_ready": segment_map_ready,
        },
    }
    ready = [name for name in REQUIRED_FILES if states[name]["ready"]]
    return {
        "active": len(ready) == len(REQUIRED_FILES),
        "logger_count": len(ready),
        "ready_files": ready,
        "file_status": states,
    }
