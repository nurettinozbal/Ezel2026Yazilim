"""Persistent, strictly validated Arducam B0495 field-control profile."""

from __future__ import annotations

import json
import math
import os
import secrets
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCHEMA_VERSION = 1
CONTROL_LIMITS = {
    "auto_exposure": (0, 1),
    "exposure": (5, 660),
    "gain": (168, 1600),
    "auto_wb": (0, 1),
    "wb_kelvin": (2300, 6500),
    "saturation": (0, 15),
    "brightness": (0, 128),
    "contrast": (0, 20),
    "power_line_frequency": (0, 2),
}


def validate_camera_controls(payload: Any, *, complete: bool = True) -> dict[str, int]:
    if not isinstance(payload, dict) or not payload:
        raise ValueError("camera controls must be a non-empty object")
    keys = set(payload)
    expected = set(CONTROL_LIMITS)
    if keys.difference(expected) or (complete and keys != expected):
        raise ValueError("camera controls do not match the field profile schema")
    result: dict[str, int] = {}
    for name, raw in payload.items():
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"{name} must be numeric")
        value = float(raw)
        low, high = CONTROL_LIMITS[name]
        if not math.isfinite(value) or int(value) != value or not low <= int(value) <= high:
            raise ValueError(f"{name} outside [{low}, {high}]")
        result[name] = int(value)
    return result


def controls_to_ros_parameters(controls: Any) -> dict[str, Any]:
    values = validate_camera_controls(controls, complete=True)
    return {
        "auto_exposure": 0 if values["auto_exposure"] else 1,
        "exposure_time_absolute": values["exposure"],
        "gain": values["gain"],
        "white_balance_automatic": bool(values["auto_wb"]),
        "white_balance_temperature": values["wb_kelvin"],
        "saturation": values["saturation"],
        "brightness": values["brightness"] - 64,
        "contrast": values["contrast"],
        "power_line_frequency": values["power_line_frequency"],
    }


def load_camera_control_profile(path: str | Path) -> dict[str, Any] | None:
    profile_path = Path(path).expanduser()
    if not profile_path.exists():
        return None
    if not profile_path.is_file() or profile_path.stat().st_size > 4096:
        raise ValueError("camera field profile is not a bounded regular file")
    try:
        payload = json.loads(profile_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError("camera field profile could not be read") from exc
    if not isinstance(payload, dict) or set(payload) != {
        "schema_version", "saved_at_utc", "controls"
    }:
        raise ValueError("camera field profile keys are invalid")
    if payload["schema_version"] != SCHEMA_VERSION:
        raise ValueError("camera field profile schema version is unsupported")
    saved_at = payload["saved_at_utc"]
    if not isinstance(saved_at, str) or not 20 <= len(saved_at) <= 40 or not saved_at.endswith("Z"):
        raise ValueError("camera field profile timestamp is invalid")
    return {
        "schema_version": SCHEMA_VERSION,
        "saved_at_utc": saved_at,
        "controls": validate_camera_controls(payload["controls"], complete=True),
    }


def save_camera_control_profile(
    path: str | Path, controls: Any, *, saved_at_utc: str | None = None
) -> dict[str, Any]:
    profile_path = Path(path).expanduser()
    parent = profile_path.parent
    parent.mkdir(parents=True, exist_ok=True)
    saved_at = saved_at_utc or datetime.now(timezone.utc).isoformat(timespec="seconds").replace(
        "+00:00", "Z"
    )
    payload = {
        "schema_version": SCHEMA_VERSION,
        "saved_at_utc": saved_at,
        "controls": validate_camera_controls(controls, complete=True),
    }
    if not isinstance(saved_at, str) or not 20 <= len(saved_at) <= 40 or not saved_at.endswith("Z"):
        raise ValueError("camera field profile timestamp is invalid")
    temporary = parent / f".{profile_path.name}.{secrets.token_hex(6)}.tmp"
    data = (json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode(
        "utf-8"
    )
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, profile_path)
        try:
            os.chmod(profile_path, 0o600)
        except OSError:
            pass
    finally:
        if temporary.exists():
            temporary.unlink()
    return payload


def save_versioned_camera_control_profile(
    active_path: str | Path, controls: Any, *, now: datetime | None = None
) -> tuple[dict[str, Any], Path]:
    moment = now or datetime.now().astimezone()
    if moment.tzinfo is None or moment.utcoffset() is None:
        raise ValueError("camera profile timestamp must include a timezone")
    saved_at_utc = moment.astimezone(timezone.utc).isoformat(timespec="seconds").replace(
        "+00:00", "Z"
    )
    active = Path(active_path).expanduser()
    history = active.parent / "camera_profiles"
    stamp = moment.strftime("%Y%m%d_%H%M%S_%f")
    archive = history / f"camera_{stamp}.json"
    if archive.exists():
        archive = history / f"camera_{stamp}_{secrets.token_hex(3)}.json"
    payload = save_camera_control_profile(
        archive, controls, saved_at_utc=saved_at_utc
    )
    save_camera_control_profile(active, controls, saved_at_utc=saved_at_utc)
    return payload, archive
