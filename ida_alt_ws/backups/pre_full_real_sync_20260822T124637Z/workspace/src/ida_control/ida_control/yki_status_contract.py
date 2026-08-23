"""Compact, allowlisted IDA -> YKİ MAVLink status contract.

This module is pure Python so the wire mapping can be unit-tested without ROS,
pymavlink or vehicle hardware.  Field names stay within MAVLink char[10].
"""

from __future__ import annotations

from typing import Any


STATE_CODE_MAP = {
    "WAIT_MISSION": 1,
    "MISSION_READY": 2,
    "PARKUR_1_NAV": 3,
    "PARKUR_2_AVOIDANCE": 4,
    "PARKUR_3_TARGET_LOCK": 5,
    "ENGAGE": 6,
    "FAILSAFE": 7,
    "COMPLETE": 8,
}

ACTION_CODE_MAP = {
    "idle": 1,
    "hold": 1,
    "waypoint": 2,
    "corridor": 3,
    "avoid": 4,
    "target_hold": 5,
    "engage": 6,
    "failsafe": 7,
    "complete": 8,
}

PARKUR_FROM_STATE = {
    "PARKUR_1_NAV": 1,
    "PARKUR_2_AVOIDANCE": 2,
    "PARKUR_3_TARGET_LOCK": 3,
    "ENGAGE": 3,
}


def autonomy_fields(payload: Any) -> dict[str, int]:
    if not isinstance(payload, dict):
        return {}
    state = str(payload.get("state", "") or "").upper()
    action_text = str(payload.get("action", "") or "").lower()
    action = 1
    for token, code in ACTION_CODE_MAP.items():
        if token in action_text:
            action = code
            break
    return {
        "AUTO_ST": STATE_CODE_MAP.get(state, 0),
        "AUTO_AC": action,
        "PARKUR": PARKUR_FROM_STATE.get(state, 0),
    }


def bounded_count(value: Any) -> int:
    if isinstance(value, bool):
        return 0
    try:
        count = int(value)
    except (TypeError, ValueError, OverflowError):
        return 0
    return min(10000, max(0, count))


def perception_count(payload: Any, list_key: str) -> int:
    items = payload.get(list_key, []) if isinstance(payload, dict) else []
    return 100 + bounded_count(len(items) if isinstance(items, list) else 0)


def logging_fields(payload: Any) -> dict[str, int]:
    if not isinstance(payload, dict):
        return {}
    return {
        "LOG_ACT": 1 if payload.get("active") is True else 0,
        "LOG_CNT": 100 + bounded_count(payload.get("logger_count", 0)),
    }

