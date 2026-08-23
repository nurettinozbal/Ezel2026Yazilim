"""Compact IDA autonomy fields transported over MAVLink NAMED_VALUE_INT."""

from __future__ import annotations

import re
from typing import Any


def normalize_named_value_field(raw_name: Any) -> str:
    if isinstance(raw_name, (bytes, bytearray)):
        raw_name = raw_name.decode("ascii", errors="replace")
    return str(raw_name or "").replace("\x00", "").strip().upper()


IDA_AUTONOMY_FIELD_NAMES = {
    "AUTO_ST", "AUTO_AC", "PARKUR", "PERC_DET", "PERC_OBS", "LOG_ACT", "LOG_CNT",
}

STATE_TO_CODE = {
    "WAIT_MISSION": 1, "MISSION_READY": 2, "PARKUR_1_NAV": 3,
    "PARKUR_2_AVOIDANCE": 4, "PARKUR_3_TARGET_LOCK": 5,
    "ENGAGE": 6, "FAILSAFE": 7, "COMPLETE": 8,
}
CODE_TO_STATE = {code: state for state, code in STATE_TO_CODE.items()}
CODE_TO_ACTION_LABEL = {
    1: "BEKLE", 2: "YOL NOKTA", 3: "KORİDOR", 4: "ENGEL KAÇINMA",
    5: "HEDEF KİLİT", 6: "ANGAGE", 7: "FAILSAFE", 8: "GÖREV TAMAM",
}
PARKUR_CODE_TO_LABEL = {1: "P1", 2: "P2", 3: "P3"}


def decode_ida_field(raw_name: Any, raw_value: Any) -> tuple[str | None, Any]:
    """Return normalized telemetry key/value; unknown/invalid input is ignored."""
    name = normalize_named_value_field(raw_name)
    if name not in IDA_AUTONOMY_FIELD_NAMES:
        return None, None
    if isinstance(raw_value, bool):
        return None, None
    if isinstance(raw_value, int):
        code = raw_value
    elif isinstance(raw_value, (str, bytes, bytearray)):
        try:
            raw_text = raw_value.decode("ascii", errors="strict") if isinstance(raw_value, (bytes, bytearray)) else raw_value
        except UnicodeDecodeError:
            return None, None
        if re.fullmatch(r"-?\d+", raw_text.strip()) is None:
            return None, None
        code = int(raw_text)
    else:
        return None, None

    if name == "AUTO_ST":
        value = CODE_TO_STATE.get(code)
        return ("autonomy_state", value) if value is not None else (None, None)
    if name == "AUTO_AC":
        value = CODE_TO_ACTION_LABEL.get(code)
        return ("autonomy_action", value) if value is not None else (None, None)
    if name == "PARKUR":
        value = PARKUR_CODE_TO_LABEL.get(code)
        return ("parkur", value) if value is not None else (None, None)
    if name == "PERC_DET":
        return ("perception_detection_count", code - 100) if 100 <= code <= 10100 else (None, None)
    if name == "PERC_OBS":
        return ("perception_obstacle_count", code - 100) if 100 <= code <= 10100 else (None, None)
    if name == "LOG_ACT":
        return ("logging_active", bool(code)) if code in {0, 1} else (None, None)
    if name == "LOG_CNT":
        return ("logging_count", code - 100) if 100 <= code <= 10100 else (None, None)
    return None, None
