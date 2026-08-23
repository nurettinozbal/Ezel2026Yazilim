"""Thread-safe normalized telemetry state."""

from __future__ import annotations

import copy
import math
import threading
import time
from typing import Any

try:
    from pymavlink import mavutil
except ImportError:  # Backend remains observable before field dependencies are installed.
    mavutil = None


class TelemetryState:
    def __init__(self, ida_sys_id: int, iha_sys_id: int, heartbeat_timeout: float) -> None:
        self.heartbeat_timeout = heartbeat_timeout
        self.emergency_active = False
        self._lock = threading.Lock()
        self._state: dict[str, Any] = {
            "ida": {
                "sys_id": ida_sys_id,
                "speed": 0,
                "battery_percent": 0,
                "voltage": 0.0,
                "current": 0.0,
                "mode": "DISARMED",
                "lat": 0,
                "lon": 0,
                "heading": 0,
                "target_heading": None,
                "target_speed": None,
                "current_wp": 0,
                "dist_to_wp": 0,
                "motor_left_pwm": None,
                "motor_right_pwm": None,
                "motor_left_pct": None,
                "motor_right_pct": None,
                "rpm_left": None,
                "rpm_right": None,
                "roll": None,
                "pitch": None,
                "yaw": None,
                "gps_sats": 0,
                "hdop": 9.9,
                "rssi": -99,
                "home_lat": None,
                "home_lon": None,
                "home_locked": False,
                "home_locked_at": 0,
                "return_home_pending": False,
                "return_home_status": "idle",
                "return_home_message": "",
                "armed": False,
                "connected": False,
                "last_heartbeat": 0,
                "autonomy_state": None,
                "autonomy_action": None,
                "parkur": None,
                "perception_detection_count": 0,
                "perception_obstacle_count": 0,
                "logging_active": False,
                "logging_count": 0,
                "autonomy_last_update_monotonic": 0.0,
                "autonomy_fresh": False,
            },
            "iha": {
                "sys_id": iha_sys_id,
                "battery": 0,
                "voltage": 0.0,
                "alt": 0,
                "detected_color": "BEKLENİYOR",
                "mode": "DISARMED",
                "lat": 0,
                "lon": 0,
                "gps_sats": 0,
                "hdop": 9.9,
                "rssi": -99,
                "home_lat": None,
                "home_lon": None,
                "home_locked": False,
                "home_locked_at": 0,
                "return_home_pending": False,
                "return_home_status": "idle",
                "return_home_message": "",
                "armed": False,
                "connected": False,
                "last_heartbeat": 0,
                "last_update": 0,
            },
            "system": {
                "rssi": -99,
                "gps_sats": 0,
                "hdop": 9.9,
                "failsafe_status": "UNKNOWN",
                "ida_link": "DISCONNECTED",
                "iha_link": "DISCONNECTED",
                "emergency_active": False,
            },
        }

    def set_emergency(self) -> None:
        with self._lock:
            self.emergency_active = True
            self._state["system"]["emergency_active"] = True

    def reset_emergency(self) -> None:
        with self._lock:
            self.emergency_active = False
            self._state["system"]["emergency_active"] = False

    def mark_disconnected(self, vehicle: str) -> None:
        with self._lock:
            self._state[vehicle]["connected"] = False
            self._state[vehicle]["last_heartbeat"] = 0
            if vehicle == "ida":
                self._state[vehicle]["autonomy_fresh"] = False

    def has_home_position(self, vehicle: str) -> bool:
        with self._lock:
            state = self._state[vehicle]
            return bool(state["home_locked"] and state["home_lat"] is not None and state["home_lon"] is not None)

    def mark_return_home_pending(self, vehicle: str, message: str) -> None:
        with self._lock:
            state = self._state[vehicle]
            state["return_home_pending"] = True
            state["return_home_status"] = "pending"
            state["return_home_message"] = message

    def mark_return_home_result(self, vehicle: str, status: str, message: str) -> None:
        with self._lock:
            state = self._state[vehicle]
            state["return_home_pending"] = False
            state["return_home_status"] = status
            state["return_home_message"] = message

    def is_connected(self, vehicle: str) -> bool:
        with self._lock:
            self._refresh_links_locked()
            return bool(self._state[vehicle]["connected"])

    def is_armed(self, vehicle: str) -> bool:
        with self._lock:
            return bool(self._state[vehicle]["armed"])

    def mode_is(self, vehicle: str, expected_mode: str) -> bool:
        with self._lock:
            return str(self._state[vehicle].get("mode", "")).upper() == expected_mode.upper()

    def vehicle_snapshot(self, vehicle: str) -> dict[str, Any]:
        with self._lock:
            self._refresh_links_locked()
            return copy.deepcopy(self._state[vehicle])

    def update_message(self, vehicle: str, message: Any, mode_name: str | None = None) -> None:
        now = time.time()
        msg_type = message.get_type()
        with self._lock:
            state = self._state[vehicle]
            system = self._state["system"]
            if vehicle == "iha":
                state["last_update"] = now

            if msg_type == "HEARTBEAT":
                state["connected"] = True
                state["last_heartbeat"] = now
                base_mode = int(getattr(message, "base_mode", 0))
                armed_flag = getattr(getattr(mavutil, "mavlink", None), "MAV_MODE_FLAG_SAFETY_ARMED", 128)
                state["armed"] = bool(base_mode & armed_flag)
                state["mode"] = mode_name or ("ARMED" if state["armed"] else "DISARMED")
                system["failsafe_status"] = str(getattr(message, "system_status", "UNKNOWN"))
            elif msg_type == "GLOBAL_POSITION_INT":
                state["lat"] = getattr(message, "lat", 0) / 1e7
                state["lon"] = getattr(message, "lon", 0) / 1e7
                self._lock_home_position_if_needed(state, state["lat"], state["lon"], now)
                if vehicle == "iha":
                    state["alt"] = getattr(message, "relative_alt", 0) / 1000
                else:
                    hdg = getattr(message, "hdg", 65535)
                    if hdg != 65535:
                        state["heading"] = hdg / 100
            elif msg_type == "VFR_HUD":
                state["speed"] = round(float(getattr(message, "groundspeed", 0)), 2)
                state["heading"] = int(getattr(message, "heading", state.get("heading", 0)))
                if vehicle == "iha":
                    state["alt"] = round(float(getattr(message, "alt", 0)), 2)
            elif msg_type == "ATTITUDE" and vehicle == "ida":
                state["roll"] = round(float(getattr(message, "roll", 0)), 3)
                state["pitch"] = round(float(getattr(message, "pitch", 0)), 3)
                state["yaw"] = round(float(getattr(message, "yaw", 0)), 3)
            elif msg_type == "SYS_STATUS":
                voltage = int(getattr(message, "voltage_battery", 0))
                current = int(getattr(message, "current_battery", -1))
                remaining = int(getattr(message, "battery_remaining", -1))
                if voltage not in (0, 65535):
                    state["voltage"] = round(voltage / 1000, 2)
                if vehicle == "ida" and current >= 0:
                    state["current"] = round(current / 100, 2)
                if remaining >= 0:
                    state["battery" if vehicle == "iha" else "battery_percent"] = remaining
            elif msg_type == "BATTERY_STATUS":
                remaining = int(getattr(message, "battery_remaining", -1))
                current = int(getattr(message, "current_battery", -1))
                voltages = [value for value in getattr(message, "voltages", []) if 0 < value < 65535]
                if voltages:
                    state["voltage"] = round(sum(voltages) / 1000, 2)
                if remaining >= 0:
                    state["battery" if vehicle == "iha" else "battery_percent"] = remaining
                if vehicle == "ida" and current >= 0:
                    state["current"] = round(current / 100, 2)
            elif msg_type == "GPS_RAW_INT":
                # Her araç kendi GPS kalitesini taşır; aksi halde İHA'nın verisi
                # İDA'nınkini ezer. system.* alanları İDA'yı yansıtır (alt bar İDA odaklı).
                sats = int(getattr(message, "satellites_visible", 0))
                eph = int(getattr(message, "eph", 999))
                hdop = round(eph / 100, 2) if eph < 65535 else 9.9
                state["gps_sats"] = sats
                state["hdop"] = hdop
                if vehicle == "ida":
                    system["gps_sats"] = sats
                    system["hdop"] = hdop
            elif msg_type == "RADIO_STATUS":
                raw_rssi = int(getattr(message, "rssi", 0))
                rssi = round(raw_rssi / 1.9 - 127) if raw_rssi else -99
                state["rssi"] = rssi
                if vehicle == "ida":
                    system["rssi"] = rssi
            elif msg_type == "MISSION_CURRENT" and vehicle == "ida":
                state["current_wp"] = int(getattr(message, "seq", 0))
            elif msg_type == "NAV_CONTROLLER_OUTPUT" and vehicle == "ida":
                state["dist_to_wp"] = int(getattr(message, "wp_dist", 0))
                state["target_heading"] = int(getattr(message, "nav_bearing", state.get("target_heading") or 0))
            elif msg_type == "SERVO_OUTPUT_RAW" and vehicle == "ida":
                self._update_motor_pwm(state, "left", getattr(message, "servo9_raw", None))
                self._update_motor_pwm(state, "right", getattr(message, "servo11_raw", None))
            elif msg_type in {"POSITION_TARGET_GLOBAL_INT", "POSITION_TARGET_LOCAL_NED"} and vehicle == "ida":
                # Otopilotun hedef hızı (şartname Dosya2 "Hız set pointi").
                # Yatay bileşenlerin bileşkesi yer hızı setpoint'ini verir.
                self._update_target_speed(state, message)

    def snapshot(self, target: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            self._refresh_links_locked()
            snapshot = copy.deepcopy(self._state)
            snapshot["iha"]["detected_color"] = target["target_color"]
            return snapshot

    def update_ida_autonomy(self, **fields: Any) -> None:
        """Apply only the compact, allowlisted autonomy telemetry fields."""
        allowed = {
            "autonomy_state", "autonomy_action", "parkur",
            "perception_detection_count", "perception_obstacle_count",
            "logging_active", "logging_count",
        }
        with self._lock:
            state = self._state["ida"]
            updated = False
            for key, value in fields.items():
                if key in allowed and value is not None:
                    state[key] = value
                    updated = True
            if updated:
                state["autonomy_last_update_monotonic"] = time.monotonic()
                state["autonomy_fresh"] = bool(state["connected"])

    def _refresh_links_locked(self) -> None:
        now = time.time()
        monotonic_now = time.monotonic()
        ida_last = self._state["ida"]["last_heartbeat"]
        iha_last = self._state["iha"]["last_heartbeat"]
        self._state["ida"]["connected"] = bool(ida_last and now - ida_last <= self.heartbeat_timeout)
        self._state["iha"]["connected"] = bool(iha_last and now - iha_last <= self.heartbeat_timeout)
        autonomy_last = self._state["ida"]["autonomy_last_update_monotonic"]
        self._state["ida"]["autonomy_fresh"] = bool(
            self._state["ida"]["connected"] and autonomy_last
            and monotonic_now - autonomy_last <= self.heartbeat_timeout
        )
        self._state["system"]["ida_link"] = "CONNECTED" if self._state["ida"]["connected"] else "DISCONNECTED"
        self._state["system"]["iha_link"] = "CONNECTED" if self._state["iha"]["connected"] else "DISCONNECTED"

    @staticmethod
    def _update_target_speed(state: dict[str, Any], message: Any) -> None:
        try:
            vx = float(getattr(message, "vx", 0.0))
            vy = float(getattr(message, "vy", 0.0))
        except (TypeError, ValueError):
            return
        # type_mask'te hız bitleri "ignore" işaretliyse otopilot hız setpoint'i yayınlamıyordur.
        VELOCITY_IGNORE_MASK = 0b111000
        try:
            type_mask = int(getattr(message, "type_mask", 0))
        except (TypeError, ValueError):
            type_mask = 0
        if type_mask & VELOCITY_IGNORE_MASK == VELOCITY_IGNORE_MASK:
            return
        state["target_speed"] = round(math.hypot(vx, vy), 2)

    @staticmethod
    def _update_motor_pwm(state: dict[str, Any], side: str, pwm_value: Any) -> None:
        if pwm_value is None:
            return
        try:
            pwm = int(pwm_value)
        except (TypeError, ValueError):
            return
        pct = max(-100.0, min(100.0, round((pwm - 1500) / 5.0, 1)))
        state[f"motor_{side}_pwm"] = pwm
        state[f"motor_{side}_pct"] = pct

    @staticmethod
    def _lock_home_position_if_needed(state: dict[str, Any], lat: Any, lon: Any, timestamp: float) -> None:
        if state["home_locked"]:
            return
        if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
            return
        if not -90 <= lat <= 90 or not -180 <= lon <= 180 or (lat == 0 and lon == 0):
            return
        state["home_lat"] = lat
        state["home_lon"] = lon
        state["home_locked"] = True
        state["home_locked_at"] = timestamp
