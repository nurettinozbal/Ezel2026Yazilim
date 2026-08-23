"""IDA mission validation and upload orchestration."""

from __future__ import annotations

import math
import threading
import time
from typing import Any


class MissionManager:
    def __init__(self, ida_vehicle: Any, logger: Any) -> None:
        self.ida_vehicle = ida_vehicle
        self.logger = logger
        self._lock = threading.Lock()
        self.has_uploaded_mission = False
        self.last_upload_time = 0.0
        self.waypoint_count = 0

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "has_uploaded_mission": self.has_uploaded_mission,
                "last_upload_time": self.last_upload_time,
                "waypoint_count": self.waypoint_count,
            }

    def invalidate_uploaded_mission(self) -> None:
        with self._lock:
            self.has_uploaded_mission = False
            self.last_upload_time = 0.0
            self.waypoint_count = 0

    def upload_ida_mission(self, waypoints: Any) -> tuple[bool, str, str]:
        if not isinstance(waypoints, list) or not waypoints:
            return False, "Görev waypoint listesi boş", "rejected"
        if len(waypoints) > 1000:
            return False, "Görev en fazla 1000 waypoint içerebilir", "rejected"

        normalized = []
        for index, waypoint in enumerate(waypoints):
            try:
                if not isinstance(waypoint, dict) or any(
                    isinstance(waypoint.get(name), bool)
                    for name in ("lat", "lon", "alt", "parkur")
                ):
                    raise ValueError
                lat = float(waypoint["lat"])
                lon = float(waypoint["lon"])
                alt = float(waypoint.get("alt", 0))
                raw_parkur = waypoint["parkur"]
                if isinstance(raw_parkur, bool):
                    raise ValueError
                parkur = int(raw_parkur)
                if isinstance(raw_parkur, float) and not raw_parkur.is_integer():
                    raise ValueError
            except (KeyError, TypeError, ValueError):
                return False, f"Waypoint #{index + 1} geçersiz", "rejected"
            if (
                not math.isfinite(lat)
                or not math.isfinite(lon)
                or not math.isfinite(alt)
                or not -90 <= lat <= 90
                or not -180 <= lon <= 180
                or not -100 <= alt <= 100
            ):
                return False, f"Waypoint #{index + 1} koordinatı geçersiz", "rejected"
            if parkur not in {1, 2, 3}:
                return False, f"Waypoint #{index + 1} parkur değeri geçersiz", "rejected"
            normalized.append({"lat": lat, "lon": lon, "alt": alt, "parkur": parkur})

        parkurs = [waypoint["parkur"] for waypoint in normalized]
        if parkurs[0] != 1 or parkurs != sorted(parkurs):
            return False, "Waypoint parkurları 1 ile başlamalı ve 1->2->3 sıralı olmalı", "rejected"

        # Upload starts by clearing the vehicle mission, so a failed attempt
        # must invalidate any previously successful mission state.
        self.invalidate_uploaded_mission()

        upload_result = self.ida_vehicle.upload_mission(normalized)
        success, message, status = _normalize_vehicle_result(upload_result)
        if success:
            p1_count = parkurs.count(1)
            p2_count = parkurs.count(2)
            metadata_result = self.ida_vehicle.write_mission_parkur_metadata(
                p1_count, p2_count
            )
            metadata_ok, metadata_message, metadata_status = _normalize_vehicle_result(
                metadata_result
            )
            if not metadata_ok:
                success = False
                message = (
                    f"{message}; parkur metadata doğrulanamadı: {metadata_message}"
                )
                status = metadata_status
        if success:
            with self._lock:
                self.has_uploaded_mission = True
                self.last_upload_time = time.time()
                self.waypoint_count = len(normalized)
        self.logger.system(message, "SUCCESS" if success else "ERROR")
        return success, message, status


def _normalize_vehicle_result(result: Any) -> tuple[bool, str, str]:
    if isinstance(result, dict):
        ok = bool(result.get("ok"))
        return ok, str(result.get("message", "")), str(result.get("status", "accepted" if ok else "failed"))
    if isinstance(result, tuple):
        if len(result) >= 3:
            return bool(result[0]), str(result[1]), str(result[2])
        if len(result) >= 2:
            ok = bool(result[0])
            return ok, str(result[1]), "accepted" if ok else "failed"
    ok = bool(result)
    return ok, str(result), "accepted" if ok else "failed"
