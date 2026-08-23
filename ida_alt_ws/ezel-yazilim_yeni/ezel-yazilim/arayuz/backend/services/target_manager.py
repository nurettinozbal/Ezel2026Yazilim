"""Competition target lock state."""

from __future__ import annotations

import threading
import time
from typing import Any


VALID_COLORS = {"KIRMIZI", "YEŞİL", "SİYAH"}
VALID_SOURCES = {"IHA", "MANUAL"}

# İHA -> YKİ NAMED_VALUE_INT ve YKİ -> İDA SCR_USER4 aynı kodları
# kullanır. Parkur-3 model/otonomi kontratı yalnız bu üç rengi kabul eder.
COLOR_CODES = {"KIRMIZI": 1, "YEŞİL": 2, "SİYAH": 4}
CODE_TO_COLOR = {code: color for color, code in COLOR_CODES.items()}

# MAVLink NAMED_VALUE_INT.name alanı char[10]'dur; "TARGET_COLOR" (12 karakter)
# hat üzerinde "TARGET_COL" olarak kırpılır. Her iki biçim de kabul edilir.
TARGET_COLOR_FIELD_NAMES = {"TARGET_COLOR", "TARGET_COL"}


def normalize_named_value_field(raw_name: Any) -> str:
    """NAMED_VALUE_INT.name alanını karşılaştırılabilir hale getirir."""
    if isinstance(raw_name, (bytes, bytearray)):
        raw_name = raw_name.decode("ascii", errors="replace")
    return str(raw_name or "").replace("\x00", "").strip().upper()


class TargetManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.target_color = "BEKLENİYOR"
        self.confidence = 0.0
        self.source = "MANUAL"
        self.locked_at = 0.0
        self.is_locked = False
        self.mission_started = False
        self.delivery_status = "not_sent"
        self.delivery_message = ""
        self.delivered_at = 0.0

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "target_color": self.target_color,
                "confidence": self.confidence,
                "source": self.source,
                "locked_at": self.locked_at,
                "is_locked": self.is_locked,
                "mission_started": self.mission_started,
                "delivery_status": self.delivery_status,
                "delivery_message": self.delivery_message,
                "delivered_at": self.delivered_at,
            }

    def lock_target(self, payload: dict[str, Any], force: bool = False) -> tuple[bool, str]:
        color = str(payload.get("color", "")).upper().strip()
        source = str(payload.get("source", "MANUAL")).upper().strip()
        try:
            confidence = float(payload.get("confidence", 1.0))
        except (TypeError, ValueError):
            return False, "confidence sayısal olmalı"

        if color not in VALID_COLORS:
            return False, f"Geçersiz hedef rengi: {color or 'BOŞ'}"
        if source not in VALID_SOURCES:
            return False, f"Geçersiz hedef kaynağı: {source}"
        if not 0.0 <= confidence <= 1.0:
            return False, "confidence 0.0-1.0 aralığında olmalı"

        with self._lock:
            if (self.is_locked or self.mission_started) and not force:
                return False, "Hedef kilitli; FORCE_UPDATE_TARGET gerekli"
            self.target_color = color
            self.confidence = confidence
            self.source = source
            self.locked_at = time.time()
            self.is_locked = True
            self.delivery_status = "pending_send"
            self.delivery_message = "Hedef kilitlendi, İDA gönderimi bekleniyor"
            self.delivered_at = 0.0
        return True, f"Hedef kilitlendi: {color}"

    def mark_mission_started(self) -> None:
        with self._lock:
            self.mission_started = True

    def reset_mission_started(self) -> None:
        with self._lock:
            self.mission_started = False

    def mark_delivery(self, status: str, message: str) -> None:
        with self._lock:
            self.delivery_status = status
            self.delivery_message = message
            self.delivered_at = time.time()

    def confirm_vehicle_target(self, color_code: int) -> tuple[bool, str]:
        """Mark delivery verified only when Jetson echoes the locked color.

        ``TGT_ACK`` is emitted by the onboard bridge after it has read SCR_USER4
        and published ``/mission/target_color``.  A stale/different acknowledgement
        must never verify the current target.
        """
        color = CODE_TO_COLOR.get(int(color_code))
        with self._lock:
            if not self.is_locked:
                return False, "Kilitli hedef yok; İDA doğrulaması yok sayıldı"
            if color != self.target_color:
                return False, (
                    f"İDA hedef doğrulaması eşleşmedi: "
                    f"beklenen={self.target_color}, gelen={color or color_code}"
                )
            self.delivery_status = "state_verified"
            self.delivery_message = f"İDA Jetson hedefi okudu ve doğruladı: {color}"
            self.delivered_at = time.time()
        return True, self.delivery_message
