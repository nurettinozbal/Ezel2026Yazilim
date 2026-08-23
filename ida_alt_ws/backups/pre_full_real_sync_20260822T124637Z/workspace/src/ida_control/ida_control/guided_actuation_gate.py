"""ROS-independent safety gate for live GUIDED actuation.

The gate deliberately uses monotonic receipt time.  A fresh ROS command alone
is not sufficient to move the vehicle: the bridge must have observed a
disarmed state since boot, a fresh armed state, fresh global/local position
health and the autopilot must still report GUIDED.  Once an active armed cycle
loses any of those conditions the gate latches closed until DISARM is observed.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be finite")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def normalize_flight_mode(value: object) -> str:
    """Return a stable uppercase MAVSDK flight-mode name."""

    raw = getattr(value, "name", value)
    text = str(raw).strip().upper()
    if "." in text:
        text = text.rsplit(".", 1)[-1]
    return text


@dataclass(frozen=True)
class GuidedGateDecision:
    allowed: bool
    latched: bool
    reason: str


class GuidedActuationGate:
    """Fail-closed state machine for one arm/GUIDED cycle."""

    def __init__(self, timeout_s: float = 2.5) -> None:
        timeout_s = _finite(timeout_s, "timeout_s")
        if not 0.5 <= timeout_s <= 10.0:
            raise ValueError("timeout_s must be in [0.5,10]")
        self.timeout_s = timeout_s
        self._lock = threading.RLock()
        self._armed: bool | None = None
        self._armed_at: float | None = None
        self._mode = ""
        self._mode_at: float | None = None
        self._global_position_ok = False
        self._local_position_ok = False
        self._health_at: float | None = None
        self._last_update: float | None = None
        self._disarmed_seen = False
        self._active_once = False
        self._latched_reason = ""

    def _accept_time(self, now_mono: float) -> float:
        now_mono = _finite(now_mono, "now_mono")
        if self._last_update is not None and now_mono < self._last_update:
            if self._active_once:
                self._latched_reason = "monotonic_clock_rollback"
            self._mode_at = None
            self._health_at = None
            self._armed_at = None
        self._last_update = now_mono
        return now_mono

    def mark_armed(self, armed: object, now_mono: float) -> None:
        if not isinstance(armed, bool):
            raise ValueError("armed must be bool")
        with self._lock:
            now_mono = self._accept_time(now_mono)
            self._armed = armed
            self._armed_at = now_mono
            if not armed:
                self._disarmed_seen = True
                self._active_once = False
                self._latched_reason = ""

    def mark_flight_mode(self, mode: object, now_mono: float) -> None:
        with self._lock:
            now_mono = self._accept_time(now_mono)
            self._mode = normalize_flight_mode(mode)
            self._mode_at = now_mono

    def mark_position_health(
        self, global_position_ok: object, local_position_ok: object, now_mono: float
    ) -> None:
        if not isinstance(global_position_ok, bool) or not isinstance(
            local_position_ok, bool
        ):
            raise ValueError("position health fields must be bool")
        with self._lock:
            now_mono = self._accept_time(now_mono)
            self._global_position_ok = global_position_ok
            self._local_position_ok = local_position_ok
            self._health_at = now_mono

    def mark_disconnected(self) -> None:
        with self._lock:
            if self._active_once:
                self._latched_reason = "vehicle_link_lost"
            self._mode_at = None
            self._health_at = None
            self._armed_at = None

    def decision(self, now_mono: float) -> GuidedGateDecision:
        now_mono = _finite(now_mono, "now_mono")
        with self._lock:
            if self._last_update is not None and now_mono < self._last_update:
                if self._active_once:
                    self._latched_reason = "monotonic_clock_rollback"
                return GuidedGateDecision(False, bool(self._latched_reason), "clock_invalid")
            if self._latched_reason:
                return GuidedGateDecision(False, True, self._latched_reason)
            if self._armed is not True:
                return GuidedGateDecision(False, False, "vehicle_disarmed")
            if not self._disarmed_seen:
                return GuidedGateDecision(False, False, "disarmed_cycle_not_seen")

            timings = (self._armed_at, self._mode_at, self._health_at)
            if any(item is None for item in timings):
                reason = "guided_health_incomplete"
            elif any(not 0.0 <= now_mono - float(item) <= self.timeout_s for item in timings):
                reason = "guided_health_stale"
            # MAVSDK maps ArduPilot Rover custom mode 15 (GUIDED) to its
            # cross-autopilot FlightMode.OFFBOARD enum.  Accept the literal
            # GUIDED name as well for deterministic mocks/future adapters.
            elif self._mode not in {"GUIDED", "OFFBOARD"}:
                reason = f"flight_mode_{self._mode.lower() or 'unknown'}"
            elif not self._global_position_ok:
                reason = "global_position_unhealthy"
            elif not self._local_position_ok:
                reason = "local_position_ekf_unhealthy"
            else:
                self._active_once = True
                return GuidedGateDecision(True, False, "ready")

            if self._active_once:
                self._latched_reason = reason
                return GuidedGateDecision(False, True, reason)
            return GuidedGateDecision(False, False, reason)

    def snapshot(self, now_mono: float) -> dict:
        with self._lock:
            decision = self.decision(now_mono)
            return {
                "allowed": decision.allowed,
                "latched": decision.latched,
                "reason": decision.reason,
                "mode": self._mode or None,
                "armed": self._armed,
                "global_position_ok": self._global_position_ok,
                "local_position_ok": self._local_position_ok,
                "disarmed_cycle_seen": self._disarmed_seen,
            }
