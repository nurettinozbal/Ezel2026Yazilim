"""Single safety gate for every frontend command."""

from __future__ import annotations

import threading
from typing import Any


IDA_COMMANDS = {
    "ARM_IDA",
    "DISARM_IDA",
    "SET_IDA_MODE",
    "UPLOAD_IDA_MISSION",
    "START_IDA_MISSION",
    "STOP_IDA_MISSION",
    "SEND_TARGET_TO_IDA",
}
IHA_COMMANDS = {"ARM_IHA", "DISARM_IHA", "SET_IHA_MODE"}
ARM_OR_START = {"ARM_IDA", "ARM_IHA", "START_IDA_MISSION"}
# İDA otonom görev başladıktan sonra (şartname §5.5.3.1) kilitlenecek komutlar.
# İHA komutları bilinçli olarak kapsam dışı: İHA görev sırasında manuel kontrol
# edilebilir olmalıdır (§5.5.3.1 "İHA'nın kontrolü manuel olarak yapılabilecektir").
MISSION_LOCKED_COMMANDS = (
    IDA_COMMANDS - {"STOP_IDA_MISSION", "DISARM_IDA"}
) | {"LOCK_TARGET", "FORCE_UPDATE_TARGET"}
RADIO_VEHICLES = {"ida", "iha"}
IDA_MODES = {"MANUAL", "GUIDED", "HOLD", "RTL"}
IHA_MODES = {"LOITER", "GUIDED", "AUTO", "RTL"}
WARNING_STATUSES = {"sent_unconfirmed", "acked_unverified", "partial_unconfirmed", "busy"}
ERROR_STATUSES = {"rejected", "failed", "timeout", "aborted", "blocked_unverified", "partial_failed"}


class CommandGate:
    def __init__(
        self,
        ida_vehicle: Any,
        iha_vehicle: Any,
        telemetry_state: Any,
        mission_manager: Any,
        target_manager: Any,
        logger: Any,
    ) -> None:
        self.ida = ida_vehicle
        self.iha = iha_vehicle
        self.state = telemetry_state
        self.missions = mission_manager
        self.targets = target_manager
        self.logger = logger
        self._safety_lock = threading.RLock()

    def execute(self, command: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        command = str(command or "").upper()
        payload = payload if isinstance(payload, dict) else {}
        # Mission upload may wait for protocol requests; emergency must not wait behind it.
        if command == "UPLOAD_IDA_MISSION":
            with self._safety_lock:
                allowed, reason = self._allow(command, payload)
                if allowed:
                    self.missions.invalidate_uploaded_mission()
            ok, message, status = self._execute_allowed(command, payload, allowed, reason)
        else:
            with self._safety_lock:
                allowed, reason = self._allow(command, payload)
                ok, message, status = self._execute_allowed(command, payload, allowed, reason)

        result = {"ok": bool(ok), "command": command, "message": message, "status": status}
        self.logger.command(command, payload, result)
        if allowed:
            self.logger.system(
                f"{command} [{status}]: {message}",
                self._log_level(ok, status),
            )
        return result

    def _execute_allowed(
        self,
        command: str,
        payload: dict[str, Any],
        allowed: bool,
        reason: str,
    ) -> tuple[bool, str, str]:
        if not allowed:
            self.logger.system(f"KOMUT REDDEDİLDİ {command}: {reason}", "WARNING")
            return False, reason, "rejected"
        try:
            return self._normalize_result(self._dispatch(command, payload))
        except Exception as exc:
            self.logger.system(f"{command} işleme hatası: {exc}", "ERROR")
            return False, f"Komut işleme hatası: {exc}", "failed"

    def _allow(self, command: str, payload: dict[str, Any]) -> tuple[bool, str]:
        known = IDA_COMMANDS | IHA_COMMANDS | {
            "EMERGENCY_STOP_ALL",
            "RESET_EMERGENCY",
            "LOCK_TARGET",
            "FORCE_UPDATE_TARGET",
            "SET_RADIO_FREQUENCY",
        }
        if command not in known:
            return False, "Bilinmeyen komut"
        if command == "EMERGENCY_STOP_ALL":
            return True, ""
        if command == "RESET_EMERGENCY":
            if not self.state.is_connected("ida") or not self.state.is_connected("iha"):
                return False, "Emergency sıfırlanamaz: iki araç bağlantısı da doğrulanmalı"
            if self.state.is_armed("ida") or self.state.is_armed("iha"):
                return False, "Emergency sıfırlanamaz: araçlardan biri armed"
            return True, ""
        if (
            command in MISSION_LOCKED_COMMANDS
            and self.targets.snapshot()["mission_started"]
            and not self.state.emergency_active
        ):
            return False, "İDA otonom görevi devam ediyor; acil durdurma dışında komut kabul edilmez (şartname §5.5.3.1)"
        if self.state.emergency_active and command in ARM_OR_START:
            return False, "Emergency kilidi aktif"
        if self.state.emergency_active and command == "UPLOAD_IDA_MISSION":
            return False, "Emergency kilidi aktifken görev yüklenemez"
        if self.state.emergency_active and command == "SET_IDA_MODE":
            if str(payload.get("mode", "")).upper() not in {"HOLD", "RTL"}:
                return False, "Emergency sırasında İDA yalnız HOLD/RTL moduna alınabilir"
        if self.state.emergency_active and command == "SET_IHA_MODE":
            if str(payload.get("mode", "")).upper() not in {"LOITER", "RTL"}:
                return False, "Emergency sırasında İHA yalnız LOITER/RTL moduna alınabilir"
        if command in IDA_COMMANDS and not self.state.is_connected("ida"):
            return False, "İDA bağlantısı yok"
        if command in IHA_COMMANDS and not self.state.is_connected("iha"):
            return False, "İHA bağlantısı yok"
        if command == "UPLOAD_IDA_MISSION" and not payload.get("waypoints"):
            return False, "Görev waypoint listesi boş"
        if command == "START_IDA_MISSION" and not self.missions.snapshot()["has_uploaded_mission"]:
            return False, "Görev başlatılamaz: İDA’ya yüklenmiş görev yok."
        if command == "START_IDA_MISSION" and not self.state.is_armed("ida"):
            return False, "Görev başlatılamaz: İDA armed değil."
        if command == "SET_IDA_MODE" and str(payload.get("mode", "")).upper() not in IDA_MODES:
            return False, "Geçersiz İDA modu"
        if command == "SET_IHA_MODE" and str(payload.get("mode", "")).upper() not in IHA_MODES:
            return False, "Geçersiz İHA modu"
        if command == "LOCK_TARGET" and self.targets.snapshot()["is_locked"]:
            return False, "Hedef zaten kilitli"
        if command == "FORCE_UPDATE_TARGET":
            safe_mode = self.state.emergency_active or (
                not self.state.is_armed("ida") and not self.state.is_armed("iha")
            )
            if not safe_mode:
                return False, "Force hedef güncellemesi yalnız güvenli/emergency modda"
        if command == "SET_RADIO_FREQUENCY":
            vehicle = str(payload.get("vehicle", "")).lower().strip()
            if vehicle not in RADIO_VEHICLES:
                return False, "Frekans yazılacak araç geçersiz (ida/iha)"
            try:
                freq_khz = int(payload.get("freq_khz", 0))
            except (TypeError, ValueError):
                return False, "Frekans sayısal olmalı"
            if freq_khz <= 0:
                return False, "Frekans sıfırdan büyük olmalı"
            # Frekans yazma Pixhawk'ı reboot eder ve linki geçici olarak keser;
            # yalnız disarmed ve emergency olmayan durumda güvenlidir.
            if self.state.emergency_active:
                return False, "Emergency kilidi aktifken frekans değiştirilemez"
            if self.state.is_armed(vehicle):
                return False, "Frekans yalnız araç disarmed iken değiştirilebilir"
            if vehicle == "ida" and self.targets.snapshot()["mission_started"]:
                return False, "İDA otonom görevi devam ederken frekans değiştirilemez"
            if not self.state.is_connected(vehicle):
                return False, f"{'İDA' if vehicle == 'ida' else 'İHA'} bağlantısı yok"
        return True, ""

    def _dispatch(self, command: str, payload: dict[str, Any]) -> Any:
        if command == "EMERGENCY_STOP_ALL":
            self.state.set_emergency()
            ida_ok, ida_message, ida_status = self._normalize_result(self.ida.send_emergency())
            iha_ok, iha_message, iha_status = self._normalize_result(self.iha.send_emergency())
            return (
                ida_ok and iha_ok,
                f"İDA [{ida_status}]: {ida_message}; İHA [{iha_status}]: {iha_message}",
                self._combine_status([ida_status, iha_status]),
            )
        if command == "RESET_EMERGENCY":
            stop_ok, stop_message, stop_status = self._normalize_result(
                self.ida.stop_mission()
            )
            if not stop_ok or stop_status != "state_verified":
                return False, (
                    "Emergency sıfırlanmadı: Jetson STOP ACK gerekli. "
                    f"{stop_message}"
                ), "blocked_unverified"
            self.state.reset_emergency()
            self.targets.reset_mission_started()
            return True, "Emergency kilidi ve Jetson mission durumu kontrollü sıfırlandı", "state_verified"
        if command == "ARM_IDA":
            return self.ida.send_arm()
        if command == "DISARM_IDA":
            return self.ida.send_disarm()
        if command == "ARM_IHA":
            return self.iha.send_arm()
        if command == "DISARM_IHA":
            return self.iha.send_disarm()
        if command == "SET_IDA_MODE":
            return self.ida.set_mode(str(payload.get("mode", "")))
        if command == "SET_IHA_MODE":
            return self.iha.set_mode(str(payload.get("mode", "")))
        if command == "UPLOAD_IDA_MISSION":
            return self.missions.upload_ida_mission(payload.get("waypoints"))
        if command == "START_IDA_MISSION":
            ok, message, status = self._normalize_result(self.ida.start_mission())
            if ok and status in {"acked", "state_verified"}:
                self.targets.mark_mission_started()
            return ok, message, status
        if command == "STOP_IDA_MISSION":
            ok, message, status = self._normalize_result(self.ida.stop_mission())
            if ok and status == "state_verified":
                self.targets.reset_mission_started()
            return ok, message, status
        if command == "LOCK_TARGET":
            return self.targets.lock_target(payload)
        if command == "FORCE_UPDATE_TARGET":
            return self.targets.lock_target(payload, force=True)
        if command == "SET_RADIO_FREQUENCY":
            vehicle = str(payload.get("vehicle", "")).lower().strip()
            target_vehicle = self.ida if vehicle == "ida" else self.iha
            return target_vehicle.run_radio_config(int(payload.get("freq_khz", 0)))
        if command == "SEND_TARGET_TO_IDA":
            target = self.targets.snapshot()
            if not target["is_locked"]:
                return False, "Gönderilecek kilitli hedef yok", "rejected"
            ok, message, status = self._normalize_result(self.ida.send_target_info(target))
            self.targets.mark_delivery(status, message)
            return ok, message, status
        return False, "Komut uygulanamadı", "failed"

    @staticmethod
    def _normalize_result(result: Any) -> tuple[bool, str, str]:
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

    @staticmethod
    def _combine_status(statuses: list[str]) -> str:
        if any(status in ERROR_STATUSES for status in statuses):
            return "partial_failed"
        if any(status in WARNING_STATUSES for status in statuses):
            return "partial_unconfirmed"
        if all(status == "state_verified" for status in statuses):
            return "state_verified"
        if all(status in {"acked", "state_verified", "accepted"} for status in statuses):
            return "acked"
        return "partial_unconfirmed"

    @staticmethod
    def _log_level(ok: bool, status: str) -> str:
        if not ok or status in ERROR_STATUSES:
            return "ERROR"
        if status in WARNING_STATUSES:
            return "WARNING"
        return "SUCCESS"
