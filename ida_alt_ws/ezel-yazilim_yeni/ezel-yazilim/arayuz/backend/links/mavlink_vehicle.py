"""Resilient single-vehicle MAVLink link."""

from __future__ import annotations

import math
import queue
import threading
import time
from typing import Any

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

try:
    import serial
except ImportError:  # Backend remains observable before field dependencies are installed.
    serial = None

from services.target_manager import (
    CODE_TO_COLOR,
    COLOR_CODES,
    TARGET_COLOR_FIELD_NAMES,
    normalize_named_value_field,
)

IDA_TARGET_COLOR_PARAM = b"SCR_USER4"
IDA_MISSION_COUNTS_PARAM = b"SCR_USER5"
IDA_MISSION_CONTROL_PARAM = b"SCR_USER6"
MISSION_COUNT_RADIX = 1001
MISSION_MAILBOX_BASE = 8_000_000
MISSION_CONTROL_MAX_SEQUENCE = 1_999_999  # encoded mailbox remains < 16m


def _decode_mission_mailbox(value: Any) -> tuple[str, int, bool] | None:
    token = MAVLinkVehicle._exact_mission_control_token(value)
    if token is None:
        return None
    code = token - MISSION_MAILBOX_BASE
    if code == 0:
        return "ack", 0, False
    residue = code % 4
    if residue == 1:
        return "command", (code + 3) // 4, True
    if residue == 2:
        return "command", (code + 2) // 4, False
    if residue == 3:
        return "ack", (code + 1) // 4, True
    return "ack", code // 4, False


def _next_mission_mailbox(value: Any, *, start: bool) -> tuple[int, int] | None:
    state = _decode_mission_mailbox(value)
    if state is None or state[0] != "ack":
        return None
    sequence = state[1] + 1
    if sequence > MISSION_CONTROL_MAX_SEQUENCE:
        sequence = 1
    command = MISSION_MAILBOX_BASE + 4 * sequence - (3 if start else 2)
    ack = MISSION_MAILBOX_BASE + 4 * sequence - (1 if start else 0)
    return command, ack


class MAVLinkVehicle:
    def __init__(
        self,
        *,
        port: str,
        baudrate: int,
        expected_sys_id: int,
        gcs_sys_id: int,
        name: str,
        state_key: str,
        telemetry_state: Any,
        logger: Any,
        emergency_action: str,
        reconnect_delay: float,
        mission_timeout: float,
        telemetry_rate_hz: int,
        command_ack_timeout: float,
        state_verify_timeout: float,
        auto_return_on_link_loss: bool,
        return_home_mode: str,
        companion_component_id: int | None = None,
        target_detection_handler: Any = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.expected_sys_id = expected_sys_id
        self.gcs_sys_id = gcs_sys_id
        self.name = name
        self.state_key = state_key
        self.telemetry_state = telemetry_state
        self.logger = logger
        self.emergency_action = emergency_action.upper()
        self.reconnect_delay = reconnect_delay
        self.mission_timeout = mission_timeout
        self.telemetry_rate_hz = telemetry_rate_hz
        self.command_ack_timeout = command_ack_timeout
        self.state_verify_timeout = state_verify_timeout
        self.auto_return_on_link_loss = auto_return_on_link_loss
        self.return_home_mode = return_home_mode.upper().strip() or "RTL"
        if companion_component_id is not None and not 1 <= int(companion_component_id) <= 255:
            raise ValueError("companion_component_id must be in [1,255]")
        self.companion_component_id = (
            int(companion_component_id) if companion_component_id is not None else None
        )
        # Yalnız İHA örneği için bağlanır: araç otonom renk tespitini bildirdiğinde çağrılır.
        self.target_detection_handler = target_detection_handler
        self.target_ack_handler: Any = None

        self.master: Any = None
        self.target_component = 1
        self.latest_state: dict[str, Any] = {}
        self._return_home_pending = False
        self._last_reported_target_color: str | None = None
        self._radio_config_active = threading.Event()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._send_lock = threading.Lock()
        self._param_transaction_lock = threading.Lock()
        self._mission_lock = threading.Lock()
        self._mission_messages: queue.Queue[Any] = queue.Queue(maxsize=100)
        self._command_ack_messages: queue.Queue[Any] = queue.Queue(maxsize=100)
        self._param_value_messages: queue.Queue[Any] = queue.Queue(maxsize=100)

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self.read_loop, name=f"{self.name}-mavlink", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._close_connection()
        if self._thread:
            self._thread.join(timeout=2)

    def connect(self) -> bool:
        if mavutil is None:
            self.logger.system(
                f"{self.name}: pymavlink kurulu değil; bağlantı beklemede",
                "ERROR",
            )
            return False
        try:
            self.logger.system(f"{self.name}: {self.port}@{self.baudrate} bağlantısı deneniyor")
            master = mavutil.mavlink_connection(
                self.port,
                baud=self.baudrate,
                source_system=self.gcs_sys_id,
                autoreconnect=True,
            )
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and not self._stop_event.is_set():
                heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                if heartbeat is None:
                    continue
                if heartbeat.get_srcSystem() != self.expected_sys_id:
                    self.logger.system(
                        f"{self.name}: beklenmeyen SYS_ID={heartbeat.get_srcSystem()}, bağlantı reddedildi",
                        "WARNING",
                    )
                    continue
                self.master = master
                self.target_component = heartbeat.get_srcComponent() or 1
                self._handle_message(heartbeat)
                self._request_telemetry_stream()
                self.logger.system(f"{self.name}: MAVLink bağlantısı aktif", "SUCCESS")
                self._reassert_emergency_after_reconnect()
                self._reassert_return_home_after_reconnect()
                return True
            master.close()
            self.logger.system(f"{self.name}: heartbeat zaman aşımı", "WARNING")
        except Exception as exc:
            self.logger.system(f"{self.name}: port açılamadı: {exc}", "ERROR")
        return False

    def read_loop(self) -> None:
        missing_dependency_logged = False
        while not self._stop_event.is_set():
            if mavutil is None:
                if not missing_dependency_logged:
                    self.logger.system(f"{self.name}: reconnect için pymavlink bekleniyor", "WARNING")
                    missing_dependency_logged = True
                self._stop_event.wait(self.reconnect_delay)
                continue
            if self._radio_config_active.is_set():
                # Radio reconfig ham seri porta ihtiyaç duyar; bu sırada reconnect denenmez.
                self._stop_event.wait(0.5)
                continue
            if self.master is None and not self.connect():
                self.telemetry_state.mark_disconnected(self.state_key)
                self._stop_event.wait(self.reconnect_delay)
                continue

            try:
                message = self.master.recv_match(blocking=True, timeout=1)
                if message is None:
                    self._handle_heartbeat_timeout()
                    continue
                if message.get_srcSystem() != self.expected_sys_id:
                    continue
                self._handle_message(message)
            except Exception as exc:
                self.logger.system(f"{self.name}: MAVLink okuma hatası: {exc}", "ERROR")
                self.telemetry_state.mark_disconnected(self.state_key)
                self._mark_return_home_pending_if_needed("MAVLink okuma hatası nedeniyle link koptu")
                self._close_connection()
                self._stop_event.wait(self.reconnect_delay)

    def _handle_heartbeat_timeout(self) -> None:
        if self._stop_event.is_set() or self.telemetry_state.is_connected(self.state_key):
            return
        self.logger.system(f"{self.name}: heartbeat zaman aşımı, reconnect deneniyor", "WARNING")
        self.telemetry_state.mark_disconnected(self.state_key)
        self._mark_return_home_pending_if_needed("Heartbeat zaman aşımı nedeniyle link koptu")
        self._close_connection()

    def _handle_message(self, message: Any) -> None:
        msg_type = message.get_type()
        if msg_type == "BAD_DATA":
            return
        try:
            self.latest_state[msg_type] = message.to_dict()
        except Exception:
            pass

        mode_name = None
        if msg_type == "HEARTBEAT" and mavutil is not None:
            try:
                mode_name = mavutil.mode_string_v10(message)
            except Exception:
                mode_name = None
        self.telemetry_state.update_message(self.state_key, message, mode_name)

        if msg_type == "STATUSTEXT":
            text = getattr(message, "text", "")
            if isinstance(text, bytes):
                text = text.decode(errors="replace")
            self.logger.system(f"{self.name} STATUSTEXT: {text}")
        if msg_type in {"MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"}:
            try:
                self._mission_messages.put_nowait(message)
            except queue.Full:
                self.logger.system(f"{self.name}: mission cevap kuyruğu dolu", "WARNING")
        if msg_type == "COMMAND_ACK":
            try:
                self._command_ack_messages.put_nowait(message)
            except queue.Full:
                self.logger.system(f"{self.name}: command ACK kuyruğu dolu", "WARNING")
        if msg_type == "PARAM_VALUE":
            try:
                self._param_value_messages.put_nowait(message)
            except queue.Full:
                self.logger.system(f"{self.name}: parametre cevap kuyruğu dolu", "WARNING")
        if msg_type == "NAMED_VALUE_INT":
            self._handle_named_value_int(message)

    def _handle_named_value_int(self, message: Any) -> None:
        """İHA'nın otonom hedef renk tespitini karşılar (şartname §5.5.3.1)."""
        name = normalize_named_value_field(getattr(message, "name", ""))
        if self.state_key == "ida":
            if name == "TGT_ACK":
                try:
                    source_component = int(message.get_srcComponent())
                except (AttributeError, TypeError, ValueError):
                    source_component = -1
                if source_component != self.companion_component_id:
                    self.logger.system(
                        f"İDA hedef doğrulaması yanlış component'ten reddedildi: "
                        f"{source_component}",
                        "WARNING",
                    )
                    return
                if self.target_ack_handler is not None:
                    try:
                        self.target_ack_handler(int(getattr(message, "value", -1)))
                    except (TypeError, ValueError) as exc:
                        self.logger.system(f"İDA hedef doğrulaması geçersiz: {exc}", "WARNING")
                return
            from services.autonomy_contract import decode_ida_field

            field, value = decode_ida_field(
                getattr(message, "name", ""), getattr(message, "value", None)
            )
            if field is not None:
                self.telemetry_state.update_ida_autonomy(**{field: value})
                return
        if self.target_detection_handler is None:
            return
        if name not in TARGET_COLOR_FIELD_NAMES:
            return

        try:
            code = int(getattr(message, "value", -1))
        except (TypeError, ValueError):
            code = -1
        color = CODE_TO_COLOR.get(code)
        if color is None:
            self.logger.system(
                f"{self.name}: geçersiz hedef renk kodu yok sayıldı: {code}",
                "WARNING",
            )
            return

        # Araç aynı rengi her tick tekrar yayınlar; yalnız değişimde ileri taşı.
        if color == self._last_reported_target_color:
            return
        self._last_reported_target_color = color

        self.logger.system(f"{self.name}: otonom hedef renk tespiti alındı: {color}", "SUCCESS")
        # Handler İDA'ya gönderim tetikleyeceği için okuma döngüsü bloklanmamalı.
        threading.Thread(
            target=self._run_target_detection_handler,
            args=(color,),
            name=f"{self.name}-target-detection",
            daemon=True,
        ).start()

    def _run_target_detection_handler(self, color: str) -> None:
        try:
            self.target_detection_handler(color)
        except Exception as exc:
            self.logger.system(f"{self.name}: hedef tespiti işlenemedi: {exc}", "ERROR")

    def _reassert_emergency_after_reconnect(self) -> None:
        if not self.telemetry_state.emergency_active:
            return
        thread = threading.Thread(
            target=self._send_emergency_reassertion,
            name=f"{self.name}-emergency-reassert",
            daemon=True,
        )
        thread.start()

    def _send_emergency_reassertion(self) -> None:
        ok, message, status = self.send_emergency()
        level = "SUCCESS" if ok and status in {"acked", "state_verified"} else "WARNING"
        self.logger.system(f"{self.name}: reconnect emergency yeniden denendi [{status}] {message}", level)

    def _mark_return_home_pending_if_needed(self, reason: str) -> None:
        if self._radio_config_active.is_set():
            # Radio reconfig sırasındaki sessizlik kasıtlıdır; gerçek link kaybı sayılmaz
            # ve otomatik return-home/RTL tetiklenmemelidir.
            return
        if not self.auto_return_on_link_loss or self.telemetry_state.emergency_active:
            return
        if not self.telemetry_state.has_home_position(self.state_key):
            self.logger.system(
                f"{self.name}: link kaybı algılandı fakat home/breakpoint konumu yok; otomatik dönüş planlanmadı",
                "WARNING",
            )
            return
        if self._return_home_pending:
            return
        self._return_home_pending = True
        message = f"{reason}; reconnect sonrası {self.return_home_mode} denenecek"
        self.telemetry_state.mark_return_home_pending(self.state_key, message)
        self.logger.system(f"{self.name}: {message}", "WARNING")

    def _reassert_return_home_after_reconnect(self) -> None:
        if not self._return_home_pending:
            return
        thread = threading.Thread(
            target=self._send_return_home_after_reconnect,
            name=f"{self.name}-return-home-reconnect",
            daemon=True,
        )
        thread.start()

    def _send_return_home_after_reconnect(self) -> None:
        if self.telemetry_state.emergency_active:
            self._return_home_pending = False
            message = "Emergency aktif olduğu için otomatik return-home denenmedi"
            self.telemetry_state.mark_return_home_result(self.state_key, "blocked_emergency", message)
            self.logger.system(f"{self.name}: {message}", "WARNING")
            return
        ok, message, status = self.set_mode(self.return_home_mode)
        self._return_home_pending = False
        self.telemetry_state.mark_return_home_result(self.state_key, status, message)
        level = "SUCCESS" if ok and status in {"acked", "state_verified"} else "WARNING"
        self.logger.system(
            f"{self.name}: reconnect sonrası return-home denendi [{status}] {message}",
            level,
        )

    def run_radio_config(self, target_freq_khz: int) -> tuple[bool, str, str]:
        """RFD900x min/max frekansını yazar.

        Aynı seri hat hem MAVLink hem RFD AT komutlarını taşıdığı için, modem komut
        moduna ancak hat sessizken girilebilir. Bu yüzden önce Pixhawk reboot edilerek
        MAVLink akışı kesilir, oluşan sessizlik penceresinde '+++' ile modem komut
        moduna alınır. Sıralama sahada donanımla doğrulanmıştır.
        """
        if mavutil is None or serial is None:
            return False, "pymavlink/pyserial kurulu değil; frekans yazılamaz", "failed"
        if target_freq_khz <= 0:
            return False, "Geçersiz frekans değeri", "rejected"
        if self._radio_config_active.is_set():
            return False, f"{self.name} için frekans yazma işlemi zaten sürüyor", "busy"

        self._radio_config_active.set()
        connection = None
        port = None
        try:
            self._close_connection()
            self.logger.system(f"{self.name}: frekans yazma başladı, Pixhawk reboot ediliyor", "WARNING")

            try:
                connection = mavutil.mavlink_connection(self.port, baud=self.baudrate)
                connection.wait_heartbeat(timeout=3)
                connection.reboot_autopilot()
                self.logger.system(f"{self.name}: reboot komutu gönderildi", "INFO")
            except Exception as exc:
                # Hat zaten sessizse reboot gönderilemeyebilir; akış yine de denenir.
                self.logger.system(f"{self.name}: reboot gönderilemedi ({exc}); yine de devam ediliyor", "WARNING")
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
                    connection = None

            time.sleep(3)

            port = serial.Serial(self.port, self.baudrate, timeout=1)
            port.reset_input_buffer()
            time.sleep(1.1)
            port.write(b"+++")
            time.sleep(1.1)
            response = port.read_all().decode(errors="ignore")

            if "OK" not in response:
                self.logger.system(
                    f"{self.name}: modem komut moduna girmedi (yanıt: {response!r})",
                    "ERROR",
                )
                return False, f"{self.name}: modem komut moduna girmedi", "failed"

            self.logger.system(f"{self.name}: modem komut modunda, frekanslar yazılıyor", "SUCCESS")

            min_freq = target_freq_khz
            max_freq = target_freq_khz + 1000
            commands = [
                f"RTATS8={min_freq}",
                f"RTATS9={max_freq}",
                "RTAT&W",
                "RTATO",
                f"ATS8={min_freq}",
                f"ATS9={max_freq}",
                "AT&W",
                "ATO",
            ]
            for command in commands:
                port.write((command + "\r\n").encode())
                time.sleep(0.3)
                self.logger.system(f"{self.name}: {command}", "INFO")

            return (
                True,
                f"{self.name}: frekans {min_freq}-{max_freq} kHz olarak yazıldı; link yeniden kurulacak",
                "accepted",
            )
        except Exception as exc:
            self.logger.system(f"{self.name}: frekans yazma hatası: {exc}", "ERROR")
            return False, f"{self.name}: frekans yazma hatası: {exc}", "failed"
        finally:
            if port is not None:
                try:
                    port.close()
                except Exception:
                    pass
            self._radio_config_active.clear()

    def _close_connection(self) -> None:
        master, self.master = self.master, None
        if master is not None:
            try:
                master.close()
            except Exception:
                pass

    def _request_telemetry_stream(self) -> None:
        """Do not inject REQUEST_DATA_STREAM into a multi-port vehicle network.

        ArduPilot stream rates are configured per physical port with SRx_*
        parameters.  Re-requesting all streams on every YKİ reconnect caused
        forwarded traffic/response contention with the Jetson companion link.
        """
        return

    def _connection(self) -> tuple[Any | None, str]:
        if self.master is None or not self.telemetry_state.is_connected(self.state_key):
            return None, f"{self.name} bağlı değil"
        return self.master, ""

    def send_arm(self) -> tuple[bool, str, str]:
        return self._send_arm_state(True)

    def send_disarm(self) -> tuple[bool, str, str]:
        return self._send_arm_state(False)

    def _send_arm_state(self, arm: bool) -> tuple[bool, str, str]:
        master, error = self._connection()
        if master is None:
            return False, error, "failed"
        command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        ok, message, status = self._send_command_long(master, command, [1 if arm else 0])
        action = "ARM" if arm else "DISARM"
        if not ok:
            return ok, f"{self.name} {action}: {message}", status
        if status != "acked":
            return True, f"{self.name} {action}: {message}", status
        if self._wait_until(lambda: self.telemetry_state.is_armed(self.state_key) == arm):
            return True, f"{self.name} {action} doğrulandı", "state_verified"
        return True, f"{self.name} {action} ACK aldı fakat telemetri durumu henüz doğrulanmadı", "acked_unverified"

    def set_mode(self, mode: str) -> tuple[bool, str, str]:
        master, error = self._connection()
        if master is None:
            return False, error, "failed"
        mode = mode.upper().strip()
        try:
            if self.state_key == "ida" and mode == "GUIDED":
                mailbox = self._read_float_param(master, IDA_MISSION_CONTROL_PARAM)
                mailbox_state = _decode_mission_mailbox(mailbox)
                if mailbox_state is None:
                    return False, (
                        "İDA GUIDED reddedildi: SCR_USER6 mailbox hazır değil"
                    ), "blocked_unverified"
                if mailbox_state[0] == "command" or mailbox_state[2]:
                    return False, (
                        "İDA GUIDED reddedildi: önce STOP ACK ile otonomiyi temizleyin"
                    ), "busy"
            mapping = master.mode_mapping() or {}
            if mode not in mapping:
                return False, f"{self.name} modu desteklenmiyor: {mode}", "rejected"
            ok, message, status = self._send_command_long(
                master,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                [
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mapping[mode],
                ],
            )
            if not ok:
                return ok, f"{self.name} {mode} modu: {message}", status
            if status != "acked":
                return True, f"{self.name} {mode} modu: {message}", status
            if self._wait_until(lambda: self.telemetry_state.mode_is(self.state_key, mode)):
                return True, f"{self.name} modu {mode} olarak doğrulandı", "state_verified"
            return True, f"{self.name} {mode} modu ACK aldı fakat heartbeat modu henüz doğrulanmadı", "acked_unverified"
        except Exception as exc:
            return False, f"{self.name} mod komutu gönderilemedi: {exc}", "failed"

    def send_emergency(self) -> tuple[bool, str, str]:
        if self.emergency_action == "DISARM":
            return self.send_disarm()
        return self.set_mode(self.emergency_action)

    def _send_command_long(
        self,
        master: Any,
        command: int,
        params: list[float | int] | None = None,
    ) -> tuple[bool, str, str]:
        params = (params or [])[:7]
        params += [0] * (7 - len(params))
        self._clear_command_acks()
        try:
            with self._send_lock:
                master.mav.command_long_send(
                    self.expected_sys_id,
                    self.target_component,
                    command,
                    0,
                    *params,
                )
        except Exception as exc:
            return False, f"MAVLink komutu gönderilemedi: {exc}", "failed"
        return self._wait_for_command_ack(command)

    def _clear_command_acks(self) -> None:
        while not self._command_ack_messages.empty():
            try:
                self._command_ack_messages.get_nowait()
            except queue.Empty:
                break

    def _wait_for_command_ack(self, command: int) -> tuple[bool, str, str]:
        deadline = time.monotonic() + self.command_ack_timeout
        while time.monotonic() < deadline:
            timeout = max(0.05, min(0.25, deadline - time.monotonic()))
            try:
                message = self._command_ack_messages.get(timeout=timeout)
            except queue.Empty:
                continue
            if int(getattr(message, "command", -1)) != int(command):
                continue
            result = int(getattr(message, "result", -1))
            accepted = getattr(mavutil.mavlink, "MAV_RESULT_ACCEPTED", 0)
            in_progress = getattr(mavutil.mavlink, "MAV_RESULT_IN_PROGRESS", 5)
            if result == accepted:
                return True, "MAVLink COMMAND_ACK kabul edildi", "acked"
            if result == in_progress:
                continue
            return False, f"MAVLink COMMAND_ACK reddetti: result={result}", "rejected"
        return True, "MAVLink frame yazıldı fakat COMMAND_ACK zaman aşımına uğradı", "sent_unconfirmed"

    def _wait_until(self, predicate: Any) -> bool:
        deadline = time.monotonic() + self.state_verify_timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.1)
        return False

    def upload_mission(self, waypoints: list[dict[str, float]]) -> tuple[bool, str, str]:
        master, error = self._connection()
        if master is None:
            return False, error, "failed"
        if self.telemetry_state.emergency_active:
            return False, f"{self.name} mission upload reddedildi: emergency aktif", "rejected"
        if not self._mission_lock.acquire(blocking=False):
            return False, f"{self.name} için başka mission işlemi sürüyor", "busy"

        try:
            mailbox = self._read_float_param(master, IDA_MISSION_CONTROL_PARAM)
            mailbox_state = _decode_mission_mailbox(mailbox)
            if mailbox_state is not None and (
                mailbox_state[0] == "command" or mailbox_state[2]
            ):
                return False, (
                    "Mission upload reddedildi: önce SCR_USER6 STOP ACK gerekli"
                ), "busy"
            if mailbox is None:
                return False, "Mission upload reddedildi: SCR_USER6 okunamadı", "timeout"
            while not self._mission_messages.empty():
                try:
                    self._mission_messages.get_nowait()
                except queue.Empty:
                    break
            self._send_mission_clear(master)
            self._send_mission_count(master, len(waypoints))
            deadline = time.monotonic() + self.mission_timeout
            sent_sequences: set[int] = set()

            while time.monotonic() < deadline:
                if self.telemetry_state.emergency_active:
                    return False, f"{self.name} mission upload emergency nedeniyle durduruldu", "aborted"
                try:
                    message = self._mission_messages.get(timeout=1)
                except queue.Empty:
                    continue
                msg_type = message.get_type()
                if msg_type in {"MISSION_REQUEST", "MISSION_REQUEST_INT"}:
                    seq = int(getattr(message, "seq", -1))
                    if not 0 <= seq < len(waypoints):
                        return False, f"{self.name} geçersiz mission seq istedi: {seq}", "rejected"
                    self._send_mission_item(master, seq, waypoints[seq], msg_type == "MISSION_REQUEST_INT")
                    sent_sequences.add(seq)
                elif msg_type == "MISSION_ACK":
                    ack_type = int(getattr(message, "type", -1))
                    accepted = getattr(mavutil.mavlink, "MAV_MISSION_ACCEPTED", 0)
                    if ack_type == accepted and len(sent_sequences) == len(waypoints):
                        return True, f"{self.name} görevi yüklendi ({len(waypoints)} waypoint)", "acked"
                    if ack_type == accepted and not sent_sequences:
                        # MISSION_CLEAR_ALL ACK may arrive before mission requests.
                        continue
                    return False, f"{self.name} mission ACK hatası: {ack_type}", "rejected"
            return False, f"{self.name} mission upload zaman aşımı", "timeout"
        except Exception as exc:
            return False, f"{self.name} mission upload hatası: {exc}", "failed"
        finally:
            self._mission_lock.release()

    def _send_mission_clear(self, master: Any) -> None:
        with self._send_lock:
            try:
                master.mav.mission_clear_all_send(
                    self.expected_sys_id,
                    self.target_component,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            except TypeError:
                master.mav.mission_clear_all_send(self.expected_sys_id, self.target_component)

    def _send_mission_count(self, master: Any, count: int) -> None:
        with self._send_lock:
            try:
                master.mav.mission_count_send(
                    self.expected_sys_id,
                    self.target_component,
                    count,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            except TypeError:
                master.mav.mission_count_send(self.expected_sys_id, self.target_component, count)

    def _send_mission_item(self, master: Any, seq: int, waypoint: dict[str, float], use_int: bool) -> None:
        common = [
            self.expected_sys_id,
            self.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            if use_int
            else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            1 if seq == 0 else 0,
            1,
            0,
            0,
            0,
            0,
        ]
        coordinates = (
            [int(waypoint["lat"] * 1e7), int(waypoint["lon"] * 1e7), waypoint["alt"]]
            if use_int
            else [waypoint["lat"], waypoint["lon"], waypoint["alt"]]
        )
        with self._send_lock:
            sender = master.mav.mission_item_int_send if use_int else master.mav.mission_item_send
            try:
                sender(*common, *coordinates, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            except TypeError:
                sender(*common, *coordinates)

    def start_mission(self) -> tuple[bool, str, str]:
        """Start Jetson autonomy; Pixhawk AUTO mission execution is not used."""
        if not self._mission_lock.acquire(blocking=False):
            return False, f"{self.name} için başka mission işlemi sürüyor", "busy"
        try:
            master, error = self._connection()
            if master is None:
                return False, error, "failed"
            mailbox_state = _decode_mission_mailbox(
                self._read_float_param(master, IDA_MISSION_CONTROL_PARAM)
            )
            if mailbox_state is None or mailbox_state[0] != "ack" or mailbox_state[2]:
                return False, (
                    "Mission start reddedildi: SCR_USER6 idle STOP_ACK değil; "
                    "önce GÖREV DURDUR kullanın"
                ), "busy"
            mode_ok, mode_message, mode_status = self.set_mode("GUIDED")
            if not mode_ok or mode_status != "state_verified":
                return False, (
                    f"Mission start gönderilmedi: GUIDED doğrulanamadı. {mode_message}"
                ), "blocked_unverified"
            return self._signal_mission_control_locked(start=True)
        finally:
            self._mission_lock.release()

    def stop_mission(self) -> tuple[bool, str, str]:
        """Physically HOLD first, then require Jetson to acknowledge stop."""
        mode_ok, mode_message, mode_status = self.set_mode("HOLD")
        if not mode_ok or mode_status != "state_verified":
            return False, (
                f"Mission stop gönderilmedi: HOLD doğrulanamadı. {mode_message}"
            ), "blocked_unverified"
        return self._signal_mission_control(start=False)

    def _signal_mission_control(self, *, start: bool) -> tuple[bool, str, str]:
        if not self._mission_lock.acquire(blocking=False):
            return False, f"{self.name} için başka mission işlemi sürüyor", "busy"
        try:
            return self._signal_mission_control_locked(start=start)
        finally:
            self._mission_lock.release()

    def _signal_mission_control_locked(self, *, start: bool) -> tuple[bool, str, str]:
        """Write SCR_USER6 mailbox command and wait for Jetson ACK state."""
        master, error = self._connection()
        if master is None:
            return False, error, "failed"
        current = self._read_float_param(master, IDA_MISSION_CONTROL_PARAM)
        transition = _next_mission_mailbox(current, start=start)
        if transition is None:
            state = _decode_mission_mailbox(current)
            status = "busy" if state is not None and state[0] == "command" else "timeout"
            return False, f"SCR_USER6 mission mailbox pending/geçersiz: {current!r}", status
        token, expected_ack = transition
        observed = self._set_mission_mailbox_command(
            master, token, expected_ack
        )
        if observed is None:
            return False, "SCR_USER6 mission command geri okunamadı", "timeout"
        if observed == expected_ack:
            action = "START" if start else "STOP"
            return True, (
                f"{self.name} GUIDED mission {action}: hızlı Jetson ACK={expected_ack}"
            ), "state_verified"

        deadline = time.monotonic() + max(5.0, self.state_verify_timeout)
        while time.monotonic() < deadline:
            ack = self._read_float_param(master, IDA_MISSION_CONTROL_PARAM, timeout=0.75)
            if self._exact_mission_control_token(ack) == expected_ack:
                action = "START" if start else "STOP"
                return True, (
                    f"{self.name} GUIDED mission {action}: "
                    f"SCR_USER6 command={token}, Jetson ACK={expected_ack}"
                ), "state_verified"
            time.sleep(0.1)

        # Start ACK kaybolursa araç GUIDED'da komut alabilir; önce HOLD ile
        # fiziksel hareketi kes. Pending token ASLA overwrite edilmez: bridge
        # henüz onu görmediyse sequence atlamak kalıcı bir control/ACK kilidi
        # üretir. Bridge geldiğinde aynı tokenı ACK'ler; operatör daha sonra
        # normal, bir-sonraki signed STOP tokenını gönderebilir.
        if start:
            self.set_mode("HOLD")
        return False, "Jetson mission control ACK zaman aşımı; araç HOLD", "timeout"

    @staticmethod
    def _exact_mission_control_token(value: Any) -> int | None:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number):
            return None
        token = int(round(number))
        maximum = MISSION_MAILBOX_BASE + 4 * MISSION_CONTROL_MAX_SEQUENCE
        if abs(number - token) > 1e-6 or not MISSION_MAILBOX_BASE <= token <= maximum:
            return None
        return token

    def send_target_info(self, target: dict[str, Any]) -> tuple[bool, str, str]:
        master, error = self._connection()
        if master is None:
            return False, error, "failed"
        color = str(target.get("target_color", "")).strip().upper()
        if color not in COLOR_CODES:
            return False, f"Hedef rengi İDA'ya yazılamadı: {color or 'BOŞ'}", "rejected"
        try:
            if self._set_float_param_with_readback(
                master, IDA_TARGET_COLOR_PARAM, float(COLOR_CODES[color])
            ):
                return True, (
                    f"Hedef {self.name} Pixhawk {IDA_TARGET_COLOR_PARAM.decode()} parametresine yazıldı; "
                    "Jetson okuma doğrulaması bekleniyor"
                ), "acked"
            return False, (
                f"Hedef {self.name} {IDA_TARGET_COLOR_PARAM.decode()} yazımı geri okunamadı; "
                "görev başlatılmamalı"
            ), "timeout"
        except Exception as exc:
            return False, f"Hedef bilgisi gönderilemedi: {exc}", "failed"

    def write_mission_parkur_metadata(
        self, p1_count: int, p2_count: int
    ) -> tuple[bool, str, str]:
        """Persist safe mission-boundary metadata without executable commands."""
        master, error = self._connection()
        if master is None:
            return False, error, "failed"
        if (
            isinstance(p1_count, bool)
            or isinstance(p2_count, bool)
            or not isinstance(p1_count, int)
            or not isinstance(p2_count, int)
            or not 1 <= p1_count <= 1000
            or not 0 <= p2_count <= 1000
        ):
            return False, "Parkur waypoint adetleri geçersiz", "rejected"
        packed_counts = p1_count * MISSION_COUNT_RADIX + p2_count
        try:
            if not self._set_float_param_with_readback(
                master, IDA_MISSION_COUNTS_PARAM, float(packed_counts)
            ):
                return False, "SCR_USER5 parkur metadata geri okunamadı", "timeout"
            mailbox = self._read_float_param(master, IDA_MISSION_CONTROL_PARAM)
            if _decode_mission_mailbox(mailbox) is None and not self._set_float_param_with_readback(
                master, IDA_MISSION_CONTROL_PARAM, float(MISSION_MAILBOX_BASE)
            ):
                return False, "SCR_USER6 mission mailbox başlatılamadı", "timeout"
        except Exception as exc:
            return False, f"Parkur metadata yazılamadı: {exc}", "failed"
        return True, "SCR_USER5 P1/P2 metadata ve SCR_USER6 mailbox doğrulandı", "acked"

    def _set_float_param_with_readback(
        self, master: Any, param_id: bytes, value: float
    ) -> bool:
        # Mission metadata upload and target delivery may originate from
        # different WebSocket threads. Queue clearing + send + readback is one
        # indivisible transaction or each command can consume the other's ACK.
        with self._param_transaction_lock:
            self._clear_param_values()
            with self._send_lock:
                master.mav.param_set_send(
                    self.expected_sys_id,
                    self.target_component,
                    param_id,
                    float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                )
            return self._wait_for_param_value(param_id, float(value))

    def _set_mission_mailbox_command(
        self, master: Any, command: int, expected_ack: int
    ) -> int | None:
        """Write SCR_USER6 and accept either command echo or a raced fast ACK."""
        with self._param_transaction_lock:
            self._clear_param_values()
            with self._send_lock:
                master.mav.param_set_send(
                    self.expected_sys_id,
                    self.target_component,
                    IDA_MISSION_CONTROL_PARAM,
                    float(command),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                )
            deadline = time.monotonic() + self.command_ack_timeout
            while time.monotonic() < deadline:
                remaining = max(0.01, min(0.2, deadline - time.monotonic()))
                value = self._wait_for_param_response(
                    IDA_MISSION_CONTROL_PARAM, remaining
                )
                token = self._exact_mission_control_token(value)
                if token in {command, expected_ack}:
                    return token
            return None

    def _read_float_param(
        self, master: Any, param_id: bytes, timeout: float | None = None
    ) -> float | None:
        """Read one exact parameter under the shared transaction lock."""
        with self._param_transaction_lock:
            self._clear_param_values()
            with self._send_lock:
                master.mav.param_request_read_send(
                    self.expected_sys_id,
                    self.target_component,
                    param_id,
                    -1,
                )
            return self._wait_for_param_response(
                param_id,
                self.command_ack_timeout if timeout is None else float(timeout),
            )

    def _wait_for_param_response(self, param_id: bytes, timeout: float) -> float | None:
        deadline = time.monotonic() + max(0.05, timeout)
        while time.monotonic() < deadline:
            wait = max(0.01, min(0.2, deadline - time.monotonic()))
            try:
                message = self._param_value_messages.get(timeout=wait)
            except queue.Empty:
                continue
            raw_id = getattr(message, "param_id", b"")
            normalized = (
                raw_id.replace(b"\x00", b"").strip()
                if isinstance(raw_id, bytes)
                else str(raw_id or "").replace("\x00", "").strip().encode()
            )
            if normalized != param_id:
                continue
            try:
                value = float(getattr(message, "param_value"))
            except (TypeError, ValueError):
                continue
            return value if math.isfinite(value) else None
        return None

    def _clear_param_values(self) -> None:
        while not self._param_value_messages.empty():
            try:
                self._param_value_messages.get_nowait()
            except queue.Empty:
                break

    def _wait_for_param_value(self, param_id: bytes, expected: float) -> bool:
        deadline = time.monotonic() + self.command_ack_timeout
        while time.monotonic() < deadline:
            timeout = max(0.05, min(0.25, deadline - time.monotonic()))
            try:
                message = self._param_value_messages.get(timeout=timeout)
            except queue.Empty:
                continue
            raw_id = getattr(message, "param_id", b"")
            if isinstance(raw_id, bytes):
                normalized = raw_id.replace(b"\x00", b"").strip()
            else:
                normalized = str(raw_id or "").replace("\x00", "").strip().encode()
            if normalized != param_id:
                continue
            try:
                value = float(getattr(message, "param_value"))
            except (TypeError, ValueError):
                continue
            if math.isfinite(value) and abs(value - expected) <= 0.01:
                return True
        return False
