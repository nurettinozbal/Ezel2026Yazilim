import csv
import asyncio
import math
import tempfile
import unittest
from unittest import mock
from pathlib import Path
from types import SimpleNamespace

from services.command_gate import CommandGate
from services.logger import BackendLogger
from services.lab_debug import (
    CONTRACT, InMemoryDebugAdapter, VehicleTestRelay, handle_producer_message,
    is_lab_authorized, normalize_snapshot, producer_token_valid,
    raw_lidar_left_to_canonical, validate_lab_config,
)
from services.mission_manager import MissionManager
from services.target_manager import TargetManager
from services.telemetry_state import TelemetryState


class FakeVehicle:
    def __init__(self):
        self.calls = []

    def _ok(self, name, *args):
        self.calls.append((name, args))
        return True, name

    def send_arm(self):
        return self._ok("arm")

    def send_disarm(self):
        return self._ok("disarm")

    def set_mode(self, mode):
        return self._ok("mode", mode)

    def send_emergency(self):
        return self._ok("emergency")

    def upload_mission(self, waypoints):
        return self._ok("upload", waypoints)

    def write_mission_parkur_metadata(self, p1_count, p2_count):
        return self._ok("mission_metadata", p1_count, p2_count)

    def start_mission(self):
        return self._ok("start")

    def stop_mission(self):
        self.calls.append(("stop", ()))
        return True, "stop", "state_verified"

    def send_target_info(self, target):
        return self._ok("target", target)

    def run_radio_config(self, freq_khz):
        return self._ok("radio_config", freq_khz)


class ConnectedState:
    def __init__(self):
        self.emergency_active = False
        self.connected = {"ida": True, "iha": True}
        self.armed = {"ida": False, "iha": False}

    def is_connected(self, vehicle):
        return self.connected[vehicle]

    def is_armed(self, vehicle):
        return self.armed[vehicle]

    def set_emergency(self):
        self.emergency_active = True

    def reset_emergency(self):
        self.emergency_active = False


class FakeMessage:
    def __init__(self, message_type, **fields):
        self._message_type = message_type
        self._src_component = fields.pop("src_component", 191)
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self):
        return self._message_type

    def get_srcComponent(self):
        return self._src_component


class BackendServiceTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.logger = BackendLogger(Path(self.temp_dir.name))
        self.ida = FakeVehicle()
        self.iha = FakeVehicle()
        self.state = ConnectedState()
        self.targets = TargetManager()
        self.missions = MissionManager(self.ida, self.logger)
        self.gate = CommandGate(
            self.ida,
            self.iha,
            self.state,
            self.missions,
            self.targets,
            self.logger,
        )

    def tearDown(self):
        # Windows keeps FileHandler targets locked until handlers are closed.
        for handler in list(self.logger._system_logger.handlers):
            handler.close()
            self.logger._system_logger.removeHandler(handler)
        self.temp_dir.cleanup()

    def test_emergency_latches_and_blocks_arm(self):
        result = self.gate.execute("EMERGENCY_STOP_ALL")
        self.assertTrue(result["ok"])
        self.assertIn("status", result)
        self.assertTrue(self.state.emergency_active)
        self.assertFalse(self.gate.execute("ARM_IDA")["ok"])

    def test_invalid_mode_is_rejected(self):
        result = self.gate.execute("SET_IHA_MODE", {"mode": "MANUAL"})
        self.assertFalse(result["ok"])

    def test_ida_auto_mode_is_not_a_yki_control_option(self):
        result = self.gate.execute("SET_IDA_MODE", {"mode": "AUTO"})
        self.assertFalse(result["ok"])

    def test_emergency_rejects_unsafe_mode_change(self):
        self.gate.execute("EMERGENCY_STOP_ALL")
        result = self.gate.execute("SET_IDA_MODE", {"mode": "AUTO"})
        self.assertFalse(result["ok"])

    def test_target_lock_requires_force_for_change(self):
        first = self.gate.execute(
            "LOCK_TARGET",
            {"color": "KIRMIZI", "source": "MANUAL", "confidence": 1.0},
        )
        second = self.gate.execute(
            "LOCK_TARGET",
            {"color": "YEŞİL", "source": "MANUAL", "confidence": 1.0},
        )
        forced = self.gate.execute(
            "FORCE_UPDATE_TARGET",
            {"color": "YEŞİL", "source": "MANUAL", "confidence": 1.0},
        )
        self.assertTrue(first["ok"])
        self.assertFalse(second["ok"])
        self.assertTrue(forced["ok"])

    def test_locked_target_can_be_sent_to_ida(self):
        self.ida.send_target_info = lambda _target: (True, "target stored", "acked")
        locked = self.gate.execute(
            "LOCK_TARGET",
            {"color": "KIRMIZI", "source": "MANUAL", "confidence": 1.0},
        )
        sent = self.gate.execute("SEND_TARGET_TO_IDA")

        self.assertTrue(locked["ok"])
        self.assertTrue(sent["ok"])
        self.assertEqual(sent["status"], "acked")
        self.assertEqual(self.targets.snapshot()["delivery_status"], "acked")

    def test_empty_mission_is_rejected(self):
        self.assertFalse(self.gate.execute("UPLOAD_IDA_MISSION", {"waypoints": []})["ok"])

    def test_mission_upload_is_rejected_during_emergency(self):
        self.state.set_emergency()
        result = self.gate.execute(
            "UPLOAD_IDA_MISSION",
            {"waypoints": [{"lat": 41.0, "lon": 29.0, "alt": 0, "parkur": 1}]},
        )

        self.assertFalse(result["ok"])
        self.assertEqual(result["status"], "rejected")

    def test_mission_start_requires_successful_upload(self):
        rejected = self.gate.execute("START_IDA_MISSION")
        uploaded = self.gate.execute(
            "UPLOAD_IDA_MISSION",
            {"waypoints": [{"lat": 41.0, "lon": 29.0, "alt": 0, "parkur": 1}]},
        )
        self.state.armed["ida"] = True
        started = self.gate.execute("START_IDA_MISSION")

        self.assertFalse(rejected["ok"])
        self.assertEqual(rejected["message"], "Görev başlatılamaz: İDA’ya yüklenmiş görev yok.")
        self.assertTrue(uploaded["ok"])
        self.assertTrue(started["ok"])
        self.assertIn(("mission_metadata", (1, 0)), self.ida.calls)
        mission = self.missions.snapshot()
        self.assertTrue(mission["has_uploaded_mission"])
        self.assertEqual(mission["waypoint_count"], 1)
        self.assertGreater(mission["last_upload_time"], 0)

    def test_mission_requires_ordered_explicit_parkur_metadata(self):
        missing = self.gate.execute(
            "UPLOAD_IDA_MISSION",
            {"waypoints": [{"lat": 41.0, "lon": 29.0, "alt": 0}]},
        )
        reversed_route = self.gate.execute(
            "UPLOAD_IDA_MISSION",
            {"waypoints": [
                {"lat": 41.0, "lon": 29.0, "alt": 0, "parkur": 2},
                {"lat": 41.1, "lon": 29.1, "alt": 0, "parkur": 1},
            ]},
        )
        self.assertFalse(missing["ok"])
        self.assertFalse(reversed_route["ok"])
        self.assertFalse(self.missions.snapshot()["has_uploaded_mission"])

    def test_mission_metadata_failure_blocks_start(self):
        self.ida.write_mission_parkur_metadata = (
            lambda _p1, _p2: (False, "metadata timeout", "timeout")
        )
        result = self.gate.execute(
            "UPLOAD_IDA_MISSION",
            {"waypoints": [{"lat": 41.0, "lon": 29.0, "alt": 0, "parkur": 1}]},
        )
        self.assertFalse(result["ok"])
        self.assertEqual(result["status"], "timeout")
        self.assertFalse(self.gate.execute("START_IDA_MISSION")["ok"])

    def test_mission_start_requires_armed_ida(self):
        self.gate.execute(
            "UPLOAD_IDA_MISSION",
            {"waypoints": [{"lat": 41.0, "lon": 29.0, "alt": 0, "parkur": 1}]},
        )

        result = self.gate.execute("START_IDA_MISSION")

        self.assertFalse(result["ok"])
        self.assertEqual(result["message"], "Görev başlatılamaz: İDA armed değil.")

    def test_failed_upload_invalidates_previous_mission(self):
        self.gate.execute(
            "UPLOAD_IDA_MISSION",
            {"waypoints": [{"lat": 41.0, "lon": 29.0, "alt": 0, "parkur": 1}]},
        )
        self.ida.upload_mission = lambda _waypoints: (False, "upload failed")
        failed = self.gate.execute(
            "UPLOAD_IDA_MISSION",
            {"waypoints": [{"lat": 41.1, "lon": 29.1, "alt": 0, "parkur": 1}]},
        )

        self.assertFalse(failed["ok"])
        self.assertFalse(self.missions.snapshot()["has_uploaded_mission"])
        self.assertFalse(self.gate.execute("START_IDA_MISSION")["ok"])

    def test_emergency_reset_requires_connected_disarmed_vehicles(self):
        self.state.set_emergency()
        self.state.armed["iha"] = True
        armed_result = self.gate.execute("RESET_EMERGENCY")
        self.assertFalse(armed_result["ok"])
        self.assertTrue(self.state.emergency_active)

        self.state.armed["iha"] = False
        reset_result = self.gate.execute("RESET_EMERGENCY")
        self.assertTrue(reset_result["ok"])
        self.assertFalse(self.state.emergency_active)

    def test_emergency_reset_rejects_unknown_disconnected_state(self):
        self.state.set_emergency()
        self.state.connected["ida"] = False
        result = self.gate.execute("RESET_EMERGENCY")
        self.assertFalse(result["ok"])
        self.assertTrue(self.state.emergency_active)

    def test_emergency_reset_requires_verified_jetson_stop(self):
        self.state.set_emergency()
        self.targets.mark_mission_started()
        self.ida.stop_mission = lambda: (False, "Jetson ACK timeout", "timeout")

        result = self.gate.execute("RESET_EMERGENCY")

        self.assertFalse(result["ok"])
        self.assertEqual(result["status"], "blocked_unverified")
        self.assertTrue(self.state.emergency_active)
        self.assertTrue(self.targets.snapshot()["mission_started"])

    def test_disconnected_snapshot_matches_contract(self):
        state = TelemetryState(1, 2, heartbeat_timeout=5)
        snapshot = state.snapshot(self.targets.snapshot())
        self.assertEqual(snapshot["system"]["ida_link"], "DISCONNECTED")
        self.assertEqual(snapshot["ida"]["sys_id"], 1)
        self.assertEqual(snapshot["iha"]["sys_id"], 2)

    def test_iha_connection_requires_heartbeat(self):
        state = TelemetryState(1, 2, heartbeat_timeout=5)
        state.update_message("iha", FakeMessage("GLOBAL_POSITION_INT", lat=410000000, lon=290000000, relative_alt=1000))

        self.assertFalse(state.is_connected("iha"))

        state.update_message("iha", FakeMessage("HEARTBEAT", base_mode=0, system_status=3))

        self.assertTrue(state.is_connected("iha"))

    def test_ida_extended_telemetry_fields_are_normalized(self):
        state = TelemetryState(1, 2, heartbeat_timeout=5)
        state.update_message("ida", FakeMessage("NAV_CONTROLLER_OUTPUT", wp_dist=42, nav_bearing=178))
        state.update_message("ida", FakeMessage("SERVO_OUTPUT_RAW", servo9_raw=1600, servo11_raw=1450))

        snapshot = state.snapshot(self.targets.snapshot())

        self.assertEqual(snapshot["ida"]["dist_to_wp"], 42)
        self.assertEqual(snapshot["ida"]["target_heading"], 178)
        self.assertEqual(snapshot["ida"]["motor_left_pwm"], 1600)
        self.assertEqual(snapshot["ida"]["motor_right_pwm"], 1450)
        self.assertEqual(snapshot["ida"]["motor_left_pct"], 20.0)
        self.assertEqual(snapshot["ida"]["motor_right_pct"], -10.0)

    def test_home_position_locks_first_valid_gps(self):
        state = TelemetryState(1, 2, heartbeat_timeout=5)
        state.update_message("ida", FakeMessage("GLOBAL_POSITION_INT", lat=0, lon=0))
        self.assertFalse(state.has_home_position("ida"))

        state.update_message("ida", FakeMessage("GLOBAL_POSITION_INT", lat=410000000, lon=290000000))
        state.update_message("ida", FakeMessage("GLOBAL_POSITION_INT", lat=420000000, lon=300000000))

        snapshot = state.snapshot(self.targets.snapshot())
        self.assertTrue(snapshot["ida"]["home_locked"])
        self.assertEqual(snapshot["ida"]["home_lat"], 41.0)
        self.assertEqual(snapshot["ida"]["home_lon"], 29.0)

    def test_return_home_status_fields_are_recorded(self):
        state = TelemetryState(1, 2, heartbeat_timeout=5)

        state.mark_return_home_pending("ida", "link lost")
        pending = state.snapshot(self.targets.snapshot())
        self.assertTrue(pending["ida"]["return_home_pending"])
        self.assertEqual(pending["ida"]["return_home_status"], "pending")

        state.mark_return_home_result("ida", "state_verified", "RTL doğrulandı")
        result = state.snapshot(self.targets.snapshot())
        self.assertFalse(result["ida"]["return_home_pending"])
        self.assertEqual(result["ida"]["return_home_status"], "state_verified")
        self.assertEqual(result["ida"]["return_home_message"], "RTL doğrulandı")

    def test_mission_started_blocks_ida_commands_except_emergency(self):
        self.targets.mark_mission_started()

        self.assertFalse(self.gate.execute("SET_IDA_MODE", {"mode": "HOLD"})["ok"])
        self.assertFalse(self.gate.execute("ARM_IDA")["ok"])
        self.assertFalse(
            self.gate.execute(
                "UPLOAD_IDA_MISSION",
                {"waypoints": [{"lat": 41.0, "lon": 29.0, "alt": 0, "parkur": 1}]},
            )["ok"]
        )
        self.assertFalse(
            self.gate.execute(
                "LOCK_TARGET",
                {"color": "KIRMIZI", "source": "MANUAL", "confidence": 1.0},
            )["ok"]
        )

        # STOP/DISARM are safety exits and must never be hidden behind the
        # active-mission command lock. STOP also clears the lock after ACK.
        self.assertTrue(self.gate.execute("DISARM_IDA")["ok"])
        self.assertTrue(self.gate.execute("STOP_IDA_MISSION")["ok"])
        self.assertFalse(self.targets.snapshot()["mission_started"])

        result = self.gate.execute("EMERGENCY_STOP_ALL")
        self.assertTrue(result["ok"])

    def test_verified_stop_clears_mission_started_lock(self):
        self.targets.mark_mission_started()
        self.ida.stop_mission = lambda: (True, "Jetson stop ack", "state_verified")

        result = self.gate.execute("STOP_IDA_MISSION")

        self.assertTrue(result["ok"])
        self.assertFalse(self.targets.snapshot()["mission_started"])

    def test_mission_started_does_not_block_iha_commands(self):
        self.targets.mark_mission_started()

        result = self.gate.execute("SET_IHA_MODE", {"mode": "LOITER"})
        self.assertTrue(result["ok"])

    def test_reset_emergency_clears_mission_started_lock(self):
        self.targets.mark_mission_started()
        self.gate.execute("EMERGENCY_STOP_ALL")
        # Emergency aktifken DISARM_IDA, emergency-özel kısıtlara takılmaz.
        disarm_result = self.gate.execute("DISARM_IDA")
        self.assertTrue(disarm_result["ok"])

        reset_result = self.gate.execute("RESET_EMERGENCY")

        self.assertTrue(reset_result["ok"])
        self.assertFalse(self.targets.snapshot()["mission_started"])
        # Kilit kalktı: normal SET_IDA_MODE tekrar çalışmalı.
        mode_result = self.gate.execute("SET_IDA_MODE", {"mode": "HOLD"})
        self.assertTrue(mode_result["ok"])

    def test_upload_is_rejected_while_mission_started_even_after_emergency(self):
        # Emergency aktifken UPLOAD_IDA_MISSION zaten ayrı bir kuralla reddediliyor;
        # mission-lock ile çakışmadığını doğrula.
        self.targets.mark_mission_started()
        self.gate.execute("EMERGENCY_STOP_ALL")

        result = self.gate.execute(
            "UPLOAD_IDA_MISSION",
            {"waypoints": [{"lat": 41.0, "lon": 29.0, "alt": 0, "parkur": 1}]},
        )

        self.assertFalse(result["ok"])
        self.assertTrue(self.targets.snapshot()["mission_started"])

    def test_target_speed_is_derived_from_position_target(self):
        state = TelemetryState(1, 2, heartbeat_timeout=5)
        state.update_message(
            "ida",
            FakeMessage("POSITION_TARGET_GLOBAL_INT", vx=3.0, vy=4.0, vz=0.0, type_mask=0),
        )

        snapshot = state.snapshot(self.targets.snapshot())

        self.assertEqual(snapshot["ida"]["target_speed"], 5.0)

    def test_target_speed_ignores_masked_velocity_setpoints(self):
        state = TelemetryState(1, 2, heartbeat_timeout=5)
        state.update_message(
            "ida",
            FakeMessage("POSITION_TARGET_GLOBAL_INT", vx=3.0, vy=4.0, vz=0.0, type_mask=0b111000),
        )

        snapshot = state.snapshot(self.targets.snapshot())

        self.assertIsNone(snapshot["ida"]["target_speed"])

    def test_telemetry_csv_includes_required_ida_fields(self):
        state = TelemetryState(1, 2, heartbeat_timeout=5)
        state.update_message("ida", FakeMessage("ATTITUDE", roll=0.0, pitch=math.pi / 2, yaw=0.3))
        state.update_message("ida", FakeMessage("NAV_CONTROLLER_OUTPUT", wp_dist=10, nav_bearing=90))
        state.update_message(
            "ida",
            FakeMessage("POSITION_TARGET_GLOBAL_INT", vx=3.0, vy=4.0, vz=0.0, type_mask=0),
        )
        snapshot = state.snapshot(self.targets.snapshot())

        self.logger.telemetry(snapshot)
        self.logger.flush()

        with self.logger.telemetry_path.open(encoding="utf-8") as file:
            rows = list(csv.reader(file))

        header = rows[0]
        for column in (
            "ida_heading_deg",
            "ida_roll_deg",
            "ida_pitch_deg",
            "ida_target_speed_mps",
            "ida_target_heading_deg",
        ):
            self.assertIn(column, header)

        data_row = dict(zip(header, rows[1]))
        self.assertEqual(float(data_row["ida_roll_deg"]), 0.0)
        # ATTITUDE radyan yayınlar; CSV dereceye çevirmelidir.
        # telemetry_state radyanı 3 haneye yuvarladığı için ~0.06° sapma normaldir.
        self.assertAlmostEqual(float(data_row["ida_pitch_deg"]), 90.0, delta=0.1)
        self.assertEqual(int(data_row["ida_target_heading_deg"]), 90)
        self.assertEqual(float(data_row["ida_target_speed_mps"]), 5.0)

    def test_radio_frequency_requires_disarmed_connected_vehicle(self):
        self.state.armed["ida"] = True
        armed_result = self.gate.execute("SET_RADIO_FREQUENCY", {"vehicle": "ida", "freq_khz": 915000})
        self.assertFalse(armed_result["ok"])

        self.state.armed["ida"] = False
        ok_result = self.gate.execute("SET_RADIO_FREQUENCY", {"vehicle": "ida", "freq_khz": 915000})
        self.assertTrue(ok_result["ok"])
        self.assertIn(("radio_config", (915000,)), self.ida.calls)

        self.state.connected["iha"] = False
        disconnected = self.gate.execute("SET_RADIO_FREQUENCY", {"vehicle": "iha", "freq_khz": 915000})
        self.assertFalse(disconnected["ok"])

    def test_radio_frequency_rejects_invalid_payload_and_emergency(self):
        self.assertFalse(self.gate.execute("SET_RADIO_FREQUENCY", {"vehicle": "yok", "freq_khz": 915000})["ok"])
        self.assertFalse(self.gate.execute("SET_RADIO_FREQUENCY", {"vehicle": "ida", "freq_khz": 0})["ok"])
        self.assertFalse(self.gate.execute("SET_RADIO_FREQUENCY", {"vehicle": "ida", "freq_khz": "abc"})["ok"])

        self.gate.execute("EMERGENCY_STOP_ALL")
        self.assertFalse(self.gate.execute("SET_RADIO_FREQUENCY", {"vehicle": "ida", "freq_khz": 915000})["ok"])

    def test_radio_frequency_rejected_while_ida_mission_running(self):
        self.targets.mark_mission_started()

        ida_result = self.gate.execute("SET_RADIO_FREQUENCY", {"vehicle": "ida", "freq_khz": 915000})
        self.assertFalse(ida_result["ok"])

        # İHA görev sırasında manuel kontrol edilebildiği için kilitlenmez.
        iha_result = self.gate.execute("SET_RADIO_FREQUENCY", {"vehicle": "iha", "freq_khz": 915000})
        self.assertTrue(iha_result["ok"])

    def test_gps_and_radio_are_tracked_per_vehicle(self):
        state = TelemetryState(1, 2, heartbeat_timeout=5)

        state.update_message("ida", FakeMessage("GPS_RAW_INT", satellites_visible=14, eph=80))
        state.update_message("ida", FakeMessage("RADIO_STATUS", rssi=200))
        # İHA verisi geldikten sonra İDA'nın değerleri korunmalı.
        state.update_message("iha", FakeMessage("GPS_RAW_INT", satellites_visible=6, eph=250))
        state.update_message("iha", FakeMessage("RADIO_STATUS", rssi=100))

        snapshot = state.snapshot(self.targets.snapshot())

        self.assertEqual(snapshot["ida"]["gps_sats"], 14)
        self.assertEqual(snapshot["ida"]["hdop"], 0.8)
        self.assertEqual(snapshot["iha"]["gps_sats"], 6)
        self.assertEqual(snapshot["iha"]["hdop"], 2.5)
        self.assertNotEqual(snapshot["ida"]["rssi"], snapshot["iha"]["rssi"])
        # system.* İDA'yı yansıtır; İHA onu ezmemeli.
        self.assertEqual(snapshot["system"]["gps_sats"], 14)
        self.assertEqual(snapshot["system"]["hdop"], 0.8)
        self.assertEqual(snapshot["system"]["rssi"], snapshot["ida"]["rssi"])

    def test_named_value_int_decodes_target_color_for_both_name_lengths(self):
        received = []
        vehicle = self._make_iha_link(received.append)

        # MAVLink char[10] kırpması: hem tam ad hem kırpılmış ad kabul edilmeli.
        vehicle._handle_named_value_int(FakeMessage("NAMED_VALUE_INT", name=b"TARGET_COLOR", value=1))
        self._join_detection_threads()
        vehicle._last_reported_target_color = None
        vehicle._handle_named_value_int(FakeMessage("NAMED_VALUE_INT", name="TARGET_COL", value=4))
        self._join_detection_threads()

        self.assertEqual(received, ["KIRMIZI", "SİYAH"])

    def test_named_value_int_ignores_invalid_code_and_repeats(self):
        received = []
        vehicle = self._make_iha_link(received.append)

        vehicle._handle_named_value_int(FakeMessage("NAMED_VALUE_INT", name=b"TARGET_COL", value=99))
        vehicle._handle_named_value_int(FakeMessage("NAMED_VALUE_INT", name=b"OTHER_FIELD", value=1))
        vehicle._handle_named_value_int(FakeMessage("NAMED_VALUE_INT", name=b"TARGET_COL", value=2))
        vehicle._handle_named_value_int(FakeMessage("NAMED_VALUE_INT", name=b"TARGET_COL", value=2))
        self._join_detection_threads()

        # Geçersiz kod ve ilgisiz alan yutulur; aynı renk tekrarı yeniden iletilmez.
        self.assertEqual(received, ["YEŞİL"])

    def test_iha_detection_locks_target_with_iha_source(self):
        ok, _ = self.targets.lock_target({"color": "KIRMIZI", "source": "IHA", "confidence": 1.0})
        snapshot = self.targets.snapshot()

        self.assertTrue(ok)
        self.assertTrue(snapshot["is_locked"])
        self.assertEqual(snapshot["source"], "IHA")
        self.assertEqual(snapshot["target_color"], "KIRMIZI")

        # İlk tespit kazanır; ikinci tespit force olmadan hedefi değiştiremez.
        second_ok, _ = self.targets.lock_target({"color": "YEŞİL", "source": "IHA", "confidence": 1.0})
        self.assertFalse(second_ok)
        self.assertEqual(self.targets.snapshot()["target_color"], "KIRMIZI")

    def _make_iha_link(self, handler):
        from links.mavlink_vehicle import MAVLinkVehicle

        return MAVLinkVehicle(
            port="/dev/null",
            baudrate=57600,
            expected_sys_id=2,
            gcs_sys_id=255,
            name="İHA",
            state_key="iha",
            telemetry_state=TelemetryState(1, 2, heartbeat_timeout=5),
            logger=self.logger,
            emergency_action="RTL",
            reconnect_delay=1.0,
            mission_timeout=5.0,
            telemetry_rate_hz=2,
            command_ack_timeout=1.0,
            state_verify_timeout=1.0,
            auto_return_on_link_loss=False,
            return_home_mode="RTL",
            target_detection_handler=handler,
        )

    def _make_ida_link(self):
        from links.mavlink_vehicle import MAVLinkVehicle

        state = TelemetryState(1, 2, heartbeat_timeout=5)
        state.update_message("ida", FakeMessage("HEARTBEAT", base_mode=0, system_status=3))
        vehicle = MAVLinkVehicle(
            port="/dev/null", baudrate=57600, expected_sys_id=1, gcs_sys_id=255,
            name="İDA", state_key="ida", telemetry_state=state, logger=self.logger,
            emergency_action="HOLD", reconnect_delay=1.0, mission_timeout=5.0,
            telemetry_rate_hz=2, command_ack_timeout=0.05, state_verify_timeout=1.0,
            auto_return_on_link_loss=False, return_home_mode="RTL",
            companion_component_id=191,
        )
        return vehicle

    def test_send_target_uses_scr_user4_and_requires_param_readback(self):
        import links.mavlink_vehicle as module

        vehicle = self._make_ida_link()
        sent = []

        class FakeMav:
            def param_set_send(self, *args):
                sent.append(args)
                # Aynı portta bekleyen eski/uyuşmayan param cevabı doğru
                # SCR_USER4 readback'ini gölgelememeli.
                vehicle._param_value_messages.put_nowait(
                    FakeMessage("PARAM_VALUE", param_id=b"SCR_USER4\x00", param_value=4.0)
                )
                vehicle._param_value_messages.put_nowait(
                    FakeMessage("PARAM_VALUE", param_id=b"SCR_USER4\x00", param_value=1.0)
                )

        vehicle.master = SimpleNamespace(mav=FakeMav())
        old_mavutil = module.mavutil
        module.mavutil = SimpleNamespace(
            mavlink=SimpleNamespace(MAV_PARAM_TYPE_REAL32=9)
        )
        try:
            ok, _message, status = vehicle.send_target_info({"target_color": "KIRMIZI"})
        finally:
            module.mavutil = old_mavutil

        self.assertTrue(ok)
        self.assertEqual(status, "acked")
        self.assertEqual(sent[0][2:5], (b"SCR_USER4", 1.0, 9))

    def test_send_target_without_matching_readback_fails_closed(self):
        import links.mavlink_vehicle as module

        vehicle = self._make_ida_link()
        vehicle.master = SimpleNamespace(mav=SimpleNamespace(param_set_send=lambda *args: None))
        old_mavutil = module.mavutil
        module.mavutil = SimpleNamespace(
            mavlink=SimpleNamespace(MAV_PARAM_TYPE_REAL32=9)
        )
        try:
            ok, _message, status = vehicle.send_target_info({"target_color": "KIRMIZI"})
        finally:
            module.mavutil = old_mavutil
        self.assertFalse(ok)
        self.assertEqual(status, "timeout")

    def test_yki_reconnect_does_not_request_all_telemetry_streams(self):
        """YKİ must not amplify traffic on the Pixhawk multi-port network."""
        vehicle = self._make_ida_link()
        request = mock.MagicMock()
        vehicle.master = SimpleNamespace(
            mav=SimpleNamespace(request_data_stream_send=request)
        )
        vehicle._request_telemetry_stream()
        request.assert_not_called()

    def test_mission_parkur_counts_are_packed_in_scr_user5(self):
        import links.mavlink_vehicle as module

        vehicle = self._make_ida_link()
        sent = []

        class FakeMav:
            def param_set_send(self, *args):
                sent.append(args)
                vehicle._param_value_messages.put_nowait(FakeMessage(
                    "PARAM_VALUE", param_id=args[2], param_value=args[3]
                ))

        vehicle.master = SimpleNamespace(mav=FakeMav())
        vehicle._read_float_param = mock.MagicMock(return_value=8_000_000.0)
        old_mavutil = module.mavutil
        module.mavutil = SimpleNamespace(
            mavlink=SimpleNamespace(MAV_PARAM_TYPE_REAL32=9)
        )
        try:
            ok, _message, status = vehicle.write_mission_parkur_metadata(4, 1)
        finally:
            module.mavutil = old_mavutil

        self.assertTrue(ok)
        self.assertEqual(status, "acked")
        self.assertEqual([(call[2], call[3]) for call in sent], [
            (b"SCR_USER5", 4005.0),
        ])

    def test_legacy_scr_user6_is_initialized_as_idle_mailbox(self):
        vehicle = self._make_ida_link()
        vehicle.master = SimpleNamespace(mav=SimpleNamespace())
        vehicle._read_float_param = mock.MagicMock(return_value=1.0)
        vehicle._set_float_param_with_readback = mock.MagicMock(return_value=True)

        result = vehicle.write_mission_parkur_metadata(4, 1)

        self.assertTrue(result[0])
        self.assertEqual(vehicle._set_float_param_with_readback.call_args_list, [
            mock.call(vehicle.master, b"SCR_USER5", 4005.0),
            mock.call(vehicle.master, b"SCR_USER6", 8_000_000.0),
        ])

    def test_ida_start_uses_guided_and_token_not_auto_mission_start(self):
        vehicle = self._make_ida_link()
        vehicle.master = SimpleNamespace(mav=SimpleNamespace())
        vehicle._read_float_param = mock.MagicMock(return_value=8_000_000.0)
        vehicle.set_mode = mock.MagicMock(
            return_value=(True, "GUIDED verified", "state_verified")
        )
        vehicle._signal_mission_control_locked = mock.MagicMock(
            return_value=(True, "Jetson ACK", "state_verified")
        )

        result = vehicle.start_mission()

        self.assertTrue(result[0])
        vehicle.set_mode.assert_called_once_with("GUIDED")
        vehicle._signal_mission_control_locked.assert_called_once_with(start=True)

    def test_late_start_ack_requires_stop_before_restart(self):
        vehicle = self._make_ida_link()
        vehicle.master = SimpleNamespace(mav=SimpleNamespace())
        vehicle._read_float_param = mock.MagicMock(return_value=8_000_003.0)
        vehicle.set_mode = mock.MagicMock()

        result = vehicle.start_mission()

        self.assertFalse(result[0])
        self.assertEqual(result[2], "busy")
        vehicle.set_mode.assert_not_called()

    def test_start_cannot_enter_guided_during_mission_upload(self):
        vehicle = self._make_ida_link()
        vehicle.set_mode = mock.MagicMock()
        vehicle._mission_lock.acquire()
        try:
            result = vehicle.start_mission()
        finally:
            vehicle._mission_lock.release()

        self.assertFalse(result[0])
        self.assertEqual(result[2], "busy")
        vehicle.set_mode.assert_not_called()

    def test_direct_guided_is_blocked_while_start_token_is_pending(self):
        vehicle = self._make_ida_link()
        vehicle.master = SimpleNamespace(mode_mapping=lambda: {"GUIDED": 15})
        vehicle._read_float_param = mock.MagicMock(return_value=8_000_001.0)
        vehicle._send_command_long = mock.MagicMock()

        result = vehicle.set_mode("GUIDED")

        self.assertFalse(result[0])
        self.assertEqual(result[2], "busy")
        vehicle._send_command_long.assert_not_called()

    def test_ida_stop_holds_before_persistent_stop_token(self):
        vehicle = self._make_ida_link()
        vehicle.set_mode = mock.MagicMock(
            return_value=(True, "HOLD verified", "state_verified")
        )
        vehicle._signal_mission_control = mock.MagicMock(
            return_value=(True, "Jetson ACK", "state_verified")
        )

        result = vehicle.stop_mission()

        self.assertTrue(result[0])
        vehicle.set_mode.assert_called_once_with("HOLD")
        vehicle._signal_mission_control.assert_called_once_with(start=False)

    def test_ida_mission_control_does_not_overwrite_pending_token(self):
        vehicle = self._make_ida_link()
        vehicle.master = SimpleNamespace(mav=SimpleNamespace())
        vehicle._read_float_param = mock.MagicMock(return_value=8_000_001.0)
        vehicle._set_mission_mailbox_command = mock.MagicMock()

        result = vehicle._signal_mission_control(start=True)

        self.assertFalse(result[0])
        self.assertEqual(result[2], "busy")
        vehicle._set_mission_mailbox_command.assert_not_called()

    def test_mission_upload_does_not_overwrite_pending_control(self):
        vehicle = self._make_ida_link()
        vehicle.master = SimpleNamespace(mav=SimpleNamespace())
        vehicle._read_float_param = mock.MagicMock(return_value=8_000_001.0)
        vehicle._send_mission_clear = mock.MagicMock()

        result = vehicle.upload_mission([
            {"lat": 41.0, "lon": 29.0, "alt": 0.0, "parkur": 1}
        ])

        self.assertFalse(result[0])
        self.assertEqual(result[2], "busy")
        vehicle._send_mission_clear.assert_not_called()

    def test_start_ack_timeout_holds_without_skipping_sequence(self):
        vehicle = self._make_ida_link()
        vehicle.master = SimpleNamespace(mav=SimpleNamespace())
        vehicle._read_float_param = mock.MagicMock(return_value=8_000_000.0)
        vehicle._set_mission_mailbox_command = mock.MagicMock(return_value=8_000_001)
        vehicle.set_mode = mock.MagicMock(return_value=(True, "HOLD", "state_verified"))

        with mock.patch(
            # _connection() freshness kontrolü de aynı modüldeki monotonic'i
            # bir kez okur; sonra deadline ve ilk loop kontrolü gelir.
            "links.mavlink_vehicle.time.monotonic", side_effect=[100.0, 100.0, 106.0]
        ):
            result = vehicle._signal_mission_control(start=True)

        self.assertFalse(result[0])
        self.assertEqual(result[2], "timeout")
        vehicle.set_mode.assert_called_once_with("HOLD")
        vehicle._set_mission_mailbox_command.assert_called_once_with(
            vehicle.master, 8_000_001, 8_000_003
        )

    def test_fast_jetson_ack_during_command_readback_is_success(self):
        vehicle = self._make_ida_link()
        vehicle.master = SimpleNamespace(mav=SimpleNamespace())
        vehicle._read_float_param = mock.MagicMock(return_value=8_000_000.0)
        vehicle._set_mission_mailbox_command = mock.MagicMock(return_value=8_000_003)

        result = vehicle._signal_mission_control(start=True)

        self.assertTrue(result[0])
        self.assertEqual(result[2], "state_verified")
        vehicle._read_float_param.assert_called_once()

    def test_mailbox_writer_accepts_ack_that_races_command_echo(self):
        import links.mavlink_vehicle as module

        vehicle = self._make_ida_link()

        class FakeMav:
            def param_set_send(self, *_args):
                vehicle._param_value_messages.put_nowait(FakeMessage(
                    "PARAM_VALUE", param_id=b"SCR_USER6\x00", param_value=8_000_003.0
                ))

        vehicle.master = SimpleNamespace(mav=FakeMav())
        old_mavutil = module.mavutil
        module.mavutil = SimpleNamespace(mavlink=SimpleNamespace(MAV_PARAM_TYPE_REAL32=9))
        try:
            observed = vehicle._set_mission_mailbox_command(
                vehicle.master, 8_000_001, 8_000_003
            )
        finally:
            module.mavutil = old_mavutil

        self.assertEqual(observed, 8_000_003)

    def test_ida_target_ack_only_verifies_matching_locked_color(self):
        vehicle = self._make_ida_link()
        received = []
        vehicle.target_ack_handler = received.append
        vehicle._handle_named_value_int(
            FakeMessage("NAMED_VALUE_INT", name=b"TGT_ACK", value=2)
        )
        self.assertEqual(received, [2])

        vehicle._handle_named_value_int(
            FakeMessage("NAMED_VALUE_INT", name=b"TGT_ACK", value=1, src_component=1)
        )
        self.assertEqual(received, [2])

        self.targets.lock_target({"color": "KIRMIZI", "source": "IHA", "confidence": 1.0})
        mismatch, _ = self.targets.confirm_vehicle_target(2)
        matched, _ = self.targets.confirm_vehicle_target(1)
        self.assertFalse(mismatch)
        self.assertTrue(matched)
        self.assertEqual(self.targets.snapshot()["delivery_status"], "state_verified")

    @staticmethod
    def _join_detection_threads():
        import threading

        for thread in threading.enumerate():
            if thread.name.endswith("-target-detection"):
                thread.join(timeout=2)

    def test_websocket_token_validation_requires_configured_match(self):
        import main as backend_main

        old_required = backend_main.config.WS_AUTH_REQUIRED
        old_token = backend_main.config.WS_AUTH_TOKEN
        try:
            backend_main.config.WS_AUTH_REQUIRED = True
            backend_main.config.WS_AUTH_TOKEN = "field-token"

            self.assertTrue(backend_main._token_valid("field-token"))
            self.assertFalse(backend_main._token_valid("wrong-token"))

            backend_main.config.WS_AUTH_TOKEN = ""
            self.assertFalse(backend_main._token_valid("field-token"))
        finally:
            backend_main.config.WS_AUTH_REQUIRED = old_required
            backend_main.config.WS_AUTH_TOKEN = old_token

    def test_autonomy_named_value_contract_updates_ida_state(self):
        from links.mavlink_vehicle import MAVLinkVehicle

        state = TelemetryState(1, 2, heartbeat_timeout=5)
        vehicle = MAVLinkVehicle(
            port="/dev/null", baudrate=57600, expected_sys_id=1, gcs_sys_id=255,
            name="IDA", state_key="ida", telemetry_state=state, logger=self.logger,
            emergency_action="HOLD", reconnect_delay=1.0, mission_timeout=5.0,
            telemetry_rate_hz=2, command_ack_timeout=1.0, state_verify_timeout=1.0,
            auto_return_on_link_loss=False, return_home_mode="RTL",
        )
        vehicle._handle_named_value_int(FakeMessage("NAMED_VALUE_INT", name=b"AUTO_ST\x00", value=4))
        vehicle._handle_named_value_int(FakeMessage("NAMED_VALUE_INT", name="PERC_OBS", value=103))
        snapshot = state.vehicle_snapshot("ida")
        self.assertEqual(snapshot["autonomy_state"], "PARKUR_2_AVOIDANCE")
        self.assertEqual(snapshot["perception_obstacle_count"], 3)
        self.assertGreater(snapshot["autonomy_last_update_monotonic"], 0)
        self.assertFalse(snapshot["autonomy_fresh"])

        state.update_message("ida", FakeMessage("HEARTBEAT", base_mode=0, system_status=3))
        vehicle._handle_named_value_int(FakeMessage("NAMED_VALUE_INT", name="AUTO_ST", value=3))
        self.assertTrue(state.vehicle_snapshot("ida")["autonomy_fresh"])
        state.mark_disconnected("ida")
        self.assertFalse(state.vehicle_snapshot("ida")["autonomy_fresh"])

    def test_autonomy_decoder_rejects_nonexact_and_out_of_bounds_values(self):
        from services.autonomy_contract import decode_ida_field

        invalid = [
            ("AUTO_ST", 4.0), ("AUTO_ST", 99), ("LOG_ACT", 2),
            ("PERC_OBS", 99), ("PERC_OBS", 10101), ("PERC_DET", True),
        ]
        for name, value in invalid:
            with self.subTest(name=name, value=value):
                self.assertEqual(decode_ida_field(name, value), (None, None))

    @staticmethod
    def _valid_debug_snapshot():
        return {
            "schema_version": 1, "source_stamp": 100.0, "frame_id": "base_link",
            "lidar": {
                "status": "ok",
                "points": [{"forward_m": 1.0, "lateral_right_m": 2.0}],
                "clusters": [{"id": "cluster-1", "forward_m": 2.0, "lateral_right_m": -1.0}],
            },
            "camera": {"status": "ok", "bearing_rad": 0.2},
            "fusion": {"status": "ok", "objects": [{
                "id": "orange-1", "forward_m": 2.0, "lateral_right_m": -1.0,
                "color": "orange", "source": "fusion", "status": "ok",
            }]},
            "autonomy": {"status": "ok", "state": "PARKUR_1_NAV", "action": "dwa", "current_waypoint": 4, "failsafe_reason": ""},
            "chosen_command": {"status": "ok", "forward_mps": 0.6, "yaw_rate_rps": 0.2},
        }

    def test_lab_debug_adapter_is_strict_latest_only_with_ages(self):
        adapter = InMemoryDebugAdapter()
        snapshot = adapter.ingest(self._valid_debug_snapshot(), now_epoch=100.5, now_monotonic=10.0)
        self.assertEqual(snapshot["fusion"]["objects"][0]["lateral_right_m"], -1.0)
        displayed = adapter.display_snapshot(now_epoch=101.0, now_monotonic=10.75)
        self.assertEqual(displayed["server_receive_age_s"], 0.75)
        self.assertEqual(displayed["source_age_s"], 1.0)

    def test_lab_debug_rejects_malformed_nonfinite_range_color_status_id_and_bounds(self):
        mutations = []
        bad = self._valid_debug_snapshot(); bad["schema_version"] = 99; mutations.append(bad)
        bad = self._valid_debug_snapshot(); bad["lidar"]["points"][0]["forward_m"] = math.inf; mutations.append(bad)
        bad = self._valid_debug_snapshot(); bad["lidar"]["points"][0]["lateral_right_m"] = 101; mutations.append(bad)
        bad = self._valid_debug_snapshot(); bad["fusion"]["objects"][0]["color"] = "purple"; mutations.append(bad)
        bad = self._valid_debug_snapshot(); bad["camera"]["status"] = "ready"; mutations.append(bad)
        bad = self._valid_debug_snapshot(); bad["fusion"]["objects"][0]["id"] = "bad id"; mutations.append(bad)
        bad = self._valid_debug_snapshot(); bad["lidar"]["points"] *= 721; mutations.append(bad)
        bad = self._valid_debug_snapshot(); bad["unexpected"] = True; mutations.append(bad)
        for payload in mutations:
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    normalize_snapshot(payload, now_epoch=100.5, now_monotonic=10.0)

    def test_canonical_vehicle_test_manifest_and_session_bound_results(self):
        relay = VehicleTestRelay()
        expected_ids = {"comms", "telemetry", "camera_p1p2", "camera_p3", "lidar", "fusion_shadow", "autonomy_shadow", "logging"}
        self.assertEqual({item["id"] for item in CONTRACT["tests"]}, expected_ids)
        self.assertEqual({item["id"] for item in relay.manifest()["tests"]}, expected_ids)
        expectations = {"profile": "lidar_bottle", "case": "positive", "expected_color": None, "expected_range_m": 2.0, "expected_bearing_deg": 5.0, "range_tolerance_m": .25, "bearing_tolerance_deg": 3.0, "expected_id": "bottle-1"}
        pending = relay.start("lidar", 20, expectations)
        self.assertEqual((pending["event"], pending["seq"]), ("pending", 0))
        request = relay.pending_requests()[0]
        self.assertEqual(set(request), {"action", "run_id", "seq", "test", "timeout_s", "expectations"})
        relay.mark_sent(request)
        self.assertEqual(relay.pending_requests(), [])
        forwarded = relay.acknowledge({"action": "start", "run_id": pending["run_id"], "seq": 1, "accepted": True})
        self.assertEqual(forwarded["event"], "forwarded")
        replay = relay.acknowledge({"action": "start", "run_id": pending["run_id"], "seq": 1, "accepted": True})
        self.assertEqual(replay, forwarded)
        with self.assertRaises(ValueError):
            relay.acknowledge({"action": "start", "run_id": pending["run_id"], "seq": 1, "accepted": False})
        cancel_event = relay.cancel(pending["run_id"])
        delayed_replay = relay.acknowledge({"action": "start", "run_id": pending["run_id"], "seq": 1, "accepted": True})
        self.assertEqual(delayed_replay, forwarded)
        self.assertLess(delayed_replay["seq"], cancel_event["seq"])
        with self.assertRaises(ValueError):
            relay.accept_ros("/vehicle_test/events", {"schema_version": 1, "run_id": "spoof", "seq": 1, "test": "lidar", "event": "STARTED", "monotonic_s": 1.0})
        raw = {"schema_version": 1, "run_id": pending["run_id"], "seq": 1, "test": "lidar", "state": "PASS", "actuation_enabled": False, "remaining_s": 0.0, "reasons": [], "expectations": expectations, "stamp": 2.0, "pass": True, "artifact": {"path": "run/result.json", "sha256": "a" * 64}, "evidence_manifest": {}, "bundle_correlations": [], "channels": {}}
        result = relay.accept_ros("/vehicle_test/result", raw)
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["artifact"]["sha256"], "a" * 64)
        self.assertEqual(result["raw"], raw)
        # Lost application ACK: exact terminal replay is idempotent, while a
        # conflicting payload with the same identity remains rejected.
        self.assertEqual(relay.accept_ros("/vehicle_test/result", raw), result)
        conflicting = dict(raw); conflicting["reasons"] = ["changed"]
        with self.assertRaises(ValueError):
            relay.accept_ros("/vehicle_test/result", conflicting)
        queued = relay.start("logging", 20, None)
        self.assertEqual(relay.pending_requests()[0]["seq"], 3)
        cancelled = relay.cancel(queued["run_id"])
        self.assertEqual(cancelled["status"], "CANCELLED")
        self.assertEqual(relay.pending_requests(), [])

    def test_async_producer_handler_rejects_spoof_and_accepts_strict_snapshot(self):
        adapter = InMemoryDebugAdapter(); relay = VehicleTestRelay()
        kind, snapshot = asyncio.run(handle_producer_message(
            {"type": "debug_snapshot", "data": self._valid_debug_snapshot()}, adapter, relay,
        ))
        self.assertEqual(kind, "debug_snapshot")
        self.assertEqual(snapshot["frame_id"], "base_link")
        with self.assertRaises(ValueError):
            asyncio.run(handle_producer_message({"type": "debug_snapshot", "data": {}}, adapter, relay))
        pending = relay.start("comms", 15, None)
        request = relay.pending_requests()[0]; relay.mark_sent(request)
        kind, forwarded = asyncio.run(handle_producer_message({"type": "vehicle_test_ack", "data": {"action": "start", "run_id": pending["run_id"], "seq": 1, "accepted": True}}, adapter, relay))
        self.assertEqual((kind, forwarded["event"]), ("vehicle_test", "forwarded"))
        monitor = {"schema_version": 1, "run_id": pending["run_id"], "seq": 1, "test": "comms", "event": "STARTED", "monotonic_s": 1.0, "stamp": 2.0, "actuation_enabled": False}
        kind, result = asyncio.run(handle_producer_message({"type": "vehicle_test_ros", "data": {"topic": "/vehicle_test/events", "payload": monitor}}, adapter, relay))
        self.assertEqual((kind, result["event"]), ("vehicle_test", "monitor_event"))
        with self.assertRaises(ValueError):
            asyncio.run(handle_producer_message({"type": "vehicle_test_ros", "data": {"topic": "/bad", "payload": monitor}}, adapter, relay))

    def test_vehicle_test_reconnect_redelivery_cancel_order_and_bounded_history(self):
        relay = VehicleTestRelay()
        pending = relay.start("comms", 15, None)
        start_request = relay.pending_requests()[0]
        relay.mark_sent(start_request)
        cancelled = relay.cancel(pending["run_id"])
        self.assertEqual(cancelled["event"], "cancel_requested")
        # While the start write is in flight only cancel is newly deliverable.
        self.assertEqual([item["action"] for item in relay.pending_requests()], ["cancel"])
        relay.release_inflight()
        requests = relay.pending_requests()
        self.assertEqual([item["action"] for item in requests], ["start", "cancel"])
        self.assertEqual(requests, relay.pending_requests())
        relay.mark_sent(requests[1])
        relay.acknowledge({"action": "start", "run_id": pending["run_id"], "seq": 1, "accepted": True})
        relay.acknowledge({"action": "cancel", "run_id": pending["run_id"], "seq": 2, "accepted": True})
        result = {"schema_version": 1, "run_id": pending["run_id"], "seq": 1, "test": "comms", "state": "CANCELLED", "actuation_enabled": False, "remaining_s": 0.0, "reasons": ["cancelled"], "expectations": None, "stamp": 2.0, "pass": False, "artifact": None, "evidence_manifest": {}, "bundle_correlations": [], "channels": {}}
        relay.accept_ros("/vehicle_test/result", result)
        for _ in range(33):
            item = relay.start("logging", 20, None)
            relay.cancel(item["run_id"])
        self.assertEqual(len(relay.terminal_history()), CONTRACT["terminal_history_limit"])

    def test_lidar_left_is_negated_but_canonical_fusion_is_not(self):
        self.assertEqual(raw_lidar_left_to_canonical({"forward_m": 2, "lateral_left_m": 1.5}), {"forward_m": 2.0, "lateral_right_m": -1.5})
        snapshot = normalize_snapshot(self._valid_debug_snapshot(), now_epoch=100.5, now_monotonic=1)
        self.assertEqual(snapshot["fusion"]["objects"][0]["lateral_right_m"], -1.0)

    def test_debug_clock_rollback_is_explicitly_invalid(self):
        adapter = InMemoryDebugAdapter()
        adapter.ingest(self._valid_debug_snapshot(), now_epoch=100.5, now_monotonic=10.0)
        snapshot = adapter.display_snapshot(now_epoch=90.0, now_monotonic=5.0)
        self.assertFalse(snapshot["clock_valid"])
        self.assertLess(snapshot["server_receive_age_s"], 0.0)
        self.assertLess(snapshot["source_age_s"], 0.0)

    def test_passive_test_start_requires_existing_ws_authorization(self):
        self.assertFalse(is_lab_authorized(False, True, "field-token"))
        self.assertFalse(is_lab_authorized(True, False, "field-token"))
        self.assertFalse(is_lab_authorized(True, True, ""))
        self.assertTrue(is_lab_authorized(True, True, "field-token"))
        self.assertFalse(producer_token_valid("browser-token", "jetson-token", True))
        self.assertFalse(producer_token_valid("jetson-token", "jetson-token", False))
        self.assertTrue(producer_token_valid("jetson-token", "jetson-token", True))
        validate_lab_config(True, True, "browser-token", "jetson-token")
        for browser, producer in [("", "jetson"), ("browser", ""), ("same", "same")]:
            with self.assertRaises(RuntimeError):
                validate_lab_config(True, True, browser, producer)


if __name__ == "__main__":
    unittest.main()
