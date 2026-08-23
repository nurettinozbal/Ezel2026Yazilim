import json
import math
import os
import sys
import unittest

PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from ida_control.bridge_health import BridgeHealth, strict_json, telemetry_payload


class BridgeHealthTests(unittest.TestCase):
    def _complete(self, health, stamp=100.0, mono=10.0):
        for source in ("position", "attitude", "velocity", "armed"):
            health.mark_telemetry(source, stamp, mono)

    def test_never_connected_or_partial_telemetry_is_not_fresh(self):
        health = BridgeHealth(False, 0.5)
        self.assertFalse(health.snapshot(10.0, 100.0)["connected"])
        health.mark_connection()
        health.mark_telemetry("position", 100.0, 10.0)
        snapshot = health.snapshot(10.0, 100.0)
        self.assertFalse(snapshot["connected"])
        self.assertFalse(snapshot["telemetry_ready"])

    def test_real_complete_acquisition_is_fresh_then_expires(self):
        health = BridgeHealth(False, 0.5)
        health.mark_connection()
        self._complete(health)
        fresh = health.snapshot(10.49, 100.49)
        self.assertTrue(fresh["connected"])
        self.assertTrue(fresh["heartbeat_fresh"])
        self.assertEqual(fresh["acquisition_stamp"], 100.0)
        self.assertFalse(health.snapshot(10.501, 100.501)["connected"])

    def test_dry_run_never_reports_real_connection(self):
        health = BridgeHealth(True, 0.5)
        health.mark_connection()
        self._complete(health)
        snapshot = health.snapshot(10.0, 100.0)
        self.assertTrue(snapshot["dry_run"])
        self.assertFalse(snapshot["connected"])
        self.assertFalse(snapshot["heartbeat_fresh"])

    def test_disconnect_and_clock_rollback_fail_closed(self):
        health = BridgeHealth(False, 0.5)
        health.mark_connection()
        self._complete(health)
        health.mark_disconnected()
        self.assertFalse(health.snapshot(10.0, 100.0)["connected"])
        health.mark_connection()
        health.mark_telemetry("position", 99.0, 9.0)
        self.assertFalse(health.snapshot(9.0, 99.0)["telemetry_ready"])
        self._complete(health, 100.0, 10.0)
        health.invalidate_telemetry()
        self.assertFalse(health.snapshot(10.0, 100.0)["connected"])

    def test_one_live_source_cannot_hide_three_stale_sources(self):
        health = BridgeHealth(False, 0.5)
        health.mark_connection()
        self._complete(health, 100.0, 0.0)
        health.mark_telemetry("armed", 110.0, 10.0)

        snapshot = health.snapshot(10.01, 110.01)

        self.assertFalse(snapshot["connected"])
        self.assertFalse(snapshot["heartbeat_fresh"])
        self.assertFalse(snapshot["source_health"]["position"]["fresh"])
        self.assertTrue(snapshot["source_health"]["armed"]["fresh"])

    def test_telemetry_payload_includes_optional_motor_pwm(self):
        state = {
            "lat": 41.0, "lon": 29.0, "heading_deg": 1.0,
            "ground_speed": 0.0, "roll_deg": 0.0, "pitch_deg": 0.0,
            "mode": "DISARMED",
            "motor_left_pwm": 1620, "motor_right_pwm": 1500,
        }
        payload = telemetry_payload(state, 123.0)
        self.assertEqual(payload["motor_left_pwm"], 1620)
        self.assertEqual(payload["motor_right_pwm"], 1500)

    def test_telemetry_payload_rejects_implausible_pwm(self):
        state = {
            "lat": 41.0, "lon": 29.0, "heading_deg": 1.0,
            "ground_speed": 0.0, "roll_deg": 0.0, "pitch_deg": 0.0,
            "mode": "DISARMED",
            "motor_left_pwm": 99999,
        }
        with self.assertRaises(ValueError):
            telemetry_payload(state, 123.0)

    def test_telemetry_payload_omits_pwm_when_absent(self):
        state = {
            "lat": 41.0, "lon": 29.0, "heading_deg": 1.0,
            "ground_speed": 0.0, "roll_deg": 0.0, "pitch_deg": 0.0,
            "mode": "DISARMED",
        }
        payload = telemetry_payload(state, 123.0)
        self.assertNotIn("motor_left_pwm", payload)
        self.assertNotIn("motor_right_pwm", payload)

    def test_strict_serializers_reject_nonfinite(self):
        state = {
            "lat": 41.0, "lon": 29.0, "heading_deg": 1.0,
            "ground_speed": 0.0, "roll_deg": 0.0, "pitch_deg": 0.0,
            "mode": "DISARMED",
        }
        payload = telemetry_payload(state, 123.0)
        self.assertEqual(payload["stamp"], payload["acquisition_stamp"])
        self.assertEqual(json.loads(strict_json(payload))["lat"], 41.0)
        with self.assertRaises(ValueError):
            telemetry_payload({**state, "lat": math.nan}, 123.0)


if __name__ == "__main__":
    unittest.main()
