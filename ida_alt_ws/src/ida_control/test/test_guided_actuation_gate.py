import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ida_control.guided_actuation_gate import (  # noqa: E402
    GuidedActuationGate,
    normalize_flight_mode,
)


class GuidedActuationGateTests(unittest.TestCase):
    def healthy_armed_gate(self) -> GuidedActuationGate:
        gate = GuidedActuationGate(2.5)
        gate.mark_armed(False, 1.0)
        gate.mark_position_health(True, True, 1.1)
        gate.mark_flight_mode("GUIDED", 1.1)
        gate.mark_armed(True, 1.2)
        return gate

    def test_requires_disarmed_cycle_health_guided_and_arm(self):
        gate = GuidedActuationGate(2.5)
        gate.mark_position_health(True, True, 1.0)
        gate.mark_flight_mode("GUIDED", 1.0)
        gate.mark_armed(True, 1.0)
        self.assertEqual(gate.decision(1.1).reason, "disarmed_cycle_not_seen")

        gate.mark_armed(False, 1.2)
        self.assertEqual(gate.decision(1.2).reason, "vehicle_disarmed")
        gate.mark_armed(True, 1.3)
        self.assertTrue(gate.decision(1.4).allowed)

    def test_hold_or_ekf_loss_latches_until_disarm(self):
        for change, expected in (
            (lambda g: g.mark_flight_mode("HOLD", 2.0), "flight_mode_hold"),
            (
                lambda g: g.mark_position_health(True, False, 2.0),
                "local_position_ekf_unhealthy",
            ),
        ):
            with self.subTest(expected=expected):
                gate = self.healthy_armed_gate()
                self.assertTrue(gate.decision(1.3).allowed)
                change(gate)
                decision = gate.decision(2.1)
                self.assertFalse(decision.allowed)
                self.assertTrue(decision.latched)
                self.assertEqual(decision.reason, expected)
                # Merely returning to GUIDED/healthy cannot reopen an armed cycle.
                gate.mark_flight_mode("GUIDED", 2.2)
                gate.mark_position_health(True, True, 2.2)
                self.assertTrue(gate.decision(2.3).latched)
                gate.mark_armed(False, 2.4)
                self.assertFalse(gate.decision(2.4).latched)

    def test_stale_disconnect_and_clock_rollback_fail_closed(self):
        gate = self.healthy_armed_gate()
        self.assertTrue(gate.decision(1.3).allowed)
        self.assertEqual(gate.decision(4.0).reason, "guided_health_stale")
        self.assertTrue(gate.decision(4.0).latched)

        gate.mark_armed(False, 4.1)
        gate.mark_position_health(True, True, 4.2)
        gate.mark_flight_mode("GUIDED", 4.2)
        gate.mark_armed(True, 4.2)
        self.assertTrue(gate.decision(4.3).allowed)
        gate.mark_disconnected()
        self.assertEqual(gate.decision(4.4).reason, "vehicle_link_lost")

        gate.mark_armed(False, 5.0)
        gate.mark_position_health(True, True, 5.1)
        gate.mark_flight_mode("GUIDED", 5.1)
        gate.mark_armed(True, 5.1)
        self.assertTrue(gate.decision(5.2).allowed)
        self.assertEqual(gate.decision(4.9).reason, "clock_invalid")

    def test_mode_normalization_and_input_validation(self):
        self.assertEqual(normalize_flight_mode("FlightMode.GUIDED"), "GUIDED")
        self.assertEqual(
            normalize_flight_mode(type("Mode", (), {"name": "HOLD"})()), "HOLD"
        )
        gate = GuidedActuationGate()
        gate.mark_armed(False, 1.0)
        gate.mark_position_health(True, True, 1.0)
        gate.mark_flight_mode("FlightMode.OFFBOARD", 1.0)
        gate.mark_armed(True, 1.0)
        self.assertTrue(gate.decision(1.1).allowed)
        with self.assertRaises(ValueError):
            GuidedActuationGate(float("nan"))
        gate = GuidedActuationGate()
        with self.assertRaises(ValueError):
            gate.mark_armed(1, 0.0)
        with self.assertRaises(ValueError):
            gate.mark_position_health(True, True, math.inf)


if __name__ == "__main__":
    unittest.main()
