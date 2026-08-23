"""Regression tests for the P1/P2 single navigation behavior authority."""

import math
import os
import sys
import unittest


_PACKAGE_ROOT = os.path.dirname(__file__)
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

from ida_planning.behavior import (  # noqa: E402
    CenterObstacleEscapeController,
    MODE_ALIGN,
    MODE_CRUISE,
    MODE_RECOVERY,
    MODE_SAFETY_STOP,
    NavigationBehaviorArbiter,
    motion_expected_for_stuck,
    navigation_safety_envelope,
)
from ida_planning.dwa import Candidate, DwaPlanner  # noqa: E402
from ida_planning.planner import Command, near_field_command  # noqa: E402


def _command(vx=0.7, yaw_deg_s=0.0, action="dwa score=1"):
    return Command(vx, 0.0, math.radians(yaw_deg_s), action)


def _obstacle(forward, lateral=0.0, *, hard=True):
    return {
        "forward_m": forward,
        "lateral_m": lateral,
        "hard_obstacle": hard,
    }


class FinalMotionEnvelopeTest(unittest.TestCase):
    def test_nonfinite_command_fails_closed(self):
        final, reason = navigation_safety_envelope(
            _command(float("nan"), 0.0), [], high_yaw_deg_s=22.5,
            obstacle_stop_m=2.0,
        )
        self.assertEqual((final.vx, final.vy, final.yaw_rate), (0.0, 0.0, 0.0))
        self.assertEqual(reason, "invalid_command")

    def test_high_yaw_never_carries_forward_thrust(self):
        final, reason = navigation_safety_envelope(
            _command(0.7, 25.0), [], high_yaw_deg_s=22.5,
            obstacle_stop_m=2.0,
        )
        self.assertEqual(final.vx, 0.0)
        self.assertAlmostEqual(final.yaw_rate, math.radians(25.0))
        self.assertIn("high_yaw", reason)

    def test_close_forward_hard_obstacle_never_carries_forward_thrust(self):
        final, reason = navigation_safety_envelope(
            _command(0.7, 10.0), [_obstacle(1.6, 0.8)],
            high_yaw_deg_s=22.5, obstacle_stop_m=2.0,
        )
        self.assertEqual(final.vx, 0.0)
        self.assertIn("obstacle_", reason)

    def test_soft_or_behind_obstacles_do_not_mask_clear_path(self):
        obstacles = [_obstacle(1.0, hard=False), _obstacle(-1.0, hard=True)]
        final, reason = navigation_safety_envelope(
            _command(0.7, 10.0), obstacles, high_yaw_deg_s=22.5,
            obstacle_stop_m=2.0,
        )
        self.assertEqual(final.vx, 0.7)
        self.assertEqual(reason, "")

    def test_close_gate_buoys_outside_hull_corridor_do_not_force_stop(self):
        obstacles = [_obstacle(0.7, -1.7), _obstacle(0.7, 1.7)]
        final, reason = navigation_safety_envelope(
            _command(0.4, 0.0), obstacles,
            high_yaw_deg_s=22.5,
            obstacle_stop_m=2.0,
            lateral_clearance_m=1.0,
        )
        self.assertEqual(final.vx, 0.4)
        self.assertEqual(reason, "")

    def test_rear_buoy_inside_pivot_sweep_uses_straight_escape(self):
        final, reason = navigation_safety_envelope(
            _command(0.7, -50.0),
            [_obstacle(-0.84, 0.04)],
            high_yaw_deg_s=22.5,
            obstacle_stop_m=2.0,
            rotational_sweep_radius_m=1.0,
            rear_sweep_escape_speed_mps=0.2,
        )
        self.assertAlmostEqual(final.vx, 0.2)
        self.assertEqual(final.yaw_rate, 0.0)
        self.assertTrue(final.action.startswith("near_field_slow rear_sweep_escape"))
        self.assertIn("rear_sweep_escape", reason)

    def test_zero_surge_near_field_pivot_cannot_sweep_rear_buoy(self):
        final, reason = navigation_safety_envelope(
            _command(0.0, -30.0, "near_field_stop"),
            [_obstacle(-0.84, 0.04)],
            high_yaw_deg_s=22.5,
            obstacle_stop_m=2.0,
            rotational_sweep_radius_m=1.0,
            rear_sweep_escape_speed_mps=0.2,
        )
        self.assertAlmostEqual(final.vx, 0.2)
        self.assertEqual(final.yaw_rate, 0.0)
        self.assertIn("rear_sweep_escape", reason)

    def test_front_or_side_buoy_inside_pivot_sweep_stops_rotation(self):
        final, reason = navigation_safety_envelope(
            _command(0.7, 50.0),
            [_obstacle(0.3, 0.7)],
            high_yaw_deg_s=22.5,
            obstacle_stop_m=2.0,
            rotational_sweep_radius_m=1.0,
            rear_sweep_escape_speed_mps=0.2,
        )
        self.assertEqual((final.vx, final.yaw_rate), (0.0, 0.0))
        self.assertIn("rotational_sweep", reason)

    def test_normal_p2_yaw_inside_sweep_pivots_away_before_contact(self):
        final, reason = navigation_safety_envelope(
            _command(1.0, -20.0),
            [_obstacle(0.75, -0.90)],
            high_yaw_deg_s=22.5,
            obstacle_stop_m=1.0,
            rotational_sweep_radius_m=1.3,
            rear_sweep_escape_speed_mps=0.2,
        )
        self.assertEqual(final.vx, 0.0)
        self.assertAlmostEqual(final.yaw_rate, math.radians(-20.0))
        self.assertTrue(final.action.startswith("navigation_safety_stop pivot_away"))
        self.assertIn("pivot_away_sweep", reason)

    def test_normal_p2_yaw_uses_same_sign_as_positive_lateral_obstacle(self):
        final, _reason = navigation_safety_envelope(
            _command(1.0, -20.0),
            [_obstacle(0.75, 0.90)],
            high_yaw_deg_s=22.5,
            obstacle_stop_m=1.0,
            rotational_sweep_radius_m=1.3,
            rear_sweep_escape_speed_mps=0.2,
        )
        self.assertEqual(final.vx, 0.0)
        self.assertAlmostEqual(final.yaw_rate, math.radians(20.0))


class CenterObstacleEscapeControllerTest(unittest.TestCase):
    def _controller(self, **overrides):
        values = dict(
            trigger_s=1.5,
            reverse_s=1.5,
            cooldown_s=1.0,
            reverse_speed_mps=0.2,
            yaw_deg_s=30.0,
            rear_stop_m=1.5,
        )
        values.update(overrides)
        return CenterObstacleEscapeController(
            **values,
        )

    def test_stable_center_stop_starts_bounded_reverse_arc(self):
        controller = self._controller()
        stop = _command(0.0, 30.0, "near_field_stop_center")
        first, reason = controller.apply(stop, [_obstacle(1.4)], 10.0)
        early, _ = controller.apply(stop, [_obstacle(1.4)], 11.4)
        active, reason_active = controller.apply(stop, [_obstacle(1.4)], 11.5)
        expired, reason_expired = controller.apply(stop, [_obstacle(1.4)], 13.0)
        self.assertEqual((first.vx, early.vx), (0.0, 0.0))
        self.assertEqual(reason, "arming")
        self.assertEqual(active.vx, -0.2)
        self.assertAlmostEqual(active.yaw_rate, math.radians(30.0))
        self.assertEqual(reason_active, "reverse_started")
        self.assertEqual(expired.vx, 0.0)
        self.assertEqual(reason_expired, "cooldown")

    def test_close_rear_obstacle_refuses_reverse(self):
        controller = self._controller()
        stop = _command(0.0, 30.0, "near_field_stop_center")
        obstacles = [_obstacle(1.4), _obstacle(-1.0)]
        controller.apply(stop, obstacles, 1.0)
        blocked, reason = controller.apply(stop, obstacles, 3.0)
        self.assertEqual(blocked.vx, 0.0)
        self.assertEqual(reason, "rear_blocked")

    def test_off_center_near_stop_also_escapes_pure_pivot_deadlock(self):
        controller = self._controller()
        stop = _command(0.0, -30.0, "near_field_stop")
        controller.apply(stop, [_obstacle(1.4, 0.5)], 1.0)
        active, reason = controller.apply(stop, [_obstacle(1.4, 0.5)], 2.5)
        self.assertEqual(active.vx, -0.2)
        self.assertLess(active.yaw_rate, 0.0)
        self.assertEqual(reason, "reverse_started")

    def test_clear_path_or_clock_rollback_resets_arming(self):
        controller = self._controller()
        stop = _command(0.0, 30.0, "near_field_stop_center")
        controller.apply(stop, [_obstacle(1.4)], 10.0)
        clear, reason = controller.apply(_command(), [], 10.5)
        self.assertEqual(clear.vx, 0.7)
        self.assertEqual(reason, "clear")
        controller.apply(stop, [_obstacle(1.4)], 11.0)
        rolled, rolled_reason = controller.apply(stop, [_obstacle(1.4)], 1.0)
        self.assertEqual(rolled.vx, 0.0)
        self.assertEqual(rolled_reason, "arming")

    def test_clear_path_after_reverse_gets_opposite_yaw_forward_commit(self):
        controller = self._controller(
            forward_commit_s=1.5,
            forward_commit_speed_mps=0.4,
            forward_commit_yaw_deg_s=20.0,
        )
        stop = _command(0.0, 30.0, "near_field_stop_center")
        controller.apply(stop, [_obstacle(1.4)], 1.0)
        reverse, _ = controller.apply(stop, [_obstacle(1.4)], 2.5)
        commit, reason = controller.apply(_command(0.7, 0.0), [], 4.0)
        held, held_reason = controller.apply(_command(0.7, 0.0), [], 4.5)
        self.assertEqual(reverse.vx, -0.2)
        self.assertEqual(commit.vx, 0.4)
        self.assertLess(commit.yaw_rate, 0.0)
        self.assertEqual(reason, "forward_commit_started")
        self.assertEqual((held.vx, held_reason), (0.4, "forward_commit_active"))

    def test_blocked_commit_stops_and_flips_next_reverse_side(self):
        controller = self._controller(
            trigger_s=0.5,
            reverse_s=1.0,
            cooldown_s=0.5,
            forward_commit_s=1.0,
            forward_commit_speed_mps=0.4,
            forward_commit_yaw_deg_s=20.0,
        )
        right_stop = _command(0.0, 30.0, "near_field_stop_center")
        controller.apply(right_stop, [_obstacle(1.4)], 1.0)
        first_reverse, _ = controller.apply(right_stop, [_obstacle(1.4)], 1.5)
        blocked, reason = controller.apply(right_stop, [_obstacle(1.4)], 2.5)
        controller.apply(right_stop, [_obstacle(1.4)], 3.0)
        second_reverse, _ = controller.apply(right_stop, [_obstacle(1.4)], 3.5)
        self.assertGreater(first_reverse.yaw_rate, 0.0)
        self.assertEqual((blocked.vx, reason), (0.0, "cooldown"))
        self.assertLess(second_reverse.yaw_rate, 0.0)


class WaypointAlignmentArcTest(unittest.TestCase):
    def test_stationary_align_becomes_bounded_forward_arc(self):
        result = near_field_command(
            [], _command(0.0, 30.0, "dwa_align_waypoint_right"),
            heading_error_deg=52.0,
            heading_align_max_yaw_deg_s=12.0,
            heading_align_speed_mps=0.20,
        )
        self.assertTrue(result.active)
        self.assertAlmostEqual(result.command.vx, 0.20)
        self.assertLessEqual(
            abs(result.command.yaw_rate), math.radians(12.0) + 1e-9
        )

    def test_cruise_alignment_is_capped_not_left_at_full_speed(self):
        result = near_field_command(
            [], _command(0.6, 0.0, "dwa score=1"),
            heading_error_deg=-25.0,
            heading_align_max_yaw_deg_s=12.0,
            heading_align_speed_mps=0.20,
        )
        self.assertAlmostEqual(result.command.vx, 0.20)
        self.assertLess(result.command.yaw_rate, 0.0)


class NavigationBehaviorArbiterTest(unittest.TestCase):
    def _arbiter(self):
        return NavigationBehaviorArbiter(
            min_dwell_s={MODE_ALIGN: 0.6, MODE_RECOVERY: 1.0},
            high_yaw_deg_s=40.0,
            obstacle_stop_m=2.0,
        )

    def test_align_dwell_blocks_lower_priority_cruise_chatter(self):
        arbiter = self._arbiter()
        first = arbiter.select(_command(0.0, 20.0, "dwa_align_waypoint"), [], 10.0)
        held = arbiter.select(_command(0.7, 0.0, "dwa score=2"), [], 10.2)
        released = arbiter.select(_command(0.7, 0.0, "dwa score=2"), [], 10.7)
        self.assertEqual(first.mode, MODE_ALIGN)
        self.assertTrue(held.held)
        self.assertEqual(held.mode, MODE_ALIGN)
        self.assertEqual(held.command.vx, 0.0)
        self.assertFalse(released.held)
        self.assertEqual(released.mode, MODE_CRUISE)

    def test_recovery_preempts_align_and_holds_over_align(self):
        arbiter = self._arbiter()
        arbiter.select(_command(0.0, 20.0, "dwa_align_waypoint"), [], 1.0)
        recovery = arbiter.select(_command(0.3, 15.0, "stuck_recovery"), [], 1.1)
        held = arbiter.select(_command(0.0, -20.0, "dwa_align_waypoint"), [], 1.5)
        self.assertEqual(recovery.mode, MODE_RECOVERY)
        self.assertFalse(recovery.held)
        self.assertEqual(held.mode, MODE_RECOVERY)
        self.assertTrue(held.held)

    def test_dwell_can_be_disabled_without_bypassing_safety_envelope(self):
        arbiter = self._arbiter()
        arbiter.select(_command(0.3, 15.0, "stuck_recovery"), [], 1.0)
        released = arbiter.select(
            _command(0.7, 0.0, "dwa score=2"), [], 1.1, allow_dwell=False
        )
        self.assertFalse(released.held)
        self.assertEqual(released.mode, MODE_CRUISE)

        stopped = arbiter.select(
            _command(0.7, 0.0, "dwa score=2"),
            [_obstacle(1.0)],
            1.2,
            allow_dwell=False,
        )
        self.assertEqual(stopped.mode, MODE_SAFETY_STOP)
        self.assertEqual(stopped.command.vx, 0.0)

    def test_current_obstacle_safety_preempts_held_recovery(self):
        arbiter = self._arbiter()
        arbiter.select(_command(0.3, 15.0, "stuck_recovery"), [], 1.0)
        stopped = arbiter.select(
            _command(0.7, 0.0, "dwa score=2"), [_obstacle(1.0)], 1.2
        )
        self.assertFalse(stopped.held)
        self.assertEqual(stopped.mode, MODE_SAFETY_STOP)
        self.assertEqual(stopped.command.vx, 0.0)

    def test_clock_rollback_discards_old_dwell_command(self):
        arbiter = self._arbiter()
        arbiter.select(_command(0.3, 15.0, "stuck_recovery"), [], 10.0)
        after_rollback = arbiter.select(_command(0.7, 0.0, "dwa score=2"), [], 1.0)
        self.assertFalse(after_rollback.held)
        self.assertEqual(after_rollback.mode, MODE_CRUISE)
        self.assertEqual(after_rollback.command.vx, 0.7)

    def test_stuck_expectation_excludes_deliberate_align_and_safety_stop(self):
        arbiter = self._arbiter()
        align = arbiter.select(_command(0.0, 20.0, "dwa_align_waypoint"), [], 1.0)
        self.assertFalse(motion_expected_for_stuck(align, 0.1))
        arbiter.reset()
        stop = arbiter.select(_command(0.7, 0.0), [_obstacle(1.0)], 2.0)
        self.assertFalse(motion_expected_for_stuck(stop, 0.1))

    def test_forward_acceleration_is_slewed_but_stop_is_immediate(self):
        arbiter = NavigationBehaviorArbiter(
            high_yaw_deg_s=40.0,
            obstacle_stop_m=2.0,
            forward_accel_mps2=0.8,
        )
        first = arbiter.select(_command(0.6), [], 1.0)
        rising = arbiter.select(_command(0.6), [], 1.1)
        stopped = arbiter.select(_command(0.6), [_obstacle(1.0)], 1.2)
        self.assertEqual(first.command.vx, 0.0)
        self.assertAlmostEqual(rising.command.vx, 0.08, places=6)
        self.assertEqual(stopped.command.vx, 0.0)
        self.assertEqual(stopped.mode, MODE_SAFETY_STOP)


class DwaParkurAlignContractTest(unittest.TestCase):
    def test_p2_can_disable_place_alignment_without_disabling_dwa(self):
        planner = DwaPlanner(
            align_before_drive_deg=42.0,
            align_yaw_rate_deg_s=30.0,
            min_drive_speed_mps=0.2,
        )
        # A 90-degree goal would normally return the early stationary align.
        aligned = planner.plan((0.0, 10.0), object(), None)
        self.assertTrue(aligned.action.startswith("dwa_align_waypoint"))

        candidate = Candidate(vx=0.3, yaw_rate=math.radians(20.0), score=1.0)
        planner.candidates = lambda goal, last: [candidate]
        planner.score_candidate = lambda cand, goal, last, costmap: cand
        planner._nearest_obstacle_lateral = lambda costmap: None
        planned = planner.plan(
            (0.0, 10.0), object(), None, allow_heading_align=False
        )
        self.assertFalse(planned.action.startswith("dwa_align_waypoint"))
        self.assertGreater(planned.vx, 0.0)

    def test_call_scoped_yaw_limit_constrains_p2_and_restores_shared_planner(self):
        planner = DwaPlanner(
            max_yaw_rate_deg_s=50.0,
            recovery_yaw_deg_s=25.0,
            min_turn_rate_deg_s=20.0,
        )
        observed_limits = []

        def candidates(_goal, _last):
            observed_limits.append(planner.max_yaw_rate)
            return [Candidate(vx=0.3, yaw_rate=planner.max_yaw_rate, score=1.0)]

        planner.candidates = candidates
        planner.score_candidate = lambda cand, goal, last, costmap: cand
        planner._nearest_obstacle_lateral = lambda costmap: None
        planned = planner.plan(
            (5.0, 1.0),
            object(),
            None,
            allow_heading_align=False,
            yaw_rate_limit_deg_s=22.0,
        )
        self.assertAlmostEqual(abs(planned.yaw_rate), math.radians(22.0))
        self.assertEqual(observed_limits, [math.radians(22.0)])
        self.assertAlmostEqual(planner.max_yaw_rate, math.radians(50.0))
        self.assertAlmostEqual(planner.recovery_yaw, math.radians(25.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
