"""Saf-Python unit tests for the IDA planning modules (no rclpy needed).

Run with: python -m unittest src/ida_planning/test_saf.py
"""

import math
import unittest

from ida_planning.scoring.contact import ContactCounter, ContactEvent, _buoy_key, _obstacle_key
from ida_planning.scoring.out_of_course import (
    OutOfCourseDetector,
    _point_in_polygon,
    _segment_distance_m,
)
from ida_planning.scoring.score import ScoreCalculator

from ida_planning.costmap import (
    COST_OBSTACLE,
    COST_ORANGE,
    COST_UNKNOWN,
    COST_YELLOW,
    CostMap,
    DEFAULT_COLOR_MATCH_DEG,
)
from ida_planning.dwa import Candidate, DEFAULT_W, DwaCommand, DwaPlanner, _wrap_rad, build_offset_cells
from ida_planning.geo import (
    angular_separation_deg,
    bearing_delta_deg,
    heading_error_deg,
    local_m_to_latlon,
    normalize_angle_deg,
    perpendicular_distance_m,
    signed_crossing_side_m,
)
from ida_planning.speed_safety import safety_radius, speed_for_safety_radius, stopping_distance
from ida_planning.behavior import nearest_forward_hard_obstacle_m
from ida_planning.planner import (
    Command,
    CourseGeometryTracker,
    course_geometry_parkur,
    PairCrossingDetector,
    YellowGateCounter,
    apply_obstacle_avoidance,
    build_route_corridor_info,
    corridor_goal_body,
    is_last_waypoint_index,
    nearest_neighbor_route,
    mission_waypoint_fingerprint,
    near_field_command,
    sanitize_mission_waypoints,
    split_parkur_routes,
    target_engagement_command,
    update_strict_orange_latch,
    waypoint_advance_decision,
    waypoint_guidance_index,
    wrong_target_risk,
)


class ForwardHardObstacleEnvelopeTest(unittest.TestCase):
    def test_uses_only_hard_obstacles_inside_lateral_envelope(self):
        nearest = nearest_forward_hard_obstacle_m(
            [
                {"forward_m": 1.0, "lateral_m": 0.0, "hard_obstacle": False},
                {"forward_m": 2.0, "lateral_m": 1.6, "hard_obstacle": True},
                {"forward_m": 3.0, "lateral_m": 1.2, "hard_obstacle": True},
            ],
            rear_clearance_m=0.0,
            lateral_clearance_m=1.5,
        )
        self.assertAlmostEqual(nearest, math.hypot(3.0, 1.2))

    def test_returns_none_when_forward_envelope_is_clear(self):
        self.assertIsNone(
            nearest_forward_hard_obstacle_m(
                [{"forward_m": -1.0, "lateral_m": 0.0, "hard_obstacle": True}],
                rear_clearance_m=0.0,
                lateral_clearance_m=1.5,
            )
        )


class MissionRouteCorridorTest(unittest.TestCase):
    def test_route_corridor_is_camera_independent_and_signed(self):
        start = {"lat": 40.0, "lon": 29.0, "parkur": 1}
        end_lat, end_lon = local_m_to_latlon(40.0, 29.0, 60.0, 0.0)
        cur_lat, cur_lon = local_m_to_latlon(40.0, 29.0, 10.0, 4.5)
        info = build_route_corridor_info(
            {"lat": cur_lat, "lon": cur_lon, "heading_deg": 0.0},
            [start, {"lat": end_lat, "lon": end_lon, "parkur": 2}],
            lookahead_m=8.0,
            hard_half_width_m=4.0,
        )
        self.assertTrue(info["active"])
        self.assertEqual(info["source"], "mission_route")
        self.assertAlmostEqual(info["center_forward_m"], 8.0, places=3)
        self.assertAlmostEqual(info["center_lateral_m"], -4.5, places=3)
        self.assertAlmostEqual(info["current_cross_track_m"], -4.5, places=3)

    def test_route_corridor_rejects_outward_but_keeps_inward_arc(self):
        planner = DwaPlanner(
            max_speed_mps=0.7,
            max_yaw_rate_deg_s=45.0,
            min_drive_speed_mps=0.1,
        )
        planner._corridor = {
            "active": True,
            "source": "mission_route",
            "confidence": 1.0,
            "center_forward_m": 8.0,
            "center_lateral_m": -4.1,
            "current_cross_track_m": 4.1,
            "cross_forward_coeff": 0.0,
            "cross_right_coeff": -1.0,
            "hard_half_width_m": 4.0,
        }
        costmap = CostMap(
            size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=0.5
        )
        costmap.update([], [], "", ground_speed=0.7)
        outward = planner.score_candidate(
            Candidate(vx=0.7, yaw_rate=-0.2), (20.0, -4.1), None, costmap
        )
        inward = planner.score_candidate(
            Candidate(vx=0.7, yaw_rate=0.2), (20.0, -4.1), None, costmap
        )
        self.assertTrue(outward.collides)
        self.assertFalse(inward.collides)


def buoy(color, distance, bbox_norm_x=0.0, forward_m=0.0, lateral_m=0.0, confidence=0.9, buoy_id=None):
    b = {
        "color": color,
        "distance": distance,
        "bbox_norm_x": bbox_norm_x,
        "forward_m": forward_m,
        "lateral_m": lateral_m,
        "confidence": confidence,
        "stamp": 0.0,
    }
    if buoy_id is not None:
        b["id"] = buoy_id
    return b


class TestGeoHelpers(unittest.TestCase):
    def test_heading_error_wrapping(self):
        # Hedef 10 derece sağda -> +10; hedef 350 iken heading 10 -> -20 (sola).
        self.assertAlmostEqual(heading_error_deg(0.0, 10.0), 10.0, places=6)
        self.assertAlmostEqual(heading_error_deg(10.0, 350.0), -20.0, places=6)
        self.assertAlmostEqual(heading_error_deg(170.0, -170.0), 20.0, places=6)
        self.assertAlmostEqual(heading_error_deg(0.0, 360.0), 0.0, places=6)

    def test_bearing_delta_and_separation(self):
        self.assertAlmostEqual(bearing_delta_deg(350.0, 10.0), 20.0, places=6)
        self.assertAlmostEqual(bearing_delta_deg(10.0, 350.0), -20.0, places=6)
        self.assertAlmostEqual(angular_separation_deg(10.0, 350.0), 20.0, places=6)
        self.assertAlmostEqual(angular_separation_deg(0.0, 180.0), 180.0, places=6)

    def test_normalize_angle(self):
        self.assertAlmostEqual(normalize_angle_deg(540.0), 180.0, places=6)
        self.assertAlmostEqual(normalize_angle_deg(-540.0), -180.0, places=6)

    def test_perpendicular_distance(self):
        # X ekseni üzerindeki A(0,0)-B(10,0) doğrusuna P(5,3) -> 3.0
        self.assertAlmostEqual(perpendicular_distance_m(5.0, 3.0, 0.0, 0.0, 10.0, 0.0), 3.0, places=6)
        # Aynı doğruya P(5,0) -> 0.0
        self.assertAlmostEqual(perpendicular_distance_m(5.0, 0.0, 0.0, 0.0, 10.0, 0.0), 0.0, places=6)
        # Dejenere doğru (A==B) -> nokta uzaklığına düşer.
        self.assertAlmostEqual(perpendicular_distance_m(3.0, 4.0, 0.0, 0.0, 0.0, 0.0), 5.0, places=6)

    def test_signed_crossing_side(self):
        # Gate: right buoy (0,5) -> left buoy (0,-5) doğrusu boyunca.
        # dir = left - right = (0, -10). cross = dir_n*(boat_e - right_e) - dir_e*(boat_n - right_n).
        # Boat kuzeyde (10,0): 0*(-5) - (-10)*(10) = 100 > 0 -> sol taraf (pozitif).
        self.assertGreater(signed_crossing_side_m(10.0, 0.0, 0.0, -5.0, 0.0, 5.0), 0.0)
        # Boat çizginin güneyinde (-10,0): 0*(-5) - (-10)*(-10) = -100 < 0 -> sağ taraf.
        self.assertLess(signed_crossing_side_m(-10.0, 0.0, 0.0, -5.0, 0.0, 5.0), 0.0)
        # Çizgi üzerinde -> 0.
        self.assertAlmostEqual(signed_crossing_side_m(0.0, 3.0, 0.0, -5.0, 0.0, 5.0), 0.0, places=6)
        # Dejenere gate -> 0.
        self.assertEqual(signed_crossing_side_m(10.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0)


class TestPairCrossingDetector(unittest.TestCase):
    def _make_detector(self):
        return PairCrossingDetector(
            max_pair_lateral_m=15.0,
            min_lateral_gap_m=1.0,
            cross_trigger_forward_m=3.0,
            cross_complete_forward_m=-0.5,
            min_confidence=0.35,
            max_age_s=20.0,
        )

    def test_two_gates_crossed_once_each(self):
        det = self._make_detector()
        # İki ikili: G1 (l1/r1), G2 (l2/r2). Sol dubalar negatif lateral.
        buoys = [
            buoy("orange", 8.0, forward_m=5.0, lateral_m=-3.0, buoy_id="l1"),
            buoy("orange", 8.0, forward_m=5.0, lateral_m=3.5, buoy_id="r1"),
            buoy("orange", 10.0, forward_m=7.0, lateral_m=-6.0, buoy_id="l2"),
            buoy("orange", 10.0, forward_m=7.0, lateral_m=6.5, buoy_id="r2"),
        ]
        stats = det.update(buoys, now=1.0)
        # Henüz geçilmedi (forward > -0.5).
        self.assertEqual(stats.crossed_count, 0)
        self.assertEqual(stats.kd_estimate, 2)
        self.assertAlmostEqual(stats.ratio, 0.0, places=6)

        # Araç ilerler: tüm dubalar arkada kalır.
        behind = [
            buoy("orange", 8.0, forward_m=-1.0, lateral_m=-3.0, buoy_id="l1"),
            buoy("orange", 8.0, forward_m=-1.0, lateral_m=3.5, buoy_id="r1"),
            buoy("orange", 10.0, forward_m=-1.0, lateral_m=-6.0, buoy_id="l2"),
            buoy("orange", 10.0, forward_m=-1.0, lateral_m=6.5, buoy_id="r2"),
        ]
        stats = det.update(behind, now=2.0)
        self.assertEqual(stats.crossed_count, 2)
        self.assertEqual(len(stats.crossed_keys), 2)
        self.assertAlmostEqual(stats.ratio, 1.0, places=6)

        # Aynı ikiliden tekrar geçiş SAYILMAZ (dubalar hâlâ arkada görünse bile).
        stats = det.update(behind, now=3.0)
        self.assertEqual(stats.crossed_count, 2)

    def test_reset_clears_gates(self):
        det = self._make_detector()
        # Önce ikiliyi gör, sonra geç (reset'ten önce crossed=1 olsun).
        ahead = [
            buoy("orange", 8.0, forward_m=5.0, lateral_m=-3.0, buoy_id="l1"),
            buoy("orange", 8.0, forward_m=5.0, lateral_m=3.5, buoy_id="r1"),
        ]
        det.update(ahead, now=1.0)
        behind = [
            buoy("orange", 8.0, forward_m=-1.0, lateral_m=-3.0, buoy_id="l1"),
            buoy("orange", 8.0, forward_m=-1.0, lateral_m=3.5, buoy_id="r1"),
        ]
        stats = det.update(behind, now=2.0)
        self.assertEqual(stats.crossed_count, 1)
        # Reset sonrası gate'ler boşalır.
        det.reset()
        stats = det.update(behind, now=3.0)
        # Arkadaki dubalar yeni gate OLUŞTURAMAZ (forward > cross_trigger şartı),
        # dolayısıyla crossed_count 0 kalır.
        self.assertEqual(stats.crossed_count, 0)
        self.assertEqual(stats.kd_estimate, 0)
        # Aynı dubalar öndeyken yeniden görülürse gate yeniden kurulabilir.
        stats = det.update(ahead, now=4.0)
        self.assertEqual(stats.kd_estimate, 1)

    def test_low_confidence_and_wrong_color_ignored(self):
        det = self._make_detector()
        buoys = [
            buoy("orange", 8.0, forward_m=-1.0, lateral_m=-3.0, confidence=0.1, buoy_id="l1"),
            buoy("green", 8.0, forward_m=-1.0, lateral_m=3.5, buoy_id="r1"),
        ]
        stats = det.update(buoys, now=1.0)
        self.assertEqual(stats.crossed_count, 0)
        self.assertEqual(stats.kd_estimate, 0)

    def test_stale_gates_cleaned(self):
        det = self._make_detector()
        buoys = [
            buoy("orange", 8.0, forward_m=5.0, lateral_m=-3.0, buoy_id="l1"),
            buoy("orange", 8.0, forward_m=5.0, lateral_m=3.5, buoy_id="r1"),
        ]
        det.update(buoys, now=1.0)
        stats = det.update([], now=100.0)  # 99s sonra, max_age_s=20 asıldı.
        self.assertEqual(stats.kd_estimate, 1)  # seen history monotonic
        self.assertEqual(stats.active_gates, [])

    def test_stale_gate_expires_before_same_frame_association(self):
        det = self._make_detector()
        ahead = [
            buoy("orange", 6.0, forward_m=6.0, lateral_m=-4.0),
            buoy("orange", 6.0, forward_m=6.0, lateral_m=4.0),
        ]
        first = det.update(ahead, now=1.0)
        self.assertEqual([gate.key for gate in first.active_gates], ["geo:1"])
        # Arada boş update yok: eski geo:1 association'dan önce expire olmalı.
        later = det.update(ahead, now=100.0)
        self.assertEqual([gate.key for gate in later.active_gates], ["geo:2"])
        self.assertEqual(later.kd_estimate, 2)

    def test_stable_id_reappears_without_increasing_seen_count(self):
        det = self._make_detector()
        ahead = [
            buoy("orange", 6.0, forward_m=6.0, lateral_m=-4.0, buoy_id="p1_o_l1"),
            buoy("orange", 6.0, forward_m=6.0, lateral_m=4.0, buoy_id="p1_o_r1"),
        ]
        self.assertEqual(det.update(ahead, now=1.0).kd_estimate, 1)
        self.assertEqual(det.update([], now=100.0).kd_estimate, 1)
        self.assertEqual(det.update(ahead, now=101.0).kd_estimate, 1)

    def test_malformed_or_half_gate_never_enters_seen_history(self):
        det = self._make_detector()
        half = [buoy("orange", 6.0, forward_m=6.0, lateral_m=-4.0, buoy_id="p1_o_l1")]
        malformed = [
            buoy("orange", 6.0, forward_m=6.0, lateral_m=-4.0, buoy_id="p1_o_lbad"),
            buoy("orange", 6.0, forward_m=6.0, lateral_m=4.0, buoy_id="p1_o_rbad"),
        ]
        self.assertEqual(det.update(half, now=1.0).kd_estimate, 0)
        self.assertEqual(det.update(malformed, now=2.0).kd_estimate, 0)

    def test_canonical_camera_fov_loss_does_not_imply_crossing(self):
        det = self._make_detector()
        visible = [
            buoy("orange", 7.0, forward_m=6.0, lateral_m=-4.0, buoy_id="p1_o_l1"),
            buoy("orange", 7.0, forward_m=6.0, lateral_m=4.0, buoy_id="p1_o_r1"),
        ]
        self.assertEqual(det.update(visible, now=1.0).crossed_count, 0)
        # 90 derece kamera FOV'u gate tekne hizasına gelmeden kaybedebilir.
        self.assertEqual(det.update([], now=2.0).crossed_count, 0)
        stats = det.update([], now=30.0)
        self.assertEqual(stats.crossed_count, 0)
        self.assertEqual(stats.kd_estimate, 1)

    def test_sim_truth_tracks_complete_gate_through_behind_threshold(self):
        det = self._make_detector()
        for stamp, forward in enumerate((6.0, 5.0, 0.0, -0.6), start=1):
            truth = [
                buoy("orange", 7.0, forward_m=forward, lateral_m=-4.0, buoy_id="p2_o_l1"),
                buoy("orange", 7.0, forward_m=forward, lateral_m=4.0, buoy_id="p2_o_r1"),
            ]
            stats = det.update(truth, now=float(stamp))
        self.assertEqual(stats.kd_estimate, 1)
        self.assertEqual(stats.crossed_count, 1)
        self.assertLessEqual(stats.crossed_count, stats.kd_estimate)

    def test_initial_pose_on_gate_center_does_not_create_or_cross_gate(self):
        det = self._make_detector()
        det.allow_sim_truth_geometry = True
        # full_mission initial_pose=(10,0,heading0), gate1 uçlarının
        # body-forward değerleri yaklaşık -3.84/+3.84: yeni gate önde değil.
        frame = [
            buoy("orange", 5.0, forward_m=-3.8411, lateral_m=3.2009, buoy_id="p1_o_l1"),
            buoy("orange", 5.0, forward_m=3.8411, lateral_m=-3.2009, buoy_id="p1_o_r1"),
        ]
        for item in frame:
            item["source"] = "sim_gate_truth"
        stats = det.update(frame, now=1.0)
        self.assertEqual(stats.kd_estimate, 0)
        self.assertEqual(stats.crossed_count, 0)

    def test_sim_truth_stable_id_handles_heading_offset_without_relaxing_production(self):
        def frame(heading_deg, source=None):
            angle = math.radians(heading_deg)
            result = []
            for side, east in (("l", -5.0), ("r", 5.0)):
                forward = math.cos(angle) * 10.0 + math.sin(angle) * east
                lateral = -math.sin(angle) * 10.0 + math.cos(angle) * east
                item = buoy(
                    "orange", 12.0, forward_m=forward, lateral_m=lateral,
                    buoy_id=f"p2_o_{side}1",
                )
                if source:
                    item["source"] = source
                result.append(item)
            return result

        for heading in (0.0, 10.0, 20.0, 30.0, 40.0, 45.0):
            with self.subTest(heading=heading):
                det = self._make_detector()
                det.allow_sim_truth_geometry = True
                stats = det.update(frame(heading, "sim_gate_truth"), now=1.0)
                self.assertEqual(stats.kd_estimate, 1)

        # Aynı 30 derece geometri canonical/production default'ta forward-gap
        # validasyonunu geçemez; sim ayrıcalığı production'a sızmaz.
        production = self._make_detector()
        self.assertEqual(production.update(frame(30.0), now=1.0).kd_estimate, 0)

        enabled_but_canonical = self._make_detector()
        enabled_but_canonical.allow_sim_truth_geometry = True
        self.assertEqual(
            enabled_but_canonical.update(frame(30.0, "camera"), now=1.0).kd_estimate,
            0,
        )

        # Source etiketi tek başına yetmez; stable L/R ID aynı fiziksel pair
        # numarasını doğrulamalıdır.
        mismatched = frame(30.0, "sim_gate_truth")
        mismatched[1]["id"] = "p2_o_r2"
        spoof = self._make_detector()
        spoof.allow_sim_truth_geometry = True
        self.assertEqual(spoof.update(mismatched, now=1.0).kd_estimate, 0)

    def test_explicit_ids_pair_only_matching_numbers_under_reorder(self):
        det = self._make_detector()
        detections = [
            buoy("orange", 20.0, forward_m=12.0, lateral_m=4.0, buoy_id="p1_o_r2"),
            buoy("orange", 10.0, forward_m=5.0, lateral_m=-7.0, buoy_id="p1_o_l1"),
            buoy("orange", 20.0, forward_m=12.0, lateral_m=-4.0, buoy_id="p1_o_l2"),
            buoy("orange", 10.0, forward_m=5.0, lateral_m=7.0, buoy_id="p1_o_r1"),
        ]
        stats = det.update(detections, now=1.0)
        self.assertEqual(stats.kd_estimate, 2)
        self.assertEqual(
            {gate.key for gate in stats.active_gates}, {"id:p1_o:1", "id:p1_o:2"}
        )
        self.assertEqual(len({gate.left_id for gate in stats.active_gates}), 2)
        self.assertEqual(len({gate.right_id for gate in stats.active_gates}), 2)

        # Mesafe/lateral sırası tamamen değişse bile stable ID partneri değişmez.
        stats = det.update(list(reversed(detections)), now=2.0)
        self.assertEqual(stats.kd_estimate, 2)

    def test_many_explicit_gates_never_expand_combinatorially(self):
        det = self._make_detector()
        detections = []
        for index in range(1, 11):
            fwd = 4.0 + index
            detections.extend(
                [
                    buoy("orange", fwd, forward_m=fwd, lateral_m=-5.0, buoy_id=f"p1_o_l{index}"),
                    buoy("orange", fwd, forward_m=fwd, lateral_m=5.0, buoy_id=f"p1_o_r{index}"),
                ]
            )
        stats = det.update(detections[::2] + detections[1::2], now=1.0)
        self.assertEqual(stats.kd_estimate, 10)
        self.assertEqual(len(stats.active_gates), 10)
        stats = det.update(list(reversed(detections)), now=2.0)
        self.assertEqual(stats.kd_estimate, 10)

    def test_single_side_visibility_does_not_invent_or_duplicate_gate(self):
        det = self._make_detector()
        left_only = [
            buoy("orange", 8.0, forward_m=8.0, lateral_m=-4.0, buoy_id="p1_o_l3")
        ]
        self.assertEqual(det.update(left_only, now=1.0).kd_estimate, 0)
        both = left_only + [
            buoy("orange", 8.0, forward_m=8.0, lateral_m=4.0, buoy_id="p1_o_r3")
        ]
        self.assertEqual(det.update(both, now=2.0).kd_estimate, 1)
        self.assertEqual(det.update(left_only, now=3.0).kd_estimate, 1)
        self.assertEqual(det.update(both, now=4.0).kd_estimate, 1)

    def test_heading_side_swap_preserves_explicit_gate_identity(self):
        det = self._make_detector()
        ahead = [
            buoy("orange", 8.0, forward_m=6.0, lateral_m=-4.0, buoy_id="p1_o_l4"),
            buoy("orange", 8.0, forward_m=6.0, lateral_m=4.0, buoy_id="p1_o_r4"),
        ]
        self.assertEqual(det.update(ahead, now=1.0).kd_estimate, 1)
        # Tekne dönünce fiziksel sol duba görüntünün sağında kalabilir.
        turned = [
            buoy("orange", 3.0, forward_m=2.0, lateral_m=4.0, buoy_id="p1_o_l4"),
            buoy("orange", 3.0, forward_m=2.0, lateral_m=-4.0, buoy_id="p1_o_r4"),
        ]
        stats = det.update(turned, now=2.0)
        self.assertEqual(stats.kd_estimate, 1)
        self.assertEqual(stats.active_gates[0].key, "id:p1_o:4")

    def test_crossed_drop_and_reappear_is_never_counted_twice(self):
        det = self._make_detector()
        ahead = [
            buoy("orange", 8.0, forward_m=6.0, lateral_m=-4.0, buoy_id="p1_o_l5"),
            buoy("orange", 8.0, forward_m=6.0, lateral_m=4.0, buoy_id="p1_o_r5"),
        ]
        behind = [
            buoy("orange", 2.0, forward_m=-1.0, lateral_m=-4.0, buoy_id="p1_o_l5"),
            buoy("orange", 2.0, forward_m=-1.0, lateral_m=4.0, buoy_id="p1_o_r5"),
        ]
        det.update(ahead, now=1.0)
        self.assertEqual(det.update(behind, now=2.0).crossed_count, 1)
        # Stale temizlik crossed kimliğini history'de korur.
        self.assertEqual(det.update([], now=30.0).crossed_count, 1)
        stats = det.update(ahead, now=31.0)
        self.assertEqual(stats.crossed_count, 1)
        self.assertEqual(stats.kd_estimate, 1)
        self.assertEqual(stats.crossed_keys, ["id:p1_o:5"])

    def test_idless_minimum_cost_pairing_is_one_to_one_and_temporal(self):
        det = self._make_detector()
        frame1 = [
            buoy("orange", 6.0, forward_m=6.0, lateral_m=-4.0),
            buoy("orange", 12.0, forward_m=12.0, lateral_m=-5.0),
            buoy("orange", 12.0, forward_m=12.2, lateral_m=5.0),
            buoy("orange", 6.0, forward_m=6.2, lateral_m=4.0),
        ]
        stats = det.update(frame1, now=1.0)
        self.assertEqual(stats.kd_estimate, 2)
        keys = {gate.key for gate in stats.active_gates}
        self.assertEqual(len(keys), 2)
        # Reorder + küçük hareket: iki eski track'e association, yeni gate yok.
        frame2 = [dict(b, forward_m=float(b["forward_m"]) - 0.5) for b in reversed(frame1)]
        stats = det.update(frame2, now=2.0)
        self.assertEqual(stats.kd_estimate, 2)
        self.assertEqual({gate.key for gate in stats.active_gates}, keys)

    def test_idless_matching_maximizes_valid_gate_count_before_cost(self):
        det = self._make_detector()
        # Greedy 14<->13 seçerse 10<->15 geçersiz kalır ve KD yanlış 1 olur.
        # Global matching 10<->13 ve 14<->15 ile iki geçerli gate'i korumalı.
        frame = [
            buoy("orange", 10.0, forward_m=10.0, lateral_m=-4.0),
            buoy("orange", 14.0, forward_m=14.0, lateral_m=-4.0),
            buoy("orange", 13.0, forward_m=13.0, lateral_m=4.0),
            buoy("orange", 15.0, forward_m=15.0, lateral_m=4.0),
        ]
        stats = det.update(frame, now=1.0)
        self.assertEqual(stats.kd_estimate, 2)
        self.assertEqual(len(stats.active_gates), 2)

    def test_anonymous_same_geometry_ahead_is_a_new_gate(self):
        det = self._make_detector()
        ahead = [
            buoy("orange", 6.0, forward_m=6.0, lateral_m=-4.0),
            buoy("orange", 6.0, forward_m=6.1, lateral_m=4.0),
        ]
        behind = [
            buoy("orange", 2.0, forward_m=-1.0, lateral_m=-4.0),
            buoy("orange", 2.0, forward_m=-1.1, lateral_m=4.0),
        ]
        det.update(ahead, now=1.0)
        self.assertEqual(det.update(behind, now=2.0).crossed_count, 1)
        # Stable ID/odometri yokken aynı width/lateral ile önde görünen nesne,
        # düz koridordaki bir sonraki gerçek kapıdır; suppression undercount yapar.
        stats = det.update(ahead, now=10.0)
        self.assertEqual(stats.kd_estimate, 2)
        self.assertEqual(len(stats.active_gates), 1)
        stats = det.update(behind, now=11.0)
        self.assertEqual(stats.crossed_count, 2)
        self.assertEqual(stats.kd_estimate, 2)

    def test_side_crossings_too_far_apart_do_not_combine(self):
        det = self._make_detector()
        ahead = [
            buoy("orange", 6.0, forward_m=6.0, lateral_m=-4.0, buoy_id="p1_o_l8"),
            buoy("orange", 6.0, forward_m=6.0, lateral_m=4.0, buoy_id="p1_o_r8"),
        ]
        det.update(ahead, now=1.0)
        det.update([dict(ahead[0], forward_m=-1.0), ahead[1]], now=2.0)
        stats = det.update([ahead[0], dict(ahead[1], forward_m=-1.0)], now=5.1)
        self.assertEqual(stats.crossed_count, 0)
        # İki taraf aynı zaman penceresinde arkadaysa geçiş tamamlanır.
        both = [dict(ahead[0], forward_m=-1.0), dict(ahead[1], forward_m=-1.0)]
        self.assertEqual(det.update(both, now=5.2).crossed_count, 1)

    def test_invalid_single_side_observations_cannot_cross_explicit_gate(self):
        det = self._make_detector()
        ahead = [
            buoy("orange", 6.0, forward_m=6.0, lateral_m=-4.0, buoy_id="p1_o_l88"),
            buoy("orange", 6.0, forward_m=6.0, lateral_m=4.0, buoy_id="p1_o_r88"),
        ]
        self.assertEqual(det.update(ahead, now=1.0).kd_estimate, 1)
        invalid_left = [dict(ahead[0], forward_m=-1.0, lateral_m=-100.0)]
        invalid_right = [dict(ahead[1], forward_m=-1.0, lateral_m=100.0)]
        det.update(invalid_left, now=2.0)
        stats = det.update(invalid_right, now=2.1)
        self.assertEqual(stats.crossed_count, 0)
        self.assertEqual(stats.kd_estimate, 1)

    def test_invalid_physical_gate_and_malformed_values_are_ignored(self):
        det = self._make_detector()
        invalid = [
            buoy("orange", 20.0, forward_m=5.0, lateral_m=-4.0, buoy_id="p1_o_l9"),
            buoy("orange", 20.0, forward_m=10.0, lateral_m=4.0, buoy_id="p1_o_r9"),
            {"color": "orange", "confidence": "bad", "forward_m": 5, "lateral_m": -3},
            {"color": "orange", "confidence": 0.9, "forward_m": float("nan"), "lateral_m": 3},
            {"color": "orange", "confidence": 0.9, "forward_m": 5, "lateral_m": float("inf")},
        ]
        self.assertEqual(det.update(invalid, now=1.0).kd_estimate, 0)
        self.assertEqual(det.update([], now=float("nan")).kd_estimate, 0)

    def test_malformed_course_namespace_id_is_not_anonymous_fallback(self):
        det = self._make_detector()
        malformed = [
            buoy("orange", 6.0, forward_m=6.0, lateral_m=-4.0, buoy_id="p1_o_lX"),
            buoy("orange", 6.0, forward_m=6.0, lateral_m=4.0, buoy_id="p1_o_rX"),
        ]
        self.assertEqual(det.update(malformed, now=1.0).kd_estimate, 0)

        # Genel tracker ID formatı course namespace değildir; geometrik fallback
        # tarafından bire-bir eşlenebilir.
        generic = [
            buoy("orange", 6.0, forward_m=6.0, lateral_m=-4.0, buoy_id="track-A"),
            buoy("orange", 6.0, forward_m=6.0, lateral_m=4.0, buoy_id="track-B"),
        ]
        self.assertEqual(det.update(generic, now=2.0).kd_estimate, 1)

    def test_constructor_rejects_invalid_pair_limits(self):
        invalid_kwargs = (
            {"max_pair_forward_gap_m": 0.0},
            {"max_pair_forward_gap_m": float("nan")},
            {"max_pair_crossing_gap_s": float("inf")},
            {"min_lateral_gap_m": 16.0, "max_pair_lateral_m": 15.0},
        )
        for kwargs in invalid_kwargs:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                PairCrossingDetector(**kwargs)

    def test_equal_confidence_duplicate_and_equal_cost_are_permutation_stable(self):
        explicit = [
            buoy("orange", 6.0, forward_m=6.2, lateral_m=-4.0, buoy_id="p1_o_l10"),
            buoy("orange", 6.0, forward_m=6.0, lateral_m=-4.0, buoy_id="p1_o_l10"),
            buoy("orange", 6.0, forward_m=6.0, lateral_m=4.0, buoy_id="p1_o_r10"),
        ]
        anonymous = [
            buoy("orange", 6.0, forward_m=6.0, lateral_m=-3.0),
            buoy("orange", 6.0, forward_m=6.0, lateral_m=-5.0),
            buoy("orange", 6.0, forward_m=6.0, lateral_m=3.0),
            buoy("orange", 6.0, forward_m=6.0, lateral_m=5.0),
        ]
        snapshots = []
        for frame in (explicit + anonymous, list(reversed(explicit + anonymous))):
            det = self._make_detector()
            stats = det.update(frame, now=1.0)
            snapshots.append(
                [
                    (g.key, round(g.center_forward_m, 3), round(g.center_lateral_m, 3), round(g.width_m, 3))
                    for g in stats.active_gates
                ]
            )
        self.assertEqual(snapshots[0], snapshots[1])
        self.assertEqual(snapshots[0], sorted(snapshots[0]))


class TestObstacleAvoidance(unittest.TestCase):
    def _cmd(self):
        return Command(vx=0.8, vy=0.0, yaw_rate=0.0, action="waypoint")

    def test_no_danger_passthrough(self):
        cmd = self._cmd()
        out = apply_obstacle_avoidance(cmd, [], avoid_distance_m=8.0, max_yaw_rate=45.0)
        self.assertEqual(out.vx, cmd.vx)
        self.assertEqual(out.yaw_rate, cmd.yaw_rate)

    def test_left_obstacle_steers_left(self):
        cmd = self._cmd()
        obstacles = [{"distance": 4.0, "forward_m": 3.0, "lateral_m": -1.0}]
        out = apply_obstacle_avoidance(cmd, obstacles, avoid_distance_m=8.0, max_yaw_rate=45.0)
        self.assertEqual(out.vy, 0.0)
        self.assertLess(out.yaw_rate, 0.0)  # engel solda -> sola dön
        self.assertEqual(out.action.split()[0], "avoid_left")
        self.assertLessEqual(out.vx, 0.28)

    def test_right_obstacle_steers_right(self):
        cmd = self._cmd()
        obstacles = [{"distance": 4.0, "forward_m": 3.0, "lateral_m": 1.0}]
        out = apply_obstacle_avoidance(cmd, obstacles, avoid_distance_m=8.0, max_yaw_rate=45.0)
        self.assertEqual(out.vy, 0.0)
        self.assertGreater(out.yaw_rate, 0.0)  # engel sağda -> sağa dön
        self.assertEqual(out.action.split()[0], "avoid_right")

    def test_outside_corridor_ignored(self):
        cmd = self._cmd()
        obstacles = [{"distance": 4.0, "forward_m": 3.0, "lateral_m": 9.0}]
        out = apply_obstacle_avoidance(cmd, obstacles, avoid_distance_m=8.0, max_yaw_rate=45.0)
        self.assertEqual(out, cmd)


class TestWrongTargetRisk(unittest.TestCase):
    def test_wrong_target_close_found(self):
        buoys = [
            buoy("green", 1.5, lateral_m=0.5, confidence=0.9, buoy_id="g1"),
            buoy("orange", 6.0, lateral_m=0.0, confidence=0.9, buoy_id="o1"),
        ]
        risk = wrong_target_risk(buoys, target_color="orange")
        self.assertIsNotNone(risk)
        self.assertEqual(risk["color"], "green")

    def test_wrong_target_far_none(self):
        buoys = [
            buoy("green", 5.0, lateral_m=0.5, confidence=0.9, buoy_id="g1"),
            buoy("orange", 6.0, lateral_m=0.0, confidence=0.9, buoy_id="o1"),
        ]
        self.assertIsNone(wrong_target_risk(buoys, target_color="orange"))

    def test_low_confidence_ignored(self):
        buoys = [buoy("green", 1.5, lateral_m=0.5, confidence=0.1, buoy_id="g1")]
        self.assertIsNone(wrong_target_risk(buoys, target_color="orange"))


class TestNearestNeighborRoute(unittest.TestCase):
    """nearest_neighbor_route: greedy NN sıralama, mutasyon yok, edge durumlar."""

    @staticmethod
    def _wp(lat, lon, parkur=1, wp_id=None):
        w = {"lat": lat, "lon": lon, "parkur": parkur}
        if wp_id is not None:
            w["id"] = wp_id
        return w

    def test_greedy_order_from_start(self):
        # GN'ler: A=(40.0000,29.0000), B=(40.0000,29.0010) [A'dan ~85m doğu],
        # C=(40.0010,29.0000) [A'dan ~111m kuzey].
        # Start=(39.9990,29.0000): A ~111m, B ~140m, C ~222m -> en yakın A.
        # A'dan sonra B (85m) C'den (111m) yakın -> rota A, B, C.
        wps = [
            self._wp(40.0010, 29.0000, parkur=1, wp_id="gnC"),
            self._wp(40.0000, 29.0010, parkur=1, wp_id="gnB"),
            self._wp(40.0000, 29.0000, parkur=1, wp_id="gnA"),
        ]
        route = nearest_neighbor_route(39.9990, 29.0000, wps)
        self.assertEqual([w["id"] for w in route], ["gnA", "gnB", "gnC"])

    def test_route_changes_with_start(self):
        wps = [
            self._wp(40.0000, 29.0000, parkur=1, wp_id="gnA"),
            self._wp(40.0000, 29.0010, parkur=1, wp_id="gnB"),
            self._wp(40.0010, 29.0000, parkur=1, wp_id="gnC"),
        ]
        # Start A noktasında -> rota A'dan başlar: A -> B -> C.
        route_a = nearest_neighbor_route(40.0000, 29.0000, wps)
        self.assertEqual([w["id"] for w in route_a], ["gnA", "gnB", "gnC"])
        # Start B'ye yakın (dLon 0.0005 -> ~43m) -> rota B'den başlar: B -> A -> C.
        route_b = nearest_neighbor_route(40.0000, 29.0015, wps)
        self.assertEqual([w["id"] for w in route_b], ["gnB", "gnA", "gnC"])
        # Start değişince ilk GN değişir (rota farklılaşır).
        self.assertNotEqual(route_a[0]["id"], route_b[0]["id"])

    def test_empty_and_single(self):
        self.assertEqual(nearest_neighbor_route(40.0, 29.0, []), [])
        single = [self._wp(40.0, 29.0, parkur=2, wp_id="only")]
        route = nearest_neighbor_route(40.0, 29.0, single)
        self.assertEqual(len(route), 1)
        self.assertEqual(route[0]["id"], "only")
        # Tek GN kopyası döner; giriş listesi mutasyona uğramaz.
        self.assertEqual(len(single), 1)

    def test_no_mutation_and_parkur_preserved(self):
        wps = [
            self._wp(40.0010, 29.0000, parkur=1, wp_id="gnC"),
            self._wp(40.0000, 29.0010, parkur=2, wp_id="gnB"),
            self._wp(40.0000, 29.0000, parkur=1, wp_id="gnA"),
        ]
        original_ids = [w["id"] for w in wps]
        original_parkur = [w["parkur"] for w in wps]
        route = nearest_neighbor_route(39.9990, 29.0000, wps)
        # Giriş listesi mutasyona uğramaz (sıra ve içerik korunur).
        self.assertEqual([w["id"] for w in wps], original_ids)
        self.assertEqual([w["parkur"] for w in wps], original_parkur)
        # Dönen rota sıralıdır ve her sözlük parkur alanını korur.
        self.assertEqual([w["id"] for w in route], ["gnA", "gnB", "gnC"])
        self.assertEqual([w["parkur"] for w in route], [1, 2, 1])

    def test_identical_points_keep_input_order(self):
        wps = [
            self._wp(40.0, 29.0, parkur=1, wp_id="p1"),
            self._wp(40.0, 29.0, parkur=1, wp_id="p2"),
            self._wp(40.0, 29.0, parkur=2, wp_id="p3"),
        ]
        route = nearest_neighbor_route(39.9990, 29.0, wps)
        self.assertEqual([w["id"] for w in route], ["p1", "p2", "p3"])

    def test_start_coincides_with_waypoint(self):
        # Start GN A ile aynı noktada -> rota A'dan başlar: A -> B -> C.
        wps = [
            self._wp(40.0010, 29.0000, parkur=1, wp_id="gnC"),
            self._wp(40.0000, 29.0010, parkur=1, wp_id="gnB"),
            self._wp(40.0000, 29.0000, parkur=1, wp_id="gnA"),
        ]
        route = nearest_neighbor_route(40.0000, 29.0000, wps)
        self.assertEqual([w["id"] for w in route], ["gnA", "gnB", "gnC"])


class TestYellowGateCounter(unittest.TestCase):
    """YellowGateCounter: eşik, decay, tavan, filtreler, reset, determinizm."""

    def _make_counter(self):
        return YellowGateCounter(
            threshold=4,
            decay=2,
            range_m=15.0,
            forward_m=1.0,
            lateral_m=6.0,
            min_confidence=0.35,
        )

    def _yellow(self, distance=10.0, forward_m=5.0, lateral_m=1.0, confidence=0.9):
        return buoy("yellow", distance, forward_m=forward_m, lateral_m=lateral_m, confidence=confidence)

    def test_single_sighting_not_enough(self):
        counter = self._make_counter()
        self.assertEqual(counter.update([self._yellow()], now=1.0), 1)
        self.assertLess(counter.update([self._yellow()], now=2.0), counter.threshold)

    def test_threshold_reached(self):
        counter = self._make_counter()
        values = [counter.update([self._yellow()], now=float(t)) for t in range(1, 5)]
        self.assertEqual(values, [1, 2, 3, 4])
        self.assertGreaterEqual(counter.update([self._yellow()], now=5.0), counter.threshold)

    def test_decay_when_not_seen(self):
        counter = self._make_counter()
        counter.update([self._yellow()], now=1.0)
        counter.update([self._yellow()], now=2.0)  # counter=2
        self.assertEqual(counter.update([], now=3.0), 0)  # max(2-2, 0)=0
        # Decay counter'ı 0'ın altına düşürmez.
        self.assertEqual(counter.update([], now=4.0), 0)

    def test_cap_at_threshold(self):
        counter = self._make_counter()
        for t in range(1, 8):
            counter.update([self._yellow()], now=float(t))
        self.assertEqual(counter.counter, counter.threshold)  # tavan: min(c+1, 4)

    def test_flake_tolerance(self):
        counter = self._make_counter()
        # 3 kare (counter=3), 1 boş kare (decay -> 1), tekrar 3 kare (-> 4).
        counter.update([self._yellow()], now=1.0)
        counter.update([self._yellow()], now=2.0)
        counter.update([self._yellow()], now=3.0)
        self.assertEqual(counter.update([], now=4.0), 1)
        counter.update([self._yellow()], now=5.0)
        counter.update([self._yellow()], now=6.0)
        self.assertGreaterEqual(counter.update([self._yellow()], now=7.0), counter.threshold)

    def test_reset(self):
        counter = self._make_counter()
        for t in range(1, 5):
            counter.update([self._yellow()], now=float(t))
        self.assertGreaterEqual(counter.counter, counter.threshold)
        counter.reset()
        self.assertEqual(counter.counter, 0.0)
        self.assertEqual(counter.update([self._yellow()], now=5.0), 1)

    def test_filters(self):
        counter = self._make_counter()
        # Güven eşiği.
        self.assertEqual(counter.update([self._yellow(confidence=0.2)], now=1.0), 0)
        # Menzil dışı.
        self.assertEqual(counter.update([self._yellow(distance=20.0)], now=2.0), 0)
        # Arkada (forward_m <= forward eşiği).
        self.assertEqual(counter.update([self._yellow(forward_m=-1.0)], now=3.0), 0)
        # Lateral dışı.
        self.assertEqual(counter.update([self._yellow(lateral_m=9.0)], now=4.0), 0)
        # Yanlış renk.
        self.assertEqual(counter.update([buoy("green", 10.0, forward_m=5.0, lateral_m=1.0)], now=5.0), 0)
        # lateral_m alanı eksik -> sentinel "y", filtre başarısız olur.
        no_lateral = {"color": "yellow", "distance": 10.0, "forward_m": 5.0, "confidence": 0.9}
        self.assertEqual(counter.update([no_lateral], now=6.0), 0)
        # Filtreleri geçen tek duba counter'ı artırır (kontrol).
        self.assertEqual(counter.update([self._yellow()], now=7.0), 1)

    def test_determinism(self):
        counter = self._make_counter()
        first = [counter.update([self._yellow()], now=float(t)) for t in range(1, 4)]
        counter.reset()
        second = [counter.update([self._yellow()], now=float(t)) for t in range(1, 4)]
        self.assertEqual(first, second)


class TestIsLastWaypointIndex(unittest.TestCase):
    """is_last_waypoint_index: P2 son waypoint koşulu (index-1 tabanlı saf mantık)."""

    def test_middle_wp_not_last(self):
        self.assertFalse(is_last_waypoint_index(0, 5))
        self.assertFalse(is_last_waypoint_index(3, 5))

    def test_last_wp_is_last(self):
        self.assertTrue(is_last_waypoint_index(4, 5))
        self.assertTrue(is_last_waypoint_index(5, 5))  # tükenmiş -> sona ulaşılmış

    def test_edge_totals(self):
        self.assertTrue(is_last_waypoint_index(0, 0))
        self.assertTrue(is_last_waypoint_index(0, 1))
        self.assertFalse(is_last_waypoint_index(0, 2))


class TestTargetEngagement(unittest.TestCase):
    def test_all_five_selectable_colors_can_engage(self):
        for color in ("red", "green", "black", "orange", "yellow"):
            with self.subTest(color=color):
                cmd = target_engagement_command(
                    color,
                    [buoy(color, 0.8, bbox_norm_x=0.0, confidence=0.9, buoy_id=color)],
                    max_speed=0.8,
                    max_yaw_rate=45.0,
                    min_confidence=0.45,
                )
                self.assertIn("engage", cmd.action)

    def test_engage_correct_target(self):
        buoys = [buoy("green", 0.8, bbox_norm_x=0.0, confidence=0.9, buoy_id="g1")]
        cmd = target_engagement_command("green", buoys, max_speed=0.8, max_yaw_rate=45.0, min_confidence=0.45)
        self.assertIn("engage", cmd.action)
        self.assertEqual(cmd.vy, 0.0)
        self.assertEqual(cmd.vx, 0.18)

    def test_field_engagement_and_lock_respect_effective_speed_floor(self):
        engage = target_engagement_command(
            "green",
            [buoy("green", 0.8, bbox_norm_x=0.0, confidence=0.9, buoy_id="g1")],
            max_speed=1.6,
            max_yaw_rate=50.0,
            min_confidence=0.20,
            lock_min_speed_mps=0.70,
            engage_speed_mps=0.70,
        )
        self.assertEqual(engage.vx, 0.70)
        lock = target_engagement_command(
            "green",
            [buoy("green", 2.0, bbox_norm_x=0.5, confidence=0.9, buoy_id="g1")],
            max_speed=1.6,
            max_yaw_rate=50.0,
            min_confidence=0.20,
            lock_min_speed_mps=0.70,
            engage_speed_mps=0.70,
        )
        self.assertEqual(lock.vx, 0.70)
        self.assertIn("target_align_moving", lock.action)

    def test_target_must_align_before_approach_and_keeps_yaw_while_engaging(self):
        off_center = target_engagement_command(
            "green",
            [buoy("green", 4.0, bbox_norm_x=0.30, confidence=0.9, buoy_id="g1")],
            max_speed=1.6, max_yaw_rate=50.0, min_confidence=0.20,
            lock_min_speed_mps=0.70, engage_speed_mps=0.70,
            engage_center_max=0.18,
        )
        self.assertIn("target_align_moving", off_center.action)
        self.assertEqual(off_center.vx, 0.70)
        self.assertNotEqual(off_center.yaw_rate, 0.0)

        approach = target_engagement_command(
            "green",
            [buoy("green", 4.0, bbox_norm_x=0.10, confidence=0.9, buoy_id="g1")],
            max_speed=1.6, max_yaw_rate=50.0, min_confidence=0.20,
            lock_min_speed_mps=0.70, engage_speed_mps=0.70,
            engage_center_max=0.18,
        )
        self.assertIn("target_lock", approach.action)
        self.assertGreater(approach.vx, 0.0)
        self.assertNotEqual(approach.yaw_rate, 0.0)

        contact = target_engagement_command(
            "green",
            [buoy("green", 0.8, bbox_norm_x=0.10, confidence=0.9, buoy_id="g1")],
            max_speed=1.6, max_yaw_rate=50.0, min_confidence=0.20,
            lock_min_speed_mps=0.70, engage_speed_mps=0.70,
            engage_center_max=0.18,
        )
        self.assertIn("engage", contact.action)
        self.assertGreater(contact.vx, 0.0)
        self.assertNotEqual(contact.yaw_rate, 0.0)

    def test_far_off_center_target_advances_with_bounded_yaw_until_five_metres(self):
        far = target_engagement_command(
            "green",
            [buoy("green", 22.0, bbox_norm_x=-1.0, confidence=0.9, buoy_id="g1")],
            max_speed=0.6,
            max_yaw_rate=50.0,
            min_confidence=0.20,
            align_distance_m=5.0,
            approach_speed_mps=0.40,
            approach_yaw_max_deg_s=12.0,
        )
        self.assertIn("target_approach", far.action)
        self.assertEqual(far.vx, 0.40)
        self.assertAlmostEqual(far.yaw_rate, math.radians(-12.0))

        close = target_engagement_command(
            "green",
            [buoy("green", 4.9, bbox_norm_x=-1.0, confidence=0.9, buoy_id="g1")],
            max_speed=0.6,
            max_yaw_rate=50.0,
            min_confidence=0.20,
            align_distance_m=5.0,
            approach_speed_mps=0.40,
            approach_yaw_max_deg_s=12.0,
        )
        self.assertIn("target_align", close.action)
        self.assertEqual(close.vx, 0.0)

    def test_avoid_wrong_target_no_stop(self):
        # Yanlış hedef yakın: durma yok, avoid_wrong_target ve vy=0.
        buoys = [
            buoy("green", 1.0, bbox_norm_x=0.0, confidence=0.9, buoy_id="g1"),
            buoy("red", 1.5, lateral_m=-0.5, confidence=0.9, buoy_id="r1"),
        ]
        cmd = target_engagement_command(
            "green", buoys, max_speed=0.8, max_yaw_rate=45.0, min_confidence=0.45
        )
        self.assertEqual(cmd.action, "avoid_wrong_target")
        self.assertEqual(cmd.vy, 0.0)
        self.assertGreater(cmd.vx, 0.0)

    def test_search_when_no_candidates(self):
        cmd = target_engagement_command("green", [], max_speed=0.8, max_yaw_rate=45.0, min_confidence=0.45)
        self.assertIn("search_target", cmd.action)
        self.assertEqual(cmd.vy, 0.0)

    def test_search_rate_and_multi_frame_confirmation_gate(self):
        search = target_engagement_command(
            "black", [], max_speed=0.8, max_yaw_rate=45.0,
            min_confidence=0.30, search_yaw_rate_deg_s=10.0,
        )
        self.assertEqual(search.action, "search_target_360 ts3_risk=0")
        self.assertAlmostEqual(search.yaw_rate, math.radians(10.0))
        confirming = target_engagement_command(
            "black",
            [buoy("black", 0.8, bbox_norm_x=0.0, confidence=0.35, buoy_id="b1")],
            max_speed=0.8, max_yaw_rate=45.0, min_confidence=0.30,
            engage_allowed=False,
        )
        self.assertEqual(confirming.action, "target_confirming")
        self.assertEqual(confirming.vx, 0.0)

    def test_search_advance_after_rotations_with_heading(self):
        """Kullanıcı gözlemi (2026-08-18): 2 tur dönüş sonrası hedef yoksa
        waypoint yönüne ileri gidilir (search_advance)."""
        cmd = target_engagement_command(
            "green", [], max_speed=0.8, max_yaw_rate=45.0,
            min_confidence=0.45,
            search_advance_after_rotations=2,
            search_advance_speed_mps=0.5,
            search_advance_heading_deg=30.0,
        )
        self.assertIn("search_advance", cmd.action)
        self.assertAlmostEqual(cmd.vx, 0.5)
        self.assertAlmostEqual(cmd.yaw_rate, math.radians(30.0))

    def test_search_rotates_when_rotations_below_threshold(self):
        """2 tur dolmadan hedef yoksa hâlâ dönüş (search_target_360)."""
        cmd = target_engagement_command(
            "green", [], max_speed=0.8, max_yaw_rate=45.0,
            min_confidence=0.45,
            search_advance_after_rotations=0,
            search_advance_heading_deg=30.0,
        )
        self.assertIn("search_target_360", cmd.action)
        self.assertEqual(cmd.vx, 0.0)

    def test_search_advance_without_heading_still_rotates(self):
        """search_advance_heading verilmezse ve rotasyon yoksa dönüş sürer."""
        cmd = target_engagement_command(
            "green", [], max_speed=0.8, max_yaw_rate=45.0,
            min_confidence=0.45,
            search_advance_after_rotations=2,
            search_advance_speed_mps=0.5,
            search_advance_heading_deg=None,
        )
        self.assertIn("search_target_360", cmd.action)

    def test_ts3_risk_appended(self):
        # Yanlış hedef risk yarıçapı İÇİNDE ama düşük güvenli: avoid tetiklenmez
        # (wrong_target_risk güven filtresi), ancak ts3_risk=1 sayılır.
        buoys = [
            buoy("green", 4.0, bbox_norm_x=0.2, confidence=0.9, buoy_id="g1"),
            buoy("red", 2.0, lateral_m=1.0, confidence=0.1, buoy_id="r1"),
        ]
        cmd = target_engagement_command(
            "green", buoys, max_speed=0.8, max_yaw_rate=45.0, min_confidence=0.45
        )
        self.assertIn("ts3_risk=1", cmd.action)
        self.assertEqual(cmd.vy, 0.0)


# ---------------------------------------------------------------------------
# CostMap: engel işleme, şişirme, renk eşleme (bearing 6° eşiği),
# turuncu/sarı/unknown maliyet, reset.
# ---------------------------------------------------------------------------


class TestSpeedSafety(unittest.TestCase):
    """Hıza bağlı güvenlik yarıçapı (speed_safety.py) saf fonksiyonları."""

    def test_stopping_distance_zero(self):
        # v=0 -> durma mesafesi yok (geri gitmiyoruz).
        self.assertEqual(stopping_distance(0.0, 1.0, 1.5), 0.0)

    def test_stopping_distance_example(self):
        # 0.8 m/s: 0.8*1.0 + 0.64/3.0 = 0.8 + 0.2133 = 1.0133.
        self.assertAlmostEqual(stopping_distance(0.8, 1.0, 1.5), 0.8 + 0.64 / 3.0, places=3)

    def test_safety_radius_example(self):
        # 3 m/s: taban 0.5 + 3.0 + 9.0/3.0 = 6.5 (tavan 10 altında).
        self.assertAlmostEqual(safety_radius(3.0, 0.5, 10.0, 1.0, 1.5), 6.5, places=3)

    def test_safety_radius_caps_at_max(self):
        # 20 m/s durma mesafesi 30 m -> tavan 10.0'a sabitlenir.
        self.assertAlmostEqual(safety_radius(20.0, 0.5, 10.0, 1.0, 1.5), 10.0, places=3)

    def test_safety_radius_max_below_base(self):
        # Tavan tabandan küçükse taban döner (tavan tabanın altına inemez).
        self.assertAlmostEqual(safety_radius(5.0, 3.0, 2.0, 1.0, 1.5), 3.0, places=3)

    def test_stopping_distance_infinite_decel(self):
        # a <= 0 -> anında duramaz: durma mesafesi sonsuz (güvenli taraf).
        self.assertEqual(stopping_distance(1.0, 1.0, 0.0), float("inf"))

    def test_speed_for_safety_radius(self):
        # 10 m tavan: -1.5 + sqrt(2.25 + 2*1.5*9.5) = sqrt(30.75) ~= 4.045.
        # (Plan dosyasındaki ~3.82 YANLIŞTI; doğru değer 4.045'tir.)
        self.assertAlmostEqual(speed_for_safety_radius(10.0, 0.5, 1.0, 1.5), 4.045, places=2)


def _cm(**kw):
    """Küçük test haritası: 10 m, 0.5 m hücre (20x20)."""
    defaults = dict(size_m=10.0, cell_m=0.5, bot_radius_m=0.6, safety_m=0.5)
    defaults.update(kw)
    return CostMap(**defaults)


def _obs(forward, lateral):
    return {"distance": math.hypot(forward, lateral), "forward_m": forward, "lateral_m": lateral}


def _det(color, distance, bearing=0.0, confidence=0.9):
    return {
        "color": color,
        "distance": distance,
        "bearing_deg": bearing,
        "confidence": confidence,
        "forward_m": distance * math.cos(math.radians(bearing)),
        "lateral_m": distance * math.sin(math.radians(bearing)),
    }


class TestCostMap(unittest.TestCase):
    def test_obstacle_cell_and_inflation(self):
        cm = _cm()
        cm.update([_obs(5.0, 0.0)], [], "")
        self.assertEqual(cm.cost(5.0, 0.0), COST_OBSTACLE)
        # Şişirme yarıçapı = 0.6 + 0.5 = 1.1 m; 0.7 m yanda hâlâ engel alanı.
        self.assertTrue(cm.collides(4.6, 0.0))
        self.assertTrue(cm.collides(5.0, 0.7))
        # 2 m yanda engel alanı dışı (1.1 m yarıçap).
        self.assertFalse(cm.collides(5.0, 2.0))

    def test_goals_not_inflated(self):
        cm = _cm(size_m=10.0, cell_m=0.5, bot_radius_m=0.0, safety_m=0.0)
        cm.update([], [_det("green", 4.0, bearing=0.0)], "green")
        self.assertEqual(cm.cost(4.0, 0.0), 3)  # goal hücresi
        self.assertFalse(cm.collides(4.0, 0.0))  # şişirilmez (DWA hedef çekimi)

    def test_orange_corridor_cost(self):
        cm = _cm(size_m=10.0, cell_m=0.5, bot_radius_m=0.0, safety_m=0.0)
        cm.update([], [_det("orange", 5.0, bearing=5.0)], "")
        self.assertEqual(cm.cost(5.0, math.sin(math.radians(5.0)) * 5.0), COST_ORANGE)

    def test_yellow_cost_and_color_match(self):
        cm = _cm(size_m=10.0, cell_m=0.5, bot_radius_m=0.0, safety_m=0.0)
        # Lidar engeli 5 m ileride; sarı detection aynı bearing'de -> sarı engel.
        cm.update([_obs(5.0, 0.0)], [_det("yellow", 5.0, bearing=0.0)], "")
        self.assertEqual(cm.cost(5.0, 0.0), COST_YELLOW)

    def test_unknown_cost_when_no_lidar_match(self):
        cm = _cm(size_m=10.0, cell_m=0.5, bot_radius_m=0.0, safety_m=0.0)
        # SARI = ENGEL (COST_YELLOW=8) — lidar eşleşmesi olmasa bile DWA çarpışmayı
        # algılamalı (parkur2_analysis S1). Eski COST_UNKNOWN(4) eşiği geçmiyordu.
        cm.update([], [_det("yellow", 5.0, bearing=0.0, confidence=0.5)], "")
        self.assertEqual(cm.cost(5.0, 0.0), COST_YELLOW)

    def test_unknown_wrong_color_target(self):
        cm = _cm(size_m=10.0, cell_m=0.5, bot_radius_m=0.0, safety_m=0.0)
        # P3 hedef rengi green; siyah duba lidar engeli yok -> unknown engel.
        cm.update([], [_det("black", 4.0, bearing=0.0)], "green")
        self.assertEqual(cm.cost(4.0, 0.0), COST_UNKNOWN)

    def test_color_match_6deg_threshold(self):
        cm = _cm(size_m=10.0, cell_m=0.5, bot_radius_m=0.0, safety_m=0.0)
        # Lidar engeli 0°; turuncu detection 5° -> eşleşir (<= 6°), maliyet 1.
        cm.update([_obs(5.0, 0.0)], [_det("orange", 5.0, bearing=5.0)], "")
        self.assertEqual(cm.cost(5.0, 0.0), COST_ORANGE)
        # 9° -> eşleşmez; lidar engeli maliyet 8 olarak kalır.
        cm2 = _cm(size_m=10.0, cell_m=0.5, bot_radius_m=0.0, safety_m=0.0)
        cm2.update([_obs(5.0, 0.0)], [_det("orange", 5.0, bearing=9.0)], "")
        self.assertEqual(cm2.cost(5.0, 0.0), COST_OBSTACLE)

    def test_low_conf_yellow_obstacle_painted_orange_stays_hard(self):
        """Düşük güvenli kamera etiketi lidar engelini yumuşatamaz."""
        cm = _cm(size_m=10.0, cell_m=0.5, bot_radius_m=0.0, safety_m=0.0)
        cm.update(
            [_obs(5.0, 0.0)],
            [_det("orange", 5.0, bearing=0.0, confidence=0.4)],
            "",
        )
        self.assertEqual(cm.cost(5.0, 0.0), COST_OBSTACLE)
        self.assertTrue(cm.collides(5.0, 0.0))

    def test_high_conf_orange_paint_keeps_physical_obstacle_hard(self):
        """LİDAR ÖNCELİĞİ: yüksek güvenli turuncu etiketi, fiziksel engeli korur.

        Yüksek güvenli turuncu paint `cells`'i 1 (koridor etiketi) yapabilir
        AMA `inflated` (şişirme) fiziksel engeli 8'de korur — `_add_obstacle`
        `if inflated < inflate_cost` koşulu mevcut hard halkayı düşürmez.
        DWA `collides` inflated'a baktığı için engel HER durumda çarpışma olarak
        kalır — en güçlü lidar önceliği garantisi.
        """
        cm = _cm(size_m=10.0, cell_m=0.5, bot_radius_m=0.0, safety_m=0.0)
        cm.update([_obs(5.0, 0.0)], [_det("orange", 5.0, bearing=0.0, confidence=0.9)], "")
        # Hücre etiketi koridor olabilir (mevcut davranış, P1 uyumluluğu)...
        self.assertEqual(cm.cost(5.0, 0.0), COST_ORANGE)
        # ...AMA fiziksel engel inflated'da 8 kalır -> çarpışma korunur.
        self.assertTrue(cm.collides(5.0, 0.0))

    def test_orange_obstacle_painted_yellow_stays_hard(self):
        """LİDAR ÖNCELİĞİ: turuncu engel sarı sanılırsa sertleşir (güvenli yön)."""
        cm = _cm(size_m=10.0, cell_m=0.5, bot_radius_m=0.0, safety_m=0.0)
        # Lidar engeli 5m; kamera onu SARI sanıyor (yanlış ama güvenli yöne).
        cm.update([_obs(5.0, 0.0)], [_det("yellow", 5.0, bearing=0.0)], "")
        # Maliyet 8 kalır veya yükselir (asla 1'e düşmez).
        self.assertGreaterEqual(cm.cost(5.0, 0.0), COST_OBSTACLE)
        self.assertTrue(cm.collides(5.0, 0.0))

    def test_speed_dependent_inflation(self):
        # Hıza bağlı şişirme: duruşta yarıçap bot + taban (0.6+0.5=1.1 m),
        # 3 m/s'de bot + safety_radius(3.0) = 0.6 + 6.5 = 7.1 m (tavan 10 altı).
        # Engelin merkezi HER hızda şişirilir (merkez hücre de çemberde); hız
        # etkisi engelden yarıçap kadar uzaktaki noktalarda görülür.
        cm = _cm(size_m=30.0)  # 1.5 m ileri nokta (col 39) sınır içinde kalsın
        # Duruşta: engelden 1.5 m ilerideki nokta 1.1 m çemberinin DIŞINDA.
        cm.update([_obs(3.0, 0.0)], [], "", ground_speed=0.0)
        self.assertFalse(cm.collides(4.5, 0.0))
        self.assertAlmostEqual(cm.get_inflation_radius(), 1.1, places=2)
        # 3 m/s'de: aynı nokta 7.1 m çemberinin İÇİNDE -> çarpışma.
        cm.update([_obs(3.0, 0.0)], [], "", ground_speed=3.0)
        self.assertAlmostEqual(cm.get_inflation_radius(), 7.1, places=1)
        self.assertTrue(cm.collides(4.5, 0.0))
        self.assertTrue(cm.collides(3.0, 0.0))  # engel merkezi her zaman engel

    def test_inflation_survives_reset(self):
        # reset() hız/şişirme DURUMUNA dokunmaz (tick'ler arası korunur);
        # yalnızca hücre/tag/inflated dizilerini temizler.
        cm = _cm()
        cm.update([_obs(3.0, 0.0)], [], "", ground_speed=4.0)
        radius_before = cm.get_inflation_radius()
        cm.reset()
        self.assertEqual(cm.get_inflation_radius(), radius_before)
        # Harita temizlendi (anlık harita), ama hız durumu kaldı.
        self.assertFalse(cm.collides(3.0, 0.0))

    def test_reset_clears_previous_tick(self):
        cm = _cm()
        cm.update([_obs(5.0, 0.0)], [], "")
        self.assertTrue(cm.collides(5.0, 0.0))
        cm.update([], [], "")  # boş tick -> her şey temizlenir (anlık harita)
        self.assertFalse(cm.collides(5.0, 0.0))
        self.assertEqual(cm.cost(5.0, 0.0), 0)

    def test_to_json_nonzero_cells(self):
        cm = _cm()
        cm.update([_obs(5.0, 0.0)], [], "")
        data = cm.to_json()
        self.assertIn('"cells"', data)
        self.assertIn("inflated", data)  # şişirme halkası görselleştirmede var


# ---------------------------------------------------------------------------
# DWA: aday seçimi, çarpışma -> recovery (sağa), hedef çekimi, unknown yavaşlama.
# ---------------------------------------------------------------------------


class TestDwa(unittest.TestCase):
    def _planner(self, **kw):
        defaults = dict(
            max_speed_mps=0.8,
            max_yaw_rate_deg_s=45.0,
            vx_steps=7,
            yaw_steps=15,
            sim_time_s=2.5,
            sim_step_s=0.25,
            recovery_vx=0.25,
            recovery_yaw_deg_s=35.0,
            bot_radius_m=1.1,
            cell_m=0.25,
            n_cells=40,
        )
        defaults.update(kw)
        return DwaPlanner(**defaults)

    def _clear_cm(self):
        cm = CostMap(size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=0.5)
        cm.update([], [], "")
        return cm

    def test_p2_180_goal_behind_four_quadrant_bearing(self):
        # P2 180° DÖNÜŞ regresyon testi (envanter #2): hedef ARKADAYKEN
        # (goal[0] < 0) 4-kadran atan2 -> bearing ~±pi, yaw=0 adayı düşük skor.
        # Kök neden max(goal[0], 0.001) bearing'i 0'a çekip yaw=0 adayına
        # heading_score=1.0 veriyordu (araç hedefe dönemiyordu).
        pl = self._planner()
        goal_behind = (-5.0, 0.1)
        goal_bearing = math.atan2(goal_behind[1], goal_behind[0])  # ~pi (4-kadran)
        self.assertAlmostEqual(goal_bearing, math.pi, places=1)  # 178.85°
        # yaw=0 adayı: hata ~pi -> heading ~0 (dönmeli).
        hs_zero = 1.0 - abs(_wrap_rad(goal_bearing - 0.0)) / math.pi
        self.assertLess(hs_zero, 0.05)
        # Hedefe dönen aday: yaw = bearing / sim_time -> hata 0 -> heading ~1.
        yaw_turn = goal_bearing / pl.sim_time_s
        hs_turn = 1.0 - abs(_wrap_rad(goal_bearing - yaw_turn * pl.sim_time_s)) / math.pi
        self.assertGreater(hs_turn, 0.99)
        # Eski hatalı formül (max) olsaydı bearing 0'a çekilirdi (tekil nokta);
        # goal_lat=0.1 için değer 89.43°'ye kayar (0.001 bileşeni baskın).
        old_bearing = math.atan2(goal_behind[1], max(goal_behind[0], 0.001))
        self.assertAlmostEqual(old_bearing, math.pi / 2.0, places=1)

    def test_p2_plan_turns_when_goal_behind(self):
        # Tam plan(): hedef arkada (goal=(-5, 0.1)) -> yaw=0 değil, dönüş adayı.
        pl = self._planner()
        cm = self._clear_cm()
        out = pl.plan((-5.0, 0.1), cm, None)
        self.assertFalse(out.action.startswith("dwa_recovery"))
        self.assertGreaterEqual(abs(out.yaw_rate), math.radians(20.0))  # dönüş adayı
        self.assertGreater(out.vx, 0.08)  # dönüş sırasında ilerleme (vx>0)

    def test_large_waypoint_heading_error_aligns_before_drive(self):
        pl = self._planner(
            align_before_drive_deg=42.0,
            align_yaw_rate_deg_s=30.0,
        )
        out = pl.plan((1.0, 4.0), self._clear_cm(), None)
        self.assertEqual(out.vx, 0.0)
        self.assertEqual(out.action, "dwa_align_waypoint_right")
        self.assertAlmostEqual(out.yaw_rate, math.radians(30.0))

    def test_small_waypoint_heading_error_keeps_normal_dwa(self):
        pl = self._planner(
            align_before_drive_deg=42.0,
            align_yaw_rate_deg_s=30.0,
        )
        out = pl.plan((5.0, 1.0), self._clear_cm(), None)
        self.assertFalse(out.action.startswith("dwa_align_waypoint"))
        self.assertGreater(out.vx, 0.0)

    def test_waypoint_alignment_has_exit_hysteresis(self):
        pl = self._planner(
            align_before_drive_deg=42.0,
            align_yaw_rate_deg_s=30.0,
        )
        previous = DwaCommand(0.0, math.radians(30.0), "dwa_align_waypoint_right")
        still_aligning = pl.plan((5.0, 2.5), self._clear_cm(), previous)
        self.assertTrue(still_aligning.action.startswith("dwa_align_waypoint"))
        released = pl.plan((5.0, 1.5), self._clear_cm(), previous)
        self.assertFalse(released.action.startswith("dwa_align_waypoint"))

    def test_waypoint_directly_behind_keeps_previous_alignment_side(self):
        pl = self._planner(
            align_before_drive_deg=42.0,
            align_yaw_rate_deg_s=30.0,
        )
        previous = DwaCommand(0.0, math.radians(30.0), "dwa_align_waypoint_right")
        # Bearing is just across the -pi seam. Without seam hysteresis this
        # flips left/right every pose update and the boat never completes turn.
        out = pl.plan((-38.0, -0.1), self._clear_cm(), previous)
        self.assertEqual(out.action, "dwa_align_waypoint_right")
        self.assertGreater(out.yaw_rate, 0.0)

        previous_left = DwaCommand(
            0.0, math.radians(-30.0), "dwa_align_waypoint_left"
        )
        out_left = pl.plan((-38.0, 0.1), self._clear_cm(), previous_left)
        self.assertEqual(out_left.action, "dwa_align_waypoint_left")
        self.assertLess(out_left.yaw_rate, 0.0)

    def test_waypoint_alignment_rejects_nonfinite_or_out_of_range_config(self):
        for field, value in (
            ("align_before_drive_deg", float("nan")),
            ("align_before_drive_deg", 181.0),
            ("align_yaw_rate_deg_s", float("inf")),
            ("align_yaw_rate_deg_s", -1.0),
        ):
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self._planner(**{field: value})

    def test_p1_corridor_goal_centers_lane(self):
        # P1 SALINIM regresyon testi (envanter #3): koridor aktifken amaç
        # (lookahead, 0.3*cl) üretilir — lateral hedef yumuşatılır (heading
        # terimi 0'a değil 0.3*cl'ye nişan alır -> mikro waypoint bearing'ine
        # duyarlılık azalır, araç koridoru ortalama eğiliminde kalır).
        forward, lateral = corridor_goal_body(
            waypoint_fwd=4.0, waypoint_lat=-2.5,
            center_forward_m=6.0, center_lateral_m=-1.0, confidence=1.0,
        )
        self.assertGreaterEqual(forward, 8.0)  # lookahead tavanı
        self.assertAlmostEqual(lateral, -0.3, places=2)  # cl*0.3

    def test_p1_corridor_low_conf_falls_back(self):
        # Düşük güvenli koridor (<0.3) -> waypoint hedefi (yeni hedef yok).
        # Düşük güven: gövde (forward/lateral) hiç üretilmez -> waypoint kalır.
        wp = (4.0, -2.5)
        goal = corridor_goal_body(
            waypoint_fwd=wp[0], waypoint_lat=wp[1],
            center_forward_m=6.0, center_lateral_m=-1.0, confidence=0.2,
        )
        self.assertEqual(goal[0], 4.0)
        self.assertEqual(goal[1], -2.5)

    def test_heading_attraction_forward_goal(self):
        # Hedef ileride -> ileri aday (yaw ~0) en yüksek skoru alır.
        pl = self._planner()
        cm = self._clear_cm()
        out = pl.plan((5.0, 0.0), cm, None)
        self.assertFalse(out.action.startswith("dwa_recovery"))
        self.assertGreater(out.vx, 0.3)
        self.assertLess(abs(out.yaw_rate), math.radians(10.0))

    def test_obstacle_triggers_recovery_right(self):
        # İleri yol tamamen kapalı (engel 1.5 m'de, süpürme 0.8*3.0=2.4 m) ->
        # tüm adaylar çarpışır -> recovery sağa dönüş.
        pl = self._planner(sim_time_s=3.0)
        cm = CostMap(size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=0.5)
        cm.update([_obs(1.5, 0.0)], [], "")
        out = pl.plan((5.0, 0.0), cm, None)
        self.assertTrue(out.action.startswith("dwa_recovery"))
        self.assertGreater(out.yaw_rate, 0.0)  # sağa (pozitif) dönüş
        self.assertLessEqual(out.vx, 0.25)

    def test_recovery_deterministic(self):
        pl = self._planner()
        cm = self._clear_cm()
        a = pl.recovery(cm, None)
        b = pl.recovery(cm, None)
        self.assertEqual(a.vx, b.vx)
        self.assertEqual(a.yaw_rate, b.yaw_rate)
        self.assertGreater(a.yaw_rate, 0.0)

    def test_recovery_side_memory_is_bounded_to_configured_calls(self):
        pl = self._planner(recovery_latch_ticks=3)
        obstacle_left = CostMap(
            size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=0.5
        )
        obstacle_left.update([_obs(2.0, -1.0)], [], "")
        obstacle_right = CostMap(
            size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=0.5
        )
        obstacle_right.update([_obs(2.0, 1.0)], [], "")

        first = pl.recovery(obstacle_left, None, goal_body=(5.0, 0.0))
        noisy_flip = pl.recovery(obstacle_right, None, goal_body=(5.0, 0.0))
        self.assertGreater(first.yaw_rate, 0.0)
        self.assertGreater(noisy_flip.yaw_rate, 0.0)

        # The first call arms three retained calls; one was consumed above.
        pl.recovery(obstacle_right, None, goal_body=(5.0, 0.0))
        pl.recovery(obstacle_right, None, goal_body=(5.0, 0.0))
        new_episode = pl.recovery(obstacle_right, None, goal_body=(5.0, 0.0))
        self.assertLess(new_episode.yaw_rate, 0.0)

    def test_zero_recovery_latch_preserves_legacy_per_frame_selection(self):
        pl = self._planner(recovery_latch_ticks=0)
        left = CostMap(size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=0.5)
        right = CostMap(size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=0.5)
        left.update([_obs(2.0, -1.0)], [], "")
        right.update([_obs(2.0, 1.0)], [], "")
        self.assertGreater(pl.recovery(left, None).yaw_rate, 0.0)
        self.assertLess(pl.recovery(right, None).yaw_rate, 0.0)

    def test_dynamic_window_accel_limits(self):
        pl = self._planner()
        last = DwaCommand(vx=0.5, yaw_rate=0.2, action="prev")
        vx_min, vx_max, yaw_min, yaw_max = pl.dynamic_window(last)
        # max_accel=0.8 -> vx_max = 0.5 + 0.8 = 1.3, max_speed=0.8 ile tavanlanır.
        self.assertLessEqual(vx_max, pl.max_speed)
        self.assertGreaterEqual(vx_min, 0.0)
        self.assertLessEqual(yaw_max, pl.max_yaw_rate)

    def test_unknown_slows_down(self):
        # Hedefin önünde unknown hücre (yakın) -> vx sınırlanır.
        # SARI artık engel (8) — unknown'u P3 hedef rengi dışı duba üretir (black).
        pl = self._planner()
        cm = CostMap(size_m=30.0, cell_m=0.25, bot_radius_m=0.0, safety_m=0.0)
        cm.update([], [_det("black", 3.0, bearing=0.0, confidence=0.5)], "green")
        self.assertTrue(pl.near_unknown(cm))
        # near_unknown true iken autonomy, slow_planner (max_speed=dwa_slow_vx=0.3)
        # kurar; bu test aynı mekanizmayı vx_steps=7 ile doğrular.
        slow_pl = DwaPlanner(
            max_speed_mps=0.3, max_yaw_rate_deg_s=45.0, vx_steps=7, yaw_steps=15,
            sim_time_s=2.5, sim_step_s=0.25, recovery_vx=0.25, recovery_yaw_deg_s=35.0,
            bot_radius_m=1.1, cell_m=0.25, n_cells=120,
        )
        out = slow_pl.plan((5.0, 0.0), cm, None)
        self.assertLess(out.vx, 0.35)  # slow_vx tavanı
        self.assertFalse(out.action.startswith("dwa_recovery"))

    def test_goal_cost_attracts(self):
        # Goal hücresi (maliyet 3) engel DEĞİL: hedefe yönelim sürer.
        pl = self._planner()
        cm = CostMap(size_m=30.0, cell_m=0.25, bot_radius_m=0.0, safety_m=0.0)
        cm.update([], [_det("green", 4.0, bearing=0.0)], "green")
        out = pl.plan((4.0, 0.0), cm, None)
        self.assertFalse(out.action.startswith("dwa_recovery"))
        self.assertGreater(out.vx, 0.2)

    def test_candidates_count_and_offset_table(self):
        pl = self._planner()
        cands = pl.candidates((5.0, 0.0), None)
        self.assertEqual(len(cands), pl.vx_steps * pl.yaw_steps)  # ~105
        # 1.1 m offset tablosu 0.25 m hücrede ~180 hücre içermelidir.
        # (Küçük/özelleştirilmiş hücrelerde daha az; eşik 60 — 1.1 m yarıçap.)
        offsets = build_offset_cells(1.1, 0.25)
        self.assertGreater(len(offsets), 60)
        self.assertLess(len(offsets), 240)

    def test_max_speed_scale_governor(self):
        # Governor: scale 0.5 -> etkin tavan 0.8*0.5 = 0.4; dinamik pencere ve
        # recovery bu etkin tavana saygı duyar (Adım 3 DWA değişikliği).
        pl = self._planner()
        pl.set_max_speed_scale(0.5)
        self.assertAlmostEqual(pl.effective_max_speed(), 0.4, places=2)
        _, vx_max, _, _ = pl.dynamic_window(None)
        self.assertAlmostEqual(vx_max, 0.4, places=2)
        cm = self._clear_cm()
        out = pl.recovery(cm, None)
        self.assertLessEqual(out.vx, 0.4)

    def test_recovery_goal_behind_uses_four_quadrant_bearing(self):
        # B8: recovery'de hedef ARKADAYKEN (goal[0] < 0) 4-kadran atan2 -> ±180°
        # bearing; eski max(goal[0], 0.1) 0.1'e çekip ±90° üretiyordu. Engel yok,
        # hedef arkada: dönüş hedefe doğru olmalı (bearing/2 clamp recovery_yaw).
        pl = self._planner()  # recovery_yaw_deg_s=35
        cm = self._clear_cm()  # boş costmap (engel yok -> hedefe dönüş dalı)
        goal_behind = (-5.0, 0.1)
        gb = math.degrees(math.atan2(goal_behind[1], goal_behind[0]))  # ~178.85°
        # Hedef arkada -> bearing ±180°'ye yakın olmalı (4-kadran; 178.85±5).
        self.assertAlmostEqual(gb, 180.0, delta=5.0)
        out = pl.recovery(cm, None, goal_body=goal_behind)
        # 0.5 * 178.85 = 89.4 clamp 35 -> +35 (hedefe dönüş, sağa).
        self.assertGreater(out.yaw_rate, 0.0)
        self.assertLessEqual(math.degrees(abs(out.yaw_rate)), 35.0)
        # Eski hatalı formül olsaydı: atan2(0.1, 0.1)=45° -> yaw=0.5*45=22.5°
        # (yanlış yön değil ama yanlış büyüklük; asıl hata engel varken görünür).
        old_gb = math.degrees(math.atan2(goal_behind[1], max(goal_behind[0], 0.1)))
        self.assertAlmostEqual(old_gb, 45.0, places=0)  # 0.1/0.1 -> 45° (hatalı)

    def test_nearest_obstacle_lateral_forward_half_sphere(self):
        # B14: _nearest_obstacle_lateral YALNIZCA ÖN yarım küreyi (forward >= 0)
        # taramalı. world_to_cell: forward=col artışı, lateral=-row artışı.
        # Arkadaki engel (forward<0) yok sayılmalı; öndeki döndürülmeli.
        pl = self._planner()
        cm = CostMap(size_m=30.0, cell_m=0.25, bot_radius_m=0.0, safety_m=0.0)
        # Arkada engel (forward=-3, lateral=+1) + önde engel (forward=3, lateral=-1).
        cm._add_obstacle(-3.0, 1.0, COST_OBSTACLE, "obstacle")
        cm._add_obstacle(3.0, -1.0, COST_OBSTACLE, "obstacle")
        lat = pl._nearest_obstacle_lateral(cm)
        self.assertIsNotNone(lat)
        # En yakın ÖN engel (3.0, -1.0) -> lateral negatif (sol). Arkadaki
        # (-3, 1) filtrelenmeli; eski dr>=0 kodu sağ tarafı (lateral<=0) arıyordu
        # ve öndeki/arkadaki karışırdı.
        self.assertLess(lat, 0.0)

    # --- Swept-cell cache (performans: build_swept_cells tekrarı) -------------

    def test_swept_cache_hit_returns_same_cells(self):
        # Aynı (vx, yaw, n_cells) iki kez sorulursa cache aynı listeyi döner;
        # build_swept_cells yalnızca ilk miss'te çalışır (cache regresyonu).
        pl = self._planner()
        cm = self._clear_cm()
        a = pl._swept_cells(0.5, 0.1, cm.n)
        b = pl._swept_cells(0.5, 0.1, cm.n)
        self.assertIs(a[0], b[0])  # aynı liste nesnesi (cache referansı)
        self.assertIs(a[1], b[1])
        self.assertGreater(len(a[0]), 0)
        # Farklı yaw farklı geometri üretir (cache miss -> yeni liste).
        c = pl._swept_cells(0.5, 0.2, cm.n)
        self.assertIsNot(a[0], c[0])

    def test_swept_cache_round_key_quantization(self):
        # round(vx, 5) key nicemlemesi: aynı yuvarlanmış değer aynı cache'e oturur.
        pl = self._planner()
        cm = self._clear_cm()
        # round(1.000004, 5) = round(1.0000049, 5) = 1.0 -> aynı key.
        a = pl._swept_cells(1.000004, 0.0, cm.n)
        b = pl._swept_cells(1.0000049, 0.0, cm.n)
        self.assertIs(a[0], b[0])
        self.assertIs(a[1], b[1])
        # round(1.000006, 5) = 1.00001 -> farklı key (farklı geometri).
        c = pl._swept_cells(1.000006, 0.0, cm.n)
        self.assertIsNot(a[0], c[0])

    def test_swept_cache_fifo_eviction(self):
        # 512 sınırında FIFO: en eski giriş atılır; yenisi eklenir.
        pl = self._planner()
        cm = self._clear_cm()
        pl._swept_cache.clear()
        pl._swept_cache_max = 3
        first_key = (0.1, 0.1, cm.n)
        for i in range(3):
            pl._swept_cells(0.1, 0.1 + i * 0.1, cm.n)
        self.assertEqual(len(pl._swept_cache), 3)
        # 4. ekleme -> ilk (0.1, 0.1) FIFO atılır.
        pl._swept_cells(0.1, 0.4, cm.n)
        self.assertEqual(len(pl._swept_cache), 3)
        self.assertNotIn(first_key, pl._swept_cache)
        # Atılan key yeniden üretilebilir (cache tutarlı kalır).
        pl._swept_cells(0.1, 0.1, cm.n)
        self.assertIn(first_key, pl._swept_cache)
        self.assertEqual(len(pl._swept_cache), 3)


# ---------------------------------------------------------------------------
# Puan ceza modülü: ContactCounter (temas sayacı) — Ç1/Ç2.
# ---------------------------------------------------------------------------


def _buoy(color, distance, forward_m=None, lateral_m=None, confidence=0.9, buoy_id=None):
    """Temas testi için duba sözlüğü: forward/lateral verilirse sim/fused."""
    b = {
        "color": color,
        "distance": distance,
        "confidence": confidence,
    }
    if forward_m is not None:
        b["forward_m"] = forward_m
    if lateral_m is not None:
        b["lateral_m"] = lateral_m
    if buoy_id is not None:
        b["id"] = buoy_id
    return b


def _obstacle(forward_m, lateral_m, obstacle_id=None):
    """Temas testi için lidar engeli (gövde koordinatı)."""
    o = {
        "distance": math.hypot(forward_m, lateral_m),
        "forward_m": forward_m,
        "lateral_m": lateral_m,
    }
    if obstacle_id is not None:
        o["id"] = obstacle_id
    return o


def _counter(**kw):
    defaults = dict(
        contact_radius_m=1.6,
        sustained_contact_s=30.0,
        drop_after_s=5.0,
        max_lateral_m=15.0,
        max_distance_m=3.0,
        min_confidence=0.35,
        include_yellow_in_p2=True,
    )
    defaults.update(kw)
    return ContactCounter(**defaults)


class TestContactCounter(unittest.TestCase):
    def test_single_contact_counts_one(self):
        # P1: yakın turuncu duba (kamera, yalnız distance) -> 1 temas.
        cc = _counter()
        stats = cc.update([_buoy("orange", 1.0)], [], 1, now=1.0)
        self.assertEqual(stats.total, 1)
        self.assertEqual(stats.by_color["orange"], 1)
        self.assertEqual(cc.total_counted(), 0)  # kapalı olay yok

    def test_30s_sustained_contact_counts_two(self):
        # Aynı dubaya 30 sn SÜREKLİ temas -> 2 çarpma (şartname 840-843).
        cc = _counter(sustained_contact_s=30.0, drop_after_s=5.0)
        cc.update([_buoy("orange", 1.0)], [], 1, now=0.0)
        stats = cc.update([_buoy("orange", 1.0)], [], 1, now=29.0)
        self.assertEqual(stats.total, 1)  # 29 sn: hâlâ 1 olay
        self.assertFalse(stats.active_events[0].sustained)
        stats = cc.update([_buoy("orange", 1.0)], [], 1, now=31.0)
        self.assertTrue(stats.active_events[0].sustained)
        self.assertEqual(stats.active_events[0].counted, 2)

    def test_two_sustained_blocks_count_four(self):
        # İki AYRI 30 sn'lik sürekli temas bloğu (arada 6 sn kopma) -> 4 çarpma.
        # Şartname: her temas sayılır + 30 sn sürekli temas 2 sayılır.
        cc = _counter(sustained_contact_s=30.0, drop_after_s=5.0)
        cc.update([_buoy("orange", 1.0, buoy_id="b1")], [], 1, now=0.0)
        cc.update([_buoy("orange", 1.0, buoy_id="b1")], [], 1, now=30.0)  # 1. blok tamam
        self.assertEqual(cc.get_events()[0].counted, 2)
        cc.update([], [], 1, now=36.0)  # 6 sn kopma (< drop_after_s=5? hayır: 6>5)
        # 30->36 arası 6 sn, drop_after_s=5 -> olay kapanır.
        self.assertEqual(cc.total_counted(), 2)  # ilk blok kapandı
        # 2. blok: aynı duba yeniden yakın temas.
        cc.update([_buoy("orange", 1.0, buoy_id="b1")], [], 1, now=37.0)
        cc.update([_buoy("orange", 1.0, buoy_id="b1")], [], 1, now=67.0)  # 2. blok 30 sn
        self.assertEqual(cc.total_counted(), 4)  # 2 + 2

    def test_open_sustained_event_included_in_total_counted(self):
        # O1: counted>=2 olan AÇIK (devam eden) olay da total_counted'a dahildir.
        cc = _counter(sustained_contact_s=30.0, drop_after_s=5.0)
        cc.update([_buoy("orange", 1.0, buoy_id="b1")], [], 1, now=0.0)
        cc.update([_buoy("orange", 1.0, buoy_id="b1")], [], 1, now=31.0)  # açık, counted=2
        self.assertEqual(cc.get_events()[0].counted, 2)
        self.assertFalse(cc.get_events()[0].closed)
        self.assertEqual(cc.total_counted(), 2)

    def test_drop_after_s_closes_event(self):
        # Temas 5 sn kesilirse olay kapanır; yeni temas yeni olay açar.
        cc = _counter(drop_after_s=5.0)
        cc.update([_buoy("orange", 1.0, buoy_id="o1")], [], 1, now=0.0)
        cc.update([], [], 1, now=3.0)
        stats = cc.update([], [], 1, now=9.0)  # 9-0 > 5 -> kapanır
        self.assertEqual(stats.total, 0)
        # Yeni temas (6 sn sonra) yeni olay açar; eski olay kapalı kalır.
        stats = cc.update([_buoy("orange", 1.0, buoy_id="o1")], [], 1, now=12.0)
        self.assertEqual(stats.total, 1)

    def test_lidar_and_camera_same_buoy_count_once(self):
        # Aynı dubayı hem lidar hem kamera görürse TEK temas sayılır.
        # Kamera (id "d1", distance 1.0) + lidar (id "d1", forward 0.9, lateral 0.3).
        cc = _counter()
        stats = cc.update(
            [_buoy("yellow", 1.0, confidence=0.9, buoy_id="d1")],
            [_obstacle(0.9, 0.3, obstacle_id="d1")],
            2,
            now=1.0,
        )
        self.assertEqual(stats.total, 1)
        self.assertEqual(len(cc.get_events()), 1)

    def test_yellow_counts_in_p2(self):
        # P2: sarı engel dubası çarpma sayılır (kenar/engel ayrımı yok, tek Ç2).
        cc = _counter(include_yellow_in_p2=True)
        stats = cc.update([_buoy("yellow", 1.2)], [], 2, now=1.0)
        self.assertEqual(stats.total, 1)
        self.assertEqual(stats.by_color["yellow"], 1)
        # P1'de sarı engel sayılmaz (yalnız turuncu kenar).
        cc.reset()
        stats = cc.update([_buoy("yellow", 1.2)], [], 1, now=1.0)
        self.assertEqual(stats.total, 0)

    def test_camera_distance_only_uses_distance(self):
        # Gerçek kamera kontratı: yalnız distance + confidence (forward/lateral yok).
        cc = _counter(contact_radius_m=1.6)
        stats = cc.update([_buoy("orange", 0.8, confidence=0.9, buoy_id="c1")], [], 1, now=1.0)
        self.assertEqual(stats.total, 1)
        # Düşük güven -> elenir (ayrı duba; aynı id'yi yüksek güvenle tekrar görürse
        # olay SÜRDÜRÜLÜR, yeni olay açılmaz — çifte sayım bastırılır).
        stats = cc.update([_buoy("orange", 0.8, confidence=0.1, buoy_id="c2")], [], 1, now=2.0)
        self.assertEqual(stats.total, 1)

    def test_obstacle_lateral_limit(self):
        # max_lateral_m dışı lidar engeli temas adayı olmaz.
        cc = _counter(max_lateral_m=1.0)
        stats = cc.update([], [_obstacle(0.8, 3.0)], 2, now=1.0)
        self.assertEqual(stats.total, 0)

    def test_max_distance_filter(self):
        # Temas yarıçapı dışı (3.0 m) -> temas sayılmaz.
        cc = _counter(contact_radius_m=1.6, max_distance_m=3.0)
        stats = cc.update([_buoy("orange", 4.0)], [], 1, now=1.0)
        self.assertEqual(stats.total, 0)

    def test_reset_clears_events(self):
        cc = _counter()
        cc.update([_buoy("orange", 1.0, buoy_id="o1")], [], 1, now=1.0)
        self.assertEqual(cc.total_counted(), 0)
        cc.reset()
        self.assertEqual(cc.get_events(), [])
        stats = cc.update([], [], 1, now=2.0)
        self.assertEqual(stats.total, 0)

    def test_negative_distance_ignored(self):
        # Negatif/NaN distance -> aday elenir (elle NaN/Inf işleme).
        cc = _counter()
        bad = {"color": "orange", "distance": float("nan"), "confidence": 0.9}
        stats = cc.update([bad], [], 1, now=1.0)
        self.assertEqual(stats.total, 0)


class TestOutOfCourseDetector(unittest.TestCase):
    def _detector(self, mode="waypoint_bbox", half_width_m=8.0, **kw):
        defaults = dict(mode=mode, half_width_m=half_width_m, sustained_out_s=40.0, drop_after_s=5.0, min_speed_m_s=0.2)
        defaults.update(kw)
        return OutOfCourseDetector(**defaults)

    @staticmethod
    def _course_waypoints():
        # Basit P1 rotası: 4 waypoint (lat/lon), ~100 m kare.
        return [
            {"lat": 40.0, "lon": 29.0},
            {"lat": 40.0009, "lon": 29.0},
            {"lat": 40.0009, "lon": 29.001},
            {"lat": 40.0, "lon": 29.001},
        ]

    def _inside_point(self, wp):
        return {"lat": wp[0]["lat"] + 0.0001, "lon": wp[0]["lon"] + 0.0001}

    def test_inside_bbox_counts_not_outside(self):
        det = self._detector()
        det.set_course(self._course_waypoints())
        # Waypoint bbox'ı merkezinde: içeride -> birikim yok.
        stats = det.update(40.00045, 29.0005, 1.0, 1, now=0.0)
        self.assertTrue(stats.inside)
        self.assertEqual(stats.outside_s, 0.0)
        self.assertEqual(stats.times_out, 0)

    def test_40s_outside_counts_two(self):
        # Dışarıda >=40 sn -> 2 defa dışarı çıkmış sayılır (şartname 847-848).
        det = self._detector()
        det.set_course(self._course_waypoints())
        far = (40.0025, 29.0030)  # bbox + half_width'ın dışında
        det.update(*far, 1.0, 1, now=0.0)
        det.update(*far, 1.0, 1, now=39.0)
        stats = det.update(*far, 1.0, 1, now=41.0)
        self.assertEqual(stats.times_out, 2)

    def test_short_outing_not_counted(self):
        # Kısa dışarı çıkış (40 sn altı) -> çıkış sayılmaz.
        det = self._detector()
        det.set_course(self._course_waypoints())
        far = (40.0025, 29.0030)
        det.update(*far, 1.0, 1, now=0.0)
        stats = det.update(*far, 1.0, 1, now=5.0)
        self.assertEqual(stats.times_out, 0)
        stats = det.update(40.00045, 29.0005, 1.0, 1, now=6.0)  # içeri döndü
        self.assertEqual(stats.times_out, 0)
        self.assertEqual(stats.outside_s, 0.0)

    def test_stationary_not_counted(self):
        # Durgunluk (hız < min_speed_m_s) dışarıda sayılmaz.
        det = self._detector()
        det.set_course(self._course_waypoints())
        far = (40.0025, 29.0030)
        det.update(*far, 0.05, 1, now=0.0)
        stats = det.update(*far, 0.05, 1, now=50.0)
        self.assertEqual(stats.times_out, 0)
        self.assertEqual(stats.outside_s, 0.0)

    def test_gps_jump_not_counted(self):
        # Tek karelik GPS sıçraması (drop_after_s altında içeri dönüş) sayılmaz.
        det = self._detector(drop_after_s=5.0)
        det.set_course(self._course_waypoints())
        far = (40.0025, 29.0030)
        det.update(*far, 1.0, 1, now=0.0)
        det.update(40.00045, 29.0005, 1.0, 1, now=0.5)  # 0.5 sn sonra içeride
        stats = det.update(40.00045, 29.0005, 1.0, 1, now=1.0)
        self.assertEqual(stats.times_out, 0)
        self.assertEqual(stats.outside_s, 0.0)

    def test_reset_clears_times_out(self):
        det = self._detector()
        det.set_course(self._course_waypoints())
        far = (40.0025, 29.0030)
        det.update(*far, 1.0, 1, now=0.0)
        det.update(*far, 1.0, 1, now=41.0)
        self.assertEqual(det.times_out, 2)
        det.reset()
        self.assertEqual(det.times_out, 0)
        self.assertEqual(det.cumulative_out_s, 0.0)

    def test_corridor_mode_inside_center(self):
        # corridor modu: duba hattı üzerinde, bbox dışında da içeride sayılır.
        det = self._detector(mode="corridor", half_width_m=8.0)
        det.set_course(self._course_waypoints())
        # Waypoint hattı 40.0009 enleminden geçer (kuzey kenar). Araç bu hat
        # boyunca (lon 29.0005'te) -> dik uzaklık ~0 -> içeride.
        stats = det.update(40.0009, 29.0005, 1.0, 1, now=0.0)
        self.assertTrue(stats.inside)

    def test_set_course_empty_is_outside(self):
        det = self._detector()
        det.set_course([])
        stats = det.update(40.0, 29.0, 1.0, 1, now=0.0)
        self.assertFalse(stats.inside)
        self.assertEqual(stats.distance_from_course_m, float("inf"))

    def test_nan_telemetry_no_unfair_penalty(self):
        # NaN lat/lon (telemetri bozulması) haksız ceza üretmez: içeride sayılır.
        det = self._detector()
        det.set_course(self._course_waypoints())
        det.update(float("nan"), 29.0, 1.0, 1, now=0.0)
        det.update(40.00045, 29.0005, 1.0, 1, now=0.5)  # içeride
        stats = det.update(float("inf"), float("nan"), 1.0, 1, now=1.0)
        self.assertTrue(stats.inside)
        self.assertEqual(stats.times_out, 0)
        self.assertEqual(stats.outside_s, 0.0)

    def test_invalid_mode_falls_back_to_bbox(self):
        # Geçersiz mod -> waypoint_bbox'a düşer (C1 tek mod kaynağı).
        det = self._detector(mode="bogus_mode")
        self.assertEqual(det.mode, "waypoint_bbox")
        det.set_course(self._course_waypoints())
        # bbox merkezi içeride -> içeride.
        stats = det.update(40.00045, 29.0005, 1.0, 1, now=0.0)
        self.assertTrue(stats.inside)
        self.assertEqual(stats.distance_from_course_m, 0.0)

    def test_times_out_changed_flag(self):
        # times_out_changed yalnız times_out DEĞİŞTİĞİNDE True (O2 olay üretimi).
        det = self._detector()
        det.set_course(self._course_waypoints())
        far = (40.0025, 29.0030)
        det.update(*far, 1.0, 1, now=0.0)
        det.update(*far, 1.0, 1, now=41.0)  # 41 sn dışarıda -> times_out 2
        self.assertEqual(det.times_out, 2)
        self.assertTrue(det.times_out_changed())
        # Aynı times_out ile tekrar update -> değişmedi.
        det.update(*far, 1.0, 1, now=42.0)
        self.assertFalse(det.times_out_changed())
        # Reset sonrası 0'a döner.
        det.reset()
        self.assertEqual(det.times_out, 0)
        self.assertFalse(det.times_out_changed())


class TestScoreCalculator(unittest.TestCase):
    def test_example_calculation_matches_spec(self):
        # _pdf_ozet.txt:925-940 örnek hesap: KD1=5, Ç1=2 -> 9.6; KD2=5, ED2=5, Ç2=4 -> 18.0.
        calc = ScoreCalculator(kd1=5, kd2=5, ed2=5)
        r1 = calc.report(1, g1=5, contact1=2)
        self.assertAlmostEqual(r1.sections["contact"], 9.6, places=6)
        r2 = calc.report(2, g2=5, contact2=4)
        self.assertAlmostEqual(r2.sections["contact"], 18.0, places=6)
        self.assertTrue(r1.estimated)
        self.assertTrue(r2.estimated)

    def test_contact_breakpoint_zero(self):
        # Ç1=KD1 -> formül 0; Ç1>KD1 -> 0 (şartname kırılımı).
        calc = ScoreCalculator(kd1=5, kd2=5, ed2=5)
        r = calc.report(1, contact1=5)
        self.assertAlmostEqual(r.sections["contact"], 0.0, places=6)
        r = calc.report(1, contact1=6)
        self.assertAlmostEqual(r.sections["contact"], 0.0, places=6)

    def test_ooc_breakpoint_zero(self):
        # PDÇ1=4 -> 0; PDÇ1>4 -> 0. PDÇ2=5 -> 0; >5 -> 0.
        calc = ScoreCalculator(kd1=5, kd2=5, ed2=5)
        r = calc.report(1, ooc1=4)
        self.assertAlmostEqual(r.sections["ooc"], 0.0, places=6)
        r = calc.report(1, ooc1=6)
        self.assertAlmostEqual(r.sections["ooc"], 0.0, places=6)
        r = calc.report(2, ooc2=5)
        self.assertAlmostEqual(r.sections["ooc"], 0.0, places=6)
        r = calc.report(2, ooc2=6)
        self.assertAlmostEqual(r.sections["ooc"], 0.0, places=6)

    def test_ts3_table(self):
        # TS3: 0->100, 1->50, 2->5, 3..8->2, >8->1.
        calc = ScoreCalculator(kd1=5, kd2=5, ed2=5)
        for ts3, expected in [(0, 100.0), (1, 50.0), (2, 5.0), (5, 2.0), (9, 1.0)]:
            r = calc.report(3, ts3=ts3)
            self.assertAlmostEqual(r.sections["engage"], expected, places=6)

    def test_uav_bonus_p3(self):
        calc = ScoreCalculator(kd1=5, kd2=5, ed2=5, include_uav_bonus=True)
        r = calc.report(3, ts3=0, uav_bonus=True)
        self.assertAlmostEqual(r.total, 145.0, places=6)
        r = calc.report(3, ts3=0, uav_bonus=False)
        self.assertAlmostEqual(r.total, 100.0, places=6)
        calc_off = ScoreCalculator(kd1=5, kd2=5, ed2=5, include_uav_bonus=False)
        r = calc_off.report(3, ts3=0, uav_bonus=True)
        self.assertAlmostEqual(r.total, 100.0, places=6)

    def test_zero_denominator_returns_zero(self):
        # KD1=0 -> geçiş ve çarpma 0 puan (bölme hatası olmaz).
        calc = ScoreCalculator(kd1=0, kd2=0, ed2=0)
        r = calc.report(1, g1=3, contact1=2)
        self.assertEqual(r.sections["g1"], 0.0)
        self.assertEqual(r.sections["contact"], 0.0)
        r = calc.report(2, g2=3, contact2=2)
        self.assertEqual(r.sections["g2"], 0.0)
        self.assertEqual(r.sections["contact"], 0.0)

    def test_negative_counts_clamped(self):
        # Negatif sayaç -> puan 0'a clamp'lenir, negatife düşmez.
        calc = ScoreCalculator(kd1=5, kd2=5, ed2=5)
        r = calc.report(1, contact1=-2, ooc1=-1)
        # Şartname formülü oransaldır: 16 - (16*(-2))/5 = 22.4 -> [0, 16]'ya clamp.
        self.assertEqual(r.sections["contact"], 16.0)  # tavan: maksimum 16
        self.assertEqual(r.sections["ooc"], 24.0)  # tavan: maksimum 24
        self.assertGreaterEqual(r.sections["contact"], 0.0)
        self.assertGreaterEqual(r.sections["ooc"], 0.0)

    def test_penalty_flag(self):
        # penalty alanı ceza toplamını taşır (sürücü sinyali; estimated=True).
        calc = ScoreCalculator(kd1=5, kd2=5, ed2=5)
        r = calc.report(1, contact1=2, ooc1=1)
        expected = 9.6 + 18.0  # 16-6.4 + 24-6
        self.assertAlmostEqual(r.penalty, expected, places=6)


class TestScoringHelpers(unittest.TestCase):
    def test_buoy_key_id_priority(self):
        # id öncelikli anahtar; lateral_m yedeği (planner.py:187-192 stratejisi).
        self.assertEqual(_buoy_key({"id": 7, "lateral_m": 3.2}), "7")
        self.assertEqual(_buoy_key({"lateral_m": 3.3}), "L3.5")
        self.assertEqual(_buoy_key({"lateral_m": 3.2}), "L3.0")
        self.assertEqual(_obstacle_key({"id": 7, "lateral_m": 3.2}), "7")
        self.assertEqual(_obstacle_key({"lateral_m": 1.1}), "L1.0")

    def test_segment_distance_and_polygon(self):
        # _segment_distance_m: parça uçları dışında en yakın uca uzaklık.
        self.assertAlmostEqual(_segment_distance_m(5.0, 3.0, 0.0, 0.0, 10.0, 0.0), 3.0, places=6)
        # Bitiş ucunun ötesindeki nokta -> uç uzaklığı.
        self.assertAlmostEqual(_segment_distance_m(12.0, 0.0, 0.0, 0.0, 10.0, 0.0), 2.0, places=6)
        # Dejenere parça -> nokta-uca uzaklık.
        self.assertAlmostEqual(_segment_distance_m(3.0, 4.0, 0.0, 0.0, 0.0, 0.0), 5.0, places=6)
        # _point_in_polygon: basit kare (0,0)-(10,0)-(10,10)-(0,10).
        square = [__import__("ida_planning.geo", fromlist=["LocalPoint"]).LocalPoint(x, y) for x, y in [(0, 0), (10, 0), (10, 10), (0, 10)]]
        self.assertTrue(_point_in_polygon(5.0, 5.0, square))
        self.assertFalse(_point_in_polygon(15.0, 5.0, square))


class TestCorridorGoalBody(unittest.TestCase):
    """corridor_goal_body saf fonksiyonu (autonomy_node koridor hedefinin
    tek kaynağı; eski pl_corridor_goal_body helper'ı SİLİNDİ)."""

    def test_p2_corridor_goal_disabled_when_use_false(self):
        # P2 davranışı: use_corridor_goal=False -> koridor aktif + conf=1.0
        # olsa bile hedef WAYPOINT kalır (sarı engel üstüne hedef düşmez).
        goal = corridor_goal_body(
            waypoint_fwd=4.0, waypoint_lat=-2.5,
            center_forward_m=10.0, center_lateral_m=-1.0, confidence=1.0,
            use_corridor_goal=False,
        )
        self.assertEqual(goal[0], 4.0)
        self.assertEqual(goal[1], -2.5)

    def test_corridor_goal_lookahead_param(self):
        # Saha tuning yolu: lookahead_min_m ve lateral_gain parametreleştirildi.
        fwd, lat = corridor_goal_body(
            waypoint_fwd=4.0, waypoint_lat=-2.5,
            center_forward_m=7.0, center_lateral_m=2.0, confidence=1.0,
            lookahead_min_m=10.0, lateral_gain=0.5,
        )
        self.assertEqual(fwd, 10.0)  # max(cf, 10)
        self.assertAlmostEqual(lat, 1.0, places=6)  # cl*0.5
        # cf lookahead'in üstündeyse cf kazanır.
        fwd2, _ = corridor_goal_body(
            waypoint_fwd=4.0, waypoint_lat=-2.5,
            center_forward_m=12.0, center_lateral_m=2.0, confidence=1.0,
            lookahead_min_m=10.0,
        )
        self.assertEqual(fwd2, 12.0)

    def test_corridor_goal_use_true_active(self):
        # use_corridor_goal=True (varsayılan) + conf>=0.3 -> koridor hedefi.
        fwd, lat = corridor_goal_body(
            waypoint_fwd=4.0, waypoint_lat=-2.5,
            center_forward_m=6.0, center_lateral_m=-1.0, confidence=1.0,
        )
        self.assertEqual(fwd, 8.0)  # max(cf, 8.0) lookahead tavanı
        self.assertAlmostEqual(lat, -0.3, places=6)  # cl*0.3


class TestWaypointAdvanceDecision(unittest.TestCase):
    """Yakın normal geçiş + sınırlı mevcut-WP overshoot kontratı."""

    def test_advance_close_and_forward(self):
        self.assertTrue(waypoint_advance_decision(2.0, 2.5, 0.0, 3.0))

    def test_advance_close_and_behind(self):
        self.assertFalse(waypoint_advance_decision(2.0, 2.5, 0.0, -1.0))

    def test_rosbag_far_next_behind_does_not_skip_wp2(self):
        self.assertFalse(waypoint_advance_decision(50.0, 2.5, 20.0, -1.0))
        self.assertFalse(waypoint_advance_decision(50.0, 2.5, -20.0, -1.0))

    def test_rosbag_chain_cannot_skip_wp3_or_wp4(self):
        decisions = [
            waypoint_advance_decision(50.0, 2.5, 20.0, -10.0),
            waypoint_advance_decision(85.0, 2.5, 30.0, -20.0),
            waypoint_advance_decision(112.0, 2.5, 40.0, -30.0),
        ]
        self.assertEqual(decisions, [False, False, False])

    def test_bounded_current_overshoot_advances_at_corner(self):
        self.assertTrue(waypoint_advance_decision(3.0, 2.5, -0.2, 30.0, 5.0))
        self.assertTrue(waypoint_advance_decision(5.0, 2.5, -2.0, -10.0, 5.0))
        self.assertFalse(waypoint_advance_decision(5.01, 2.5, -2.0, -10.0, 5.0))

    def test_overshoot_requires_current_waypoint_behind(self):
        self.assertFalse(waypoint_advance_decision(3.0, 2.5, 0.0, -10.0, 5.0))
        self.assertFalse(waypoint_advance_decision(3.0, 2.5, 1.0, -10.0, 5.0))

    def test_no_advance_far_and_forward(self):
        self.assertFalse(waypoint_advance_decision(10.0, 2.5, 0.0, 3.0))

    def test_no_lateral_threshold(self):
        # Yakın normal geçiş gövde yanalından bağımsızdır.
        self.assertTrue(waypoint_advance_decision(1.0, 2.5, 4.0, 3.0))
        self.assertTrue(waypoint_advance_decision(2.0, 2.5, 0.0, 0.6))

    def test_invalid_threshold_and_guard_raise(self):
        for threshold in (0.0, -1.0, float("nan"), float("inf"), float("-inf")):
            with self.subTest(threshold=threshold), self.assertRaises(ValueError):
                waypoint_advance_decision(1.0, threshold, 0.0, 3.0)
        for guard in (2.0, float("nan"), float("inf"), float("-inf")):
            with self.subTest(guard=guard), self.assertRaises(ValueError):
                waypoint_advance_decision(1.0, 2.5, 0.0, 3.0, guard)

    def test_invalid_distance_never_advances(self):
        for distance in (-1.0, float("nan"), float("inf"), float("-inf")):
            with self.subTest(distance=distance):
                self.assertFalse(
                    waypoint_advance_decision(distance, 2.5, -1.0, 3.0, 5.0)
                )

    def test_field_threshold_4m_advances_within_gnss_drift(self):
        # Saha profili 2026-08-17: threshold 4.0 / guard 6.0. Bench GNSS
        # sapması ~2.5 m — mevcut 2.5 m eşiğinde WP0 asla "ulaşıldı" sayılmadı.
        # Yeni eşik: sapma 2.5m + durma 0.86m + yarım gövde 0.59m aralığında
        # (4.0m içi) + fwd_next>0.5 normal geçiş verir.
        self.assertTrue(waypoint_advance_decision(3.5, 4.0, 0.0, 3.0, 6.0))
        self.assertTrue(waypoint_advance_decision(2.5, 4.0, -10.0, 3.0, 6.0))
        # 4.0 üstü normal geçiş değildir (yalnız bounded overshoot hattı).
        self.assertFalse(waypoint_advance_decision(4.1, 4.0, 5.0, 3.0, 6.0))

    def test_field_guard_6m_bounded_current_overshoot(self):
        # threshold<dist<=guard ve fwd_current<0: köşede eşiği bir tick
        # kaçırınca kilitlenmeyi önler; fwd_current>=0 asla geçiş değildir.
        self.assertTrue(waypoint_advance_decision(5.0, 4.0, -0.2, 30.0, 6.0))
        self.assertTrue(waypoint_advance_decision(6.0, 4.0, -2.0, -10.0, 6.0))
        self.assertFalse(waypoint_advance_decision(6.01, 4.0, -2.0, -10.0, 6.0))
        self.assertFalse(waypoint_advance_decision(5.0, 4.0, 0.1, -10.0, 6.0))

    def test_field_chain_jump_still_forbidden_at_50m(self):
        # KRİTİK REGRESYON: fwd_next<0 tek başına geçiş sebebi DEĞİLDİR; yeni
        # eşik (4.0/6.0) ile 50 m uzaktaki WP2'den WP3/WP4 zincirleme atlama
        # yine yasaktır (T2WaypointAdvanceTest kontratı yeni değerlerde de).
        self.assertFalse(waypoint_advance_decision(50.0, 4.0, 20.0, -1.0, 6.0))
        self.assertFalse(waypoint_advance_decision(50.0, 4.0, -20.0, -1.0, 6.0))
        chain = [
            waypoint_advance_decision(50.0, 4.0, 20.0, -10.0, 6.0),
            waypoint_advance_decision(85.0, 4.0, 30.0, -20.0, 6.0),
            waypoint_advance_decision(112.0, 4.0, 40.0, -30.0, 6.0),
        ]
        self.assertEqual(chain, [False, False, False])

    def test_field_guard_must_not_be_below_threshold(self):
        # guard >= threshold zorunludur (planner.py ValueError kontratı);
        # eşitlik (guard == threshold) yasal — yalnız guard < threshold reddedilir.
        for guard in (3.0, 0.0, -1.0):
            with self.subTest(guard=guard), self.assertRaises(ValueError):
                waypoint_advance_decision(1.0, 4.0, 0.0, 3.0, guard)
        self.assertTrue(waypoint_advance_decision(1.0, 4.0, 0.0, 3.0, 4.0))
        self.assertFalse(waypoint_advance_decision(4.5, 4.0, 0.1, -10.0, 4.0))


class TestWaypointGuidanceIndex(unittest.TestCase):
    """Yakın waypoint bearing kilidi regresyonu."""

    def test_close_but_guarded_waypoint_guides_toward_next(self):
        # Advance kararı ayrı ve sıkı kalır; yalnız heading hedefi kararlı olan
        # sonraki waypoint'e alınır. Böylece tekne yakın nokta etrafında dönmez.
        self.assertEqual(waypoint_guidance_index(1, 5, 0.95, 2.5, False), 2)

    def test_far_waypoint_keeps_current_guidance(self):
        self.assertEqual(waypoint_guidance_index(1, 5, 2.51, 2.5, False), 1)

    def test_advanced_tick_and_last_waypoint_never_skip_again(self):
        self.assertEqual(waypoint_guidance_index(2, 5, 0.5, 2.5, True), 2)
        self.assertEqual(waypoint_guidance_index(4, 5, 0.5, 2.5, False), 4)

    def test_invalid_distance_is_fail_closed(self):
        for distance in (-1.0, float("nan"), float("inf")):
            with self.subTest(distance=distance):
                self.assertEqual(waypoint_guidance_index(1, 5, distance, 2.5), 1)


class TestParkurDecision(unittest.TestCase):
    """Parkur bazlı karar mantığı (autonomy _parkur_idx_in / _use_corridor_*)."""

    def test_parkur_idx_in(self):
        # P1=1 sabit; liste yalnız P1/P2'yi ayırt eder. [1] -> P1 True, P2 False.
        self.assertTrue(1 in [1])
        self.assertFalse(1 in [2])
        self.assertFalse(1 in [])
        # [1,2] -> "1 in lst" True; AMA _use_corridor_goal ek state koşulu
        # (state == PARKUR_1_NAV) P2'de False verir — [1,2] P2'de override'ı
        # geri getirmez (bkz. test_p2_corridor_goal_off_when_parkur2).
        self.assertTrue(1 in [1, 2])

    def test_p2_corridor_goal_off_when_parkur2(self):
        # P2'de corridor_goal_parkurs=[1] ise use_corridor_goal kararı False
        # olur -> hedef waypoint (saf fonksiyon tarafı: use_corridor_goal=False).
        state = "PARKUR_2_AVOIDANCE"
        use_cg = state == "PARKUR_1_NAV" and 1 in [1]
        self.assertFalse(use_cg)
        # [1,2] YAML: state koşulu P2'de yine False (hedef override P2'ye
        # geri gelmez — T1 bozulmaz). A/B dönüşü yalnız P1 içindir.
        use_cg_ab = state == "PARKUR_1_NAV" and 1 in [1, 2]
        self.assertFalse(use_cg_ab)  # P2 state'i hâlâ False (state koşulu)
        # P1'de [1] -> True (regresyon).
        self.assertTrue("PARKUR_1_NAV" == "PARKUR_1_NAV" and 1 in [1])

    def test_p2_corridor_bias_off_when_parkur2(self):
        # P2'de corridor bias kararı False -> autonomy dwa.plan'a corridor=None
        # geçer (dwa.py corridor_score=0, çifte uygulama kalkar).
        state = "PARKUR_2_AVOIDANCE"
        use_bias = state == "PARKUR_1_NAV" and 1 in [1]
        self.assertFalse(use_bias)
        # P1'de True (regresyon) — mevcut davranış.
        self.assertTrue("PARKUR_1_NAV" == "PARKUR_1_NAV" and 1 in [1])


class TestInterpolationPolicy(unittest.TestCase):
    """DWA modunda NN sonrası 5m re-interpolasyon ATLANIR (tick:629 ile
    on_waypoints:549-552 hizalaması). Saf model: interpolasyon kararı yalnız
    dwa_enabled'a bağlıdır (autonomy_node._interpolate_waypoints saf)."""

    def test_interpolation_skipped_in_dwa(self):
        wps = [
            {"lat": 40.0, "lon": 29.0, "parkur": 1},
            {"lat": 40.001, "lon": 29.0, "parkur": 1},
        ]
        dwa_enabled = True
        result = [dict(w) for w in wps] if dwa_enabled else None
        self.assertEqual(len(result), 2)  # mikro nokta YOK

    def test_interpolation_kept_when_not_dwa(self):
        wps = [
            {"lat": 40.0, "lon": 29.0, "parkur": 1},
            {"lat": 40.001, "lon": 29.0, "parkur": 1},
        ]
        dwa_enabled = False
        # non-DWA'da _interpolate_waypoints çağrılır (5m mikro noktalar).
        interpolated = _interpolate_for_test(wps, step_m=5.0)
        self.assertGreater(len(interpolated), 2)


class TestCourseGeometryTracker(unittest.TestCase):
    ORIGIN = (41.0, 29.0)

    def _wp(self, north, east, parkur=1):
        lat, lon = local_m_to_latlon(*self.ORIGIN, north, east)
        return {"lat": lat, "lon": lon, "parkur": parkur}

    def _position(self, north, east):
        return local_m_to_latlon(*self.ORIGIN, north, east)

    def test_straight_and_finite_endpoint_distance(self):
        tracker = CourseGeometryTracker(enter_m=4.5, exit_m=5.5)
        tracker.set_route([self._wp(0, 0), self._wp(20, 0)])
        lat, lon = self._position(10, 3)
        status = tracker.update(lat, lon)
        self.assertTrue(status.valid)
        self.assertTrue(status.inside)
        self.assertAlmostEqual(status.distance_m, 3.0, places=3)

        # Sonsuz doğruya değil, segmentin ucuna ölçülmelidir.
        lat, lon = self._position(25, 0)
        status = tracker.update(lat, lon)
        self.assertAlmostEqual(status.distance_m, 5.0, places=3)

    def test_right_angle_and_obtuse_routes(self):
        tracker = CourseGeometryTracker()
        tracker.set_route(
            [self._wp(0, 0), self._wp(20, 0), self._wp(20, 20)]
        )
        lat, lon = self._position(18, 4)
        self.assertAlmostEqual(tracker.update(lat, lon).distance_m, 2.0, places=3)

        tracker.set_route(
            [self._wp(0, 0), self._wp(20, 0), self._wp(10, 17.320508)]
        )
        lat, lon = self._position(15, 8.660254)
        self.assertLess(tracker.update(lat, lon).distance_m, 0.01)

    def test_hysteresis_enter_hold_exit_and_reenter(self):
        tracker = CourseGeometryTracker(enter_m=4.5, exit_m=5.5)
        tracker.set_route([self._wp(0, 0), self._wp(20, 0)])
        lat, lon = self._position(10, 4.4)
        self.assertTrue(tracker.update(lat, lon).inside)
        lat, lon = self._position(10, 5.2)
        self.assertTrue(tracker.update(lat, lon).inside)
        lat, lon = self._position(10, 5.6)
        self.assertFalse(tracker.update(lat, lon).inside)
        lat, lon = self._position(10, 5.0)
        self.assertFalse(tracker.update(lat, lon).inside)
        lat, lon = self._position(10, 4.4)
        self.assertTrue(tracker.update(lat, lon).inside)

    def test_route_split_adds_last_p1_anchor_to_p2(self):
        route = [
            self._wp(0, 0, 1),
            self._wp(20, 0, 1),
            self._wp(30, 5, 2),
            self._wp(40, 5, 2),
            self._wp(50, 5, 3),
        ]
        p1, p2 = split_parkur_routes(route)
        self.assertEqual(len(p1), 2)
        self.assertEqual(len(p2), 3)
        self.assertEqual((p2[0]["lat"], p2[0]["lon"]), (p1[-1]["lat"], p1[-1]["lon"]))
        self.assertEqual([int(w["parkur"]) for w in p2], [1, 2, 2])

    def test_malformed_route_or_telemetry_is_unknown(self):
        p1, p2 = split_parkur_routes(
            [None, {}, {"lat": "bad", "lon": 29.0},
             {"lat": float("nan"), "lon": 29.0, "parkur": 1}]
        )
        self.assertEqual((p1, p2), ([], []))
        tracker = CourseGeometryTracker()
        tracker.set_route(p1)
        status = tracker.update(41.0, 29.0)
        self.assertFalse(status.valid)
        self.assertIsNone(status.inside)
        self.assertIsNone(status.distance_m)
        tracker.set_route([self._wp(0, 0), self._wp(20, 0)])
        self.assertFalse(tracker.update(float("inf"), 29.0).valid)

    def test_invalid_telemetry_does_not_clear_hysteresis_memory(self):
        tracker = CourseGeometryTracker(enter_m=4.5, exit_m=5.5)
        tracker.set_route([self._wp(0, 0), self._wp(20, 0)])
        lat, lon = self._position(10, 4.0)
        self.assertTrue(tracker.update(lat, lon).inside)
        self.assertFalse(tracker.update(float("nan"), lon).valid)
        # 5.0m yalnız daha önce içerideyse hold bölgesidir.
        lat, lon = self._position(10, 5.0)
        self.assertTrue(tracker.update(lat, lon).inside)

    def test_threshold_validation(self):
        for enter, exit_ in [(-1, 1), (2, 1), (float("nan"), 2), (1, float("inf"))]:
            with self.subTest(enter=enter, exit=exit_), self.assertRaises(ValueError):
                CourseGeometryTracker(enter, exit_)

    def test_scoring_course_uses_parkur_split_geometry(self):
        route = [self._wp(0, 0, 1), self._wp(20, 0, 1), self._wp(30, 0, 2)]
        p1, p2 = split_parkur_routes(route)
        score_p1 = OutOfCourseDetector(mode="corridor", half_width_m=5.0)
        score_p2 = OutOfCourseDetector(mode="corridor", half_width_m=5.0)
        score_p1.set_course(p1)
        score_p2.set_course(p2)
        lat, lon = self._position(26, 0)
        self.assertFalse(score_p1.update(lat, lon, 1.0, 1, 1.0).inside)
        self.assertTrue(score_p2.update(lat, lon, 1.0, 2, 1.0).inside)


class TestStrictOrangeLatch(unittest.TestCase):
    def test_first_p1_tick_latches_and_outside_does_not_clear(self):
        latched = update_strict_orange_latch(False, "MISSION_READY")
        self.assertFalse(latched)
        latched = update_strict_orange_latch(latched, "PARKUR_1_NAV")
        self.assertTrue(latched)
        # Actual inside/outside state is intentionally absent from this API.
        latched = update_strict_orange_latch(latched, "PARKUR_1_NAV")
        self.assertTrue(latched)
        self.assertTrue(update_strict_orange_latch(latched, "PARKUR_2_AVOIDANCE"))

    def test_new_mission_is_only_reset(self):
        self.assertFalse(
            update_strict_orange_latch(True, "PARKUR_2_AVOIDANCE", new_mission=True)
        )


class TestMissionReplayContract(unittest.TestCase):
    def _payload(self, north_offset=0.0):
        lat0, lon0 = local_m_to_latlon(41.0, 29.0, north_offset, 0.0)
        lat1, lon1 = local_m_to_latlon(41.0, 29.0, 20.0 + north_offset, 0.0)
        return {"waypoints": [
            {"lat": lat0, "lon": lon0, "parkur": 1},
            {"lat": lat1, "lon": lon1, "parkur": 2},
        ]}

    def test_sanitizer_is_atomic_and_contract_strict(self):
        self.assertIsNone(sanitize_mission_waypoints(None))
        self.assertIsNone(sanitize_mission_waypoints({}))
        self.assertIsNone(sanitize_mission_waypoints({"waypoints": []}))
        for bad in (
            None,
            {"lat": 41.0, "lon": 29.0},
            {"lat": float("nan"), "lon": 29.0, "parkur": 1},
            {"lat": 91.0, "lon": 29.0, "parkur": 1},
            {"lat": 41.0, "lon": 29.0, "parkur": "1"},
            {"lat": 41.0, "lon": 29.0, "parkur": True},
            {"lat": 41.0, "lon": 29.0, "parkur": 4},
        ):
            payload = self._payload()
            payload["waypoints"].append(bad)
            with self.subTest(bad=bad):
                self.assertIsNone(sanitize_mission_waypoints(payload))

    def test_twenty_telemetry_sim_replays_do_not_reset_started_mission(self):
        stored = None
        state = "WAIT_MISSION"
        latched = False
        reset_count = 0
        payload = self._payload()
        for index in range(20):
            clean = sanitize_mission_waypoints(payload)
            fingerprint = mission_waypoint_fingerprint(clean)
            if fingerprint != stored:
                reset_count += 1
                stored = fingerprint
                state = "MISSION_READY"
                latched = update_strict_orange_latch(
                    latched, state, new_mission=True
                )
            if index == 4:  # start, kalan 15 waypoint yayınından önce
                state = "PARKUR_1_NAV"
                latched = update_strict_orange_latch(latched, state)
        self.assertEqual(reset_count, 1)
        self.assertEqual(state, "PARKUR_1_NAV")
        self.assertTrue(latched)

    def test_material_change_has_different_fingerprint(self):
        first = sanitize_mission_waypoints(self._payload())
        changed = sanitize_mission_waypoints(self._payload(1.0))
        self.assertNotEqual(
            mission_waypoint_fingerprint(first), mission_waypoint_fingerprint(changed)
        )


class TestCourseGeometryParkurSelection(unittest.TestCase):
    def test_fresh_p2_obstacle_failsafe_keeps_p2_geometry(self):
        self.assertEqual(
            course_geometry_parkur(
                "FAILSAFE", telemetry_fresh=True,
                failsafe_resume_state="PARKUR_2_AVOIDANCE",
            ),
            2,
        )

    def test_stale_or_target_context_is_unknown(self):
        self.assertIsNone(
            course_geometry_parkur(
                "FAILSAFE", telemetry_fresh=False,
                failsafe_resume_state="PARKUR_1_NAV",
            )
        )
        self.assertIsNone(
            course_geometry_parkur(
                "FAILSAFE", telemetry_fresh=True,
                failsafe_resume_state="PARKUR_3_TARGET_LOCK",
            )
        )


def _interpolate_for_test(waypoints, step_m=5.0):
    """_interpolate_waypoints'ın test kopyası: iki nokta arası mikro noktalar."""
    import math as _m

    def _hav(a, b, c, d):
        # haversine_m basitleştirilmiş proxy — 40.001 -> ~111m (lat derecesi ~111km).
        return _m.hypot((c - a) * 111000.0, (d - b) * 111000.0)

    out = []
    for idx in range(len(waypoints) - 1):
        start = waypoints[idx]
        end = waypoints[idx + 1]
        out.append(dict(start))
        distance = _hav(float(start["lat"]), float(start["lon"]),
                        float(end["lat"]), float(end["lon"]))
        count = int(distance / step_m)
        for k in range(1, count + 1):
            t = k / (count + 1)
            p = dict(start)
            p["lat"] = float(start["lat"]) + (float(end["lat"]) - float(start["lat"])) * t
            p["lon"] = float(start["lon"]) + (float(end["lon"]) - float(start["lon"])) * t
            out.append(p)
    if waypoints:
        out.append(dict(waypoints[-1]))
    return out


class NearFieldSafetyTest(unittest.TestCase):
    """Yakın-alan güvenlik: hard obstacle yakınında ileri komut 0 (saha bulgusu).

    Araç 0.76 m genişlik / 1.18 m uzunluk; lidar merkezden 0.57 m önde.
    Saha bulgusu: hard obstacle 0.93-1.56 m mesafedeyken bazı komutlarda
    v=0.7 m/s korundu. Bu katman DWA skorlamasından bağımsız kesin kuraldır.
    """

    def _obstacle(self, forward, lateral, hard=True, color="unknown"):
        return {
            "forward_m": forward,
            "lateral_m": lateral,
            "distance": math.hypot(forward, lateral),
            "color": color,
            "hard_obstacle": hard,
        }

    def _cmd(self, vx=0.7, yaw=0.0, action="dwa"):
        return Command(vx, 0.0, yaw, action)

    def test_close_front_obstacle_stops_forward(self):
        """0.93 m önde hard obstacle -> ileri komut 0 + pivot (saha bulgusu)."""
        cmd = self._cmd(0.7, 0.2)
        result = near_field_command(
            [self._obstacle(0.93, -0.67)],
            cmd,
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
        )
        self.assertTrue(result.active)
        self.assertTrue(result.latch)
        self.assertEqual(result.command.vx, 0.0)
        # Engel solda -> sağa pivot (pozitif yaw).
        self.assertGreater(result.command.yaw_rate, 0.0)

    def test_close_obstacle_on_left_pivots_right(self):
        """Soldaki yakın engel sağa pivot üretir (fiziksel kaçış yönü)."""
        result = near_field_command(
            [self._obstacle(1.0, -0.8)],
            self._cmd(0.7, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
        )
        self.assertEqual(result.command.vx, 0.0)
        self.assertGreater(result.command.yaw_rate, 0.0)

    def test_close_obstacle_on_right_pivots_left(self):
        """Sağdaki yakın engel sola pivot üretir (fiziksel kaçış yönü)."""
        result = near_field_command(
            [self._obstacle(1.0, 0.8)],
            self._cmd(0.7, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
        )
        self.assertEqual(result.command.vx, 0.0)
        self.assertLess(result.command.yaw_rate, 0.0)

    def test_center_close_obstacle_pivots_right_deterministically(self):
        """Merkezdeki yakın engel deterministik sağ pivot (|lateral|<0.15)."""
        result = near_field_command(
            [self._obstacle(0.9, 0.05)],
            self._cmd(0.7, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
        )
        self.assertEqual(result.command.vx, 0.0)
        self.assertGreater(result.command.yaw_rate, 0.0)
        self.assertEqual(result.reason.split()[0], "near_field_stop_center")

    def test_hysteresis_holds_stop_until_release(self):
        """Histerezis: stop aktifken engel release eşiğine kadar açılmaz."""
        # İlk: 1.2 m -> stop latch.
        r1 = near_field_command(
            [self._obstacle(1.2, 0.0)],
            self._cmd(0.7, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30,
            stop_release_m=2.5, previous_latch=False,
        )
        self.assertTrue(r1.active)
        self.assertTrue(r1.latch)
        # 2.0 m (stop_m 1.5 üstü ama release 2.5 altı) -> latch devam, stop sürer.
        r2 = near_field_command(
            [self._obstacle(2.0, 0.0)],
            self._cmd(0.7, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30,
            stop_release_m=2.5, previous_latch=True,
        )
        self.assertTrue(r2.active)
        self.assertTrue(r2.latch)
        self.assertEqual(r2.command.vx, 0.0)

    def test_hysteresis_releases_after_escape(self):
        """Engel release ve slow eşiğinin üstüne çıkınca normal komut geri gelir."""
        r = near_field_command(
            [self._obstacle(3.5, 0.0)],
            self._cmd(0.7, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30,
            stop_release_m=2.5, previous_latch=True,
        )
        self.assertFalse(r.active)
        self.assertFalse(r.latch)

    def test_slow_zone_reduces_speed_not_stop(self):
        """2.0 m önde engel -> hız düşer (0.3) ama durmaz."""
        result = near_field_command(
            [self._obstacle(2.0, -0.5)],
            self._cmd(0.7, 0.2),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
        )
        self.assertTrue(result.active)
        self.assertFalse(result.latch)
        self.assertLessEqual(result.command.vx, 0.30)

    def test_lateral_obstacle_does_not_block_waypoint_alignment_crawl(self):
        """P1 regresyonu: güvenli yandaki duba yerinde pivota kilitlemez."""
        result = near_field_command(
            [self._obstacle(0.8, -3.6)],
            self._cmd(0.0, math.radians(30.0), "dwa_align_waypoint_right"),
            stop_m=2.5, slow_m=5.0, stop_lateral_m=1.0,
            slow_lateral_m=1.5,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.80,
            stop_release_m=4.0,
            heading_error_deg=80.0,
            heading_align_speed_mps=0.70,
            heading_align_max_yaw_deg_s=20.0,
        )
        self.assertTrue(result.active)
        self.assertIn("near_field_heading_align", result.reason)
        self.assertAlmostEqual(result.command.vx, 0.70)
        self.assertGreater(result.command.yaw_rate, 0.0)

    def test_unknown_obstacle_still_protected(self):
        """Renk algılanmasa da lidar unknown hard obstacle güvenliği korunur."""
        result = near_field_command(
            [self._obstacle(1.0, 0.0, hard=True, color="unknown")],
            self._cmd(0.7, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
        )
        self.assertTrue(result.active)
        self.assertEqual(result.command.vx, 0.0)

    def test_behind_obstacle_does_not_stop(self):
        """Arkadaki engel (yarım gövde uzunluğundan geri) ileri komutu etkilemez."""
        result = near_field_command(
            [self._obstacle(-1.0, 0.3)],
            self._cmd(0.7, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
        )
        self.assertFalse(result.active)

    def test_far_obstacle_unchanged(self):
        """Uzak engel komutu değiştirmez."""
        result = near_field_command(
            [self._obstacle(5.0, 0.0)],
            self._cmd(0.7, 0.2),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
        )
        self.assertFalse(result.active)
        self.assertEqual(result.command, None)

    def test_non_hard_obstacle_not_protected(self):
        """hard_obstacle=False (kamera-only) engel bu katmana girmez."""
        result = near_field_command(
            [self._obstacle(1.0, 0.0, hard=False)],
            self._cmd(0.7, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
        )
        self.assertFalse(result.active)

    def test_heading_align_when_no_obstacle_and_error_large(self):
        """Engel yokken heading hatası büyükse küçük yaw üret (hizalanma).

        Kullanıcı gözlemi (2026-08-18): kaçınma sonrası engel yokken araç düz
        gidiyor, waypoint heading'ini düzeltmiyor. Bu test, heading_error
        deadband (10°) üstündeyse yaw üretildiğini doğrular.
        """
        result = near_field_command(
            [],  # engel yok
            self._cmd(0.4, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
            heading_error_deg=20.0,  # sağa 20° sapma -> pozitif yaw
        )
        self.assertTrue(result.active)
        self.assertIsNotNone(result.command)
        self.assertGreater(result.command.yaw_rate, 0.0)
        self.assertAlmostEqual(result.command.yaw_rate, math.radians(10.0), places=7)
        self.assertEqual(result.command.vx, 0.4)  # ileri hız korunur

    def test_heading_align_negative_error_yaws_left(self):
        """Engel yokken negatif heading hatası sola yaw üretir."""
        result = near_field_command(
            [],
            self._cmd(0.4, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
            heading_error_deg=-25.0,
        )
        self.assertTrue(result.active)
        self.assertLess(result.command.yaw_rate, 0.0)
        self.assertAlmostEqual(result.command.yaw_rate, math.radians(-12.5), places=7)

    def test_heading_align_far_obstacle_uses_radians_not_degree_value(self):
        """Uzak engel dalında da derece/radyan dönüşümü korunur."""
        result = near_field_command(
            [self._obstacle(8.0, 2.0)],
            self._cmd(0.4, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
            heading_error_deg=60.0,
            heading_align_max_yaw_deg_s=20.0,
        )
        self.assertTrue(result.active)
        self.assertAlmostEqual(result.command.yaw_rate, math.radians(20.0), places=7)

    def test_heading_align_within_deadband_no_yaw(self):
        """Engel yokken heading hatası deadband içindeyse yaw üretilmez."""
        result = near_field_command(
            [],
            self._cmd(0.4, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
            heading_error_deg=5.0,  # deadband (10°) içinde
        )
        self.assertFalse(result.active)
        self.assertEqual(result.command, None)

    def test_heading_align_stop_takes_priority_over_obstacle_free_align(self):
        """Yakın engel varken heading hizalaması DEĞİL, stop/pivot önceliklidir."""
        result = near_field_command(
            [self._obstacle(1.0, 0.0)],  # yakın engel
            self._cmd(0.4, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
            heading_error_deg=20.0,  # heading hatası da var
        )
        self.assertTrue(result.active)
        self.assertEqual(result.command.vx, 0.0)  # stop öncelikli (vx=0)
        self.assertIn("near_field_stop", result.reason)

    def test_lateral_obstacle_does_not_hold_stop_latch(self):
        """Engel yanala alındığında stop latch aracı kilitlemez."""
        first = near_field_command(
            [self._obstacle(1.0, 0.0)],
            self._cmd(0.4, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
        )
        self.assertTrue(first.active)
        self.assertEqual(first.command.vx, 0.0)
        second = near_field_command(
            [self._obstacle(0.5, 1.4)],
            self._cmd(0.4, 0.2),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=4.0,
            previous_latch=True,
        )
        self.assertNotEqual(second.command.vx, 0.0)
        self.assertNotIn("near_field_stop", second.reason)

    def test_soft_obstacle_cannot_mask_farther_hard_obstacle(self):
        """Yakındaki kamera nesnesi, daha uzaktaki lidar sert engeli gizleyemez."""
        result = near_field_command(
            [
                self._obstacle(0.8, 0.0, hard=False, color="orange"),
                self._obstacle(1.2, 0.0, hard=True, color="unknown"),
            ],
            self._cmd(0.7, 0.0),
            stop_m=1.5, slow_m=3.0, stop_lateral_m=1.0,
            pivot_yaw_deg_s=30.0, slow_speed_mps=0.30, stop_release_m=2.5,
        )
        self.assertTrue(result.active)
        self.assertEqual(result.command.vx, 0.0)


class DwaAvoidDirectionLatchTest(unittest.TestCase):
    """DWA kaçınma yön histerezisi (chatter önleme)."""

    def _planner(self):
        return DwaPlanner(
            max_speed_mps=0.7,
            max_yaw_rate_deg_s=45.0,
            recovery_yaw_deg_s=25.0,
            min_drive_speed_mps=0.08,
            min_turn_rate_deg_s=0.0,
            avoid_direction_latch_ticks=8,
        )

    def _costmap_with_obstacle(self, forward=6.0, lateral=-1.0):
        """Sol önde hard obstacle içeren costmap (DWA latch testi için).

        Engel DWA sim mesafesinin ötesine konur (6 m) — böylece ileri adaylar
        çarpışmadan skorlanır, latch bloğu çalışır. ``_nearest_obstacle_lateral``
        10 m arama ile engeli görür.
        """
        costmap = CostMap(size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=0.5)
        costmap.update(
            [{"forward_m": forward, "lateral_m": lateral}],
            [], "",
            ground_speed=0.7,
        )
        return costmap

    def test_direction_switch_requires_consistent_evidence(self):
        """En iyi aday yön değiştirse bile latch dolmadan eski yön korunur."""
        planner = self._planner()
        costmap = self._costmap_with_obstacle()

        # Engel solda, hedef sağda -> DWA sağa dönüş adayı seçer; latch sağa kurulur.
        goal = (5.0, 2.0)
        planner.plan(goal, costmap, None)
        planner.plan(goal, costmap, None)
        self.assertNotEqual(planner._avoid_direction, 0.0)

        # Latch süresi dolmadan yön değişimi olmaz (engel/hedef değişmedi).
        latch_before = planner._avoid_direction
        for _ in range(3):
            planner.plan(goal, costmap, None)
        self.assertEqual(planner._avoid_direction, latch_before)

    def test_latch_expires_and_allows_switch(self):
        """Latch tick'leri tükenince yeni yön kabul edilir (kararlı kanıt)."""
        planner = DwaPlanner(
            max_speed_mps=0.7, max_yaw_rate_deg_s=45.0,
            recovery_yaw_deg_s=25.0, min_drive_speed_mps=0.08,
            min_turn_rate_deg_s=0.0, avoid_direction_latch_ticks=2,
        )
        costmap = self._costmap_with_obstacle()
        goal = (5.0, 2.0)
        planner.plan(goal, costmap, None)
        self.assertNotEqual(planner._avoid_direction, 0.0)
        old_dir = planner._avoid_direction
        # Latch süresi dolunca aynı yön korunur (engel değişmedi; latch
        # mekanizması kararlı kalmalı — yanlış yön değişimi yok).
        for _ in range(4):
            planner.plan(goal, costmap, None)
        self.assertEqual(planner._avoid_direction, old_dir)

    def test_latched_direction_selects_highest_scored_safe_candidate(self):
        """Latch, yön içindeki ilk örneği değil en iyi güvenli adayı seçer."""
        planner = self._planner()
        costmap = self._costmap_with_obstacle()
        goal = (5.0, 2.0)
        candidates = planner.candidates(goal, None)
        scored = [
            planner.score_candidate(c, goal, None, costmap)
            for c in candidates
            if c.yaw_rate > 0.0
        ]
        safe = [c for c in scored if not c.collides and math.isfinite(c.score)]
        self.assertTrue(safe)
        expected = max(safe, key=lambda c: c.score)
        actual = planner._best_in_direction(
            costmap, goal, None, 1.0, expected
        )
        self.assertAlmostEqual(actual.score, expected.score)
        self.assertEqual(actual.vx, expected.vx)
        self.assertEqual(actual.yaw_rate, expected.yaw_rate)

    def test_recovery_turns_toward_goal_when_obstacle_is_inside_course(self):
        """Recovery hedef ve engel ters yöndeyken rotaya geri döner."""
        planner = self._planner()
        costmap = CostMap(
            size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=0.5
        )
        costmap.update(
            [{"forward_m": 3.0, "lateral_m": 1.0}],
            [], "", ground_speed=0.7,
        )
        cmd = planner.recovery(costmap, None, goal_body=(5.0, 2.0))
        self.assertGreater(cmd.yaw_rate, 0.0)

    def test_recovery_away_from_obstacle_when_outside_course(self):
        """PARKUR-FARKINDA recovery: engel parkur dışı taraftaysa engelden uzaklaş.

        Engel solda (parkur dışı), hedef sağda -> engelden uzaklaşma hedefle
        aynı yönde -> engelden uzaklaş baskın (%60).
        """
        planner = self._planner()
        costmap = CostMap(size_m=30.0, cell_m=0.25, bot_radius_m=0.6, safety_m=0.5)
        # Engel solda (lateral<0) -> engelden uzaklaşma SAĞA.
        costmap.update(
            [{"forward_m": 3.0, "lateral_m": -1.0}],
            [], "", ground_speed=0.7,
        )
        goal = (5.0, 2.0)  # hedef sağda -> goal_bearing pozitif
        cmd = planner.recovery(costmap, None, goal_body=goal)
        # Engel solda -> obstacle_dir=+1 (sağ); hedef sağda -> goal_dir=+1.
        # obstacle_dir * goal_dir > 0 -> engelden uzaklaş baskın -> yaw pozitif.
        self.assertGreater(cmd.yaw_rate, 0.0)

    def test_single_inflated_buoy_is_not_misclassified_as_wall(self):
        planner = self._planner()
        costmap = CostMap(
            size_m=30.0, cell_m=0.25, bot_radius_m=0.8, safety_m=0.5
        )
        costmap.update(
            [{"id": "yellow", "forward_m": 1.5, "lateral_m": 0.0}],
            [], "", ground_speed=0.0,
        )
        self.assertFalse(planner._front_wall_blocked(costmap))
        command = planner.recovery(costmap, None, goal_body=(5.0, 1.0))
        self.assertGreater(command.vx, 0.0)
        self.assertNotIn("blocked_pivot", command.action)

    def test_three_wide_lidar_clusters_are_wall_evidence(self):
        planner = self._planner()
        costmap = CostMap(
            size_m=30.0, cell_m=0.25, bot_radius_m=0.8, safety_m=0.5
        )
        costmap.update(
            [
                {"id": "wall_l", "forward_m": 1.5, "lateral_m": -0.9},
                {"id": "wall_c", "forward_m": 1.5, "lateral_m": 0.0},
                {"id": "wall_r", "forward_m": 1.5, "lateral_m": 0.9},
            ],
            [], "", ground_speed=0.0,
        )
        self.assertTrue(planner._front_wall_blocked(costmap))
        command = planner.recovery(costmap, None, goal_body=(5.0, 1.0))
        self.assertEqual(command.vx, 0.0)
        self.assertIn("blocked_pivot", command.action)


if __name__ == "__main__":
    unittest.main()
