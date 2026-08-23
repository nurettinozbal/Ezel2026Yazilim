"""Acceptance matrix for the ROS-independent sensor fusion core."""

import math
import os
import sys
import time
import unittest

PACKAGE_ROOT = os.path.abspath(os.path.dirname(__file__))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from ida_sensor_fusion.core import (
    CameraDetection,
    FusionCore,
    LidarCluster,
    ReadinessFlags,
    associate_one_to_one,
    classify_time_delta,
    fuse_frames,
    ros_left_to_stack_right,
    sanitize_camera_frame,
    sanitize_lidar_frame,
)
from ida_sensor_fusion.contracts import (
    camera_payload,
    lidar_payload,
    merge_camera_roles,
    result_payloads,
)


READY = ReadinessFlags(True, True, True, True)


def camera(source_id, bearing, color="orange", confidence=0.9):
    return CameraDetection(source_id, bearing, color, confidence)


def lidar(source_id, bearing, distance=10.0):
    angle = math.radians(bearing)
    forward = distance * math.cos(angle)
    right = distance * math.sin(angle)
    return LidarCluster(source_id, forward, right, distance, bearing)


class CoordinateAndTimeContractTest(unittest.TestCase):
    def test_ros_left_to_stack_right_sign_at_plus_minus_30_degrees(self):
        root3 = math.sqrt(3.0)
        forward, right = ros_left_to_stack_right(root3, 1.0)
        self.assertAlmostEqual(math.degrees(math.atan2(right, forward)), -30.0, places=6)
        forward, right = ros_left_to_stack_right(root3, -1.0)
        self.assertAlmostEqual(math.degrees(math.atan2(right, forward)), 30.0, places=6)

    def test_exact_temporal_boundaries(self):
        cases = [
            (0.049, "healthy", 49.0),
            (0.050, "healthy", 50.0),
            (0.099, "degraded", 99.0),
            (0.100, "degraded", 100.0),
            (0.101, "reject", 101.0),
        ]
        for delta, expected, milliseconds in cases:
            with self.subTest(delta=delta):
                health, dt_ms = classify_time_delta(10.0, 10.0 + delta)
                self.assertEqual(health, expected)
                self.assertAlmostEqual(dt_ms, milliseconds, places=6)

    def test_invalid_time_and_coordinates_are_safe(self):
        self.assertEqual(classify_time_delta(float("nan"), 1.0), ("reject", None))
        self.assertIsNone(ros_left_to_stack_right(float("inf"), 1.0))
        self.assertIsNone(sanitize_camera_frame(float("nan"), []))
        self.assertIsNone(sanitize_lidar_frame(1.0, None))


class AssociationTest(unittest.TestCase):
    def test_maximum_cardinality_precedes_global_minimum_cost(self):
        # Greedy C0->L0 seçerse C1 eşleşemez. Global optimum iki eşleşme:
        # C0->L1 (2deg), C1->L0 (6deg).
        cameras = [camera("c0", 0.0), camera("c1", 7.0)]
        lidars = [lidar("l0", 1.0), lidar("l1", -2.0)]
        matches, amb_c, amb_l = associate_one_to_one(cameras, lidars, 6.0, 0.25)
        self.assertEqual(amb_c, set())
        self.assertEqual(amb_l, set())
        self.assertEqual({(cameras[c].source_id, lidars[l].source_id) for c, l, _ in matches}, {
            ("c0", "l1"), ("c1", "l0")
        })

    def test_permutations_produce_same_identity_mapping(self):
        base_cameras = [camera("c0", 0.0), camera("c1", 7.0)]
        base_lidars = [lidar("l0", 1.0), lidar("l1", -2.0)]
        expected = None
        for cameras in (base_cameras, list(reversed(base_cameras))):
            for lidars in (base_lidars, list(reversed(base_lidars))):
                matches, _, _ = associate_one_to_one(cameras, lidars, 6.0, 0.25)
                identities = {(cameras[c].source_id, lidars[l].source_id) for c, l, _ in matches}
                expected = identities if expected is None else expected
                self.assertEqual(identities, expected)

    def test_ambiguity_and_duplicates_are_rejected(self):
        cameras = [camera("c0", 0.0)]
        lidars = [lidar("l0", -0.1), lidar("l1", 0.1)]
        matches, amb_c, _ = associate_one_to_one(cameras, lidars, 2.0, 0.25)
        self.assertEqual(matches, [])
        self.assertEqual(amb_c, {0})

        duplicate_cameras = [camera("dup", 0.0), camera("dup", 0.0)]
        matches, _, amb_l = associate_one_to_one(duplicate_cameras, [lidar("l0", 0.0)])
        self.assertEqual(matches, [])
        self.assertEqual(amb_l, set())  # duplicate camera identity reddi yeterlidir

        matches, amb_c, _ = associate_one_to_one(
            duplicate_cameras, [lidar("l0", -0.2), lidar("l1", 0.2)]
        )
        self.assertEqual(matches, [])
        self.assertEqual(amb_c, {0, 1})

    def test_global_tie_is_ambiguous_for_all_permutations(self):
        base_cameras = [camera("ca", 0.0), camera("cb", 0.0)]
        base_lidars = [lidar("la", 0.0), lidar("lb", 0.0)]
        for cameras in (base_cameras, list(reversed(base_cameras))):
            for lidars in (base_lidars, list(reversed(base_lidars))):
                matches, amb_c, amb_l = associate_one_to_one(
                    cameras, lidars, 1.0, 0.0
                )
                self.assertEqual(matches, [])
                self.assertEqual(amb_c, {0, 1})
                self.assertEqual(amb_l, {0, 1})

    def test_three_cameras_one_lidar_partner_and_unmatched_possibilities(self):
        base_cameras = [camera("ca", -0.1), camera("cb", 0.0), camera("cc", 0.1)]
        single_lidar = [lidar("l", 0.0)]
        for cameras in (base_cameras, list(reversed(base_cameras))):
            matches, amb_c, amb_l = associate_one_to_one(
                cameras, single_lidar, 1.0, 0.1
            )
            self.assertEqual(matches, [])
            self.assertEqual({cameras[i].source_id for i in amb_c}, {"ca", "cb", "cc"})
            self.assertEqual({single_lidar[i].source_id for i in amb_l}, {"l"})

    def test_one_camera_three_lidars_partner_and_unmatched_possibilities(self):
        single_camera = [camera("c", 0.0)]
        base_lidars = [lidar("la", -0.1), lidar("lb", 0.0), lidar("lc", 0.1)]
        for lidars in (base_lidars, list(reversed(base_lidars))):
            matches, amb_c, amb_l = associate_one_to_one(
                single_camera, lidars, 1.0, 0.1
            )
            self.assertEqual(matches, [])
            self.assertEqual({single_camera[i].source_id for i in amb_c}, {"c"})
            self.assertEqual({lidars[i].source_id for i in amb_l}, {"la", "lb", "lc"})

    def test_alternating_cycle_global_tie_rejects_every_identity(self):
        cameras = [camera(f"c{i}", 0.0) for i in range(3)]
        lidars = [lidar(f"l{i}", 0.0) for i in range(3)]
        matches, amb_c, amb_l = associate_one_to_one(cameras, lidars, 1.0, 0.0)
        self.assertEqual(matches, [])
        self.assertEqual(amb_c, {0, 1, 2})
        self.assertEqual(amb_l, {0, 1, 2})

    def test_global_near_margin_boundary(self):
        cameras = [camera("c", 0.0)]
        lidars = [lidar("best", 0.0), lidar("near", 0.25)]
        matches, amb_c, amb_l = associate_one_to_one(cameras, lidars, 1.0, 0.25)
        self.assertEqual(matches, [])
        self.assertEqual(amb_c, {0})
        self.assertEqual(amb_l, {0, 1})

        matches, amb_c, amb_l = associate_one_to_one(cameras, lidars, 1.0, 0.249)
        self.assertEqual([(c, l) for c, l, _ in matches], [(0, 0)])
        self.assertEqual(amb_c, set())
        self.assertEqual(amb_l, set())


class FusionOutputTest(unittest.TestCase):
    def _frames(self, dt=0.02):
        camera_frame = sanitize_camera_frame(10.0, [
            {"id": "cam-orange", "bearing_deg": 10.0, "color": "orange", "confidence": 0.8},
            {"id": "cam-only", "bearing_deg": -60.0, "color": "green", "confidence": 0.9},
        ])
        # ROS left=-1.763 at forward=10 -> stack right positive, bearing ~+10.
        lidar_frame = sanitize_lidar_frame(10.0 + dt, [
            {"id": "lidar-match", "forward_m": 10.0, "lateral_left_m": -1.7632698},
            {"id": "lidar-only", "forward_m": 8.0, "lateral_left_m": 3.0},
        ])
        return camera_frame, lidar_frame

    def test_matched_uses_lidar_metric_and_camera_color_unmatched_policy(self):
        camera_frame, lidar_frame = self._frames()
        result = fuse_frames(camera_frame, lidar_frame, READY)
        self.assertTrue(result.status.accepted)
        self.assertEqual(result.status.camera_stamp, 10.0)
        self.assertAlmostEqual(result.status.lidar_stamp, 10.02)
        self.assertEqual(result.status.matched_count, 1)
        by_id = {obstacle.lidar_id: obstacle for obstacle in result.obstacles}
        matched = by_id["lidar-match"]
        self.assertEqual(matched.color, "orange")
        self.assertEqual(matched.source, "camera_lidar_fused")
        self.assertAlmostEqual(matched.forward_m, 10.0)
        self.assertAlmostEqual(matched.lateral_m, 1.7632698)
        unknown = by_id["lidar-only"]
        self.assertEqual(unknown.color, "unknown")
        self.assertTrue(unknown.hard_obstacle)
        self.assertEqual({d.camera_id for d in result.camera_diagnostics}, {"cam-only"})
        self.assertEqual(len(result.obstacles), 2)  # camera-only obstacle üretmez

    def test_rejected_time_or_readiness_never_colors_lidar(self):
        camera_frame, lidar_frame = self._frames(dt=0.101)
        rejected = fuse_frames(camera_frame, lidar_frame, READY)
        self.assertFalse(rejected.status.accepted)
        self.assertTrue(all(item.color == "unknown" for item in rejected.obstacles))
        camera_frame, lidar_frame = self._frames(dt=0.01)
        not_ready = fuse_frames(camera_frame, lidar_frame, ReadinessFlags(True, True, True, False))
        self.assertFalse(not_ready.status.accepted)
        self.assertTrue(all(item.color == "unknown" for item in not_ready.obstacles))

    def test_invalid_detections_are_dropped_and_reported(self):
        camera_frame = sanitize_camera_frame(1.0, [
            {"id": "ok", "bearing_deg": 0.0, "color": "orange", "confidence": 0.8},
            {"bearing_deg": float("nan"), "color": "orange", "confidence": 0.8},
            {"bearing_deg": 0.0, "color": "", "confidence": 0.8},
        ])
        lidar_frame = sanitize_lidar_frame(1.0, [
            {"id": "ok", "forward_m": 5.0, "lateral_left_m": 0.0},
            {"forward_m": float("inf"), "lateral_left_m": 0.0},
            {"forward_m": 0.0, "lateral_left_m": 0.0},
        ])
        result = fuse_frames(camera_frame, lidar_frame, READY)
        self.assertEqual(result.status.invalid_camera_count, 2)
        self.assertEqual(result.status.invalid_lidar_count, 2)
        self.assertEqual(result.status.matched_count, 1)

    def test_lidar_requires_explicit_ros_left_field(self):
        frame = sanitize_lidar_frame(1.0, [
            {"id": "ambiguous-contract", "forward_m": 5.0, "lateral_m": 1.0},
            {"id": "explicit", "forward_m": 5.0, "lateral_left_m": 1.0},
        ])
        self.assertEqual(frame.invalid_count, 1)
        self.assertEqual(len(frame.clusters), 1)
        self.assertEqual(frame.clusters[0].source_id, "explicit")

    def test_readiness_contract_flags(self):
        flags = ReadinessFlags(True, True, False, True)
        self.assertFalse(flags.ready)
        self.assertEqual(flags.as_dict()["lidar_calibrated"], False)
        self.assertEqual(flags.as_dict()["ready"], False)
        with self.assertRaises(ValueError):
            ReadinessFlags("true", True, True, True)

    def test_association_overflow_is_bounded_and_leaves_lidar_unknown(self):
        camera_frame = sanitize_camera_frame(1.0, [
            {"id": f"c{i}", "bearing_deg": float(i), "color": "orange", "confidence": 0.9}
            for i in range(17)
        ])
        lidar_frame = sanitize_lidar_frame(1.0, [
            {"id": f"l{i}", "forward_m": 10.0, "lateral_left_m": -0.1 * i}
            for i in range(17)
        ])
        result = fuse_frames(camera_frame, lidar_frame, READY, max_association_items=16)
        self.assertTrue(result.status.association_overflow)
        self.assertEqual(result.status.matched_count, 0)
        self.assertTrue(all(item.color == "unknown" for item in result.obstacles))
        self.assertEqual(result.status.overflow_component_count, 1)
        self.assertEqual(
            sum(item.reason == "association_overflow" for item in result.camera_diagnostics),
            result.status.overflow_camera_count,
        )

    def test_association_limit_validation(self):
        camera_frame, lidar_frame = self._frames()
        for invalid in (0, -1, True, 1.5):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                fuse_frames(camera_frame, lidar_frame, READY, max_association_items=invalid)

    def test_non_candidate_360_lidar_clusters_do_not_trigger_overflow(self):
        camera_frame = sanitize_camera_frame(1.0, [
            {"id": "front-camera", "bearing_deg": 0.0, "color": "orange", "confidence": 0.9}
        ])
        clusters = [{"id": "front", "forward_m": 5.0, "lateral_left_m": 0.0}]
        for index in range(21):
            angle = math.radians(30.0 + index * 6.0)
            clusters.append({
                "id": f"outside-fov-{index}",
                "forward_m": 8.0 * math.cos(angle),
                "lateral_left_m": -8.0 * math.sin(angle),
            })
        lidar_frame = sanitize_lidar_frame(1.0, clusters)
        result = fuse_frames(camera_frame, lidar_frame, READY, max_association_items=10)
        self.assertFalse(result.status.association_overflow)
        self.assertEqual(result.status.matched_count, 1)
        self.assertEqual(len(result.obstacles), 22)

    def test_oversized_dense_component_does_not_discard_sparse_components(self):
        dense_angles = [index * 0.2 for index in range(11)]
        sparse_angles = [30.0, 60.0]
        camera_payload = [
            {
                "id": f"c-{angle:.1f}",
                "bearing_deg": angle,
                "color": "orange" if angle < 20.0 else "green",
                "confidence": 0.9,
            }
            for angle in dense_angles + sparse_angles
        ]
        lidar_payload = [
            {
                "id": f"l-{angle:.1f}",
                "forward_m": 10.0,
                "lateral_left_m": -10.0 * math.tan(math.radians(angle)),
            }
            for angle in dense_angles + sparse_angles
        ]

        def run(cameras, lidars):
            return fuse_frames(
                sanitize_camera_frame(1.0, cameras),
                sanitize_lidar_frame(1.0, lidars),
                READY,
                max_bearing_error_deg=1.0,
                max_association_items=10,
            )

        result = run(camera_payload, lidar_payload)
        self.assertTrue(result.status.association_overflow)
        self.assertEqual(result.status.overflow_component_count, 1)
        self.assertEqual(result.status.overflow_camera_count, 11)
        self.assertEqual(result.status.overflow_lidar_count, 11)
        self.assertEqual(result.status.matched_count, 2)
        fused = {
            item.lidar_id: item.camera_id
            for item in result.obstacles if item.camera_id is not None
        }
        self.assertEqual(fused, {"l-30.0": "c-30.0", "l-60.0": "c-60.0"})
        dense_obstacles = [item for item in result.obstacles if item.lidar_id not in fused]
        self.assertTrue(all(item.color == "unknown" for item in dense_obstacles))
        overflow_diagnostics = [
            item for item in result.camera_diagnostics
            if item.reason == "association_overflow"
        ]
        self.assertEqual(len(overflow_diagnostics), 11)

        reversed_result = run(list(reversed(camera_payload)), list(reversed(lidar_payload)))
        reversed_fused = {
            item.lidar_id: item.camera_id
            for item in reversed_result.obstacles if item.camera_id is not None
        }
        self.assertEqual(reversed_fused, fused)
        self.assertEqual(reversed_result.status, result.status)

        started = time.perf_counter()
        for _ in range(20):
            run(camera_payload, lidar_payload)
        self.assertLess(time.perf_counter() - started, 2.0)

    def test_duplicate_camera_id_is_quarantined_across_full_frame(self):
        cameras = [
            {"id": "duplicate", "bearing_deg": 0.0, "color": "orange", "confidence": 0.9},
            {"id": "duplicate", "bearing_deg": 90.0, "color": "green", "confidence": 0.9},
        ]
        lidars = [
            {"id": "lidar", "forward_m": 5.0, "lateral_left_m": 0.0},
        ]

        def run(camera_items):
            return fuse_frames(
                sanitize_camera_frame(1.0, camera_items),
                sanitize_lidar_frame(1.0, lidars),
                READY,
            )

        result = run(cameras)
        reversed_result = run(list(reversed(cameras)))
        for current in (result, reversed_result):
            self.assertEqual(current.status.matched_count, 0)
            self.assertEqual(current.status.ambiguous_camera_count, 2)
            self.assertEqual(len(current.obstacles), 1)
            self.assertEqual(current.obstacles[0].color, "unknown")
            self.assertTrue(all(item.reason == "ambiguous" for item in current.camera_diagnostics))
        self.assertEqual(result, reversed_result)

    def test_duplicate_lidar_id_is_quarantined_and_emitted_once(self):
        cameras = [
            {"id": "camera", "bearing_deg": 0.0, "color": "orange", "confidence": 0.9},
        ]
        lidars = [
            {"id": "duplicate", "forward_m": 5.0, "lateral_left_m": 0.0},
            {"id": "duplicate", "forward_m": 0.0, "lateral_left_m": -5.0},
        ]

        def run(lidar_items):
            return fuse_frames(
                sanitize_camera_frame(1.0, cameras),
                sanitize_lidar_frame(1.0, lidar_items),
                READY,
            )

        result = run(lidars)
        reversed_result = run(list(reversed(lidars)))
        for current in (result, reversed_result):
            self.assertEqual(current.status.matched_count, 0)
            self.assertEqual(current.status.ambiguous_lidar_count, 2)
            self.assertEqual(len(current.obstacles), 1)
            self.assertEqual(current.obstacles[0].lidar_id, "duplicate")
            self.assertEqual(current.obstacles[0].color, "unknown")
            self.assertEqual(len({item.lidar_id for item in current.obstacles}), 1)
        self.assertEqual(result, reversed_result)


class BufferAndClockTest(unittest.TestCase):
    @staticmethod
    def camera_payload(identifier="c"):
        return [{"id": identifier, "bearing_deg": 0.0, "color": "orange", "confidence": 0.9}]

    @staticmethod
    def lidar_payload(identifier="l"):
        return [{"id": identifier, "forward_m": 5.0, "lateral_left_m": 0.0}]

    def test_bounded_ordered_and_duplicate_stamp_replace(self):
        core = FusionCore(buffer_size=2)
        self.assertTrue(core.ingest_camera(1.0, self.camera_payload("c1")))
        self.assertTrue(core.ingest_camera(2.0, self.camera_payload("c2")))
        self.assertTrue(core.ingest_camera(2.0, self.camera_payload("c2-new")))
        self.assertTrue(core.ingest_camera(3.0, self.camera_payload("c3")))
        frames = core.camera_buffer.snapshot()
        self.assertEqual([frame.stamp for frame in frames], [2.0, 3.0])
        self.assertEqual(frames[0].detections[0].source_id, "c2-new")

    def test_buffer_size_requires_exact_positive_integer(self):
        for invalid in (True, 0, -1, 1.5):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                FusionCore(buffer_size=invalid)

    def test_out_of_order_source_frames_insert_without_global_reset(self):
        core = FusionCore(buffer_size=4)
        core.ingest_camera(10.0, self.camera_payload())
        core.ingest_lidar(10.0, self.lidar_payload())
        core.ingest_camera(1.0, self.camera_payload("after-reset"))
        self.assertEqual(core.clock_rollback_resets, 0)
        self.assertEqual([f.stamp for f in core.camera_buffer.snapshot()], [1.0, 10.0])
        self.assertEqual(len(core.lidar_buffer.snapshot()), 1)

    def test_observe_clock_threshold_and_explicit_epoch_reset(self):
        core = FusionCore(buffer_size=4, clock_rollback_threshold_s=0.5)
        core.ingest_camera(10.0, self.camera_payload())
        core.ingest_lidar(10.0, self.lidar_payload())
        self.assertFalse(core.observe_clock(10.0))
        self.assertFalse(core.observe_clock(9.6))  # 400ms jitter, epoch değil
        self.assertEqual(len(core.camera_buffer.snapshot()), 1)
        self.assertTrue(core.observe_clock(9.4))   # >500ms gerçek rollback
        self.assertEqual(core.clock_rollback_resets, 1)
        self.assertEqual(core.camera_buffer.snapshot(), ())
        self.assertEqual(core.lidar_buffer.snapshot(), ())
        core.ingest_camera(9.5, self.camera_payload())
        self.assertTrue(core.observe_clock(9.6, force_epoch_reset=True))
        self.assertEqual(core.clock_rollback_resets, 2)
        self.assertEqual(core.camera_buffer.snapshot(), ())

    def test_best_pair_and_status_carries_rollback_count(self):
        core = FusionCore(buffer_size=4)
        core.ingest_camera(10.0, self.camera_payload())
        core.ingest_lidar(10.049, self.lidar_payload())
        result = core.fuse_best(READY)
        self.assertEqual(result.status.temporal_health, "healthy")
        self.assertEqual(result.status.matched_count, 1)

    def test_newest_accepted_pair_wins_and_consume_is_explicit(self):
        core = FusionCore(buffer_size=6)
        core.ingest_camera(1.000, self.camera_payload("old-camera"))
        core.ingest_lidar(1.000, self.lidar_payload("old-lidar"))
        core.ingest_camera(2.000, self.camera_payload("new-camera"))
        core.ingest_lidar(2.049, self.lidar_payload("new-lidar"))
        result = core.fuse_best(READY)
        self.assertEqual(result.status.camera_stamp, 2.000)
        self.assertEqual(result.status.lidar_stamp, 2.049)
        # Default non-consuming: aynı en yeni pair tekrar okunabilir.
        again = core.fuse_best(READY)
        self.assertEqual(again.status.camera_stamp, 2.000)
        consumed = core.fuse_best(READY, consume=True)
        self.assertEqual(consumed.status.camera_stamp, 2.000)
        fallback = core.fuse_best(READY)
        self.assertEqual(fallback.status.camera_stamp, 1.000)

    def test_old_accepted_pair_cannot_override_new_epoch_data(self):
        core = FusionCore(buffer_size=6)
        core.ingest_camera(1.0, self.camera_payload("old-camera"))
        core.ingest_lidar(1.0, self.lidar_payload("old-lidar"))
        core.ingest_lidar(100.0, self.lidar_payload("new-lidar"))
        self.assertIsNone(core.fuse_best(READY, now=100.0, max_frame_age_s=0.5))
        self.assertEqual(core.camera_buffer.snapshot(), ())

    def test_age_filter_rejects_future_and_invalid_age_configuration(self):
        core = FusionCore()
        core.ingest_camera(11.0, self.camera_payload())
        core.ingest_lidar(11.0, self.lidar_payload())
        self.assertIsNone(core.fuse_best(READY, now=10.0, max_frame_age_s=1.0))
        with self.assertRaises(ValueError):
            core.fuse_best(READY, now=10.0, max_frame_age_s=-1.0)


class RosContractTest(unittest.TestCase):
    def test_raw_payloads_require_acquisition_stamp_and_reject_stale(self):
        self.assertEqual(camera_payload({"acquisition_stamp": 1.0, "detections": []}), (1.0, []))
        self.assertEqual(lidar_payload({"acquisition_stamp": 2.0, "clusters": []}), (2.0, []))
        self.assertIsNone(lidar_payload({"stamp": 2.0, "clusters": []}))
        self.assertIsNone(camera_payload({"stamp": 1.0, "detections": [], "stale": True}))
        self.assertIsNone(lidar_payload({"stamp": float("nan"), "clusters": []}))

    def test_camera_roles_merge_only_same_time_window_and_deduplicate(self):
        merged = merge_camera_roles([
            ("p1p2", 1.020, [{"id": "same", "color": "orange"}, {"id": "p1", "color": "yellow"}]),
            ("p3", 1.020, [{"id": "same", "color": "orange"}, {"id": "p3", "color": "green"}]),
            ("old", 0.800, [{"id": "old", "color": "red"}]),
        ], max_role_delta_s=0.0)
        self.assertEqual(merged[0], 1.020)
        self.assertEqual(
            {item["id"] for item in merged[1]},
            {"p1p2:same", "p1p2:p1", "p3:same", "p3:p3"},
        )

    def test_different_camera_acquisition_times_are_not_relabelled_as_newest(self):
        merged = merge_camera_roles([
            ("old-role", 1.000, [{"id": "old", "color": "orange"}]),
            ("new-role", 1.050, [{"id": "new", "color": "green"}]),
        ], max_role_delta_s=0.0)
        self.assertEqual(merged[0], 1.050)
        self.assertEqual([item["id"] for item in merged[1]], ["new-role:new"])

    def test_result_payloads_keep_lidar_metric_and_camera_color(self):
        camera_frame = sanitize_camera_frame(1.0, [
            {
                "id": "camera", "bearing_deg": 0.0, "color": "orange",
                "confidence": 0.9, "bbox_norm_x": 0.35, "bbox_size": 0.2,
            }
        ])
        lidar_frame = sanitize_lidar_frame(1.0, [
            {"id": "lidar", "forward_m": 5.0, "lateral_left_m": 0.0}
        ])
        buoys, obstacles, status = result_payloads(fuse_frames(camera_frame, lidar_frame, READY))
        self.assertEqual(buoys["detections"][0]["color"], "orange")
        self.assertEqual(buoys["detections"][0]["distance"], 5.0)
        self.assertEqual(obstacles["obstacles"][0]["id"], "lidar")
        self.assertEqual(buoys["detections"][0]["bbox_norm_x"], 0.35)
        self.assertEqual(buoys["detections"][0]["bbox_size"], 0.2)
        self.assertTrue(status["ready"])

    def test_result_bearing_uses_all_four_quadrants(self):
        camera_frame = sanitize_camera_frame(1.0, [])
        lidar_frame = sanitize_lidar_frame(1.0, [
            {"id": "behind-left", "forward_m": -1.0, "lateral_left_m": 1.0}
        ])
        _buoys, obstacles, _status = result_payloads(
            fuse_frames(camera_frame, lidar_frame, READY)
        )
        self.assertAlmostEqual(obstacles["obstacles"][0]["bearing_deg"], -135.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
