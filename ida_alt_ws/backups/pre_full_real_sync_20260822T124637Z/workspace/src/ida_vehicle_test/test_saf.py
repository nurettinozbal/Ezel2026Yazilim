"""Safety and lifecycle tests for the passive vehicle-test monitor core."""

import math
import os
import sys
import tempfile
import unittest
import hashlib
from pathlib import Path

PACKAGE_ROOT = os.path.abspath(os.path.dirname(__file__))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from ida_vehicle_test.contracts import (
    ALLOWED_TESTS,
    RequestError,
    bounded_json_loads,
    parse_request,
)
from ida_vehicle_test.core import (
    CHANNEL_RULES,
    DEFAULT_PUBLISHER_NODE_ALLOWLIST,
    TEST_RULES,
    PassiveTestCore,
    validate_payload,
)
from ida_vehicle_test.storage import atomic_persist, probe_log_files


def request(test="telemetry", run_id="run-1", seq=1, timeout=10.0):
    value = {"action": "start", "run_id": run_id, "seq": seq, "test": test, "timeout_s": timeout}
    fixtures = {
        "camera_p1p2": {
            "profile": "camera_p1p2_fixture", "case": "positive",
            "expected_color": "orange", "expected_range_m": math.sqrt(10.0),
            "expected_bearing_deg": 1.0,
        },
        "camera_p3": {
            "profile": "camera_p3_fixture", "case": "positive",
            "expected_color": "green", "expected_range_m": math.sqrt(10.0),
            "expected_bearing_deg": 1.0,
        },
        "lidar": {
            "profile": "lidar_bottle", "case": "positive",
            "expected_range_m": math.sqrt(10.0), "expected_bearing_deg": math.degrees(math.atan2(1.0, 3.0)),
        },
        "fusion_shadow": {
            "profile": "fusion_buoy", "case": "positive", "expected_color": "yellow",
            "expected_range_m": math.sqrt(10.0), "expected_bearing_deg": math.degrees(math.atan2(1.0, 3.0)),
        },
    }
    if test in fixtures:
        value["expectations"] = fixtures[test]
    return parse_request(value)


def graph_for(channels, count=1, node_override=None, type_override=None):
    return {
        channel: {
            "count": count,
            "types": [type_override or "std_msgs/msg/String"],
            "nodes": [node_override or sorted(DEFAULT_PUBLISHER_NODE_ALLOWLIST[channel])[0]],
        }
        for channel in channels
    }


def commit_if_ready(core):
    if core.pass_ready:
        core.commit_pass({"path": "/evidence/run.json", "sha256": "a" * 64})


def telemetry_channels():
    return TEST_RULES["telemetry"].channels


def observe_telemetry_bundle(core, when, ros_stamp):
    core.observe("bridge_status", payload("bridge_status", ros_stamp), when, ros_stamp)
    core.observe("telemetry", payload("telemetry", ros_stamp), when, ros_stamp)


def detection(color="orange"):
    return {
        "id": "camera-1", "color": color, "bearing_deg": 1.0,
        "confidence": 0.9, "distance": math.sqrt(10.0),
    }


def payload(channel, stamp):
    common = {"stamp": stamp}
    if channel == "bridge_status":
        return {
            **common,
            "acquisition_stamp": stamp,
            "dry_run": False,
            "connected": True,
            "heartbeat_fresh": True,
            "telemetry_ready": True,
            "last_receive_age_s": 0.01,
            "source_age_s": 0.01,
        }
    if channel in {"comms_telemetry", "telemetry"}:
        return {
            **common,
            "lat": 41.0,
            "lon": 29.0,
            "heading_deg": 10.0,
            "ground_speed": 0.0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "mode": "DISARMED",
        }
    if channel == "camera_p1p2":
        return {"acquisition_stamp": stamp, "detections": [detection("orange")]}
    if channel == "camera_p3":
        return {"acquisition_stamp": stamp, "detections": [detection("green")]}
    if channel == "lidar":
        return {
            "acquisition_stamp": stamp,
            "clusters": [{"id": "lidar-1", "forward_m": 3.0, "lateral_left_m": -1.0}],
        }
    if channel == "fusion_status":
        return {
            **common,
            "shadow_mode": True,
            "camera_fresh": True,
            "lidar_fresh": True,
            "accepted": True,
            "source_health": "ok",
            "matched_count": 1,
            "dt_ms": 20.0,
        }
    if channel == "fusion_buoys":
        item = detection("yellow")
        item.update({"bearing_deg": math.degrees(math.atan2(1.0, 3.0)), "lidar_obstacle_id": "lidar-1"})
        return {**common, "detections": [item]}
    if channel == "fusion_obstacles":
        return {
            **common,
            "obstacles": [
                {
                    "id": "lidar-1", "forward_m": 3.0, "lateral_m": 1.0,
                    "hard_obstacle": True, "color": "yellow",
                }
            ],
        }
    if channel == "autonomy_state":
        return {**common, "state": "PARKUR_1_NAV", "failsafe_reason": ""}
    if channel == "autonomy_debug":
        return {
            **common,
            "costmap": True,
            "command": {"vx": 0.0, "vy": 0.0, "yaw_rate": 0.0, "action": "shadow"},
        }
    if channel == "costmap":
        return {"cell_m": 0.25, "cells": [[stamp, 1.0, 8, "orange"]]}
    if channel == "logging":
        return {
            **common,
            "active": True,
            "logger_count": 3,
            "growing_file_count": 3,
            "path_policy_valid": True,
            "files": ["telemetry.csv", "processed_video.mp4", "map.mp4"],
        }
    raise AssertionError(channel)


class RequestContractTests(unittest.TestCase):
    def test_allowlist_is_passive_and_exact(self):
        self.assertEqual(
            ALLOWED_TESTS,
            {
                "comms",
                "telemetry",
                "camera_p1p2",
                "camera_p3",
                "lidar",
                "fusion_shadow",
                "autonomy_shadow",
                "logging",
            },
        )
        for forbidden in ("arm", "disarm", "motor", "mode", "cmd_vel"):
            with self.assertRaises(RequestError):
                request(forbidden)

    def test_request_shape_and_nonfinite_are_rejected(self):
        bad = [
            {},
            {"action": "start", "run_id": "x", "seq": True, "test": "telemetry", "timeout_s": 2},
            {"action": "start", "run_id": "bad id", "seq": 1, "test": "telemetry", "timeout_s": 2},
            {"action": "start", "run_id": "x", "seq": 1, "test": "telemetry", "timeout_s": math.nan},
            {"action": "start", "run_id": "x", "seq": 1, "test": "telemetry", "timeout_s": 301},
            {"action": "cancel", "run_id": "x", "seq": 2, "test": "telemetry"},
        ]
        for value in bad:
            with self.subTest(value=value), self.assertRaises(RequestError):
                parse_request(value)

    def test_fixture_profiles_cases_and_tolerances_are_strict(self):
        self.assertEqual(request("camera_p1p2").expectations.expected_color, "orange")
        invalid = [
            {"profile": "camera_p3_fixture", "case": "positive", "expected_color": "orange", "expected_bearing_deg": 0},
            {"profile": "camera_p1p2_fixture", "case": "ambiguity", "expected_color": "orange", "expected_bearing_deg": 0},
            {"profile": "camera_p1p2_fixture", "case": "positive", "expected_color": "red", "expected_bearing_deg": 0},
            {"profile": "camera_p1p2_fixture", "case": "positive", "expected_color": "orange", "expected_bearing_deg": math.inf},
        ]
        for expectations in invalid:
            value = {
                "action": "start", "run_id": "fixture", "seq": 1,
                "test": "camera_p1p2", "timeout_s": 5, "expectations": expectations,
            }
            with self.subTest(expectations=expectations), self.assertRaises(RequestError):
                parse_request(value)
        for case in ("negative", "ambiguity", "lidar_only", "camera_only"):
            value = {
                "action": "start", "run_id": f"fusion-{case}", "seq": 1,
                "test": "fusion_shadow", "timeout_s": 5,
                "expectations": {"profile": "fusion_buoy", "case": case},
            }
            self.assertEqual(parse_request(value).expectations.case, case)

    def test_bounded_json_rejects_bytes_items_and_depth(self):
        with self.assertRaises(ValueError):
            bounded_json_loads('"' + ("x" * 20) + '"', 8, 100)
        with self.assertRaises(ValueError):
            bounded_json_loads("[1,2,3,4]", 100, 3)
        with self.assertRaises(ValueError):
            bounded_json_loads("[[[[1]]]]", 100, 100, 2)


class StateMachineTests(unittest.TestCase):
    def test_single_active_cancel_and_monotonic_sequence(self):
        core = PassiveTestCore()
        core.request(request(seq=4), 1.0, 100.0)
        with self.assertRaises(ValueError):
            core.request(request(run_id="other", seq=5), 1.1, 100.1)
        with self.assertRaises(ValueError):
            core.request(parse_request({"action": "cancel", "run_id": "other", "seq": 5}), 1.2, 100.2)
        core.request(parse_request({"action": "cancel", "run_id": "run-1", "seq": 5}), 1.3, 100.3)
        self.assertEqual(core.state, "CANCELLED")
        self.assertIsNone(core.pop_result(1.3))
        core.attach_terminal_artifact({"path": "/evidence/cancel.json", "sha256": "c" * 64})
        result = core.pop_result(1.3)
        self.assertFalse(result["pass"])
        with self.assertRaises(ValueError):
            core.request(request(run_id="next", seq=5), 1.4, 100.4)

    def test_timeout_and_clock_rollback_fail_closed(self):
        core = PassiveTestCore()
        core.request(request(timeout=0.5), 10.0, 100.0)
        core.tick(10.499999)
        self.assertEqual(core.state, "RUNNING")
        core.tick(10.5)
        self.assertEqual(core.state, "FAIL")
        self.assertIn("test_timeout", core.reasons)

        other = PassiveTestCore()
        other.request(request(), 10.0, 100.0)
        other.tick(9.9)
        self.assertEqual(other.state, "FAIL")
        self.assertIn("monotonic_clock_rollback", other.reasons)

    def test_graph_conflict_and_malformed_evidence_fail_closed(self):
        core = PassiveTestCore()
        core.request(request(), 0.0, 100.0)
        core.observe_graph(graph_for(telemetry_channels(), count=2), 0.01)
        self.assertEqual(core.state, "FAIL")
        self.assertIn("publisher_count", core.reasons[0])

        other = PassiveTestCore()
        other.request(request(seq=2), 0.0, 100.0)
        other.observe("telemetry", {"stamp": 100.0, "lat": math.nan}, 0.1, 100.0)
        self.assertEqual(other.state, "FAIL")

    def test_old_source_is_ignored_but_future_clock_anomaly_fails(self):
        old = PassiveTestCore()
        old.request(request(), 0.0, 100.0)
        old.observe("telemetry", payload("telemetry", 99.0), 0.1, 100.0)
        self.assertEqual(old.state, "RUNNING")
        self.assertEqual(old.evidence["telemetry"].receives, [])

        future = PassiveTestCore()
        future.request(request(), 0.0, 100.0)
        future.observe("telemetry", payload("telemetry", 100.3), 0.1, 100.0)
        self.assertEqual(future.state, "FAIL")

    def test_valid_evidence_and_rate_are_both_required(self):
        core = PassiveTestCore()
        core.request(request(), 0.0, 100.0)
        core.observe_graph(graph_for(telemetry_channels()), 0.01)
        for index in range(5):
            when = 0.1 + index * 0.1
            observe_telemetry_bundle(core, when, 100.0 + when)
        core.observe_graph(graph_for(telemetry_channels()), 0.5)
        core.tick(0.5)
        commit_if_ready(core)
        self.assertEqual(core.state, "PASS")
        result = core.pop_result(0.5)
        self.assertTrue(result["pass"])
        self.assertGreaterEqual(result["channels"]["telemetry"]["rate_hz"], 5.0)
        self.assertFalse(result["actuation_enabled"])

    def test_slow_samples_fail_instead_of_false_pass(self):
        core = PassiveTestCore()
        core.request(request(), 0.0, 100.0)
        core.observe_graph(graph_for(telemetry_channels()), 0.01)
        for index in range(5):
            when = 0.1 + index * 1.2
            observe_telemetry_bundle(core, when, 100.0 + when)
        core.observe_graph(graph_for(telemetry_channels()), 4.91)
        core.tick(4.91)
        self.assertEqual(core.state, "FAIL")
        self.assertIn("rate_below", core.reasons[0])

    def test_duplicate_acquisition_stamp_is_not_a_new_sample(self):
        core = PassiveTestCore()
        core.request(request(), 0.0, 100.0)
        for when in (0.1, 0.2, 0.3, 0.4, 0.5):
            core.observe("telemetry", payload("telemetry", 100.1), when, 100.0 + when)
        core.observe_graph(graph_for(telemetry_channels()), 0.5)
        core.tick(0.5)
        self.assertEqual(core.state, "RUNNING")
        self.assertEqual(len(core.evidence["telemetry"].receives), 1)

    def test_comms_dry_run_fails_and_status_replay_is_not_evidence(self):
        core = PassiveTestCore()
        core.request(request("comms"), 0.0, 100.0)
        dry = {**payload("bridge_status", 100.1), "dry_run": True, "connected": False}
        core.observe("bridge_status", dry, 0.1, 100.1)
        self.assertEqual(core.state, "FAIL")

        replay = PassiveTestCore()
        replay.request(request("comms", run_id="replay", seq=2), 0.0, 100.0)
        for when in (0.1, 0.2, 0.3):
            replay.observe("bridge_status", payload("bridge_status", 100.1), when, 100 + when)
            replay.observe("comms_telemetry", payload("comms_telemetry", 100.1), when, 100 + when)
        self.assertEqual(len(replay.evidence["bridge_status"].receives), 1)
        self.assertEqual(len(replay.evidence["comms_telemetry"].receives), 1)

    def test_pre_start_source_stamp_is_ignored_and_cannot_satisfy_pass(self):
        core = PassiveTestCore()
        core.request(request(), 0.0, 100.0)
        core.observe("telemetry", payload("telemetry", 99.999), 0.1, 100.1)
        self.assertEqual(core.state, "RUNNING")
        self.assertEqual(core.evidence["telemetry"].receives, [])
        core.observe("telemetry", payload("telemetry", 100.2), 0.2, 100.2)
        self.assertEqual(len(core.evidence["telemetry"].receives), 1)

    def test_exact_request_replay_is_idempotent_and_terminal_is_immutable(self):
        core = PassiveTestCore()
        start = request()
        core.request(start, 0.0, 100.0)
        core.request(start, 0.1, 100.1)
        self.assertEqual(len(core.pop_events()), 1)
        core.observe_graph(graph_for(telemetry_channels(), count=2), 0.2)
        self.assertEqual(core.state, "FAIL")
        reasons = list(core.reasons)
        core.observe("telemetry", payload("telemetry", 100.3), 0.3, 100.3)
        core.tick(0.4)
        core.request(start, 0.5, 100.5)
        self.assertEqual(core.state, "FAIL")
        self.assertEqual(core.reasons, reasons)
        with self.assertRaises(ValueError):
            core.request(request(run_id="changed", seq=1), 0.6, 100.6)

    def test_pass_requires_same_cycle_graph_recheck_and_emits_manifest(self):
        core = PassiveTestCore()
        core.request(request(), 0.0, 100.0)
        for index in range(5):
            when = 0.1 + index * 0.1
            observe_telemetry_bundle(core, when, 100.0 + when)
        core.observe_graph(graph_for(telemetry_channels()), 0.5)
        core.tick(0.51)
        self.assertEqual(core.state, "RUNNING")
        core.observe_graph(graph_for(telemetry_channels()), 0.6)
        core.tick(0.6)
        commit_if_ready(core)
        self.assertEqual(core.state, "PASS")
        result = core.pop_result(0.6)
        manifest = result["evidence_manifest"]["telemetry"]
        self.assertEqual(manifest["unique_sample_count"], 5)
        self.assertEqual(manifest["final_publisher_count"], 1)
        self.assertGreater(manifest["observation_span_s"], 0.0)
        self.assertEqual(result["artifact"]["sha256"], "a" * 64)

    def test_wrong_graph_type_or_node_fails(self):
        for graph in (
            graph_for(telemetry_channels(), type_override="geometry_msgs/msg/Twist"),
            graph_for(telemetry_channels(), node_override="spoofed_bridge"),
            {name: {"error": "boom"} for name in telemetry_channels()},
        ):
            core = PassiveTestCore()
            core.request(request(), 0.0, 100.0)
            core.observe_graph(graph, 0.1)
            self.assertEqual(core.state, "FAIL")

    def test_persistence_failure_cannot_publish_pass(self):
        core = PassiveTestCore()
        core.request(request(), 0.0, 100.0)
        for index in range(5):
            when = 0.1 + index * 0.1
            observe_telemetry_bundle(core, when, 100 + when)
        core.observe_graph(graph_for(telemetry_channels()), 0.5)
        core.tick(0.5)
        self.assertTrue(core.pass_ready)
        self.assertIsNone(core.pop_result(0.5))
        core.fail_persistence()
        self.assertEqual(core.state, "FAIL")
        self.assertFalse(core.pop_result(0.5)["pass"])

    def test_fail_terminal_requires_artifact_or_records_persist_failure(self):
        core = PassiveTestCore()
        core.request(request(), 0.0, 100.0)
        core.observe_graph(graph_for(telemetry_channels(), count=2), 0.1)
        self.assertTrue(core.terminal_needs_persistence)
        self.assertIsNone(core.pop_result(0.1))
        candidate = core.terminal_candidate(0.1)
        self.assertEqual(candidate["state"], "FAIL")
        core.attach_terminal_artifact(
            {"path": "/evidence/fail.json", "sha256": "f" * 64}
        )
        self.assertEqual(core.pop_result(0.1)["artifact"]["sha256"], "f" * 64)

        failed_disk = PassiveTestCore()
        failed_disk.request(request(run_id="disk-fail", seq=2), 0.0, 100.0)
        failed_disk.observe_graph(graph_for(telemetry_channels(), count=2), 0.1)
        failed_disk.mark_terminal_persistence_failed()
        result = failed_disk.pop_result(0.1)
        self.assertIn("evidence_persist_failed", result["reasons"])
        self.assertIsNone(result["artifact"])

    def test_identical_costmap_content_is_not_independent_evidence(self):
        core = PassiveTestCore()
        core.request(request("autonomy_shadow"), 0.0, 100.0)
        same = {"cell_m": 0.25, "cells": [[1.0, 0.0, 8, "orange"]]}
        core.observe("costmap", same, 0.1, 100.1)
        core.observe("costmap", same, 0.2, 100.2)
        self.assertEqual(len(core.evidence["costmap"].receives), 1)

    def test_request_history_is_bounded(self):
        core = PassiveTestCore()
        now = 0.0
        seq = 1
        for index in range(140):
            run_id = f"bounded-{index}"
            core.request(request(run_id=run_id, seq=seq), now, 1000.0 + now)
            seq += 1
            now += 0.001
            cancel = parse_request({"action": "cancel", "run_id": run_id, "seq": seq})
            core.request(cancel, now, 1000.0 + now)
            seq += 1
            core.attach_terminal_artifact(
                {"path": f"/evidence/{run_id}.json", "sha256": "d" * 64}
            )
            core.pop_result(now)
            now += 0.001
        self.assertLessEqual(len(core._request_history), 256)
        self.assertLessEqual(len(core._seen_run_ids), 256)


class AcceptanceCatalogTests(unittest.TestCase):
    def test_bridge_status_accepts_one_hz_composite_age_but_not_stale(self):
        healthy = payload("bridge_status", 100.0)
        healthy["last_receive_age_s"] = 1.1
        healthy["source_age_s"] = 1.1
        validate_payload("bridge_status", healthy)
        stale = dict(healthy)
        stale["last_receive_age_s"] = 1.501
        with self.assertRaisesRegex(ValueError, "bridge_health_stale"):
            validate_payload("bridge_status", stale)

    def test_every_catalog_can_pass_only_after_all_required_observations(self):
        base_ros = 1000.0
        seq = 10
        for test_name, test_rule in TEST_RULES.items():
            with self.subTest(test=test_name):
                core = PassiveTestCore()
                core.request(
                    request(test_name, run_id=f"run-{test_name}", seq=seq, timeout=30),
                    0.0,
                    base_ros,
                )
                seq += 1
                core.observe_graph(graph_for(test_rule.channels), 0.001)
                max_end = 0.0
                observations = []
                duration = max(
                    (CHANNEL_RULES[name].min_samples - 1) / CHANNEL_RULES[name].min_hz
                    for name in test_rule.channels
                )
                for channel in test_rule.channels:
                    rule = CHANNEL_RULES[channel]
                    interval = 1.0 / rule.min_hz
                    sample_count = max(rule.min_samples, int(math.floor(duration * rule.min_hz)) + 1)
                    for index in range(sample_count):
                        when = 0.1 + index * interval
                        observations.append((when, channel))
                        max_end = max(max_end, when)
                for when, channel in sorted(observations):
                    core.observe(channel, payload(channel, base_ros + when), when, base_ros + when)
                core.observe_graph(graph_for(test_rule.channels), max_end)
                core.tick(max_end)
                commit_if_ready(core)
                self.assertEqual(core.state, "PASS", core.reasons)
                result = core.pop_result(max_end)
                if test_name in {"camera_p1p2", "camera_p3", "lidar"}:
                    measured = [
                        item
                        for channel in test_rule.channels
                        for item in result["evidence_manifest"][channel]["measured_samples"]
                    ]
                    self.assertGreaterEqual(len(measured), 3)
                    self.assertIn("bearing_deg", measured[-1])
                    self.assertIn("range_m", measured[-1])
                if test_name == "fusion_shadow":
                    self.assertEqual(len(result["bundle_correlations"]), 3)
                    bundle = result["bundle_correlations"][-1]
                    for key in ("id", "color", "bearing_deg", "range_m", "lidar_id", "dt_ms"):
                        self.assertIn(key, bundle)

    def test_stale_empty_and_wrong_mode_payloads_fail(self):
        cases = [
            ("camera_p1p2", {"acquisition_stamp": 1.0, "detections": [], "stale": False}),
            ("lidar", {"acquisition_stamp": 1.0, "clusters": [], "stale": False}),
            ("fusion_status", {**payload("fusion_status", 1.0), "shadow_mode": False}),
            ("autonomy_state", {**payload("autonomy_state", 1.0), "failsafe_reason": "telemetry_timeout"}),
            ("logging", {**payload("logging", 1.0), "logger_count": 2}),
            ("comms_telemetry", {"stamp": 1.0, "lat": 1.0}),
        ]
        for channel, value in cases:
            with self.subTest(channel=channel), self.assertRaises(ValueError):
                validate_payload(channel, value)

    def test_fixture_wrong_color_identity_and_geometry_fail(self):
        expected = request("camera_p1p2").expectations
        wrong_color = {"acquisition_stamp": 1.0, "detections": [detection("yellow")]}
        with self.assertRaises(ValueError):
            validate_payload("camera_p1p2", wrong_color, expected)
        expected_id = parse_request(
            {
                "action": "start", "run_id": "identity", "seq": 1,
                "test": "camera_p1p2", "timeout_s": 5,
                "expectations": {
                    "profile": "camera_p1p2_fixture", "case": "positive",
                    "expected_color": "orange", "expected_range_m": math.sqrt(10.0),
                    "expected_bearing_deg": 1.0,
                    "expected_id": "wanted",
                },
            }
        ).expectations
        with self.assertRaises(ValueError):
            validate_payload(
                "camera_p1p2",
                {"acquisition_stamp": 1.0, "detections": [detection("orange")]},
                expected_id,
            )
        lidar_expected = request("lidar").expectations
        with self.assertRaises(ValueError):
            validate_payload(
                "lidar",
                {"acquisition_stamp": 1.0, "clusters": [{"id": "x", "forward_m": 8.0, "lateral_left_m": 0.0}]},
                lidar_expected,
            )
        negative = parse_request(
            {
                "action": "start", "run_id": "negative", "seq": 1,
                "test": "camera_p1p2", "timeout_s": 5,
                "expectations": {"profile": "camera_p1p2_fixture", "case": "negative"},
            }
        ).expectations
        validate_payload(
            "camera_p1p2", {"acquisition_stamp": 1.0, "detections": []}, negative
        )

    def test_fusion_identity_correlation_mismatch_fails(self):
        core = PassiveTestCore()
        core.request(request("fusion_shadow"), 0.0, 100.0)
        channels = TEST_RULES["fusion_shadow"].channels
        for index in range(3):
            when = 0.1 + index * 0.5
            for channel in channels:
                value = payload(channel, 100 + when)
                if channel == "fusion_buoys":
                    value["detections"][0]["lidar_obstacle_id"] = "wrong-lidar"
                core.observe(channel, value, when, 100 + when)
        core.observe_graph(graph_for(channels), 1.1)
        core.tick(1.1)
        self.assertEqual(core.state, "FAIL")
        self.assertIn("cross_correlation", core.reasons[0])

    def test_fusion_output_stamp_mismatch_fails(self):
        core = PassiveTestCore()
        core.request(request("fusion_shadow"), 0.0, 100.0)
        channels = TEST_RULES["fusion_shadow"].channels
        for index in range(3):
            when = 0.1 + index * 0.5
            for channel in channels:
                stamp = 100 + when + (0.2 if channel == "fusion_obstacles" else 0.0)
                core.observe(channel, payload(channel, stamp), when, stamp)
        core.observe_graph(graph_for(channels), 1.1)
        core.tick(1.1)
        self.assertEqual(core.state, "FAIL")
        self.assertIn("cross_correlation", core.reasons[0])

    def test_fusion_fail_safe_fixture_cases_are_explicit(self):
        for case in ("negative", "ambiguity", "lidar_only", "camera_only"):
            req = parse_request(
                {
                    "action": "start", "run_id": f"case-{case}", "seq": 1,
                    "test": "fusion_shadow", "timeout_s": 5,
                    "expectations": {"profile": "fusion_buoy", "case": case},
                }
            )
            expected = req.expectations
            status = {
                "stamp": 1.0, "shadow_mode": True, "matched_count": 0,
                "camera_fresh": case != "lidar_only", "lidar_fresh": case != "camera_only",
                "accepted": case in {"negative", "ambiguity"},
                "source_health": {
                    "negative": "ok", "ambiguity": "ok",
                    "lidar_only": "camera_stale", "camera_only": "lidar_stale",
                }[case],
                "dt_ms": None if case == "camera_only" else 20.0,
                "ambiguous_camera_count": 1 if case == "ambiguity" else 0,
            }
            validate_payload("fusion_status", status, expected)
            validate_payload(
                "fusion_buoys",
                {
                    "stamp": 1.0, "detections": [],
                    "stale": case in {"lidar_only", "camera_only"},
                },
                expected,
            )
            obstacles = [] if case in {"camera_only", "negative"} else [
                {
                    "id": "lidar-1", "forward_m": 3.0, "lateral_m": 0.0,
                    "hard_obstacle": True, "color": "unknown",
                }
            ]
            validate_payload(
                "fusion_obstacles",
                {"stamp": 1.0, "obstacles": obstacles, "stale": case == "camera_only"},
                expected,
            )

        negative = parse_request(
            {
                "action": "start", "run_id": "negative-overflow", "seq": 1,
                "test": "fusion_shadow", "timeout_s": 5,
                "expectations": {"profile": "fusion_buoy", "case": "negative"},
            }
        )
        with self.assertRaisesRegex(ValueError, "fusion_negative_not_observed"):
            validate_payload(
                "fusion_status",
                {
                    "stamp": 1.0, "shadow_mode": True, "matched_count": 0,
                    "camera_fresh": True, "lidar_fresh": True, "accepted": True,
                    "source_health": "association_overflow", "dt_ms": 20.0,
                },
                negative.expectations,
            )

    def test_costmap_receive_only_contract_and_nonfinite_cells(self):
        stamp, metrics = validate_payload("costmap", payload("costmap", 1.0))
        self.assertIsNone(stamp)
        self.assertEqual(metrics["cell_count"], 1)
        with self.assertRaises(ValueError):
            validate_payload("costmap", {"cell_m": 0.25, "cells": [[math.inf, 0, 8, "x"]]})

    def test_monitor_source_has_no_actuation_endpoint(self):
        source_path = os.path.join(PACKAGE_ROOT, "ida_vehicle_test", "monitor_node.py")
        with open(source_path, "r", encoding="utf-8") as handle:
            source = handle.read()
        self.assertNotIn("geometry_msgs", source)
        self.assertNotIn("/control/cmd_vel_body", source)
        self.assertNotIn("/autonomy/cmd_vel_body", source)
        self.assertNotIn("create_service", source)
        self.assertNotIn("create_client", source)
        self.assertEqual(source.count("self.create_publisher("), 4)
        for output in ("/vehicle_test/status", "/vehicle_test/metrics", "/vehicle_test/events", "/vehicle_test/result"):
            self.assertIn(output, source)


class StorageSafetyTests(unittest.TestCase):
    def test_log_probe_requires_contained_fixed_files_and_growth(self):
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root).resolve()
            run = root / "run"
            run.mkdir()
            files = [run / "telemetry.csv", run / "processed_video.mp4", run / "map.mp4"]
            for path in files:
                path.write_bytes(b"a")
            current, grown, valid = probe_log_files(root, [str(path) for path in files], {}, set())
            self.assertTrue(valid)
            self.assertEqual(grown, set())
            for path in files:
                with path.open("ab") as handle:
                    handle.write(b"b")
            _current, grown, valid = probe_log_files(root, [str(path) for path in files], current, grown)
            self.assertTrue(valid)
            self.assertEqual(len(grown), 3)
            with tempfile.TemporaryDirectory() as raw_outside:
                outside = Path(raw_outside) / "telemetry.csv"
                outside.write_bytes(b"x")
                bad = [str(outside), str(files[1]), str(files[2])]
                self.assertFalse(probe_log_files(root, bad, current, grown)[2])

    def test_log_probe_rejects_symlink_when_supported(self):
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root).resolve()
            real = root / "real"
            real.mkdir()
            link = root / "link"
            try:
                link.symlink_to(real, target_is_directory=True)
            except OSError:
                self.skipTest("symlinks unavailable")
            files = [link / "telemetry.csv", link / "processed_video.mp4", link / "map.mp4"]
            for name in ("telemetry.csv", "processed_video.mp4", "map.mp4"):
                (real / name).write_bytes(b"x")
            self.assertFalse(probe_log_files(root, [str(path) for path in files], {}, set())[2])

    def test_atomic_evidence_has_sha_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as raw_root:
            root = Path(raw_root).resolve()
            artifact = atomic_persist(root, "run-1.json", {"state": "PASS", "pass": True})
            self.assertEqual(len(artifact["sha256"]), 64)
            self.assertTrue(Path(artifact["path"]).is_file())
            self.assertEqual(
                hashlib.sha256(Path(artifact["path"]).read_bytes()).hexdigest(),
                artifact["sha256"],
            )
            with self.assertRaises(FileExistsError):
                atomic_persist(root, "run-1.json", {"state": "PASS", "pass": True})


if __name__ == "__main__":
    unittest.main()
