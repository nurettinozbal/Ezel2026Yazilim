"""Saf sim gate-truth kontrat testleri (ROS kurulumu gerektirmez)."""

import os
import sys
import types
import unittest
import json
from pathlib import Path

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
for path in (
    ROOT,
    os.path.join(ROOT, "ida_planning"),
    os.path.join(ROOT, "ida_sensor_fusion"),
):
    if path not in sys.path:
        sys.path.insert(0, path)

from ida_planning.planner import PairCrossingDetector
from ida_sensor_fusion.contracts import result_payloads
from ida_sensor_fusion.core import (
    ReadinessFlags,
    fuse_frames,
    sanitize_camera_frame,
    sanitize_lidar_frame,
)

try:
    from ida_perception_sim.perception_sim_node import PerceptionSimNode
except ModuleNotFoundError:
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy.node = rclpy_node
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {"__init__": lambda self: setattr(self, "data", "")})
    std_msgs.msg = std_msgs_msg
    sys.modules.update({
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    })
    from ida_perception_sim.perception_sim_node import PerceptionSimNode


class GateTruthTest(unittest.TestCase):
    def _node(self):
        node = PerceptionSimNode.__new__(PerceptionSimNode)
        node.gate_truth_range = 45.0
        node.gate_truth_behind = 2.0
        node.scenario = {"buoys": [
            {"id": "p2_o_l1", "color": "orange", "x": 0.0, "y": -4.0},
            {"id": "p2_o_r1", "color": "orange", "x": 0.0, "y": 4.0},
            {"id": "orphan_l2", "color": "orange", "x": 10.0, "y": -4.0},
            {"id": "yellow_1", "color": "yellow", "x": 5.0, "y": 0.0},
        ]}
        return node

    def test_canonical_camera_drops_behind_but_truth_keeps_pair(self):
        node = self._node()
        for boat_x in (-6.0, -5.0, 0.0, 0.6):
            canonical_left = node.object_to_detection(
                node.scenario["buoys"][0], boat_x, 0.0, 0.0, 35.0
            )
            truth = node.build_gate_truth(boat_x, 0.0, 0.0)
            self.assertEqual(len(truth), 2)
            self.assertEqual({item["id"] for item in truth}, {"p2_o_l1", "p2_o_r1"})
            if boat_x == 0.6:
                self.assertIsNone(canonical_left)
                self.assertAlmostEqual(truth[0]["forward_m"], -0.6, places=6)

    def test_truth_is_fov_independent_and_contains_only_complete_stable_orange_pairs(self):
        node = self._node()
        truth = node.build_gate_truth(-6.0, 0.0, 30.0)
        self.assertEqual(len(truth), 2)
        self.assertTrue(all(item["source"] == "sim_gate_truth" for item in truth))
        self.assertTrue(all(item["color"] == "orange" for item in truth))

    def test_generated_scenario_initial_pose_does_not_count_gate_one(self):
        scenario_path = (
            Path(__file__).resolve().parents[1]
            / "ida_bringup" / "scenarios" / "full_mission.yaml"
        )
        scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
        node = self._node()
        node.scenario = scenario
        initial = scenario["initial_pose"]
        truth = node.build_gate_truth(
            float(initial["x"]), float(initial["y"]), float(initial["heading_deg"])
        )
        gate1 = [item for item in truth if item["id"] in {"p1_o_l1", "p1_o_r1"}]
        # Stable pair atomik yayınlanır; ancak merkez teknenin üzerinde
        # olduğundan `_new_pair_is_ahead` gate'i bilinçli olarak kurmaz.
        self.assertEqual(len(gate1), 2)
        det = PairCrossingDetector(allow_sim_truth_geometry=True)
        stats = det.update(gate1, now=1.0)
        self.assertEqual(stats.kd_estimate, 0)
        self.assertEqual(stats.crossed_count, 0)

    def test_build_truth_to_detector_crosses_oblique_gate_once(self):
        node = self._node()
        for heading in (0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 45.0):
            with self.subTest(heading=heading):
                det = PairCrossingDetector(allow_sim_truth_geometry=True)
                for stamp, boat_x in enumerate((-10.0, -6.0, 0.0, 1.0), start=1):
                    truth = node.build_gate_truth(boat_x, 0.0, heading)
                    pair = [item for item in truth if item["id"] in {"p2_o_l1", "p2_o_r1"}]
                    self.assertEqual(len(pair), 2)
                    stats = det.update(pair, now=float(stamp))
                self.assertEqual(stats.kd_estimate, 1)
                self.assertEqual(stats.crossed_count, 1)
                self.assertLessEqual(stats.crossed_count, stats.kd_estimate)

                # Merkez truth cutoff'un arkasında kaybolur; stale/history ve
                # yeniden yaklaşma aynı stable gate'i tekrar sayamaz.
                cutoff = node.build_gate_truth(4.0, 0.0, heading)
                cutoff_pair = [
                    item for item in cutoff if item["id"] in {"p2_o_l1", "p2_o_r1"}
                ]
                self.assertEqual(cutoff_pair, [])
                det.update([], now=100.0)
                reappear = [
                    item for item in node.build_gate_truth(-10.0, 0.0, heading)
                    if item["id"] in {"p2_o_l1", "p2_o_r1"}
                ]
                final = det.update(reappear, now=101.0)
                self.assertEqual(final.crossed_count, 1)
                self.assertEqual(final.kd_estimate, 1)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))


class _ClockNow:
    nanoseconds = 12_345_000_000


class _Clock:
    def now(self):
        return _ClockNow()


class RawSensorTest(unittest.TestCase):
    def _node(self, telemetry=True):
        node = PerceptionSimNode.__new__(PerceptionSimNode)
        node.camera_range = 35.0
        node.camera_fov = 90.0
        node.lidar_range = 18.0
        node.gate_truth_range = 45.0
        node.gate_truth_behind = 2.0
        node.publish_canonical = True
        node.publish_raw = True
        node.origin_lat = 0.0
        node.origin_lon = 0.0
        node.telemetry = {"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0} if telemetry else {}
        node.scenario = {
            "buoys": [
                {"id": "orange", "color": "orange", "x": 10.0, "y": 2.0},
                {"id": "yellow", "color": "yellow", "x": 8.0, "y": -3.0},
                {"id": "blue", "color": "blue", "x": 7.0, "y": 0.0},
                {"id": "behind", "color": "orange", "x": -4.0, "y": 1.0},
            ],
            "targets": [
                {"id": "red", "color": "red", "x": 12.0, "y": -2.0},
                {"id": "green", "color": "green", "x": 13.0, "y": 2.0},
                {"id": "black", "color": "black", "x": 14.0, "y": 0.0},
                {"id": "white", "color": "white", "x": 9.0, "y": 0.0},
            ],
            "obstacles": [
                {"id": "rock", "class": "obstacle", "x": 5.0, "y": 4.0},
                {"id": "far", "class": "obstacle", "x": 30.0, "y": 0.0},
            ],
        }
        node.buoy_pub = _Publisher()
        node.obstacle_pub = _Publisher()
        node.gate_truth_pub = _Publisher()
        node.raw_p1p2_pub = _Publisher()
        node.raw_p3_pub = _Publisher()
        node.raw_lidar_pub = _Publisher()
        node.get_clock = lambda: _Clock()
        return node

    def test_publisher_topic_list_obeys_single_writer_flags(self):
        def topics(canonical, raw):
            node = PerceptionSimNode.__new__(PerceptionSimNode)
            node.publish_canonical = canonical
            node.publish_raw = raw
            created = []

            def create_publisher(_msg_type, topic, _depth):
                created.append(topic)
                return _Publisher()

            node.create_publisher = create_publisher
            node._create_output_publishers(
                "/perception/camera/p1p2/raw",
                "/perception/camera/p3/raw",
                "/perception/lidar/raw_obstacles",
            )
            return created, node

        canonical_topics, canonical_node = topics(True, False)
        self.assertEqual(canonical_topics, [
            "/perception/buoys",
            "/perception/obstacles",
            "/perception_sim/gate_truth",
        ])
        self.assertIsNone(canonical_node.raw_p1p2_pub)
        self.assertIsNone(canonical_node.raw_p3_pub)
        self.assertIsNone(canonical_node.raw_lidar_pub)

        raw_topics, raw_node = topics(False, True)
        self.assertEqual(raw_topics, [
            "/perception_sim/gate_truth",
            "/perception/camera/p1p2/raw",
            "/perception/camera/p3/raw",
            "/perception/lidar/raw_obstacles",
        ])
        self.assertIsNone(raw_node.buoy_pub)
        self.assertIsNone(raw_node.obstacle_pub)

    def test_raw_camera_role_filters_and_lidar_contains_all_physical_classes(self):
        node = self._node()
        node.tick()
        p1p2 = node.raw_p1p2_pub.messages[-1]
        p3 = node.raw_p3_pub.messages[-1]
        lidar = node.raw_lidar_pub.messages[-1]
        self.assertEqual({d["id"] for d in p1p2["detections"]}, {"orange", "yellow"})
        self.assertEqual({d["id"] for d in p3["detections"]}, {"red", "green", "black"})
        self.assertEqual(
            {d["id"] for d in lidar["clusters"]},
            {"orange", "yellow", "blue", "behind", "red", "green", "black", "white", "rock"},
        )

    def test_raw_lidar_uses_explicit_ros_left_sign_and_one_tick_stamp(self):
        node = self._node()
        node.tick()
        p1p2 = node.raw_p1p2_pub.messages[-1]
        p3 = node.raw_p3_pub.messages[-1]
        lidar = node.raw_lidar_pub.messages[-1]
        self.assertEqual(p1p2["acquisition_stamp"], 12.345)
        self.assertEqual(p1p2["acquisition_stamp"], p3["acquisition_stamp"])
        self.assertEqual(p1p2["acquisition_stamp"], lidar["acquisition_stamp"])
        clusters = {item["id"]: item for item in lidar["clusters"]}
        self.assertAlmostEqual(clusters["orange"]["lateral_left_m"], -2.0)
        self.assertAlmostEqual(clusters["yellow"]["lateral_left_m"], 3.0)
        self.assertAlmostEqual(clusters["orange"]["forward_m"], 10.0)
        self.assertTrue(all("lateral_m" not in item for item in lidar["clusters"]))
        self.assertTrue(all(item["acquisition_stamp"] == 12.345 for item in lidar["clusters"]))

    def test_canonical_target_and_raw_lidar_share_stable_physical_identity(self):
        node = self._node()
        node.tick()
        buoys = {
            item["id"]: item
            for item in node.buoy_pub.messages[-1]["detections"]
        }
        obstacles = {
            item["id"]: item
            for item in node.obstacle_pub.messages[-1]["obstacles"]
        }
        raw = {
            item["id"]: item
            for item in node.raw_lidar_pub.messages[-1]["clusters"]
        }

        target = buoys["green"]
        self.assertEqual(target["lidar_obstacle_id"], "green")
        self.assertEqual(target["source"], "sim_camera_lidar_fused")
        self.assertEqual(obstacles["green"]["lidar_obstacle_id"], "green")
        self.assertTrue(obstacles["green"]["hard_obstacle"])
        self.assertEqual(obstacles["green"]["source"], "sim_camera_lidar_fused")
        self.assertEqual(raw["green"]["id"], "green")
        self.assertAlmostEqual(target["distance"], obstacles["green"]["distance"])
        self.assertAlmostEqual(target["distance"], raw["green"]["distance_m"])

    def test_camera_only_target_does_not_claim_lidar_verification(self):
        node = self._node()
        node.lidar_range = 10.0
        node.tick()
        buoys = {
            item["id"]: item
            for item in node.buoy_pub.messages[-1]["detections"]
        }
        obstacles = {
            item["id"]: item
            for item in node.obstacle_pub.messages[-1]["obstacles"]
        }
        self.assertIn("green", buoys)
        self.assertNotIn("lidar_obstacle_id", buoys["green"])
        self.assertNotIn("green", obstacles)

    def test_raw_p3_and_lidar_frames_fuse_to_same_stable_lidar_track(self):
        node = self._node()
        node.tick()
        camera_payload = node.raw_p3_pub.messages[-1]
        lidar_payload = node.raw_lidar_pub.messages[-1]
        camera = sanitize_camera_frame(
            camera_payload["acquisition_stamp"], camera_payload["detections"]
        )
        lidar = sanitize_lidar_frame(
            lidar_payload["acquisition_stamp"], lidar_payload["clusters"]
        )
        self.assertIsNotNone(camera)
        self.assertIsNotNone(lidar)
        result = fuse_frames(
            camera,
            lidar,
            ReadinessFlags(True, True, True, True),
            max_bearing_error_deg=6.0,
        )
        fused_buoys, fused_obstacles, status = result_payloads(result)
        green = next(
            item for item in fused_buoys["detections"] if item["color"] == "green"
        )
        obstacle = next(
            item for item in fused_obstacles["obstacles"]
            if item["id"] == green["lidar_obstacle_id"]
        )
        self.assertEqual(green["lidar_obstacle_id"], "green")
        self.assertEqual(obstacle["id"], "green")
        self.assertAlmostEqual(green["distance"], obstacle["distance"])
        self.assertTrue(status["accepted"])

    def test_sim_lidar_range_override_exposes_p3_target_at_23m(self):
        node = self._node()
        target = {"id": "p3_target", "color": "green", "x": 23.0, "y": 0.0}
        node.lidar_range = 18.0
        self.assertIsNone(node.object_to_raw_lidar(target, 0.0, 0.0, 0.0, 1.0))
        node.lidar_range = 35.0
        cluster = node.object_to_raw_lidar(target, 0.0, 0.0, 0.0, 1.0)
        self.assertIsNotNone(cluster)
        self.assertEqual(cluster["id"], "p3_target")
        self.assertAlmostEqual(cluster["forward_m"], 23.0)

    def test_canonical_can_be_disabled_without_disabling_raw_or_gate_truth(self):
        node = self._node()
        node.publish_canonical = False
        node.tick()
        self.assertEqual(node.buoy_pub.messages, [])
        self.assertEqual(node.obstacle_pub.messages, [])
        self.assertEqual(len(node.raw_p1p2_pub.messages), 1)
        self.assertEqual(len(node.raw_p3_pub.messages), 1)
        self.assertEqual(len(node.raw_lidar_pub.messages), 1)
        self.assertEqual(len(node.gate_truth_pub.messages), 1)

    def test_no_telemetry_publishes_stale_empty_raw_frames(self):
        node = self._node(telemetry=False)
        node.tick()
        self.assertEqual(
            node.raw_p1p2_pub.messages[-1],
            {"acquisition_stamp": 12.345, "detections": [], "stale": True},
        )
        self.assertEqual(
            node.raw_p3_pub.messages[-1],
            {"acquisition_stamp": 12.345, "detections": [], "stale": True},
        )
        self.assertEqual(
            node.raw_lidar_pub.messages[-1],
            {"acquisition_stamp": 12.345, "clusters": [], "stale": True},
        )
        self.assertEqual(node.gate_truth_pub.messages[-1]["detections"], [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
