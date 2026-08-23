import importlib.util
import json
import pathlib
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "passive_costmap_viewer.py"
sys.path.insert(0, str(ROOT / "src" / "ida_planning"))
SPEC = importlib.util.spec_from_file_location("passive_costmap_viewer", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class PassiveCostmapViewerTests(unittest.TestCase):
    def test_fused_orange_is_visible_and_shadow_command_is_advisory(self):
        state = MODULE.PassiveShadowPlanner()
        now = 10.0
        buoy = {
            "stamp": 100.0,
            "detections": [{
                "id": "camera:orange:1", "lidar_obstacle_id": "lidar:1",
                "color": "orange", "confidence": 0.84, "distance": 4.0,
                "forward_m": 4.0, "lateral_m": 0.0, "bearing_deg": 0.0,
            }],
        }
        obstacle = {
            "stamp": 100.0,
            "obstacles": [{
                "id": "lidar:1", "color": "orange", "confidence": 0.84,
                "distance": 4.0, "forward_m": 4.0, "lateral_m": 0.0,
                "bearing_deg": 0.0, "hard_obstacle": True,
            }],
        }
        status = {
            "accepted": True, "source_health": "ok", "camera_fresh": True,
            "lidar_fresh": True, "matched_count": 1, "dt_ms": 12.0,
        }
        self.assertTrue(state.ingest("buoys", json.dumps(buoy), now))
        self.assertTrue(state.ingest("obstacles", json.dumps(obstacle), now))
        self.assertTrue(state.ingest("status", json.dumps(status), now))
        self.assertTrue(state.ingest("camera_raw", json.dumps({"detections": [{"color": "orange", "confidence": 0.84, "bearing_deg": 0.0}]}), now))
        self.assertTrue(state.ingest("lidar_raw", json.dumps({"clusters": [{"id": "lidar:1"}]}), now))
        out = state.snapshot(now + 0.1)
        self.assertTrue(out["accepted"])
        self.assertEqual(out["orange_count"], 1)
        self.assertEqual(out["hard_obstacle_count"], 1)
        self.assertEqual(out["camera_orange_count"], 1)
        self.assertEqual(out["lidar_cluster_count"], 1)
        self.assertEqual(out["camera_bearing_deg"], 0.0)
        self.assertEqual(out["nearest_lidar_delta_deg"], 0.0)
        self.assertEqual(out["orange_geometry"]["forward_m"], 4.0)
        self.assertTrue(any(cell[3] == "orange" for cell in out["costmap"]["cells"]))
        self.assertIsNotNone(out["command"])
        self.assertTrue(out["command"]["advisory_only"])

    def test_stale_data_never_produces_a_command(self):
        state = MODULE.PassiveShadowPlanner(freshness_s=1.0)
        for channel, payload in {
            "buoys": {"detections": []},
            "obstacles": {"obstacles": []},
            "status": {"accepted": True, "source_health": "ok", "camera_fresh": True, "lidar_fresh": True},
        }.items():
            state.ingest(channel, json.dumps(payload), 1.0)
        self.assertIsNone(state.snapshot(2.01)["command"])

    def test_close_obstacle_at_eighteen_degrees_cannot_choose_straight(self):
        state = MODULE.PassiveShadowPlanner()
        now = 20.0
        distance = 3.0
        bearing_deg = 18.0
        forward = distance * __import__("math").cos(__import__("math").radians(bearing_deg))
        lateral = distance * __import__("math").sin(__import__("math").radians(bearing_deg))
        buoy = {
            "detections": [{
                "id": "camera:orange", "color": "orange", "confidence": 0.8,
                "distance": distance, "forward_m": forward,
                "lateral_m": lateral, "bearing_deg": bearing_deg,
            }]
        }
        obstacle = {
            "obstacles": [{
                "id": "lidar:orange", "color": "orange", "confidence": 0.8,
                "distance": distance, "forward_m": forward,
                "lateral_m": lateral, "bearing_deg": bearing_deg,
                "hard_obstacle": True,
            }]
        }
        status = {
            "accepted": True, "source_health": "ok", "camera_fresh": True,
            "lidar_fresh": True, "matched_count": 1,
        }
        for channel, payload in (("buoys", buoy), ("obstacles", obstacle), ("status", status)):
            state.ingest(channel, json.dumps(payload), now)
        command = state.snapshot(now + 0.1)["command"]
        self.assertIsNotNone(command)
        self.assertNotEqual(command["yaw_rate_deg_s"], 0.0)

    def test_source_has_no_actuation_surface(self):
        source = MODULE_PATH.read_text(encoding="utf-8")
        forbidden = (
            "create_publisher", "create_client", "create_service",
            "pymavlink", "mavsdk", "RC_CHANNELS_OVERRIDE", "COMMAND_LONG",
        )
        for token in forbidden:
            self.assertNotIn(token, source)
        self.assertNotIn('"/control/', source)
        self.assertNotIn('"/autonomy/cmd_', source)
        self.assertIn('type="range"', MODULE.HTML)
        self.assertIn('max="8"', MODULE.HTML)


if __name__ == "__main__":
    unittest.main()
