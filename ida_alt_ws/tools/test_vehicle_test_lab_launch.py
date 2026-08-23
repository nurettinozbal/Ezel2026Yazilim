import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "src" / "ida_bringup" / "launch" / "vehicle_test_lab.launch.py"
PRODUCER = ROOT / "src" / "ida_vehicle_test" / "ida_vehicle_test" / "producer_node.py"
MONITOR = ROOT / "src" / "ida_vehicle_test" / "ida_vehicle_test" / "monitor_node.py"
REPLAY = ROOT / "src" / "ida_bringup" / "launch" / "jetson_replay_fusion.launch.py"


class VehicleTestLabLaunchTests(unittest.TestCase):
    def test_overlay_is_explicit_passive_and_contains_no_token_argument(self):
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('executable="vehicle_test_monitor"', text)
        self.assertIn('executable="vehicle_test_producer"', text)
        self.assertNotIn('DeclareLaunchArgument("token"', text)
        self.assertNotIn("cmd_vel", text)
        self.assertNotIn("ARM", text)
        self.assertIn('DeclareLaunchArgument("simulation_sources", default_value="false")', text)
        self.assertIn('"allow_sim_sources": simulation_sources', text)

    def test_producer_credential_is_environment_only(self):
        text = PRODUCER.read_text(encoding="utf-8")
        self.assertIn('os.environ.get("EZEL_JETSON_DEBUG_TOKEN"', text)
        self.assertNotIn('declare_parameter("token"', text)
        self.assertNotIn('get_parameter("token"', text)

    def test_nodes_do_not_overwrite_rclpy_subscription_registry(self):
        producer = PRODUCER.read_text(encoding="utf-8")
        monitor = (
            ROOT / "src" / "ida_vehicle_test" / "ida_vehicle_test" / "monitor_node.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("self._subscriptions =", producer)
        self.assertNotIn("self._subscriptions =", monitor)
        self.assertIn("self._request_subscription =", monitor)

    def test_monitor_graph_check_resolves_launch_remappings(self):
        text = MONITOR.read_text(encoding="utf-8")
        self.assertIn("self.resolve_topic_name(", text)
        self.assertIn("get_publishers_info_by_topic(resolved_topic)", text)

    def test_replay_launch_is_guarded_and_namespaced(self):
        text = REPLAY.read_text(encoding="utf-8")
        self.assertIn('os.environ.get("IDA_OFFLINE_REPLAY", "") != "1"', text)
        self.assertIn('os.environ.get("ROS_DOMAIN_ID", "0")', text)
        self.assertIn("offline replay ROS_DOMAIN_ID equals production domain", text)
        self.assertIn('ALT = "/ida_alt"', text)
        self.assertIn('"shadow_mode": True', text)
        self.assertIn('"raw_contract": True', text)
        self.assertIn('DeclareLaunchArgument("calibration_ready", default_value="false")', text)
        for forbidden in ("ida_control", "ida_autonomy", "ida_ws_gateway", "mavsdk_bridge"):
            self.assertNotIn(forbidden, text)


if __name__ == "__main__":
    unittest.main()
