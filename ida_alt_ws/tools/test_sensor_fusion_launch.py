"""Static launch contract tests for the opt-in sensor-fusion rollout."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
GAZEBO = ROOT / "src" / "ida_bringup" / "launch" / "sim_gazebo.launch.py"
FULL = ROOT / "src" / "ida_bringup" / "launch" / "sim_full_mission.launch.py"
REAL = ROOT / "src" / "ida_bringup" / "launch" / "real_vehicle.launch.py"
CONFIG = ROOT / "src" / "ida_bringup" / "config" / "fusion.yaml"
PERCEPTION_SIM = (
    ROOT / "src" / "ida_perception_sim" / "ida_perception_sim" / "perception_sim_node.py"
)


class SensorFusionLaunchContractTest(unittest.TestCase):
    def test_sim_launches_are_opt_in_and_wire_raw_sources(self):
        for path in (GAZEBO, FULL):
            with self.subTest(path=path.name):
                text = path.read_text(encoding="utf-8")
                self.assertIn('DeclareLaunchArgument("sensor_fusion_enabled", default_value="false")', text)
                self.assertIn('DeclareLaunchArgument("sensor_fusion_shadow_mode", default_value="true")', text)
                self.assertIn('"publish_raw": sensor_fusion_enabled', text)
                self.assertIn('"publish_canonical": perception_sim_publish_canonical', text)
                self.assertIn(
                    'DeclareLaunchArgument("perception_sim_lidar_range_m", default_value="35.0")',
                    text,
                )
                self.assertIn('"lidar_range_m": perception_sim_lidar_range_m', text)
                self.assertIn('package="ida_sensor_fusion"', text)
                self.assertIn('condition=IfCondition(sensor_fusion_enabled)', text)
                self.assertIn('"model_loaded": True', text)

    def test_production_fusion_is_takeover_scoped_and_fail_closed(self):
        text = REAL.read_text(encoding="utf-8")
        self.assertIn('package="ida_sensor_fusion"', text)
        self.assertIn('condition=IfCondition(canonical_takeover_enabled)', text)
        self.assertIn('"shadow_mode": False', text)
        for name in (
            "fusion_model_loaded",
            "camera_calibrated",
            "lidar_calibrated",
            "extrinsics_calibrated",
        ):
            self.assertIn(
                f'DeclareLaunchArgument("{name}", default_value="false")', text
            )
        self.assertIn(
            'declare_parameter("lidar_range_m", 18.0)',
            PERCEPTION_SIM.read_text(encoding="utf-8"),
        )

    def test_config_is_fail_closed_and_uses_raw_topics(self):
        text = CONFIG.read_text(encoding="utf-8")
        self.assertIn('camera_topic_p1p2: "/perception/camera/p1p2/raw"', text)
        self.assertIn('lidar_topic: "/perception/lidar/raw_obstacles"', text)
        self.assertIn("model_loaded: false", text)
        self.assertIn("extrinsics_calibrated: false", text)
        self.assertIn("max_association_items: 10", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
