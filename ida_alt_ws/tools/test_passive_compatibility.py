import ast
import tempfile
import unittest
from pathlib import Path

from check_passive_compatibility import compatibility_report, inspect_source


class PassiveCompatibilityTests(unittest.TestCase):
    def test_current_overlay_is_observe_only(self):
        report = compatibility_report()
        self.assertTrue(report["compatible"], report["errors"])
        self.assertFalse(report["actuation_enabled"])
        publishers = {
            topic
            for source in report["sources"]
            for topic in source["publishers"]
        }
        self.assertTrue(publishers)
        self.assertTrue(all(topic.startswith("/vehicle_test/") for topic in publishers))
        self.assertEqual(
            sorted(report["launch"]["executables"]),
            ["vehicle_test_monitor", "vehicle_test_producer"],
        )

    def test_production_publisher_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "unsafe.py"
            path.write_text(
                "def configure(node, String):\n"
                "    node.create_publisher(String, '/control/cmd_vel_body', 10)\n",
                encoding="utf-8",
            )
            report = inspect_source(path)
        self.assertIn(
            "production publisher forbidden: /control/cmd_vel_body",
            report["errors"],
        )

    def test_mavlink_import_and_service_are_rejected(self):
        tree = ast.parse("import pymavlink\nnode.create_service(object, 'x', print)\n")
        self.assertIsNotNone(tree)  # fixture syntax guard
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "unsafe.py"
            path.write_text(
                "import pymavlink\nnode.create_service(object, 'x', print)\n",
                encoding="utf-8",
            )
            errors = inspect_source(path)["errors"]
        self.assertIn("forbidden import: ['pymavlink']", errors)
        self.assertIn("forbidden call: create_service", errors)


if __name__ == "__main__":
    unittest.main()
