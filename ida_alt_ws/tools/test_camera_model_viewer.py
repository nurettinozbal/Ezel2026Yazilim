import math
import unittest

from camera_model_viewer import (
    GENERAL_NAMES,
    controls_from_positions,
    detection_label_color,
    nominal_bearing_deg,
    parse_v4l2_controls,
)


class TestCameraViewerGeometry(unittest.TestCase):
    def test_general_model_manifest_is_the_deployed_order(self) -> None:
        self.assertEqual(GENERAL_NAMES, ["black", "green", "orange", "red", "yellow"])

    def test_center_and_sign(self) -> None:
        self.assertAlmostEqual(nominal_bearing_deg(480.0, 960, 82.0), 0.0)
        self.assertLess(nominal_bearing_deg(0.0, 960, 82.0), 0.0)
        self.assertGreater(nominal_bearing_deg(960.0, 960, 82.0), 0.0)

    def test_nominal_edges_match_fov(self) -> None:
        self.assertAlmostEqual(nominal_bearing_deg(0.0, 960, 82.0), -41.0)
        self.assertAlmostEqual(nominal_bearing_deg(960.0, 960, 82.0), 41.0)

    def test_invalid_geometry(self) -> None:
        for width, fov in ((0, 82.0), (960, 0.0), (960, 180.0), (960, math.inf)):
            with self.subTest(width=width, fov=fov), self.assertRaises(ValueError):
                nominal_bearing_deg(480.0, width, fov)

    def test_black_detection_uses_readable_white_label(self) -> None:
        self.assertEqual(detection_label_color("black", (0, 0, 0)), (255, 255, 255))
        self.assertEqual(detection_label_color("orange", (0, 140, 255)), (0, 140, 255))


class TestCameraControlContract(unittest.TestCase):
    def test_parse_controls(self) -> None:
        parsed = parse_v4l2_controls(
            "brightness: -4\nauto_exposure: 0 (Auto Mode)\ninvalid: bad\n"
        )
        self.assertEqual(parsed, {"brightness": -4, "auto_exposure": 0})

    def test_positions_map_and_clamp(self) -> None:
        values = controls_from_positions(
            {
                "auto_exposure": 0,
                "exposure": 0,
                "gain": 0,
                "auto_wb": 0,
                "wb_kelvin": 7000,
                "saturation": 99,
                "brightness": 0,
                "contrast": 99,
            }
        )
        self.assertEqual(values["auto_exposure"], 1)
        self.assertEqual(values["exposure_time_absolute"], 5)
        self.assertEqual(values["gain"], 168)
        self.assertEqual(values["white_balance_automatic"], 0)
        self.assertEqual(values["white_balance_temperature"], 6500)
        self.assertEqual(values["brightness"], -64)
        self.assertEqual(values["saturation"], 15)
        self.assertEqual(values["contrast"], 20)


if __name__ == "__main__":
    unittest.main()
