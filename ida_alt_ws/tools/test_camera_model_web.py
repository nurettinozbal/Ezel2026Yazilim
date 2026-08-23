import unittest
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from tempfile import TemporaryDirectory

from camera_model_web import (
    CAMERA_PRESETS,
    CameraRuntime,
    CONTROL_LIMITS,
    FILTER_DEFAULTS,
    FILTER_LIMITS,
    parse_model_class_contract,
    validate_control_patch,
    validate_filter_patch,
    validate_jpeg_quality,
)
from ida_bringup.camera_profile import (
    controls_to_ros_parameters,
    load_camera_control_profile,
    save_camera_control_profile,
    save_versioned_camera_control_profile,
)


class CameraModelWebContractTests(unittest.TestCase):
    def test_accepts_bounded_integer_controls(self):
        self.assertEqual(validate_control_patch({"gain": 400}), {"gain": 400})
        self.assertEqual(validate_control_patch({"auto_exposure": 1}), {"auto_exposure": 1})
        self.assertEqual(validate_control_patch({"power_line_frequency": 2}), {"power_line_frequency": 2})

    def test_rejects_unknown_nonfinite_bool_fraction_and_range(self):
        bad = (
            {}, {"shell": 1}, {"gain": True}, {"gain": float("nan")},
            {"gain": 168.5}, {"gain": CONTROL_LIMITS["gain"][1] + 1},
        )
        for payload in bad:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                validate_control_patch(payload)

    def test_filter_defaults_are_neutral(self):
        self.assertEqual(FILTER_DEFAULTS["clahe_enabled"], 0)
        self.assertEqual(FILTER_DEFAULTS["gamma_x100"], 100)
        self.assertEqual(FILTER_DEFAULTS["grayscale_mix"], 0)
        self.assertEqual(FILTER_DEFAULTS["sharpen_x100"], 0)

    def test_filter_patch_is_bounded_and_strict(self):
        self.assertEqual(validate_filter_patch({"clahe_grid": 8}), {"clahe_grid": 8})
        bad = (
            {}, {"unknown": 1}, {"gamma_x100": True},
            {"gamma_x100": 100.5},
            {"clahe_clip_x10": FILTER_LIMITS["clahe_clip_x10"][1] + 1},
        )
        for payload in bad:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                validate_filter_patch(payload)

    def test_jpeg_quality_is_high_by_default_and_strictly_bounded(self):
        self.assertEqual(validate_jpeg_quality(92), 92)
        for value in (59, 96, True, 90.5, float("nan")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_jpeg_quality(value)

    def test_document_presets_are_complete_bounded_and_color_preserving(self):
        self.assertEqual(
            set(CAMERA_PRESETS),
            {"model_base", "indoor_auto", *(f"p{i}" for i in range(1, 11))},
        )
        for name, preset in CAMERA_PRESETS.items():
            with self.subTest(name=name):
                self.assertEqual(validate_control_patch(preset["controls"]), preset["controls"])
                self.assertEqual(validate_filter_patch(preset["filters"]), preset["filters"])
                self.assertEqual(set(preset["controls"]), set(CONTROL_LIMITS))
                self.assertEqual(set(preset["filters"]), set(FILTER_LIMITS))
                self.assertEqual(preset["filters"]["grayscale_mix"], 0)
        self.assertIn("1920×1200@50", CAMERA_PRESETS["p10"]["note"])

    def test_indoor_preset_preserves_default_auto_color_pipeline(self):
        indoor = CAMERA_PRESETS["indoor_auto"]
        self.assertEqual(indoor["controls"]["auto_exposure"], 1)
        self.assertEqual(indoor["controls"]["auto_wb"], 1)
        self.assertEqual(indoor["controls"]["power_line_frequency"], 1)
        self.assertEqual(indoor["filters"], FILTER_DEFAULTS)

    def test_recent_rate_prunes_old_capture_samples(self):
        samples = deque([1.0, 8.1, 9.0, 9.9])
        self.assertEqual(CameraRuntime._recent_rate(samples, 10.0, 2.0), 1.5)
        self.assertEqual(list(samples), [8.1, 9.0, 9.9])

    def test_p3_buoy_manifest_maps_to_canonical_colors(self):
        names, mapping = parse_model_class_contract(
            "black_buoy,red_buoy,green_buoy",
            '{"black_buoy":"black","red_buoy":"red","green_buoy":"green"}',
        )
        self.assertEqual(names, ["black_buoy", "red_buoy", "green_buoy"])
        self.assertEqual(mapping["green_buoy"], "green")

    def test_model_manifest_mapping_is_exact_and_one_to_one(self):
        bad = (
            ("black_buoy,red_buoy", '{"black_buoy":"black"}'),
            ("black_buoy,red_buoy", '{"black_buoy":"black","red_buoy":"black"}'),
            ("black_buoy,red_buoy", '{"black_buoy":"black","red_buoy":"purple"}'),
            ("black_buoy,red_buoy", "not-json"),
        )
        for names, mapping in bad:
            with self.subTest(names=names, mapping=mapping), self.assertRaises(ValueError):
                parse_model_class_contract(names, mapping)

    def test_camera_field_profile_round_trip_and_ros_mapping(self):
        controls = dict(CAMERA_PRESETS["indoor_auto"]["controls"])
        controls.update(auto_exposure=0, exposure=75, auto_wb=0, wb_kelvin=5100)
        with TemporaryDirectory() as temporary:
            path = Path(temporary) / "camera_controls.json"
            saved = save_camera_control_profile(
                path, controls, saved_at_utc="2026-08-20T12:00:00Z"
            )
            loaded = load_camera_control_profile(path)
        self.assertEqual(loaded, saved)
        ros = controls_to_ros_parameters(loaded["controls"])
        self.assertEqual(ros["auto_exposure"], 1)
        self.assertEqual(ros["exposure_time_absolute"], 75)
        self.assertFalse(ros["white_balance_automatic"])
        self.assertEqual(ros["white_balance_temperature"], 5100)

    def test_camera_field_profile_rejects_partial_or_malformed_data(self):
        with TemporaryDirectory() as temporary:
            path = Path(temporary) / "camera_controls.json"
            with self.assertRaises(ValueError):
                save_camera_control_profile(path, {"gain": 200})
            path.write_text('{"schema_version":1}', encoding="utf-8")
            with self.assertRaises(ValueError):
                load_camera_control_profile(path)

    def test_versioned_camera_profile_keeps_time_named_snapshot_and_active_copy(self):
        controls = CAMERA_PRESETS["model_base"]["controls"]
        moment = datetime(2026, 8, 20, 14, 5, 6, 123456, tzinfo=timezone.utc)
        with TemporaryDirectory() as temporary:
            active = Path(temporary) / "camera_controls.json"
            saved, archive = save_versioned_camera_control_profile(
                active, controls, now=moment
            )
            self.assertEqual(archive.name, "camera_20260820_140506_123456.json")
            self.assertEqual(load_camera_control_profile(active), saved)
            self.assertEqual(load_camera_control_profile(archive), saved)


if __name__ == "__main__":
    unittest.main()
