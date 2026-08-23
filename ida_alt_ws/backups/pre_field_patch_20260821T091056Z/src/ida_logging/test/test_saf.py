"""Saf-Python (rclpy'siz) unittest'ler: ida_logging.

run_dir ve csv_writer saf stdlib modüllerini test eder. Jetson'da
``python -m unittest`` ile çalışır (ament bağımlılığı yok).
"""

import csv
import tempfile
import unittest
from pathlib import Path

from ida_logging.csv_writer import CsvWriter, write_csv_rows
from ida_logging.judge_contract import (
    REQUIRED_FILES,
    build_map_row,
    build_telemetry_row,
    evaluate_referee_files,
    parkur_from_state,
)
from ida_logging.run_dir import (
    make_run_dir,
    make_run_dir_same,
    resolve_log_path,
    run_dir_or_create,
)


class TestRunDir(unittest.TestCase):
    def test_make_run_dir_creates_timestamped_dir(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = make_run_dir(tmp)
            self.assertTrue(run_dir.is_dir())
            self.assertTrue(run_dir.name.startswith("run_"))
            # YYYYmmdd_HHMMSS formatı: run_20260807_123456 gibi.
            stamp = run_dir.name[len("run_"):]
            self.assertEqual(len(stamp), 15)
            self.assertIn("_", stamp)

    def test_make_run_dir_exist_ok(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            first = make_run_dir(tmp)
            # Aynı saniyede ikinci çağrı çakışırsa da hata vermez.
            second = make_run_dir(tmp)
            self.assertTrue(first.is_dir())
            self.assertTrue(second.is_dir())

    def test_run_dir_or_create_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            explicit = run_dir_or_create(tmp, explicit_dir="custom_dir")
            self.assertEqual(explicit.name, "custom_dir")
            self.assertTrue(explicit.is_dir())

    def test_run_dir_or_create_default_timestamped(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = run_dir_or_create(tmp)
            self.assertTrue(run_dir.name.startswith("run_"))

    def test_independent_loggers_share_explicit_run_name(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            names = ["telemetry.csv", "processed_video.mp4", "map.mp4"]
            paths = []
            for name in names:
                run_dir = make_run_dir_same(tmp, "run_acceptance")
                path = run_dir / name
                path.touch()
                paths.append(path)
            self.assertEqual({path.parent for path in paths}, {Path(tmp) / "run_acceptance"})

    def test_shared_run_name_rejects_path_escape_and_unsafe_names(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            for unsafe in ("../outside", ".", "..", "outside", "run_a/b", "run_a\\b"):
                with self.subTest(unsafe=unsafe):
                    with self.assertRaises(ValueError):
                        make_run_dir_same(tmp, unsafe)

    def test_shared_run_name_rejects_absolute_path(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            absolute = str((Path(tmp) / "run_absolute").resolve())
            with self.assertRaises(ValueError):
                make_run_dir_same(tmp, absolute)

    def test_resolve_log_path(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = resolve_log_path(tmp, "telemetry.csv")
            self.assertEqual(path.parent.is_dir(), True)
            self.assertEqual(path.name, "telemetry.csv")


class TestCsvWriter(unittest.TestCase):
    def test_header_first_row(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "out.csv"
            writer = CsvWriter(path, ["a", "b"])
            writer.add_row({"a": 1, "b": 2})
            writer.close()
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
            self.assertEqual(rows[0], ["a", "b"])
            self.assertEqual(rows[1], ["1", "2"])

    def test_extra_keys_ignored(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "out.csv"
            writer = CsvWriter(path, ["a"])
            writer.add_row({"a": 1, "c": 99})
            writer.close()
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
            self.assertEqual(rows[1], ["1"])

    def test_missing_keys_empty(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "out.csv"
            writer = CsvWriter(path, ["a", "b"])
            writer.add_row({"b": 2})
            writer.close()
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
            self.assertEqual(rows[1], ["", "2"])

    def test_nan_inf_serialized_empty(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "out.csv"
            writer = CsvWriter(path, ["x"])
            writer.add_row({"x": float("nan")})
            writer.add_row({"x": float("inf")})
            writer.close()
            text = path.read_text(encoding="utf-8")
            self.assertNotIn("nan", text.lower())
            self.assertNotIn("inf", text.lower())

    def test_write_after_close_ignored(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "out.csv"
            writer = CsvWriter(path, ["a"])
            writer.add_row({"a": 1})
            writer.close()
            writer.add_row({"a": 2})  # sessizce reddedilir
            self.assertTrue(writer.closed)
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
            self.assertEqual(len(rows), 2)  # header + 1 satır

    def test_write_csv_rows_helper(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bulk.csv"
            returned = write_csv_rows(path, ["a", "b"], [{"a": 1, "b": 2}, {"a": 3, "b": 4}])
            self.assertEqual(returned, path)
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
            self.assertEqual(len(rows), 3)  # header + 2 satır

    def test_float_values_kept(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "out.csv"
            writer = CsvWriter(path, ["v"])
            writer.add_row({"v": 40.8630521})
            writer.close()
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.reader(f))
            self.assertEqual(rows[1], ["40.8630521"])


class TestJudgeContract(unittest.TestCase):
    def test_parkur_and_final_command_are_recorded(self) -> None:
        row = build_telemetry_row(
            {
                "stamp": 12.5,
                "lat": 40.0,
                "lon": 29.0,
                "mode": "GUIDED",
                "motor_left_pwm": 1630,
                "motor_right_pwm": 1580,
            },
            {
                "state": "PARKUR_2_NAV",
                "current_waypoint": 4,
                "action": "dwa_clear_left",
            },
            {"vx": 0.8, "vy": 0.0, "yaw_rate": -0.2},
            99.0,
        )
        self.assertEqual(parkur_from_state("PARKUR_3_TARGET_LOCK"), 3)
        self.assertEqual(row["parkur"], 2)
        self.assertEqual(row["current_waypoint"], 4)
        self.assertEqual(row["setpoint_vx"], 0.8)
        self.assertEqual(row["motor_left_pwm"], 1630)

    def test_map_row_keeps_decision_and_perception_together(self) -> None:
        row = build_map_row(
            stamp=5.0,
            telemetry={"lat": 40.0, "lon": 29.0, "heading_deg": 90.0},
            autonomy={"state": "PARKUR_1_NAV", "current_waypoint": 2, "action": "dwa"},
            command={"vx": 0.7, "vy": 0.0, "yaw_rate": 0.1},
            obstacles=[{"id": "lidar-1", "forward_m": 2.0}],
            buoys=[{"id": "lidar-1", "color": "orange"}],
            cells=[[1, 2, 8]],
            costmap_meta={"resolution": 0.2},
            score={"estimated_score_1": 42.0},
        )
        self.assertEqual(row["autonomy"]["parkur"], 1)
        self.assertEqual(row["command"]["yaw_rate"], 0.1)
        self.assertEqual(row["obstacles"][0]["id"], "lidar-1")
        self.assertEqual(row["buoys"][0]["color"], "orange")

    def test_status_requires_real_rows_and_a_real_video_frame(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "telemetry.csv").write_text("stamp,lat\n", encoding="utf-8")
            (root / "map.mp4").touch()
            (root / "processed_video.mp4").touch()
            (root / "frames.csv").write_text("stamp,frame_index\n", encoding="utf-8")
            empty = evaluate_referee_files(root)
            self.assertFalse(empty["active"])
            self.assertEqual(empty["logger_count"], 0)

            (root / "telemetry.csv").write_text("stamp,lat\n1.0,40.0\n", encoding="utf-8")
            (root / "map.mp4").write_bytes(b"mp4-test-data")
            (root / "processed_video.mp4").write_bytes(b"mp4-evidence")
            (root / "frames.csv").write_text("stamp,frame_index\n1.0,0\n", encoding="utf-8")
            ready = evaluate_referee_files(root)
            self.assertTrue(ready["active"])
            self.assertEqual(ready["logger_count"], 3)
            self.assertEqual(tuple(ready["ready_files"]), REQUIRED_FILES)


if __name__ == "__main__":
    unittest.main()
