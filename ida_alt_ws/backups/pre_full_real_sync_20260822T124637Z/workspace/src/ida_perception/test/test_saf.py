"""Saf-Python (rclpy'siz) unittest'ler: ida_perception.

model_loader.parse_class_names, class_mapping ve rplidar.process_scan saf
fonksiyonlarını test eder. Jetson'da ``python -m unittest`` ile çalışır
(ament bağımlılığı yok).
"""

import math
from pathlib import Path
import tempfile
import types
import unittest

from ida_perception.class_mapping import (
    DEFAULT_CLASS_NAME_MAP,
    map_class_name,
    parse_class_name_map,
    parse_class_name_map_strict,
)
from ida_perception.model_loader import (
    DEFAULT_CLASS_NAMES,
    ModelInferenceError,
    load_model,
    parse_class_names,
)
from ida_perception.rplidar import demo_scan, process_scan


class TestClassMapping(unittest.TestCase):
    def test_default_map_is_empty_identity(self) -> None:
        # Varsayılan eşleme boş: P3 modeli İngilizce (red/green/black) zaten
        # kontrat diliyle çıktı verir -> kimlik eşleme.
        self.assertEqual(DEFAULT_CLASS_NAME_MAP, {})

    def test_map_english_identity(self) -> None:
        mapping = parse_class_name_map('{"red": "red", "green": "green", "black": "black"}')
        self.assertEqual(map_class_name("red", mapping), "red")
        self.assertEqual(map_class_name("green", mapping), "green")
        self.assertEqual(map_class_name("black", mapping), "black")

    def test_unmapped_class_returns_none(self) -> None:
        mapping = parse_class_name_map('{"red": "red"}')
        self.assertIsNone(map_class_name("mavi", mapping))  # eşlenemeyen -> None

    def test_empty_map_identity(self) -> None:
        # Boş eşleme (P3 İngilizce model): kimlik eşleme — olduğu gibi geçer.
        self.assertEqual(map_class_name("red", {}), "red")
        self.assertEqual(map_class_name("green", {}), "green")

    def test_parse_map_case_insensitive(self) -> None:
        # Model RED/Black döndürebilir; normalize edilir.
        mapping = parse_class_name_map('{"RED": "red", "Black": "black"}')
        self.assertEqual(map_class_name("red", mapping), "red")
        self.assertEqual(map_class_name("black", mapping), "black")

    def test_parse_map_invalid_json_empty(self) -> None:
        self.assertEqual(parse_class_name_map(""), {})
        self.assertEqual(parse_class_name_map("not json"), {})
        self.assertEqual(parse_class_name_map("[1,2]"), {})

    def test_strict_map_requires_exact_native_keys_and_allowed_colors(self) -> None:
        self.assertEqual(
            parse_class_name_map_strict("", ["orange", "yellow"]), {}
        )
        self.assertEqual(
            parse_class_name_map_strict(
                '{"turuncu":"orange","sari":"yellow"}',
                ["turuncu", "sari"],
            ),
            {"turuncu": "orange", "sari": "yellow"},
        )
        for raw, names in (
            ("bad json", ["orange"]),
            ('{"orange":"blue"}', ["orange"]),
            ('{"orange":"orange"}', ["orange", "yellow"]),
            ('{"orange":"red","yellow":"red"}', ["orange", "yellow"]),
            ('{"orange":"RED","yellow":"red"}', ["orange", "yellow"]),
            ("", ["unsupported"]),
        ):
            with self.subTest(raw=raw, names=names), self.assertRaises(ValueError):
                parse_class_name_map_strict(raw, names)


class TestParseClassNames(unittest.TestCase):
    def test_comma_separated(self) -> None:
        self.assertEqual(
            parse_class_names("orange,yellow,black,red,green"),
            ["orange", "yellow", "black", "red", "green"],
        )

    def test_json_list(self) -> None:
        self.assertEqual(parse_class_names('["red", "green", "blue"]'), ["red", "green", "blue"])

    def test_empty_falls_back_to_default(self) -> None:
        self.assertEqual(parse_class_names(""), DEFAULT_CLASS_NAMES)
        self.assertEqual(parse_class_names("   "), DEFAULT_CLASS_NAMES)

    def test_garbage_falls_back_to_default(self) -> None:
        self.assertEqual(parse_class_names("not a list"), DEFAULT_CLASS_NAMES)
        self.assertEqual(parse_class_names("[1, 2]"), DEFAULT_CLASS_NAMES)

    def test_case_normalized(self) -> None:
        self.assertEqual(parse_class_names("RED, Green"), ["red", "green"])


class TestLoadModel(unittest.TestCase):
    def test_empty_path_returns_none(self) -> None:
        self.assertIsNone(load_model(""))
        self.assertIsNone(load_model("   "))

    def test_missing_file_returns_none(self) -> None:
        self.assertIsNone(load_model("C:/does/not/exist.pt"))

    def test_unknown_extension_returns_none(self) -> None:
        self.assertIsNone(load_model("C:/does/not/exist.txt"))

    def test_pt_and_engine_use_same_ultralytics_adapter(self) -> None:
        class Model:
            names = {0: "orange", 1: "yellow"}

            def __init__(self):
                self.calls = []

            def predict(self, **kwargs):
                self.calls.append(kwargs)
                boxes = types.SimpleNamespace(
                    xyxy=[[30.0, 40.0, 50.0, 80.0], [10.0, 20.0, 20.0, 50.0]],
                    conf=[0.8, 0.9],
                    cls=[1.0, 0.0],
                )
                return [types.SimpleNamespace(boxes=boxes)]

        with tempfile.TemporaryDirectory() as directory:
            for suffix in (".pt", ".engine"):
                with self.subTest(suffix=suffix):
                    path = Path(directory) / f"model{suffix}"
                    path.write_bytes(b"artifact")
                    model = Model()
                    loaded = load_model(
                        str(path),
                        ["orange", "yellow"],
                        0.4,
                        0.5,
                        imgsz=512,
                        device="cpu",
                        yolo_factory=lambda _path, model=model: model,
                    )
                    self.assertIsNotNone(loaded)
                    detections = loaded[2]("frame")
                    self.assertEqual([item["class"] for item in detections], ["orange", "yellow"])
                    self.assertEqual(detections[0]["bbox"], (10.0, 20.0, 10.0, 30.0))
                    self.assertEqual(model.calls[-1], {
                        "source": "frame",
                        "conf": 0.4,
                        "iou": 0.5,
                        "imgsz": 512,
                        "device": "cpu",
                        "verbose": False,
                    })

    def test_native_class_order_mismatch_and_load_exception_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.engine"
            path.write_bytes(b"artifact")
            wrong = types.SimpleNamespace(names={0: "yellow", 1: "orange"})
            self.assertIsNone(load_model(
                str(path), ["orange", "yellow"], yolo_factory=lambda _path: wrong
            ))

            def broken(_path):
                raise RuntimeError("load failed")

            self.assertIsNone(load_model(
                str(path), ["orange", "yellow"], yolo_factory=broken
            ))

    def test_predict_exception_is_exposed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.pt"
            path.write_bytes(b"artifact")

            class Model:
                names = ["red", "green", "black"]

                @staticmethod
                def predict(**_kwargs):
                    raise RuntimeError("predict failed")

            loaded = load_model(
                str(path), ["red", "green", "black"], yolo_factory=lambda _path: Model()
            )
            self.assertIsNotNone(loaded)
            with self.assertRaises(ModelInferenceError):
                loaded[2](object())

    def test_runtime_arguments_are_validated_before_model_load(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.pt"
            path.write_bytes(b"artifact")
            calls = []
            factory = lambda _path: calls.append(_path)
            for kwargs in (
                {"imgsz": 0},
                {"imgsz": True},
                {"device": " cpu "},
                {"confidence_threshold": float("nan")},
                {"confidence_threshold": 1.01},
                {"iou_threshold": -0.01},
            ):
                with self.subTest(kwargs=kwargs):
                    self.assertIsNone(load_model(
                        str(path), ["orange", "yellow"], yolo_factory=factory, **kwargs
                    ))
            self.assertEqual(calls, [])

    def test_result_schema_mismatch_and_all_invalid_are_exposed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.engine"
            path.write_bytes(b"artifact")

            def loaded_with(boxes):
                model = types.SimpleNamespace(
                    names={0: "orange"},
                    predict=lambda **_kwargs: [types.SimpleNamespace(boxes=boxes)],
                )
                return load_model(
                    str(path), ["orange"], yolo_factory=lambda _path: model
                )

            empty = loaded_with(types.SimpleNamespace(xyxy=[], conf=[], cls=[]))
            self.assertEqual(empty[2](object()), [])

            mismatch = loaded_with(types.SimpleNamespace(
                xyxy=[[0.0, 0.0, 1.0, 1.0]], conf=[], cls=[0.0]
            ))
            with self.assertRaises(ModelInferenceError):
                mismatch[2](object())

            all_invalid = loaded_with(types.SimpleNamespace(
                xyxy=[[0.0, 0.0, float("nan"), 1.0]], conf=[0.9], cls=[0.0]
            ))
            with self.assertRaises(ModelInferenceError):
                all_invalid[2](object())

            no_results_model = types.SimpleNamespace(
                names={0: "orange"}, predict=lambda **_kwargs: []
            )
            no_results = load_model(
                str(path), ["orange"], yolo_factory=lambda _path: no_results_model
            )
            with self.assertRaises(ModelInferenceError):
                no_results[2](object())


class TestProcessScan(unittest.TestCase):
    def test_single_cluster_forward(self) -> None:
        # 5 m ileride, 0° civarında 3 nokta -> tek engel, lateral ~0.
        points = [(359.0, 5.0), (0.0, 5.0), (1.0, 5.0)]
        obstacles = process_scan(points, stamp=123.0)
        self.assertEqual(len(obstacles), 1)
        obs = obstacles[0]
        self.assertAlmostEqual(obs["forward_m"], 5.0, delta=0.2)
        self.assertAlmostEqual(obs["lateral_m"], 0.0, delta=0.2)
        self.assertEqual(obs["stamp"], 123.0)

    def test_two_clusters_separated(self) -> None:
        # 5 m ileride ve 30° sağda 8 m -> iki ayrı engel.
        points = [(0.0, 5.0), (1.0, 5.0), (2.0, 5.0), (30.0, 8.0), (31.0, 8.0), (32.0, 8.0)]
        obstacles = process_scan(points)
        self.assertEqual(len(obstacles), 2)
        self.assertGreater(obstacles[1]["lateral_m"], obstacles[0]["lateral_m"])

    def test_behind_points_filtered(self) -> None:
        # 180° (arkada) ve çok yakın (0.1 m) noktalar elenir.
        points = [(180.0, 5.0), (179.0, 5.0), (181.0, 5.0), (0.0, 0.1), (1.0, 0.1), (2.0, 0.1)]
        obstacles = process_scan(points)
        self.assertEqual(obstacles, [])

    def test_min_cluster_points(self) -> None:
        # 2 noktalık küme gürültü sayılır (min=3).
        points = [(0.0, 5.0), (1.0, 5.0)]
        self.assertEqual(process_scan(points), [])
        self.assertEqual(len(process_scan(points, min_cluster_points=2)), 1)

    def test_range_filtering(self) -> None:
        # lidar_range_m üstü noktalar elenir.
        points = [(0.0, 30.0), (1.0, 30.0), (2.0, 30.0)]
        self.assertEqual(process_scan(points, lidar_range_m=18.0), [])

    def test_none_and_junk_values(self) -> None:
        points = [(0.0, 5.0), (1.0, None), ("x", "y"), (2.0, 5.0), (3.0, 5.0)]
        obstacles = process_scan(points)
        self.assertEqual(len(obstacles), 1)

    def test_angle_wrap_360(self) -> None:
        # 358..2 arası sarmalama: tek engel (yön 0° yakını).
        points = [(358.0, 4.0), (359.0, 4.0), (0.0, 4.0), (1.0, 4.0), (2.0, 4.0)]
        obstacles = process_scan(points)
        self.assertEqual(len(obstacles), 1)
        self.assertGreater(obstacles[0]["forward_m"], 3.0)

    def test_lateral_sign_positive_right(self) -> None:
        # Pozitif açı sağa (lateral_m > 0).
        points = [(30.0, 10.0), (31.0, 10.0), (32.0, 10.0)]
        obstacles = process_scan(points)
        self.assertEqual(len(obstacles), 1)
        self.assertGreater(obstacles[0]["lateral_m"], 0.0)
        self.assertAlmostEqual(
            obstacles[0]["lateral_m"], 10.0 * math.sin(math.radians(31.0)), delta=0.5
        )

    def test_demo_scan_deterministic(self) -> None:
        scan = demo_scan()
        self.assertEqual(len(scan), 6)
        # Aynı girdi aynı çıktı: determinizm.
        self.assertEqual(demo_scan(), scan)


if __name__ == "__main__":
    unittest.main()
