"""ROS-free unit regressions for camera/LaserScan acquisition timestamps."""

import json
import os
import sys
import types
import unittest
from unittest import mock


class _Stamp:
    def __init__(self, sec=0, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class _Header:
    def __init__(self):
        self.stamp = _Stamp()
        self.frame_id = ""


class _CompressedImage:
    def __init__(self):
        self.header = _Header()
        self.format = ""
        self.data = b""


class _Image:
    def __init__(self):
        self.header = _Header()


class _LaserScan:
    def __init__(self):
        self.header = _Header()


class _String:
    def __init__(self):
        self.data = ""


_rclpy = types.ModuleType("rclpy")
_rclpy_node = types.ModuleType("rclpy.node")
_rclpy_node.Node = object
_rclpy.node = _rclpy_node
sys.modules.setdefault("rclpy", _rclpy)
sys.modules.setdefault("rclpy.node", _rclpy_node)

_sensor_msgs = types.ModuleType("sensor_msgs")
_sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
_sensor_msgs_msg.CompressedImage = _CompressedImage
_sensor_msgs_msg.Image = _Image
_sensor_msgs_msg.LaserScan = _LaserScan
_sensor_msgs.msg = _sensor_msgs_msg
sys.modules.setdefault("sensor_msgs", _sensor_msgs)
sys.modules.setdefault("sensor_msgs.msg", _sensor_msgs_msg)

_std_msgs = types.ModuleType("std_msgs")
_std_msgs_msg = types.ModuleType("std_msgs.msg")
_std_msgs_msg.String = _String
_std_msgs.msg = _std_msgs_msg
sys.modules.setdefault("std_msgs", _std_msgs)
sys.modules.setdefault("std_msgs.msg", _std_msgs_msg)

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
for package in ("ida_perception", "ida_planning"):
    path = os.path.join(_ROOT, "src", package)
    if path not in sys.path:
        sys.path.insert(0, path)

from ida_perception import sllidar_bridge_node, yolo_camera_node  # noqa: E402


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class TestYoloAcquisitionStamp(unittest.TestCase):
    def test_b0495_capture_uses_v4l2_yuyv_and_requested_mode(self):
        capture = mock.Mock()
        fake_cv2 = types.SimpleNamespace(
            CAP_V4L2=200,
            CAP_PROP_FOURCC=6,
            CAP_PROP_FRAME_WIDTH=3,
            CAP_PROP_FRAME_HEIGHT=4,
            CAP_PROP_FPS=5,
            CAP_PROP_BUFFERSIZE=38,
            VideoCapture=mock.Mock(return_value=capture),
            VideoWriter_fourcc=mock.Mock(return_value=1448695129),
        )
        result = yolo_camera_node.open_v4l2_yuyv_camera(
            fake_cv2, 0, 960, 600, 80.0
        )
        self.assertIs(result, capture)
        fake_cv2.VideoCapture.assert_called_once_with(0, 200)
        fake_cv2.VideoWriter_fourcc.assert_called_once_with("Y", "U", "Y", "V")
        self.assertEqual(
            capture.set.call_args_list,
            [
                mock.call(6, 1448695129),
                mock.call(3, 960),
                mock.call(4, 600),
                mock.call(5, 80.0),
                mock.call(38, 1),
            ],
        )

    def _node(self):
        node = object.__new__(yolo_camera_node.YoloCameraNode)
        node._infer = lambda _frame: [
            {"class": "orange", "confidence": 0.9, "bbox": (300, 100, 40, 50)}
        ]
        node.class_names = ["orange"]
        node.class_name_map = {}
        node.allowed_colors = frozenset({"orange"})
        node.secondary_allowed_colors = frozenset()
        node.inference_allowed_colors = frozenset({"orange"})
        node.confidence_threshold = 0.45
        node.image_width = 640
        node.image_height = 480
        node.fov_deg = 90.0
        node.focal_length_px = 600.0
        node.target_height_m = 0.5
        node.now_seconds = lambda: 999.0
        return node

    def test_inference_and_payload_use_one_acquisition_stamp(self) -> None:
        node = self._node()
        detections = node._run_inference(object(), acquisition_stamp=12.25)
        self.assertEqual(detections[0]["stamp"], 12.25)
        self.assertEqual(detections[0]["acquisition_stamp"], 12.25)
        self.assertEqual(detections[0]["bbox_size"], 0.104)
        self.assertEqual(detections[0]["bbox_x"], 0.46875)
        self.assertEqual(detections[0]["bbox_y"], 0.208333)
        self.assertEqual(detections[0]["bbox_width"], 0.0625)
        self.assertEqual(detections[0]["bbox_height"], 0.104167)
        self.assertAlmostEqual(detections[0]["bearing_deg"], 0.0)

        node.buoy_pub = _Publisher()
        node._publish_buoys(detections, True, acquisition_stamp=12.25)
        payload = json.loads(node.buoy_pub.messages[-1].data)
        self.assertEqual(payload["stamp"], 12.25)
        self.assertEqual(payload["acquisition_stamp"], 12.25)
        self.assertEqual(payload["detections"][0]["stamp"], 12.25)

    def test_general_model_role_allowlists_are_strict_and_fan_out(self) -> None:
        available = ["black", "green", "orange", "red", "yellow"]
        self.assertEqual(
            yolo_camera_node.parse_allowed_colors("orange,yellow", available),
            frozenset({"orange", "yellow"}),
        )
        for bad in ("blue", "orange,ORANGE", "[]", '["orange", 1]'):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                yolo_camera_node.parse_allowed_colors(bad, available)

        node = self._node()
        node.allowed_colors = frozenset({"orange", "yellow"})
        node.secondary_allowed_colors = frozenset({"red", "green", "black"})
        primary = _Publisher()
        secondary = _Publisher()
        node.buoy_pub = primary
        detections = [
            {"color": "orange", "confidence": 0.9},
            {"color": "yellow", "confidence": 0.8},
            {"color": "green", "confidence": 0.7},
            {"color": "black", "confidence": 0.6},
        ]
        node._publish_buoys(detections, True, 10.0)
        node._publish_buoys(
            detections,
            True,
            10.0,
            publisher=secondary,
            allowed_colors=node.secondary_allowed_colors,
        )
        self.assertEqual(
            [item["color"] for item in json.loads(primary.messages[0].data)["detections"]],
            ["orange", "yellow"],
        )
        self.assertEqual(
            [item["color"] for item in json.loads(secondary.messages[0].data)["detections"]],
            ["green", "black"],
        )

    def test_inference_failure_is_published_stale_not_fresh_empty(self) -> None:
        node = self._node()

        def fail(_frame):
            raise RuntimeError("runtime failed")

        node._infer = fail
        detections = node._run_inference(object(), acquisition_stamp=5.0)
        self.assertEqual(detections, [])
        self.assertEqual(node._inference_status, "inference_error")
        node.buoy_pub = _Publisher()
        node._publish_buoys(detections, True, acquisition_stamp=5.0, stale=True)
        payload = json.loads(node.buoy_pub.messages[-1].data)
        self.assertTrue(payload["stale"])
        self.assertEqual(payload["source"], "inference_error")

    def test_missing_model_is_published_stale(self) -> None:
        node = self._node()
        node._infer = None
        detections = node._run_inference(object(), acquisition_stamp=6.0)
        node.buoy_pub = _Publisher()
        node._publish_buoys(detections, True, acquisition_stamp=6.0, stale=True)
        payload = json.loads(node.buoy_pub.messages[-1].data)
        self.assertEqual(payload["detections"], [])
        self.assertTrue(payload["stale"])
        self.assertEqual(payload["source"], "model_unavailable")

    def test_yolo_runtime_config_boundaries(self) -> None:
        yolo_camera_node.validate_yolo_config(640, "", 0.0, 1.0, 179.9)
        yolo_camera_node.validate_yolo_config(
            640, "cpu", 0.5, 0.5, 90.0,
            10.0, 600.0, 0.5, 640, 480, "engine", "/tmp/model.engine"
        )
        for args in (
            (0, "", 0.5, 0.5, 90.0),
            (640, " cpu ", 0.5, 0.5, 90.0),
            (640, "", -0.1, 0.5, 90.0),
            (640, "", 0.5, 1.1, 90.0),
            (640, "", 0.5, 0.5, 0.0),
            (640, "", 0.5, 0.5, 180.0),
        ):
            with self.subTest(args=args), self.assertRaises(ValueError):
                yolo_camera_node.validate_yolo_config(*args)
        invalid_extended = (
            {"publish_hz": 0.0},
            {"focal_length_px": float("inf")},
            {"target_height_m": -0.1},
            {"image_width": 0},
            {"image_width": 640.5},
            {"image_height": True},
            {"camera_fps": 0.0},
            {"camera_fps": float("nan")},
            {"camera_fps": True},
            {"model_type": "tensorrt"},
            {"model_type": "pt", "model_path": "/tmp/model.engine"},
        )
        for kwargs in invalid_extended:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                yolo_camera_node.validate_yolo_config(
                    640, "", 0.5, 0.5, 90.0, **kwargs
                )

    def test_annotation_uses_real_normalized_xyxy(self) -> None:
        node = self._node()
        frame = types.SimpleNamespace(shape=(480, 640, 3))
        rectangles = []
        fake_cv2 = types.SimpleNamespace(
            FONT_HERSHEY_SIMPLEX=0,
            rectangle=lambda _frame, p1, p2, *_args: rectangles.append((p1, p2)),
            putText=lambda *_args: None,
        )
        detections = node._run_inference(frame, acquisition_stamp=12.25)
        with mock.patch.object(yolo_camera_node, "cv2", fake_cv2):
            node._annotate(frame, detections)
        self.assertEqual(rectangles, [((300, 100), (340, 150))])

    def test_topic_frame_carries_original_header_and_is_consumed_once(self) -> None:
        node = self._node()
        frame = object()
        node._cap = None
        node._topic_frame = frame
        node._topic_acquisition_stamp = 42.125
        node._topic_header_fields = (42, 125_000_000, "camera_front")

        result = node._read_frame()

        self.assertEqual(result, (frame, 42.125, (42, 125_000_000, "camera_front")))
        self.assertEqual(node._read_frame(), (None, None, None))

    def test_raw_topic_uses_cv_bridge_and_preserves_header(self) -> None:
        node = self._node()
        frame = object()
        node._cv_bridge = types.SimpleNamespace(
            imgmsg_to_cv2=lambda _msg, desired_encoding: frame
        )
        node._cap = None
        node._topic_frame = None
        node._topic_acquisition_stamp = None
        node._topic_header_fields = None
        message = _Image()
        message.header.stamp.sec = 7
        message.header.stamp.nanosec = 250_000_000
        message.header.frame_id = "camera_optical"

        with mock.patch.object(yolo_camera_node, "CV2_AVAILABLE", True):
            node.on_frame_raw_topic(message)

        self.assertEqual(
            node._read_frame(),
            (frame, 7.25, (7, 250_000_000, "camera_optical")),
        )

    def test_zero_topic_stamp_uses_receive_clock(self) -> None:
        node = self._node()
        node._topic_frame = object()
        node._topic_acquisition_stamp = None
        node._topic_header_fields = None
        message = _Image()
        message.header.frame_id = "camera_optical"

        node._store_topic_header(message)

        self.assertEqual(node._topic_acquisition_stamp, 999.0)
        self.assertEqual(node._topic_header_fields, (999, 0, "camera_optical"))

    def test_processed_image_preserves_topic_header(self) -> None:
        node = self._node()
        node.image_pub = _Publisher()
        node._write_video_frame = lambda _frame: None
        jpeg = types.SimpleNamespace(tobytes=lambda: b"jpeg")
        fake_cv2 = types.SimpleNamespace(
            IMWRITE_JPEG_QUALITY=1,
            imencode=lambda *_args, **_kwargs: (True, jpeg),
        )

        with mock.patch.object(yolo_camera_node, "cv2", fake_cv2):
            node._publish_processed_image(
                object(), 42.125, (42, 125_000_000, "camera_front")
            )

        message = node.image_pub.messages[-1]
        self.assertEqual(message.header.stamp.sec, 42)
        self.assertEqual(message.header.stamp.nanosec, 125_000_000)
        self.assertEqual(message.header.frame_id, "camera_front")


class TestSllidarAcquisitionStamp(unittest.TestCase):
    def test_header_stamp_and_zero_fallback(self) -> None:
        scan = _LaserScan()
        scan.header.stamp.sec = 8
        scan.header.stamp.nanosec = 250_000_000
        self.assertEqual(
            sllidar_bridge_node.SllidarBridgeNode._message_stamp_seconds(scan, 99.0),
            8.25,
        )
        scan.header.stamp.sec = 0
        scan.header.stamp.nanosec = 0
        self.assertEqual(
            sllidar_bridge_node.SllidarBridgeNode._message_stamp_seconds(scan, 99.0),
            99.0,
        )

    def test_stale_raw_publish_keeps_old_acquisition_stamp(self) -> None:
        node = object.__new__(sllidar_bridge_node.SllidarBridgeNode)
        node.raw_contract = True
        node.now_seconds = lambda: 100.0
        node.obstacle_pub = _Publisher()
        obstacles = [
            {"distance": 5.0, "forward_m": 4.9, "lateral_m": 1.0, "width_m": 0.2}
        ]

        node._publish(
            obstacles,
            stale=True,
            acquisition_stamp=12.5,
            raw_points=[{"forward_m": 4.5, "lateral_left_m": -0.75}],
        )

        payload = json.loads(node.obstacle_pub.messages[-1].data)
        self.assertEqual(payload["stamp"], 12.5)
        self.assertEqual(payload["acquisition_stamp"], 12.5)
        self.assertEqual(payload["published_stamp"], 100.0)
        self.assertTrue(payload["stale"])
        self.assertEqual(payload["clusters"][0]["lateral_left_m"], -1.0)
        self.assertEqual(payload["points"][0]["lateral_left_m"], -0.75)


if __name__ == "__main__":
    unittest.main()
