#!/usr/bin/env python3
"""Arducam B0495 + IDA YOLO modelleri icin pasif, yerel ekran testi.

Bu arac ROS, MAVLink veya aktuatör topiclerine baglanmaz. Kamera karesini solda,
aynı karenin aktif model sonucunu sagda gosterir.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable


ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_SOURCE = ROOT / "src" / "ida_perception"
if str(PERCEPTION_SOURCE) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SOURCE))

from ida_perception.model_loader import ModelInferenceError, load_model  # noqa: E402


ROLE_NAMES = {
    "p12": ["orange", "yellow"],
    "p3": ["red", "green", "black"],
}
ROLE_LABELS = {
    "p12": "P1/P2: orange + yellow",
    "p3": "P3: red + green + black",
}
COLORS_BGR = {
    "orange": (0, 145, 255),
    "yellow": (0, 255, 255),
    "red": (0, 0, 255),
    "green": (0, 220, 0),
    "black": (30, 30, 30),
}


@dataclass
class LoadedRole:
    infer: Callable[[Any], list[dict[str, Any]]]
    path: Path


def parse_v4l2_controls(text: str) -> dict[str, int]:
    values: dict[str, int] = {}
    for raw_line in text.splitlines():
        if ":" not in raw_line:
            continue
        name, raw_value = raw_line.split(":", 1)
        token = raw_value.strip().split()[0] if raw_value.strip() else ""
        try:
            values[name.strip()] = int(token)
        except ValueError:
            continue
    return values


def controls_from_positions(positions: dict[str, int]) -> dict[str, int]:
    """UI slider degerlerini dogrudan V4L2 kontrol degerlerine cevir."""
    return {
        "auto_exposure": 0 if positions["auto_exposure"] else 1,
        "exposure_time_absolute": max(5, min(660, positions["exposure"])),
        "gain": max(168, min(1600, positions["gain"])),
        "white_balance_automatic": 1 if positions["auto_wb"] else 0,
        "white_balance_temperature": max(2300, min(6500, positions["wb_kelvin"])),
        "saturation": max(0, min(15, positions["saturation"])),
        "brightness": max(-64, min(64, positions["brightness"] - 64)),
        "contrast": max(0, min(20, positions["contrast"])),
    }


class LiveCameraControls:
    """B0495 V4L2 kontrollerini sabit allowlist ile canli uygular."""

    WINDOW = "IDA Camera Controls"
    CONTROL_NAMES = (
        "brightness",
        "contrast",
        "saturation",
        "white_balance_automatic",
        "white_balance_temperature",
        "gain",
        "auto_exposure",
        "exposure_time_absolute",
    )

    def __init__(self, cv2: Any, camera_index: int) -> None:
        self.cv2 = cv2
        self.device = f"/dev/video{camera_index}"
        self.error = ""
        current = self._read_current()
        self.cv2.namedWindow(self.WINDOW, self.cv2.WINDOW_NORMAL)
        self.cv2.resizeWindow(self.WINDOW, 620, 390)
        sliders = {
            "Auto exposure 1=ON": (1 if current.get("auto_exposure", 0) == 0 else 0, 1),
            "Exposure x0.1ms": (current.get("exposure_time_absolute", 5), 660),
            "Gain (min 168)": (current.get("gain", 168), 1600),
            "Auto WB 1=ON": (current.get("white_balance_automatic", 1), 1),
            "WB Kelvin (2300-6500)": (current.get("white_balance_temperature", 4500), 6500),
            "Saturation (0-15)": (current.get("saturation", 10), 15),
            "Brightness (-64..64)": (current.get("brightness", 0) + 64, 128),
            "Contrast (0-20)": (current.get("contrast", 10), 20),
        }
        for label, (value, maximum) in sliders.items():
            self.cv2.createTrackbar(label, self.WINDOW, int(value), maximum, lambda _value: None)
        self.last_applied = self.values()

    def _read_current(self) -> dict[str, int]:
        try:
            completed = subprocess.run(
                [
                    "v4l2-ctl",
                    "-d",
                    self.device,
                    "--get-ctrl=" + ",".join(self.CONTROL_NAMES),
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.0,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.error = f"control read failed: {exc}"
            return {}
        if completed.returncode != 0:
            self.error = completed.stderr.strip() or "control read failed"
            return {}
        return parse_v4l2_controls(completed.stdout)

    def _positions(self) -> dict[str, int]:
        return {
            "auto_exposure": self.cv2.getTrackbarPos("Auto exposure 1=ON", self.WINDOW),
            "exposure": self.cv2.getTrackbarPos("Exposure x0.1ms", self.WINDOW),
            "gain": self.cv2.getTrackbarPos("Gain (min 168)", self.WINDOW),
            "auto_wb": self.cv2.getTrackbarPos("Auto WB 1=ON", self.WINDOW),
            "wb_kelvin": self.cv2.getTrackbarPos("WB Kelvin (2300-6500)", self.WINDOW),
            "saturation": self.cv2.getTrackbarPos("Saturation (0-15)", self.WINDOW),
            "brightness": self.cv2.getTrackbarPos("Brightness (-64..64)", self.WINDOW),
            "contrast": self.cv2.getTrackbarPos("Contrast (0-20)", self.WINDOW),
        }

    def values(self) -> dict[str, int]:
        return controls_from_positions(self._positions())

    def _set_values(self, values: dict[str, int]) -> bool:
        commands = [
            {
                "auto_exposure": values["auto_exposure"],
                "white_balance_automatic": values["white_balance_automatic"],
            },
            {
                "gain": values["gain"],
                "saturation": values["saturation"],
                "brightness": values["brightness"],
                "contrast": values["contrast"],
                **(
                    {"exposure_time_absolute": values["exposure_time_absolute"]}
                    if values["auto_exposure"] == 1
                    else {}
                ),
                **(
                    {"white_balance_temperature": values["white_balance_temperature"]}
                    if values["white_balance_automatic"] == 0
                    else {}
                ),
            },
        ]
        for command in commands:
            assignment = ",".join(f"{name}={value}" for name, value in command.items())
            try:
                completed = subprocess.run(
                    ["v4l2-ctl", "-d", self.device, "--set-ctrl=" + assignment],
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=1.0,
                )
            except (OSError, subprocess.SubprocessError) as exc:
                self.error = f"control write failed: {exc}"
                return False
            if completed.returncode != 0:
                self.error = completed.stderr.strip() or "control write failed"
                return False
        self.error = ""
        return True

    def poll(self) -> dict[str, int]:
        current = self.values()
        if current != self.last_applied and self._set_values(current):
            self.last_applied = current
        return current

    def reset_auto(self) -> None:
        defaults = {
            "Auto exposure 1=ON": 1,
            "Exposure x0.1ms": 5,
            "Gain (min 168)": 168,
            "Auto WB 1=ON": 1,
            "WB Kelvin (2300-6500)": 4500,
            "Saturation (0-15)": 10,
            "Brightness (-64..64)": 64,
            "Contrast (0-20)": 10,
        }
        for label, value in defaults.items():
            self.cv2.setTrackbarPos(label, self.WINDOW, value)
        current = self.values()
        if self._set_values(current):
            self.last_applied = current

    @staticmethod
    def summary(values: dict[str, int]) -> str:
        exposure = "AUTO" if values["auto_exposure"] == 0 else f"{values['exposure_time_absolute'] / 10.0:.1f}ms"
        wb = "AUTO" if values["white_balance_automatic"] else f"{values['white_balance_temperature']}K"
        return (
            f"exp={exposure} gain={values['gain']} wb={wb} "
            f"sat={values['saturation']} bright={values['brightness']} contrast={values['contrast']}"
        )


def nominal_bearing_deg(center_x: float, width: int, horizontal_fov_deg: float) -> float:
    """Piksel merkezini pinhole varsayimiyla sag-pozitif aciya cevir."""
    if width <= 0 or not 0.0 < horizontal_fov_deg < 180.0:
        raise ValueError("invalid camera geometry")
    focal_px = (width / 2.0) / math.tan(math.radians(horizontal_fov_deg) / 2.0)
    return math.degrees(math.atan((center_x - width / 2.0) / focal_px))


def frame_metrics(frame: Any, cv2: Any, np: Any) -> dict[str, float]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return {
        "luma": float(gray.mean()),
        "black_pct": float(np.mean(gray <= 5) * 100.0),
        "white_pct": float(np.mean(gray >= 250) * 100.0),
        "saturation": float(hsv[:, :, 1].mean()),
    }


def _put_text(cv2: Any, image: Any, text: str, x: int, y: int, color=(255, 255, 255)) -> None:
    cv2.putText(image, text, (x + 1, y + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)


def draw_detections(frame: Any, detections: list[dict[str, Any]], cv2: Any, fov_deg: float) -> Any:
    result = frame.copy()
    height, width = result.shape[:2]
    for detection in detections:
        name = str(detection.get("class", "unknown")).lower()
        score = float(detection.get("confidence", 0.0))
        bbox = detection.get("bbox", ())
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            continue
        x, y, box_w, box_h = [float(value) for value in bbox]
        x1 = max(0, min(width - 1, int(round(x))))
        y1 = max(0, min(height - 1, int(round(y))))
        x2 = max(0, min(width - 1, int(round(x + box_w))))
        y2 = max(0, min(height - 1, int(round(y + box_h))))
        if x2 <= x1 or y2 <= y1:
            continue
        color = COLORS_BGR.get(name, (255, 255, 255))
        # Siyah sinifin kutusu gorunur kalsin diye dis beyaz cerceve ekle.
        if name == "black":
            cv2.rectangle(result, (x1 - 1, y1 - 1), (x2 + 1, y2 + 1), (255, 255, 255), 4)
        cv2.rectangle(result, (x1, y1), (x2, y2), color, 3)
        bearing = nominal_bearing_deg((x1 + x2) / 2.0, width, fov_deg)
        label = f"{name} {score:.2f}  bearing={bearing:+.1f} deg"
        _put_text(cv2, result, label, x1, max(22, y1 - 7), color)
    cv2.line(result, (width // 2, 0), (width // 2, height), (255, 255, 255), 1)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=600)
    parser.add_argument("--camera-fps", type=float, default=80.0)
    parser.add_argument("--role", choices=("p12", "p3"), default="p12")
    parser.add_argument(
        "--model-p12",
        default="/home/ezelproject/ezel-yazilim-main/teknofest_goruntu_isleme-main/models/parkur12_best.pt",
    )
    parser.add_argument("--model-p3", default="/home/ezelproject/parkur3_best.pt")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.45)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--device", default="0")
    parser.add_argument("--fov-deg", type=float, default=82.0)
    parser.add_argument("--fullscreen", action="store_true")
    parser.add_argument(
        "--capture-dir",
        default="/home/ezelproject/ida_alt_ws/camera_captures",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if (
        args.camera < 0
        or args.width <= 0
        or args.height <= 0
        or not math.isfinite(args.camera_fps)
        or args.camera_fps <= 0.0
        or args.imgsz <= 0
        or not math.isfinite(args.conf)
        or not 0.0 <= args.conf <= 1.0
        or not math.isfinite(args.iou)
        or not 0.0 <= args.iou <= 1.0
        or not math.isfinite(args.fov_deg)
        or not 0.0 < args.fov_deg < 180.0
    ):
        raise ValueError("invalid camera/model viewer arguments")


def main() -> int:
    args = parse_args()
    validate_args(args)
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except ImportError as exc:
        print(f"ERROR: OpenCV/numpy unavailable: {exc}", file=sys.stderr)
        return 2

    model_paths = {"p12": Path(args.model_p12), "p3": Path(args.model_p3)}
    loaded: dict[str, LoadedRole] = {}

    def activate(role: str) -> LoadedRole:
        if role in loaded:
            return loaded[role]
        path = model_paths[role]
        print(f"Loading {ROLE_LABELS[role]} from {path} ...", flush=True)
        result = load_model(
            str(path),
            ROLE_NAMES[role],
            args.conf,
            args.iou,
            args.imgsz,
            args.device,
        )
        if result is None:
            raise RuntimeError(f"model rejected or unavailable: {path}")
        _model, _preprocess, infer = result
        loaded[role] = LoadedRole(infer=infer, path=path)
        return loaded[role]

    role = args.role
    try:
        activate(role)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 3

    # OpenCV'nin Jetson GStreamer otomatik backend'i discrete UVC mod istegini
    # sessizce yok sayabiliyor. B0495 icin dogrudan V4L2 + YUYV kullan.
    capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.camera_fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        print(f"ERROR: camera {args.camera} could not be opened (busy or unsupported mode)", file=sys.stderr)
        return 4

    actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = float(capture.get(cv2.CAP_PROP_FPS))
    if (actual_width, actual_height) != (args.width, args.height):
        capture.release()
        print(
            f"ERROR: camera negotiated {actual_width}x{actual_height}, "
            f"requested {args.width}x{args.height}",
            file=sys.stderr,
        )
        return 4
    print(
        f"Camera open: {actual_width}x{actual_height} @ {actual_fps:.2f} FPS; "
        "keys: 1=P1/P2, 3=P3, S=save, R=auto reset, Space=pause, Q/Esc=quit",
        flush=True,
    )

    window = "IDA Arducam + YOLO viewer (RAW left / MODEL right)"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    if args.fullscreen:
        cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    controls = LiveCameraControls(cv2, args.camera)

    capture_dir = Path(args.capture_dir).expanduser().resolve()
    capture_dir.mkdir(parents=True, exist_ok=True)
    paused = False
    last_display = None
    last_metrics: dict[str, float] = {}
    last_controls = controls.values()
    frame_times: list[float] = []
    failures = 0

    try:
        while True:
            if not paused:
                last_controls = controls.poll()
                ok, frame = capture.read()
                if not ok or frame is None:
                    failures += 1
                    if failures >= 10:
                        raise RuntimeError("camera returned 10 consecutive empty frames")
                    continue
                failures = 0
                started = time.monotonic()
                try:
                    detections = activate(role).infer(frame)
                    inference_error = ""
                except ModelInferenceError as exc:
                    detections = []
                    inference_error = str(exc)
                inference_ms = (time.monotonic() - started) * 1000.0
                now = time.monotonic()
                frame_times.append(now)
                frame_times = [stamp for stamp in frame_times if now - stamp <= 2.0]
                display_fps = max(0.0, (len(frame_times) - 1) / max(0.001, frame_times[-1] - frame_times[0])) if len(frame_times) > 1 else 0.0

                raw = frame.copy()
                annotated = draw_detections(frame, detections, cv2, args.fov_deg)
                metrics = frame_metrics(frame, cv2, np)
                last_metrics = metrics
                _put_text(cv2, raw, "RAW CAMERA (model input is COLOR)", 12, 26, (255, 255, 255))
                _put_text(cv2, annotated, f"MODEL: {ROLE_LABELS[role]}", 12, 26, (255, 255, 255))
                _put_text(
                    cv2,
                    annotated,
                    f"detections={len(detections)} infer={inference_ms:.1f}ms view={display_fps:.1f}fps",
                    12,
                    50,
                    (255, 255, 255),
                )
                metrics_line = (
                    f"luma={metrics['luma']:.1f} sat={metrics['saturation']:.1f} "
                    f"black={metrics['black_pct']:.1f}% white_clip={metrics['white_pct']:.1f}%"
                )
                _put_text(cv2, raw, metrics_line, 12, raw.shape[0] - 16, (255, 255, 255))
                _put_text(
                    cv2,
                    annotated,
                    controls.summary(last_controls),
                    12,
                    annotated.shape[0] - 16,
                    (255, 255, 255),
                )
                if controls.error:
                    _put_text(cv2, annotated, controls.error, 12, 76, (0, 0, 255))
                if inference_error:
                    _put_text(cv2, annotated, f"INFERENCE ERROR: {inference_error}", 12, 76, (0, 0, 255))
                last_display = np.hstack((raw, annotated))

            if last_display is not None:
                cv2.imshow(window, last_display)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key == ord("1"):
                role = "p12"
                activate(role)
            elif key == ord("3"):
                role = "p3"
                activate(role)
            elif key == 32:
                paused = not paused
            elif key in (ord("r"), ord("R")):
                controls.reset_auto()
            elif key in (ord("s"), ord("S")) and last_display is not None:
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                target = capture_dir / f"camera_{role}_{stamp}.jpg"
                if not cv2.imwrite(str(target), last_display):
                    print(f"WARNING: snapshot could not be saved: {target}", file=sys.stderr)
                else:
                    print(f"Saved: {target}", flush=True)
                    target.with_suffix(".json").write_text(
                        json.dumps(
                            {
                                "role": role,
                                "model": str(model_paths[role]),
                                "camera": {
                                    "width": actual_width,
                                    "height": actual_height,
                                    "fps": actual_fps,
                                    "fov_deg": args.fov_deg,
                                },
                                "controls": last_controls,
                                "metrics": last_metrics,
                            },
                            indent=2,
                            sort_keys=True,
                        ),
                        encoding="utf-8",
                    )
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 5
    finally:
        capture.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
