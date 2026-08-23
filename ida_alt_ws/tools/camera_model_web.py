#!/usr/bin/env python3
"""Headless Arducam + five-colour YOLO viewer for a trusted local bench LAN.

This tool owns only /dev/videoN and V4L2 image controls.  It does not import ROS,
MAVLink or any actuator package.  Open http://JETSON_IP:8090 from another device
on the same network.  The field stack must be stopped because a UVC camera has a
single owner.
"""

from __future__ import annotations

import argparse
import hmac
import json
import math
import os
import subprocess
import secrets
import sys
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "ida_perception"))
sys.path.insert(0, str(ROOT / "src" / "ida_bringup"))

from ida_perception.model_loader import ModelInferenceError, load_model  # noqa: E402
from ida_bringup.camera_profile import (  # noqa: E402
    CONTROL_LIMITS,
    load_camera_control_profile,
    save_versioned_camera_control_profile,
    validate_camera_controls,
)
from camera_model_viewer import (  # noqa: E402
    COLORS_BGR,
    GENERAL_NAMES,
    controls_from_positions,
    draw_detections,
    frame_metrics,
    parse_v4l2_controls,
)


# These are image-processing controls, not B0495/V4L2 hardware controls. They
# are neutral by default so merely opening the bench tool cannot silently alter
# what the model sees.
FILTER_LIMITS = {
    "filter_before_detect": (0, 1),
    "preview_filtered": (0, 1),
    "clahe_enabled": (0, 1),
    "clahe_clip_x10": (10, 50),
    "clahe_grid": (2, 16),
    "gamma_x100": (50, 200),
    "grayscale_mix": (0, 100),
    "sharpen_x100": (0, 200),
}

FILTER_DEFAULTS = {
    "filter_before_detect": 1,
    "preview_filtered": 1,
    "clahe_enabled": 0,
    "clahe_clip_x10": 20,
    "clahe_grid": 8,
    "gamma_x100": 100,
    "grayscale_mix": 0,
    "sharpen_x100": 0,
}


def parse_model_class_contract(
    raw_names: str, raw_mapping: str
) -> tuple[list[str], dict[str, str]]:
    """Validate an exact model-native manifest and one-to-one color mapping."""
    if not isinstance(raw_names, str) or not isinstance(raw_mapping, str):
        raise ValueError("model class contract must be text")
    names = [item.strip().lower() for item in raw_names.split(",")]
    if not names or any(not item for item in names) or len(set(names)) != len(names):
        raise ValueError("model class names are invalid")
    try:
        mapping = json.loads(raw_mapping)
    except json.JSONDecodeError as exc:
        raise ValueError("model class mapping is invalid JSON") from exc
    if not isinstance(mapping, dict) or set(mapping) != set(names):
        raise ValueError("model class mapping must cover the exact native manifest")
    normalized: dict[str, str] = {}
    for native, target in mapping.items():
        if not isinstance(native, str) or not isinstance(target, str):
            raise ValueError("model class mapping values must be text")
        native_key, target_color = native.strip().lower(), target.strip().lower()
        if native_key not in names or target_color not in GENERAL_NAMES:
            raise ValueError("model class mapping contains an unsupported color")
        normalized[native_key] = target_color
    if len(set(normalized.values())) != len(normalized):
        raise ValueError("model class mapping must be one-to-one")
    return names, normalized


def _preset(
    label: str, exposure: int, gain: int, wb: int, saturation: int,
    contrast: int, clahe_clip_x10: int, clahe_grid: int, gamma_x100: int,
    sharpen_x100: int, brightness: int = 64, note: str = "",
) -> dict[str, Any]:
    return {
        "label": label,
        "controls": {
            "auto_exposure": 0, "exposure": exposure, "gain": gain,
            "auto_wb": 0, "wb_kelvin": wb, "saturation": saturation,
            "brightness": brightness, "contrast": contrast,
            "power_line_frequency": 1,
        },
        "filters": {
            "filter_before_detect": 1, "preview_filtered": 1,
            "clahe_enabled": 1, "clahe_clip_x10": clahe_clip_x10,
            "clahe_grid": clahe_grid, "gamma_x100": gamma_x100,
            "grayscale_mix": 0, "sharpen_x100": sharpen_x100,
        },
        "note": note,
    }


# Adapted from 10_Canli_Goruntu_Isleme_Presetleri.md. Exposure is in the
# B0495's 0.1 ms V4L2 units. Gain uses the only gain control exposed by this
# camera (168 is treated as the native 1x reference); there is no separate
# digital-gain control. Values are intentionally central, bounded choices from
# the document's ranges, not claims of radiometric calibration.
CAMERA_PRESETS: dict[str, dict[str, Any]] = {
    "model_base": {
        "label": "Model Baz / Nötr (önerilen başlangıç)",
        "controls": {
            "auto_exposure": 1, "exposure": 5, "gain": 168,
            "auto_wb": 1, "wb_kelvin": 4500, "saturation": 10,
            "brightness": 64, "contrast": 10, "power_line_frequency": 1,
        },
        "filters": dict(FILTER_DEFAULTS),
        "note": "Eğitim dağılımını en az değiştiren başlangıç; önce bununla ölçün.",
    },
    "indoor_auto": {
        "label": "Kapalı Alan / Yapay Işık (önerilen)",
        "controls": {
            "auto_exposure": 1, "exposure": 5, "gain": 168,
            "auto_wb": 1, "wb_kelvin": 4500, "saturation": 10,
            "brightness": 64, "contrast": 10, "power_line_frequency": 1,
        },
        "filters": dict(FILTER_DEFAULTS),
        "note": (
            "Default görüntüyü korur: auto exposure/WB, 50 Hz yapay ışık "
            "bastırma ve modele nötr yazılım filtreleri."
        ),
    },
    "p1": _preset("1 · Öğle güneşi", 5, 168, 5800, 12, 11, 18, 8, 95, 175,
                  note="Belgedeki 0,2–0,4 ms kamera minimumu nedeniyle 0,5 ms'e yükseltildi."),
    "p2": _preset("2 · Erken öğleden sonra", 5, 190, 6000, 12, 12, 20, 9, 92, 150),
    "p3": _preset("3 · İkindi", 8, 231, 6250, 12, 12, 23, 8, 88, 150),
    "p4": _preset("4 · Altın saat", 15, 294, 4750, 13, 12, 25, 9, 82, 140, 65),
    "p5": _preset("5 · Hafif bulutlu / ara güneş", 12, 252, 6250, 12, 12, 28, 10, 85, 150),
    "p6": _preset("6 · Kapalı bulutlu", 30, 420, 6500, 12, 14, 33, 14, 78, 150,
                  note="Belgedeki 7000 K kamera üst sınırı nedeniyle 6500 K ile sınırlandı."),
    "p7": _preset("7 · Gün batımı öncesi", 45, 504, 5250, 13, 13, 30, 12, 70, 110, 66),
    "p8": _preset("8 · Gün batımı", 65, 546, 4500, 13, 13, 33, 12, 68, 100, 66),
    "p9": _preset("9 · Sivil alacakaranlık başlangıcı", 100, 630, 4750, 14, 14, 38, 15, 60, 85, 67,
                  note="Bu preset için 1920×1200@50 önerilir; 960×600@80'de pozlama payı dardır."),
    "p10": _preset("10 · Alacakaranlık", 130, 672, 5000, 14, 14, 40, 16, 58, 60, 67,
                   note="960×600@80 ile kullanılmamalı; 1920×1200@50 ve saha doğrulaması gerekir."),
}


def validate_control_patch(payload: Any) -> dict[str, int]:
    return validate_camera_controls(payload, complete=False)


def validate_filter_patch(payload: Any) -> dict[str, int]:
    if not isinstance(payload, dict) or not payload or len(payload) > len(FILTER_LIMITS):
        raise ValueError("filters must be a non-empty object")
    if set(payload).difference(FILTER_LIMITS):
        raise ValueError("unknown software filter")
    result: dict[str, int] = {}
    for name, raw in payload.items():
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"{name} must be numeric")
        value = float(raw)
        low, high = FILTER_LIMITS[name]
        if not math.isfinite(value) or int(value) != value or not low <= int(value) <= high:
            raise ValueError(f"{name} outside [{low}, {high}]")
        result[name] = int(value)
    return result


def validate_jpeg_quality(raw: Any) -> int:
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise ValueError("jpeg_quality must be numeric")
    value = float(raw)
    if not math.isfinite(value) or int(value) != value or not 60 <= int(value) <= 95:
        raise ValueError("jpeg_quality outside [60, 95]")
    return int(value)


class V4L2Controls:
    NAMES = (
        "brightness", "contrast", "saturation", "white_balance_automatic",
        "white_balance_temperature", "gain", "auto_exposure",
        "exposure_time_absolute", "power_line_frequency",
    )

    def __init__(self, device: str) -> None:
        self.device = device
        self.lock = threading.Lock()
        self.error = ""
        current = self._read()
        self.positions = {
            "auto_exposure": 1 if current.get("auto_exposure", 0) == 0 else 0,
            "exposure": current.get("exposure_time_absolute", 5),
            "gain": current.get("gain", 168),
            "auto_wb": current.get("white_balance_automatic", 1),
            "wb_kelvin": current.get("white_balance_temperature", 4500),
            "saturation": current.get("saturation", 10),
            "brightness": current.get("brightness", 0) + 64,
            "contrast": current.get("contrast", 10),
            "power_line_frequency": current.get("power_line_frequency", 1),
        }
        self.positions = {
            name: max(bounds[0], min(bounds[1], int(self.positions[name])))
            for name, bounds in CONTROL_LIMITS.items()
        }

    def _run(self, argument: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["v4l2-ctl", "-d", self.device, argument], check=False,
            capture_output=True, text=True, timeout=1.0,
        )

    def _read(self) -> dict[str, int]:
        try:
            completed = self._run("--get-ctrl=" + ",".join(self.NAMES))
        except (OSError, subprocess.SubprocessError) as exc:
            self.error = f"control read failed: {exc}"
            return {}
        if completed.returncode:
            self.error = completed.stderr.strip() or "control read failed"
            return {}
        return parse_v4l2_controls(completed.stdout)

    def snapshot(self) -> dict[str, int]:
        with self.lock:
            return dict(self.positions)

    def reset(self) -> dict[str, int]:
        return self.update({
            "auto_exposure": 1, "exposure": 5, "gain": 168, "auto_wb": 1,
            "wb_kelvin": 4500, "saturation": 10, "brightness": 64,
            "contrast": 10,
            "power_line_frequency": 1,
        })

    def update(self, patch: Any) -> dict[str, int]:
        validated = validate_control_patch(patch)
        with self.lock:
            proposed = {**self.positions, **validated}
            values = controls_from_positions(proposed)
            commands = [
                {
                    "auto_exposure": values["auto_exposure"],
                    "white_balance_automatic": values["white_balance_automatic"],
                },
                {
                    "gain": values["gain"], "saturation": values["saturation"],
                    "brightness": values["brightness"], "contrast": values["contrast"],
                    "power_line_frequency": proposed["power_line_frequency"],
                    **({"exposure_time_absolute": values["exposure_time_absolute"]}
                       if values["auto_exposure"] == 1 else {}),
                    **({"white_balance_temperature": values["white_balance_temperature"]}
                       if values["white_balance_automatic"] == 0 else {}),
                },
            ]
            for command in commands:
                assignment = ",".join(f"{name}={value}" for name, value in command.items())
                try:
                    completed = self._run("--set-ctrl=" + assignment)
                except (OSError, subprocess.SubprocessError) as exc:
                    self.error = f"control write failed: {exc}"
                    raise ValueError(self.error) from exc
                if completed.returncode:
                    self.error = completed.stderr.strip() or "control write failed"
                    raise ValueError(self.error)
            self.positions = proposed
            self.error = ""
            return dict(self.positions)


class CameraRuntime:
    def __init__(self, args: argparse.Namespace) -> None:
        import cv2  # type: ignore
        import numpy as np  # type: ignore

        self.cv2, self.np, self.args = cv2, np, args
        self.model_class_names, self.model_class_map = parse_model_class_contract(
            args.class_names, args.class_map
        )
        loaded = load_model(
            args.model, self.model_class_names, 0.03, args.iou, args.imgsz, args.device
        )
        if loaded is None:
            raise RuntimeError(f"model unavailable or rejected: {args.model}")
        _model, _preprocess, self.infer = loaded
        self.controls = V4L2Controls(f"/dev/video{args.camera}")
        self.camera_profile_path = (
            Path(args.camera_profile).expanduser() if args.camera_profile else None
        )
        self.camera_profile_state: dict[str, Any] = {
            "configured": self.camera_profile_path is not None,
            "loaded": False,
            "saved_at_utc": "",
            "last_snapshot": "",
            "path": str(self.camera_profile_path) if self.camera_profile_path else "",
        }
        self.capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        self.capture.set(cv2.CAP_PROP_FPS, args.camera_fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.capture.isOpened():
            raise RuntimeError("camera could not be opened (busy or unsupported mode)")
        actual = (
            int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )
        if actual != (args.width, args.height):
            self.capture.release()
            raise RuntimeError(f"camera negotiated {actual[0]}x{actual[1]}")
        # Format negotiation may reset UVC controls on some cameras. Reapply
        # the authoritative field profile only after the stream is open.
        if self.camera_profile_path is not None:
            persisted = load_camera_control_profile(self.camera_profile_path)
            if persisted is not None:
                self.controls.update(persisted["controls"])
                self.camera_profile_state.update(
                    loaded=True, saved_at_utc=persisted["saved_at_utc"]
                )
        self.frame_condition = threading.Condition()
        self.latest_frame = None
        self.capture_sequence = 0
        self.capture_times: deque[float] = deque(maxlen=240)
        self.output_times: deque[float] = deque(maxlen=120)
        self.dropped_frames = 0
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None
        self.sequence = 0
        self.confidence = args.conf
        self.jpeg_lock = threading.Lock()
        self.jpeg_quality = args.jpeg_quality
        self.filter_lock = threading.Lock()
        self.filters = dict(FILTER_DEFAULTS)
        self.status: dict[str, Any] = {
            "healthy": False, "reason": "starting", "detections": [], "metrics": {},
            "model": args.model, "width": args.width, "height": args.height,
            "camera_profile": dict(self.camera_profile_state),
        }
        self.stop_event = threading.Event()
        self.capture_thread = threading.Thread(
            target=self._capture_loop, name="camera-capture", daemon=True
        )
        self.thread = threading.Thread(target=self._loop, name="camera-process", daemon=True)
        self.capture_thread.start()
        self.thread.start()

    def set_confidence(self, raw: Any) -> float:
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError("confidence must be numeric")
        value = float(raw)
        if not math.isfinite(value) or not 0.03 <= value <= 0.95:
            raise ValueError("confidence outside [0.03, 0.95]")
        self.confidence = value
        return value

    def set_jpeg_quality(self, raw: Any) -> int:
        value = validate_jpeg_quality(raw)
        with self.jpeg_lock:
            self.jpeg_quality = value
            return self.jpeg_quality

    def set_filters(self, raw: Any) -> dict[str, int]:
        patch = validate_filter_patch(raw)
        with self.filter_lock:
            self.filters.update(patch)
            return dict(self.filters)

    def reset_filters(self) -> dict[str, int]:
        with self.filter_lock:
            self.filters = dict(FILTER_DEFAULTS)
            return dict(self.filters)

    def apply_preset(self, raw: Any) -> dict[str, Any]:
        if not isinstance(raw, str) or raw not in CAMERA_PRESETS:
            raise ValueError("unknown camera preset")
        preset = CAMERA_PRESETS[raw]
        # Constants are still passed through the same strict validators and
        # hardware writer used by individual slider changes.
        controls = self.controls.update(preset["controls"])
        filters = self.set_filters(preset["filters"])
        return {
            "preset": raw, "label": preset["label"], "note": preset["note"],
            "controls": controls, "filters": filters,
        }

    def save_field_profile(self) -> dict[str, Any]:
        if self.camera_profile_path is None:
            raise ValueError("persistent camera profile path is not configured")
        saved, archive = save_versioned_camera_control_profile(
            self.camera_profile_path, self.controls.snapshot()
        )
        self.camera_profile_state.update(
            loaded=True, saved_at_utc=saved["saved_at_utc"],
            last_snapshot=str(archive),
        )
        self.status["camera_profile"] = dict(self.camera_profile_state)
        return dict(self.camera_profile_state)

    def _apply_filters(self, frame):
        with self.filter_lock:
            settings = dict(self.filters)
        cv2, np = self.cv2, self.np
        output = frame
        if settings["clahe_enabled"]:
            lab = cv2.cvtColor(output, cv2.COLOR_BGR2LAB)
            l_chan, a_chan, b_chan = cv2.split(lab)
            clahe = cv2.createCLAHE(
                clipLimit=settings["clahe_clip_x10"] / 10.0,
                tileGridSize=(settings["clahe_grid"], settings["clahe_grid"]),
            )
            output = cv2.cvtColor(
                cv2.merge((clahe.apply(l_chan), a_chan, b_chan)), cv2.COLOR_LAB2BGR
            )
        gamma = settings["gamma_x100"] / 100.0
        if abs(gamma - 1.0) > 1e-6:
            # The preset document uses gamma < 1 to lift shadows.
            table = np.array(
                [((index / 255.0) ** gamma) * 255 for index in range(256)],
                dtype=np.uint8,
            )
            output = cv2.LUT(output, table)
        gray_mix = settings["grayscale_mix"] / 100.0
        if gray_mix > 0:
            gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
            output = cv2.addWeighted(output, 1.0 - gray_mix, gray, gray_mix, 0)
        sharpen = settings["sharpen_x100"] / 100.0
        if sharpen > 0:
            blurred = cv2.GaussianBlur(output, (0, 0), 1.0)
            output = cv2.addWeighted(output, 1.0 + sharpen, blurred, -sharpen, 0)
        return output, settings

    @staticmethod
    def _recent_rate(samples: deque[float], now: float, window_s: float = 2.0) -> float:
        while samples and now - samples[0] > window_s:
            samples.popleft()
        return len(samples) / window_s

    def _capture_loop(self) -> None:
        failures = 0
        while not self.stop_event.is_set():
            ok, frame = self.capture.read()
            if not ok or frame is None:
                failures += 1
                if failures >= 10:
                    self.status.update(healthy=False, reason="camera read failure")
                    time.sleep(0.1)
                continue
            failures = 0
            now = time.monotonic()
            with self.frame_condition:
                self.latest_frame = frame
                self.capture_sequence += 1
                self.capture_times.append(now)
                self.frame_condition.notify_all()

    def _loop(self) -> None:
        seen_sequence = 0
        while not self.stop_event.is_set():
            with self.frame_condition:
                self.frame_condition.wait_for(
                    lambda: self.stop_event.is_set() or self.capture_sequence != seen_sequence,
                    timeout=0.5,
                )
                if self.stop_event.is_set():
                    break
                if self.latest_frame is None or self.capture_sequence == seen_sequence:
                    continue
                current_sequence = self.capture_sequence
                frame = self.latest_frame.copy()
            if seen_sequence and current_sequence > seen_sequence + 1:
                self.dropped_frames += current_sequence - seen_sequence - 1
            seen_sequence = current_sequence
            started = time.monotonic()
            try:
                filtered_frame, filter_settings = self._apply_filters(frame)
                model_frame = filtered_frame if filter_settings["filter_before_detect"] else frame
                display_frame = filtered_frame if filter_settings["preview_filtered"] else frame
                infer_started = time.monotonic()
                raw = self.infer(model_frame)
                inference_ms = (time.monotonic() - infer_started) * 1000.0
                detections = []
                for det in raw:
                    native_name = str(
                        det.get("class_name", det.get("class", ""))
                    ).strip().lower()
                    mapped_name = self.model_class_map.get(native_name)
                    if mapped_name is None or float(det.get("confidence", 0.0)) < self.confidence:
                        continue
                    mapped = dict(det)
                    mapped["class_name"] = mapped_name
                    mapped["class"] = mapped_name
                    detections.append(mapped)
                normalized = []
                for det in detections:
                    item = dict(det)
                    item["class"] = str(det.get("class_name", det.get("class", "unknown"))).lower()
                    normalized.append(item)
                annotated = draw_detections(display_frame, normalized, self.cv2, self.args.fov_deg)
                metrics = frame_metrics(display_frame, self.cv2, self.np)
                with self.jpeg_lock:
                    jpeg_quality = self.jpeg_quality
                ok_jpeg, encoded = self.cv2.imencode(
                    ".jpg", annotated, [int(self.cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
                )
                if not ok_jpeg:
                    raise RuntimeError("JPEG encode failed")
                completed_at = time.monotonic()
                self.output_times.append(completed_at)
                with self.frame_condition:
                    capture_fps = self._recent_rate(self.capture_times, completed_at)
                output_fps = self._recent_rate(self.output_times, completed_at)
                processing_ms = (completed_at - started) * 1000.0
                summary = [
                    {"color": item["class"], "confidence": round(float(item.get("confidence", 0)), 3)}
                    for item in normalized[:30]
                ]
                with self.condition:
                    self.jpeg = encoded.tobytes()
                    self.sequence += 1
                    self.status = {
                        **self.status, "healthy": True, "reason": "ok",
                        "detections": summary, "metrics": metrics,
                        "inference_ms": round(inference_ms, 1),
                        "processing_ms": round(processing_ms, 1),
                        "capture_fps": round(capture_fps, 1),
                        "output_fps": round(output_fps, 1),
                        "dropped_frames": self.dropped_frames,
                        "source_sequence": current_sequence,
                        "confidence": self.confidence,
                        "controls": self.controls.snapshot(), "control_error": self.controls.error,
                        "camera_profile": dict(self.camera_profile_state),
                        "filters": filter_settings,
                        "jpeg_quality": jpeg_quality,
                        "sequence": self.sequence,
                    }
                    self.condition.notify_all()
            except (ModelInferenceError, RuntimeError, ValueError) as exc:
                self.status.update(healthy=False, reason=str(exc))

    def close(self) -> None:
        self.stop_event.set()
        with self.frame_condition:
            self.frame_condition.notify_all()
        self.capture_thread.join(timeout=1.0)
        if self.capture_thread.is_alive():
            self.capture.release()
            self.capture_thread.join(timeout=1.0)
        self.thread.join(timeout=2.0)
        if self.capture.isOpened():
            self.capture.release()


HTML = r'''<!doctype html><html lang="tr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>IDA Kamera Ayarı</title>
<style>
body{margin:0;background:#09090b;color:#e4e4e7;font:14px system-ui}header{padding:14px 20px;background:#18181b;border-bottom:1px solid #3f3f46;display:flex;justify-content:space-between}.wrap{display:grid;grid-template-columns:minmax(0,2fr) minmax(300px,1fr);gap:14px;padding:14px}.card{background:#18181b;border:1px solid #3f3f46;border-radius:8px;padding:12px}img{display:block;width:100%;max-height:78vh;object-fit:contain;background:#000}.control{margin:9px 0}.control label{display:flex;justify-content:space-between}.control input{width:100%}button,select{background:#27272a;color:#fff;border:1px solid #52525b;border-radius:5px;padding:8px 12px}button{cursor:pointer}.ok{color:#22c55e}.bad{color:#ef4444}.note{color:#fbbf24;font-size:12px;margin:8px 0}pre{white-space:pre-wrap;font:12px monospace}@media(max-width:850px){.wrap{grid-template-columns:1fr}}
</style></head><body><header><b>IDA ARDUCAM + GENERAL 5 RENK MODELİ</b><span id="health">Bağlanıyor…</span></header><main class="wrap"><section class="card"><img src="" alt="Canlı model görüntüsü"></section><aside class="card"><h4>Işık presetleri</h4><select id="preset"></select> <button id="presetApply">Preseti uygula</button><div class="note" id="presetNote"></div><div class="control"><label>Confidence <b id="confV"></b></label><input id="conf" type="range" min="3" max="95"></div><div class="control"><label>Tarayıcı JPEG kalitesi <b id="jpegV"></b></label><input id="jpeg" type="range" min="60" max="95"></div><h4>B0495 donanım kontrolleri</h4><div id="controls"></div><button id="reset">Donanım varsayılanları</button> <button id="saveProfile">Saha ayarını kaydet</button><div class="note" id="profileState">Henüz kalıcı kayıt doğrulanmadı.</div><div class="note">Yalnız donanım kontrolleri açılış profiline kaydedilir. Confidence, JPEG ve yazılım filtreleri kaydedilmez.</div><h4>Yazılım filtreleri (yalnız bu test aracı)</h4><div id="filters"></div><button id="filterReset">Filtreleri kapat / nötrle</button><h4>Tespitler ve görüntü ölçüleri</h4><pre id="status"></pre></aside></main>
<script>const accessToken='__TOKEN__';
const presets=__PRESETS__;const presetSelect=document.querySelector('#preset');for(const [id,item] of Object.entries(presets)){presetSelect.insertAdjacentHTML('beforeend',`<option value="${id}">${item.label}</option>`)}function presetNote(){document.querySelector('#presetNote').textContent=presets[presetSelect.value].note||''}presetSelect.onchange=presetNote;presetNote();
const defs=[['auto_exposure','Otomatik exposure (0 kapalı / 1 açık)',0,1],['exposure','Exposure (0.1 ms)',5,660],['gain','Gain',168,1600],['auto_wb','Otomatik beyaz ayarı',0,1],['wb_kelvin','WB Kelvin',2300,6500],['saturation','Saturation',0,15],['brightness','Brightness (-64..64; ekranda 0..128)',0,128],['contrast','Contrast',0,20],['power_line_frequency','Şebeke frekansı (0 kapalı / 1 50Hz / 2 60Hz)',0,2]];
const box=document.querySelector('#controls');for(const [id,label,min,max] of defs){box.insertAdjacentHTML('beforeend',`<div class="control"><label>${label}<b id="${id}V"></b></label><input id="${id}" type="range" min="${min}" max="${max}"></div>`)}
const filterDefs=[['filter_before_detect','Filtre modele uygulansın (0/1)',0,1],['preview_filtered','Önizleme filtreli olsun (0/1)',0,1],['clahe_enabled','CLAHE yerel kontrast (0 kapalı / 1 açık)',0,1],['clahe_clip_x10','CLAHE clip ×10',10,50],['clahe_grid','CLAHE ızgara',2,16],['gamma_x100','Gamma ×100',50,200],['grayscale_mix','Siyah-beyaz karışımı %',0,100],['sharpen_x100','Yazılımsal keskinlik ×100',0,200]];
const filterBox=document.querySelector('#filters');for(const [id,label,min,max] of filterDefs){filterBox.insertAdjacentHTML('beforeend',`<div class="control"><label>${label}<b id="${id}V"></b></label><input id="${id}" type="range" min="${min}" max="${max}"></div>`)}
let loaded=false,timer=null;document.querySelector('img').src='/stream.mjpg?token='+encodeURIComponent(accessToken);async function post(path,data={}){const r=await fetch(path,{method:'POST',headers:{'content-type':'application/json','x-ida-camera-token':accessToken},body:JSON.stringify(data)});if(!r.ok)throw Error(await r.text());return r.json()}
function schedule(id){clearTimeout(timer);timer=setTimeout(async()=>{try{await post('/api/controls',{[id]:Number(document.querySelector('#'+id).value)})}catch(e){alert(e)}},120)}
for(const [id] of defs){document.querySelector('#'+id).oninput=e=>{document.querySelector('#'+id+'V').textContent=e.target.value;schedule(id)}}
function scheduleFilter(id){clearTimeout(timer);timer=setTimeout(async()=>{try{await post('/api/filters',{[id]:Number(document.querySelector('#'+id).value)})}catch(e){alert(e)}},120)}
for(const [id] of filterDefs){document.querySelector('#'+id).oninput=e=>{document.querySelector('#'+id+'V').textContent=e.target.value;scheduleFilter(id)}}
document.querySelector('#conf').oninput=e=>{document.querySelector('#confV').textContent=(e.target.value/100).toFixed(2);clearTimeout(timer);timer=setTimeout(()=>post('/api/confidence',{confidence:Number(e.target.value)/100}),120)};
document.querySelector('#jpeg').oninput=e=>{document.querySelector('#jpegV').textContent=e.target.value;clearTimeout(timer);timer=setTimeout(()=>post('/api/jpeg-quality',{jpeg_quality:Number(e.target.value)}),120)};
document.querySelector('#reset').onclick=()=>post('/api/reset');
document.querySelector('#filterReset').onclick=async()=>{await post('/api/filter-reset');loaded=false};
document.querySelector('#presetApply').onclick=async()=>{try{const r=await post('/api/preset',{preset:presetSelect.value});document.querySelector('#presetNote').textContent=r.note||'Preset uygulandı';loaded=false}catch(e){alert(e)}};
document.querySelector('#saveProfile').onclick=async()=>{try{const r=await post('/api/save-profile');document.querySelector('#profileState').textContent='KALICI KAYIT TAMAM · '+r.saved_at_utc}catch(e){alert(e)}};
async function refresh(){try{const s=await(await fetch('/api/status',{cache:'no-store',headers:{'x-ida-camera-token':accessToken}})).json();const h=document.querySelector('#health');h.textContent=s.healthy?'CANLI':'HATA: '+s.reason;h.className=s.healthy?'ok':'bad';const p=s.camera_profile||{};document.querySelector('#profileState').textContent=p.loaded?('KALICI PROFİL AKTİF · '+p.saved_at_utc):(p.configured?'Kalıcı profil henüz kaydedilmedi.':'Kalıcı profil yolu tanımlı değil.');document.querySelector('#status').textContent=JSON.stringify({detections:s.detections,metrics:s.metrics,pipeline:{capture_fps:s.capture_fps,output_fps:s.output_fps,inference_ms:s.inference_ms,processing_ms:s.processing_ms,dropped_frames:s.dropped_frames},control_error:s.control_error,camera_profile:p,filters:s.filters,jpeg_quality:s.jpeg_quality},null,2);if(!loaded&&s.controls&&s.filters){for(const [id] of defs){const el=document.querySelector('#'+id);el.value=s.controls[id];document.querySelector('#'+id+'V').textContent=el.value}for(const [id] of filterDefs){const el=document.querySelector('#'+id);el.value=s.filters[id];document.querySelector('#'+id+'V').textContent=el.value}document.querySelector('#conf').value=Math.round(s.confidence*100);document.querySelector('#confV').textContent=s.confidence.toFixed(2);document.querySelector('#jpeg').value=s.jpeg_quality;document.querySelector('#jpegV').textContent=s.jpeg_quality;loaded=true}}catch(e){document.querySelector('#health').textContent='BAĞLANTI YOK';document.querySelector('#health').className='bad'}setTimeout(refresh,500)}refresh();
</script></body></html>'''


def make_handler(runtime: CameraRuntime, access_token: str):
    class Handler(BaseHTTPRequestHandler):
        server_version = "IDA-Camera/1"

        def log_message(self, fmt: str, *args: Any) -> None:
            print(f"[camera-web] {self.address_string()} {fmt % args}", flush=True)

        def _json(self, payload: Any, status: int = 200) -> None:
            body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers(); self.wfile.write(body)

        def _authorized(self) -> bool:
            query = urlparse(self.path).query
            query_token = ""
            for pair in query.split("&"):
                if pair.startswith("token="):
                    from urllib.parse import unquote_plus
                    query_token = unquote_plus(pair.split("=", 1)[1])
                    break
            supplied = self.headers.get("x-ida-camera-token", "") or query_token
            return hmac.compare_digest(str(supplied), access_token)

        def _read_json(self) -> Any:
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError as exc:
                raise ValueError("invalid content length") from exc
            if not 1 <= length <= 4096:
                raise ValueError("request body outside bounds")
            return json.loads(self.rfile.read(length).decode("utf-8"))

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if not self._authorized():
                self.send_error(HTTPStatus.UNAUTHORIZED)
                return
            if path == "/":
                presets = {
                    key: {"label": value["label"], "note": value["note"]}
                    for key, value in CAMERA_PRESETS.items()
                }
                body = (
                    HTML.replace("__TOKEN__", access_token)
                    .replace("__PRESETS__", json.dumps(presets, ensure_ascii=False))
                    .encode("utf-8")
                )
                self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)
            elif path == "/api/status":
                self._json(runtime.status)
            elif path == "/stream.mjpg":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store"); self.end_headers()
                seen = -1
                try:
                    while True:
                        with runtime.condition:
                            runtime.condition.wait_for(lambda: runtime.sequence != seen, timeout=2.0)
                            frame, seen = runtime.jpeg, runtime.sequence
                        if frame is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:  # noqa: N802
            if not self._authorized():
                self.send_error(HTTPStatus.UNAUTHORIZED)
                return
            try:
                payload = self._read_json()
                path = urlparse(self.path).path
                if path == "/api/controls":
                    self._json({"controls": runtime.controls.update(payload)})
                elif path == "/api/reset":
                    self._json({"controls": runtime.controls.reset()})
                elif path == "/api/filters":
                    self._json({"filters": runtime.set_filters(payload)})
                elif path == "/api/filter-reset":
                    self._json({"filters": runtime.reset_filters()})
                elif path == "/api/preset":
                    if not isinstance(payload, dict) or set(payload) != {"preset"}:
                        raise ValueError("preset payload is invalid")
                    self._json(runtime.apply_preset(payload["preset"]))
                elif path == "/api/save-profile":
                    if payload != {}:
                        raise ValueError("save profile payload must be empty")
                    self._json(runtime.save_field_profile())
                elif path == "/api/confidence":
                    if not isinstance(payload, dict) or set(payload) != {"confidence"}:
                        raise ValueError("confidence payload is invalid")
                    self._json({"confidence": runtime.set_confidence(payload["confidence"])})
                elif path == "/api/jpeg-quality":
                    if not isinstance(payload, dict) or set(payload) != {"jpeg_quality"}:
                        raise ValueError("jpeg quality payload is invalid")
                    self._json({"jpeg_quality": runtime.set_jpeg_quality(payload["jpeg_quality"])})
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)
            except (ValueError, json.JSONDecodeError) as exc:
                self._json({"error": str(exc)}, 400)
    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=600)
    parser.add_argument("--camera-fps", type=float, default=80.0)
    parser.add_argument("--model", required=True)
    parser.add_argument(
        "--class-names", default="black,green,orange,red,yellow",
        help="Exact ordered native model.names manifest",
    )
    parser.add_argument(
        "--class-map",
        default='{"black":"black","green":"green","orange":"orange","red":"red","yellow":"yellow"}',
        help="Exact native-name to canonical-color JSON mapping",
    )
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.20)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--device", default="0")
    parser.add_argument("--fov-deg", type=float, default=82.0)
    parser.add_argument("--jpeg-quality", type=int, default=92)
    parser.add_argument(
        "--camera-profile",
        # Direct use of this tool must be persistent too.  ``ida_cli cam``
        # passes this path explicitly, but this default prevents a manually
        # launched camera panel from showing a misleading save button.
        default=str(ROOT / "state" / "camera_controls.json"),
        help="Persistent field hardware-control profile written by the explicit save action",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not (1 <= args.port <= 65535 and args.camera >= 0 and args.width > 0 and args.height > 0
            and 60 <= args.jpeg_quality <= 95):
        raise SystemExit("invalid camera web arguments")
    runtime = CameraRuntime(args)
    access_token = os.environ.get("IDA_CAMERA_WEB_TOKEN", "") or secrets.token_urlsafe(18)
    if not (16 <= len(access_token) <= 128 and access_token.isascii() and access_token.isprintable()):
        raise SystemExit("IDA_CAMERA_WEB_TOKEN must be 16..128 printable ASCII characters")
    server = ThreadingHTTPServer((args.host, args.port), make_handler(runtime, access_token))
    server.daemon_threads = True
    print(
        f"IDA camera web ready: http://JETSON_IP:{args.port}/?token={access_token}",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown(); server.server_close(); runtime.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
