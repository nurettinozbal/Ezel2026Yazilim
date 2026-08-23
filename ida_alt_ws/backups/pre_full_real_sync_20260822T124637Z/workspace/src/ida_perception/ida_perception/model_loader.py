"""Lazy Ultralytics model adapter for PyTorch and exported YOLO artifacts."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

_SUPPORTED_EXTENSIONS = {".pt", ".engine", ".onnx"}
DEFAULT_CLASS_NAMES = ["orange", "yellow", "black", "red", "green"]
LoadedModel = Tuple[Any, Callable[[Any], Any], Callable[[Any], List[Dict[str, Any]]]]


class ModelInferenceError(RuntimeError):
    """The loaded runtime failed to produce a trustworthy inference result."""


def parse_class_names(raw: str) -> List[str]:
    """Parse a comma-separated or JSON class-name list."""

    if not raw or not raw.strip():
        return list(DEFAULT_CLASS_NAMES)
    text = raw.strip()
    if text.startswith("["):
        try:
            parsed = json.loads(text)
            if isinstance(parsed, list):
                names = [
                    name.strip().lower()
                    for name in parsed
                    if isinstance(name, str) and name.strip()
                ]
                return names if names else list(DEFAULT_CLASS_NAMES)
        except Exception:
            pass
        return list(DEFAULT_CLASS_NAMES)
    names = [name.strip().lower() for name in text.split(",") if name.strip()]
    return names if len(names) > 1 else list(DEFAULT_CLASS_NAMES)


def _model_class_names(raw: Any) -> Optional[List[str]]:
    if isinstance(raw, dict):
        try:
            keys = sorted(raw)
            if keys != list(range(len(keys))):
                return None
            values: Sequence[Any] = [raw[index] for index in keys]
        except (KeyError, TypeError, ValueError):
            return None
    elif isinstance(raw, (list, tuple)):
        values = raw
    else:
        return None
    names = [str(value).strip().lower() for value in values]
    return names if names and all(names) and len(set(names)) == len(names) else None


def _to_list(value: Any) -> Any:
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    if hasattr(value, "tolist"):
        value = value.tolist()
    return value


def load_model(
    model_path: str,
    class_names: Optional[List[str]] = None,
    confidence_threshold: float = 0.45,
    iou_threshold: float = 0.45,
    imgsz: int = 640,
    device: str = "",
    *,
    yolo_factory: Optional[Callable[[str], Any]] = None,
) -> Optional[LoadedModel]:
    """Load a YOLO artifact through one fail-closed Ultralytics contract."""

    if not model_path or not model_path.strip():
        return None
    path = Path(model_path)
    if not path.is_file() or path.suffix.lower() not in _SUPPORTED_EXTENSIONS:
        return None
    expected = [
        str(name).strip().lower() for name in (class_names or DEFAULT_CLASS_NAMES)
    ]
    if (
        not expected
        or any(not name for name in expected)
        or len(set(expected)) != len(expected)
    ):
        return None
    if isinstance(imgsz, bool) or not isinstance(imgsz, int) or imgsz <= 0:
        return None
    try:
        confidence = float(confidence_threshold)
        iou = float(iou_threshold)
    except (TypeError, ValueError, OverflowError):
        return None
    if (
        not isinstance(device, str)
        or device != device.strip()
        or not math.isfinite(confidence)
        or not math.isfinite(iou)
        or not 0.0 <= confidence <= 1.0
        or not 0.0 <= iou <= 1.0
    ):
        return None
    try:
        if yolo_factory is None:
            from ultralytics import YOLO  # type: ignore

            yolo_factory = YOLO
        model = yolo_factory(str(path))
        actual = _model_class_names(getattr(model, "names", None))
    except Exception:
        return None
    if actual != expected:
        return None

    def preprocess(image: Any) -> Any:
        return image

    def infer(image: Any) -> List[Dict[str, Any]]:
        try:
            results = model.predict(
                source=image,
                conf=confidence,
                iou=iou,
                imgsz=imgsz,
                device=device,
                verbose=False,
            )
        except Exception as exc:
            raise ModelInferenceError("Ultralytics predict failed") from exc
        try:
            if not isinstance(results, (list, tuple)) or not results:
                raise ModelInferenceError("Ultralytics Results list is missing")
            boxes = getattr(results[0], "boxes", None)
            if boxes is None:
                raise ModelInferenceError("Ultralytics Results boxes are missing")
            if not all(hasattr(boxes, field) for field in ("xyxy", "conf", "cls")):
                raise ModelInferenceError("Ultralytics boxes fields are missing")
            coordinates = _to_list(boxes.xyxy)
            confidences = _to_list(boxes.conf)
            class_ids = _to_list(boxes.cls)
            if (
                not isinstance(coordinates, (list, tuple))
                or not isinstance(confidences, (list, tuple))
                or not isinstance(class_ids, (list, tuple))
                or len(coordinates) != len(confidences)
                or len(coordinates) != len(class_ids)
            ):
                raise ModelInferenceError("Ultralytics boxes schema is inconsistent")
            if not coordinates:
                return []
            parsed: List[Tuple[int, float, float, float, float, float]] = []
            for coords, raw_score, raw_class_id in zip(
                coordinates, confidences, class_ids
            ):
                if not isinstance(coords, (list, tuple)) or len(coords) != 4:
                    continue
                x1, y1, x2, y2 = [float(value) for value in coords]
                score = float(raw_score)
                class_value = float(raw_class_id)
                if not math.isfinite(class_value):
                    continue
                class_id = int(class_value)
                if (
                    not all(
                        math.isfinite(value)
                        for value in (x1, y1, x2, y2, score, class_value)
                    )
                    or not 0.0 <= score <= 1.0
                    or class_value != class_id
                    or not 0 <= class_id < len(expected)
                    or x2 <= x1
                    or y2 <= y1
                ):
                    continue
                parsed.append((class_id, x1, y1, x2, y2, score))
            if not parsed:
                raise ModelInferenceError("Ultralytics returned only invalid boxes")
            parsed.sort(
                key=lambda item: (
                    item[0], item[1], item[2], item[3], item[4], -item[5]
                )
            )
            return [
                {
                    "class": expected[class_id],
                    "confidence": score,
                    "bbox": (x1, y1, x2 - x1, y2 - y1),
                }
                for class_id, x1, y1, x2, y2, score in parsed
            ]
        except ModelInferenceError:
            raise
        except Exception as exc:
            raise ModelInferenceError("Ultralytics result parsing failed") from exc

    return model, preprocess, infer
