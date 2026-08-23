"""Pure JSON-contract helpers for the ROS sensor-fusion node."""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from .core import FusionResult, finite_float


def payload_stamp(payload: Any) -> Optional[float]:
    if not isinstance(payload, dict):
        return None
    # Processing/publish time must never masquerade as sensor acquisition time.
    return finite_float(payload.get("acquisition_stamp"))


def camera_payload(payload: Any) -> Optional[Tuple[float, List[Dict[str, Any]]]]:
    stamp = payload_stamp(payload)
    if stamp is None or not isinstance(payload, dict) or bool(payload.get("stale", False)):
        return None
    detections = payload.get("detections")
    if not isinstance(detections, list):
        return None
    return stamp, detections


def lidar_payload(payload: Any) -> Optional[Tuple[float, List[Dict[str, Any]]]]:
    stamp = payload_stamp(payload)
    if stamp is None or not isinstance(payload, dict) or bool(payload.get("stale", False)):
        return None
    clusters = payload.get("clusters")
    if not isinstance(clusters, list):
        return None
    return stamp, clusters


def merge_camera_roles(
    roles: Iterable[Tuple[str, float, Sequence[Dict[str, Any]]]],
    max_role_delta_s: float = 0.0,
) -> Optional[Tuple[float, List[Dict[str, Any]]]]:
    delta = finite_float(max_role_delta_s)
    if delta is None or delta < 0.0:
        raise ValueError("max_role_delta_s must be finite and >= 0")
    frames = [(str(role), float(stamp), list(detections)) for role, stamp, detections in roles]
    if not frames:
        return None
    newest = max(stamp for _, stamp, _ in frames)
    selected = [item for item in frames if newest - item[1] <= delta + 1e-12]
    merged: List[Dict[str, Any]] = []
    seen: set[Tuple[str, str]] = set()
    for role, _stamp, detections in sorted(selected, key=lambda item: (item[1], item[0])):
        for index, raw in enumerate(detections):
            if not isinstance(raw, dict):
                merged.append(raw)
                continue
            identity = str(raw.get("id", f"anonymous:{index}"))
            key = (role, identity, str(raw.get("color", "")))
            if key in seen:
                continue
            seen.add(key)
            detection = dict(raw)
            detection["id"] = f"{role}:{identity}"
            merged.append(detection)
    return newest, merged


def result_payloads(result: FusionResult) -> Tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any]]:
    """Convert one fusion result to canonical buoy/obstacle/status payloads."""

    stamp = max(result.status.camera_stamp, result.status.lidar_stamp)
    obstacles: List[Dict[str, Any]] = []
    buoys: List[Dict[str, Any]] = []
    for item in result.obstacles:
        bearing = math.degrees(math.atan2(item.lateral_m, item.forward_m))
        common: Dict[str, Any] = {
            "stamp": stamp,
            "id": item.lidar_id,
            "distance": item.distance_m,
            "forward_m": item.forward_m,
            "lateral_m": item.lateral_m,
            "bearing_deg": bearing,
            "color": item.color,
            "confidence": item.confidence,
            "source": item.source,
        }
        obstacles.append({**common, "hard_obstacle": True})
        if item.camera_id is not None and item.color != "unknown":
            buoy = {
                **common,
                "id": item.camera_id,
                "lidar_obstacle_id": item.lidar_id,
                "association_cost_deg": item.association_cost_deg,
            }
            if item.bbox_norm_x is not None:
                buoy["bbox_norm_x"] = item.bbox_norm_x
            if item.bbox_size is not None:
                buoy["bbox_size"] = item.bbox_size
            buoys.append(buoy)

    status = {
        "stamp": stamp,
        "temporal_health": result.status.temporal_health,
        "dt_ms": result.status.dt_ms,
        "camera_stamp": result.status.camera_stamp,
        "lidar_stamp": result.status.lidar_stamp,
        "accepted": result.status.accepted,
        "matched_count": result.status.matched_count,
        "ambiguous_camera_count": result.status.ambiguous_camera_count,
        "ambiguous_lidar_count": result.status.ambiguous_lidar_count,
        "invalid_camera_count": result.status.invalid_camera_count,
        "invalid_lidar_count": result.status.invalid_lidar_count,
        "association_overflow": result.status.association_overflow,
        "overflow_component_count": result.status.overflow_component_count,
        "overflow_camera_count": result.status.overflow_camera_count,
        "overflow_lidar_count": result.status.overflow_lidar_count,
        "clock_rollback_resets": result.status.clock_rollback_resets,
        **result.status.readiness.as_dict(),
    }
    return (
        {"stamp": stamp, "detections": buoys},
        {"stamp": stamp, "obstacles": obstacles},
        status,
    )
