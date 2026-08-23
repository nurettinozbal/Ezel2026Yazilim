#!/usr/bin/env python3
"""Read-only autonomy decision recorder for stationary/field diagnostics.

This process creates subscriptions only.  It never publishes a command, calls
an actuator service, opens MAVLink, or changes a ROS parameter.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any


MAX_JSON_BYTES = 2_000_000
VALID_COLORS = {"orange", "yellow", "red", "green", "black", "unknown"}


def _finite(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def parse_json_object(raw: str) -> dict[str, Any]:
    if not isinstance(raw, str) or len(raw.encode("utf-8")) > MAX_JSON_BYTES:
        return {}
    try:
        value = json.loads(raw)
    except (TypeError, ValueError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def turn_label(yaw_rate: Any, deadband: float = 0.02) -> str:
    """Stack convention: positive yaw is starboard/right, negative is left."""
    yaw = _finite(yaw_rate)
    if yaw is None:
        return "YOK"
    if yaw > deadband:
        return "SAG"
    if yaw < -deadband:
        return "SOL"
    return "DUZ"


def side_label(lateral_right_m: Any, deadband_m: float = 0.15) -> str:
    lateral = _finite(lateral_right_m)
    if lateral is None:
        return "YOK"
    if lateral > deadband_m:
        return "SAG"
    if lateral < -deadband_m:
        return "SOL"
    return "ON"


def _distance(forward: float, lateral: float) -> float:
    return math.hypot(forward, lateral)


def _normalized_object(
    raw: Any, source: str, *, lateral_left: bool = False
) -> dict[str, Any] | None:
    if not isinstance(raw, dict):
        return None
    forward = _finite(raw.get("forward_m"))
    lateral_key = "lateral_left_m" if lateral_left else "lateral_m"
    lateral = _finite(raw.get(lateral_key))
    if forward is None or lateral is None:
        return None
    if lateral_left:
        lateral = -lateral
    if not -100.0 <= forward <= 100.0 or not -100.0 <= lateral <= 100.0:
        return None
    distance = _finite(raw.get("distance_m", raw.get("distance")))
    if distance is None or distance < 0.0:
        distance = _distance(forward, lateral)
    color = str(raw.get("color", "unknown")).strip().lower()
    if color not in VALID_COLORS:
        color = "unknown"
    confidence = _finite(raw.get("confidence"))
    if confidence is not None and not 0.0 <= confidence <= 1.0:
        confidence = None
    identity = raw.get("lidar_obstacle_id", raw.get("id", ""))
    identity = identity if isinstance(identity, str) else ""
    return {
        "source": source,
        "id": identity[:80],
        "color": color,
        "confidence": confidence,
        "forward_m": round(forward, 3),
        "lateral_right_m": round(lateral, 3),
        "distance_m": round(distance, 3),
        "side": side_label(lateral),
        "hard_obstacle": raw.get("hard_obstacle") is True,
    }


def extract_objects(latest: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    sources = (
        ("buoys", "detections", "fused_buoy", False),
        ("obstacles", "obstacles", "fused_obstacle", False),
        ("lidar_raw", "clusters", "lidar_cluster", True),
    )
    for channel, field, source, left_positive in sources:
        payload = latest.get(channel, {})
        items = payload.get(field, []) if isinstance(payload, dict) else []
        if not isinstance(items, list):
            continue
        for item in items[:200]:
            normalized = _normalized_object(
                item, source, lateral_left=left_positive
            )
            if normalized is not None:
                rows.append(normalized)
    # Prefer canonical/fused identities; retain raw lidar objects not represented
    # by a canonical lidar_obstacle_id for physical visibility.
    unique: dict[tuple[str, str], dict[str, Any]] = {}
    for row in rows:
        key = (row["source"], row["id"] or f'{row["forward_m"]}:{row["lateral_right_m"]}')
        unique[key] = row
    return sorted(unique.values(), key=lambda item: (item["distance_m"], item["source"], item["id"]))[:40]


def extract_camera_detections(latest: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for channel, role in (("camera_p1p2", "p1p2"), ("camera_p3", "p3")):
        payload = latest.get(channel, {})
        items = payload.get("detections", []) if isinstance(payload, dict) else []
        if not isinstance(items, list):
            continue
        for index, raw in enumerate(items[:100]):
            if not isinstance(raw, dict):
                continue
            color = str(raw.get("color", raw.get("class_name", "unknown"))).lower()
            confidence = _finite(raw.get("confidence"))
            bearing = _finite(raw.get("bearing_deg"))
            if color not in VALID_COLORS or confidence is None or bearing is None:
                continue
            if not 0.0 <= confidence <= 1.0 or not -180.0 <= bearing <= 180.0:
                continue
            rows.append({
                "role": role,
                "id": str(raw.get("id", f"{role}:{index}"))[:80],
                "color": color,
                "confidence": round(confidence, 4),
                "bearing_deg": round(bearing, 3),
                "side": side_label(math.tan(math.radians(bearing))),
                "bbox_norm_x": _finite(raw.get("bbox_norm_x")),
                "bbox_size": _finite(raw.get("bbox_size")),
            })
    return sorted(rows, key=lambda item: (-item["confidence"], abs(item["bearing_deg"]), item["id"]))


def engagement_phase(action: Any, state: Any) -> str:
    text = f"{state or ''} {action or ''}".lower()
    if "engage_contact_window" in text:
        return "TEMAS_PENCERESI"
    if "engage" in text:
        return "ANGAJMAN"
    if "target_confirming" in text:
        return "HEDEF_DOGRULAMA"
    if "target_lock" in text:
        return "HEDEF_KILIDI"
    if "search_target_360" in text:
        return "360_ARAMA"
    if "parkur_2" in text or "dwa" in text or "avoid" in text:
        return "ENGELDEN_KACINMA"
    if "parkur_1" in text or "waypoint" in text:
        return "WAYPOINT_TAKIP"
    return "DIGER"


def _command(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, dict):
        return {"forward_mps": None, "yaw_rate_rps": None, "turn": "YOK"}
    forward = _finite(payload.get("forward_mps", payload.get("vx")))
    yaw = _finite(payload.get("yaw_rate_rps", payload.get("yaw_rate")))
    return {
        "forward_mps": None if forward is None else round(forward, 4),
        "yaw_rate_rps": None if yaw is None else round(yaw, 4),
        "turn": turn_label(yaw),
    }


def compose_record(
    latest: dict[str, Any], receipt_mono: dict[str, float], now_epoch: float, now_mono: float
) -> dict[str, Any]:
    objects = extract_objects(latest)
    camera = extract_camera_detections(latest)
    colored = [row for row in objects if row["color"] != "unknown"]
    hard = [row for row in objects if row["hard_obstacle"]]
    nearest = (colored or hard or objects or [None])[0]

    state = latest.get("autonomy_state", {})
    debug = latest.get("autonomy_debug", {})
    fusion = latest.get("fusion_status", {})
    telemetry = latest.get("telemetry", {})
    mission = latest.get("mission_waypoints", {})
    target_color_payload = latest.get("target_color", {})
    if not isinstance(state, dict):
        state = {}
    if not isinstance(debug, dict):
        debug = {}
    if not isinstance(fusion, dict):
        fusion = {}
    if not isinstance(telemetry, dict):
        telemetry = {}
    if not isinstance(mission, dict):
        mission = {}
    if not isinstance(target_color_payload, dict):
        target_color_payload = {}

    planned_payload = latest.get("planned_cmd", {})
    if isinstance(debug.get("command"), dict):
        planned_payload = {**debug["command"], **planned_payload}
    planned = _command(planned_payload)
    limited = _command(latest.get("limited_cmd", {}))

    left_pwm = telemetry.get("motor_left_pwm")
    right_pwm = telemetry.get("motor_right_pwm")
    left_pwm = int(left_pwm) if _finite(left_pwm) is not None else None
    right_pwm = int(right_pwm) if _finite(right_pwm) is not None else None
    current_wp = state.get("current_waypoint")
    waypoints = mission.get("waypoints", [])
    active_wp = None
    if (
        isinstance(current_wp, int) and not isinstance(current_wp, bool)
        and isinstance(waypoints, list) and 0 <= current_wp < len(waypoints)
        and isinstance(waypoints[current_wp], dict)
    ):
        active_wp = waypoints[current_wp]
    # Parkur/kordinat kanıtı: mission waypoints listesi kaçırılırsa (VOLATILE
    # dönem, geç abonelik) autonomy/state'teki durum adı (PARKUR_x_*), total_waypoints
    # + current_waypoint ile parkur tahmini yapılır. TRANSIENT_LOCAL sonrası bu yol
    # yedek olarak kalır.
    state_total = state.get("total_waypoints")
    total_waypoints = (
        state_total
        if isinstance(state_total, int) and not isinstance(state_total, bool)
        else (len(waypoints) if isinstance(waypoints, list) else None)
    )
    parkur = None
    if active_wp is not None and isinstance(active_wp.get("parkur"), int):
        parkur = active_wp["parkur"]
    else:
        state_name = str(state.get("state", "")).upper()
        for prefix, number in (
            ("PARKUR_1", 1), ("PARKUR_2", 2), ("PARKUR_3", 3), ("P3", 3),
        ):
            if prefix in state_name:
                parkur = number
                break
        if parkur is None and total_waypoints is not None and total_waypoints > 0:
            # Mission listesi yoksa, waypoint indeksinden kabaca tahmin:
            # son waypoint kümesi P3 hedefidir; ilk yarı P1, ikinci yarı P2.
            if current_wp >= total_waypoints:
                parkur = 3
            elif current_wp < total_waypoints:
                parkur = 1 if current_wp * 2 < total_waypoints else 2
    goal_body = debug.get("goal_body")
    if not isinstance(goal_body, list) or len(goal_body) != 2:
        goal_body = None
    target_color = str(
        target_color_payload.get("target_color", state.get("target_color", ""))
    ).lower()
    matching_camera = next(
        (item for item in camera if item["color"] == target_color), None
    )
    action = (
        debug.get("command", {}).get("action", state.get("action", ""))
        if isinstance(debug.get("command", {}), dict)
        else state.get("action", "")
    )
    ages = {
        key: round(max(0.0, now_mono - stamp), 3)
        for key, stamp in sorted(receipt_mono.items())
        if math.isfinite(stamp)
    }
    stamp = time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(now_epoch))
    stamp += f".{int((now_epoch % 1.0) * 1000):03d}Z"
    return {
        "schema_version": 1,
        "timestamp_utc": stamp,
        "epoch_s": round(now_epoch, 3),
        "autonomy": {
            "state": state.get("state", ""),
            "action": action,
            "phase": engagement_phase(action, state.get("state", "")),
            "current_waypoint": current_wp,
            "failsafe_reason": state.get("failsafe_reason", debug.get("failsafe_reason", "")),
        },
        "mission_intent": {
            "parkur": parkur,
            "waypoint_lat": active_wp.get("lat") if active_wp else None,
            "waypoint_lon": active_wp.get("lon") if active_wp else None,
            "goal_forward_m": _finite(goal_body[0]) if goal_body else None,
            "goal_lateral_right_m": _finite(goal_body[1]) if goal_body else None,
            "goal_side": side_label(goal_body[1]) if goal_body else "YOK",
            "total_waypoints": total_waypoints,
        },
        "engagement": {
            "requested_color": target_color,
            "camera_match": matching_camera,
            "camera_detections": camera,
        },
        "target": nearest,
        "objects": objects,
        "planned_command": planned,
        "limited_command": limited,
        "motor_output": {
            "left_pwm": left_pwm,
            "right_pwm": right_pwm,
            "measured": left_pwm is not None and right_pwm is not None,
            "source": "telemetry/state" if left_pwm is not None and right_pwm is not None else "unavailable",
        },
        "vehicle": {
            key: telemetry.get(key)
            for key in ("mode", "heading_deg", "ground_speed", "lat", "lon")
        },
        "fusion": {
            key: fusion.get(key)
            for key in (
                "accepted", "source_health", "camera_fresh", "lidar_fresh",
                "matched_count", "ambiguous_camera_count", "ambiguous_lidar_count",
                "association_overflow",
            )
        },
        "channel_age_s": ages,
    }


CSV_FIELDS = (
    "timestamp_utc", "state", "phase", "parkur", "action", "waypoint",
    "goal_forward_m", "goal_lateral_right_m", "goal_side", "requested_color",
    "camera_color", "camera_confidence", "camera_bearing_deg", "target_source", "target_id",
    "target_color", "target_side", "target_distance_m", "target_forward_m",
    "target_lateral_right_m", "planned_forward_mps", "planned_yaw_rate_rps",
    "planned_turn", "limited_forward_mps", "limited_yaw_rate_rps", "limited_turn",
    "left_pwm", "right_pwm", "pwm_measured", "fusion_health", "failsafe_reason",
)


def csv_row(record: dict[str, Any]) -> dict[str, Any]:
    target = record.get("target") or {}
    autonomy = record["autonomy"]
    mission = record["mission_intent"]
    engagement = record["engagement"]
    camera_match = engagement.get("camera_match") or {}
    planned = record["planned_command"]
    limited = record["limited_command"]
    motor = record["motor_output"]
    return {
        "timestamp_utc": record["timestamp_utc"],
        "state": autonomy.get("state"),
        "phase": autonomy.get("phase"),
        "parkur": mission.get("parkur"),
        "action": autonomy.get("action"),
        "waypoint": autonomy.get("current_waypoint"),
        "goal_forward_m": mission.get("goal_forward_m"),
        "goal_lateral_right_m": mission.get("goal_lateral_right_m"),
        "goal_side": mission.get("goal_side"),
        "requested_color": engagement.get("requested_color"),
        "camera_color": camera_match.get("color"),
        "camera_confidence": camera_match.get("confidence"),
        "camera_bearing_deg": camera_match.get("bearing_deg"),
        "target_source": target.get("source"),
        "target_id": target.get("id"),
        "target_color": target.get("color"),
        "target_side": target.get("side"),
        "target_distance_m": target.get("distance_m"),
        "target_forward_m": target.get("forward_m"),
        "target_lateral_right_m": target.get("lateral_right_m"),
        "planned_forward_mps": planned.get("forward_mps"),
        "planned_yaw_rate_rps": planned.get("yaw_rate_rps"),
        "planned_turn": planned.get("turn"),
        "limited_forward_mps": limited.get("forward_mps"),
        "limited_yaw_rate_rps": limited.get("yaw_rate_rps"),
        "limited_turn": limited.get("turn"),
        "left_pwm": motor.get("left_pwm"),
        "right_pwm": motor.get("right_pwm"),
        "pwm_measured": motor.get("measured"),
        "fusion_health": record.get("fusion", {}).get("source_health"),
        "failsafe_reason": autonomy.get("failsafe_reason"),
    }


def format_record(record: dict[str, Any]) -> str:
    row = csv_row(record)
    target = (
        f'{row["target_color"] or "unknown"}/{row["target_source"] or "yok"} '
        f'{row["target_side"] or "YOK"} {row["target_distance_m"] if row["target_distance_m"] is not None else "--"}m'
    )
    pwm = (
        f'L={row["left_pwm"]} R={row["right_pwm"]}'
        if row["pwm_measured"] else "L=YOK R=YOK"
    )
    return (
        f'{row["timestamp_utc"]} | P={row["parkur"] or "--"} {row["phase"]} '
        f'WP={row["waypoint"] if row["waypoint"] is not None else "--"} '
        f'hedef={target} kamera={row["camera_color"] or "--"}/'
        f'{row["camera_confidence"] if row["camera_confidence"] is not None else "--"} '
        f'| karar={row["action"] or "--"} '
        f'yon={row["limited_turn"] or row["planned_turn"]} '
        f'v={row["limited_forward_mps"] if row["limited_forward_mps"] is not None else row["planned_forward_mps"]} '
        f'yaw={row["limited_yaw_rate_rps"] if row["limited_yaw_rate_rps"] is not None else row["planned_yaw_rate_rps"]} '
        f'| PWM {pwm} | failsafe={row["failsafe_reason"] or "yok"}'
    )


def render_file(path: Path, last: int) -> int:
    if not path.is_file():
        raise FileNotFoundError(path)
    lines = path.read_text(encoding="utf-8").splitlines()[-last:]
    for line in lines:
        try:
            record = json.loads(line)
        except (ValueError, json.JSONDecodeError):
            continue
        if isinstance(record, dict):
            print(format_record(record))
    return 0


class DecisionRecorder:
    def __init__(self, node: Any, out_dir: Path, hz: float) -> None:
        self.node = node
        self.out_dir = out_dir
        self.hz = hz
        self.latest: dict[str, Any] = {}
        self.receipt_mono: dict[str, float] = {}
        self.lock = threading.Lock()
        out_dir.mkdir(parents=True, exist_ok=False)
        self.json_file = (out_dir / "decision_telemetry.jsonl").open("x", encoding="utf-8", buffering=1)
        self.csv_file = (out_dir / "decision_telemetry.csv").open("x", encoding="utf-8", newline="", buffering=1)
        self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()

    def ingest_json(self, channel: str, raw: str) -> None:
        payload = parse_json_object(raw)
        with self.lock:
            self.latest[channel] = payload
            self.receipt_mono[channel] = time.monotonic()

    def ingest_twist(self, channel: str, msg: Any) -> None:
        payload = {
            "forward_mps": _finite(getattr(msg.linear, "x", None)),
            "yaw_rate_rps": _finite(getattr(msg.angular, "z", None)),
        }
        with self.lock:
            self.latest[channel] = payload
            self.receipt_mono[channel] = time.monotonic()

    def tick(self) -> None:
        with self.lock:
            latest = dict(self.latest)
            receipts = dict(self.receipt_mono)
        record = compose_record(latest, receipts, time.time(), time.monotonic())
        self.json_file.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
        self.writer.writerow(csv_row(record))

    def close(self) -> None:
        self.json_file.flush()
        self.csv_file.flush()
        os.fsync(self.json_file.fileno())
        os.fsync(self.csv_file.fileno())
        self.json_file.close()
        self.csv_file.close()


def run_ros(out_dir: Path, hz: float) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    rclpy.init()
    node = Node("ida_decision_telemetry_logger")
    recorder = DecisionRecorder(node, out_dir, hz)
    json_topics = {
        "autonomy_state": "/autonomy/state",
        "autonomy_debug": "/autonomy/debug",
        "telemetry": "/telemetry/state",
        "buoys": "/perception/buoys",
        "obstacles": "/perception/obstacles",
        "lidar_raw": "/perception/lidar/raw_obstacles",
        "fusion_status": "/perception/fusion/status",
        "camera_p1p2": "/perception/camera/p1p2/raw",
        "camera_p3": "/perception/camera/p3/raw",
        "target_color": "/mission/target_color",
        "autonomy_score": "/autonomy/score",
    }
    subscriptions = []
    for channel, topic in json_topics.items():
        subscriptions.append(node.create_subscription(
            String, topic, lambda msg, key=channel: recorder.ingest_json(key, msg.data), 30
        ))
    # Görev noktaları: publisher'ı (bridge node) TRANSIENT_LOCAL (latch)
    # olduğundan, kaydedici mission upload'undan SONRA başlasa bile son waypoint
    # payload'ını alır. Geç abonelik VOLATILE iken mesaj kaçıyordu -> P=-- (saha
    # bulgusu: docs/SAHA_TEST_BULGULARI_20260817.md).
    mission_qos = QoSProfile(depth=30, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions.append(node.create_subscription(
        String, "/mission/waypoints",
        lambda msg: recorder.ingest_json("mission_waypoints", msg.data),
        mission_qos,
    ))
    subscriptions.append(node.create_subscription(
        Twist, "/autonomy/cmd_vel_body", lambda msg: recorder.ingest_twist("planned_cmd", msg), 30
    ))
    subscriptions.append(node.create_subscription(
        Twist, "/control/cmd_vel_body", lambda msg: recorder.ingest_twist("limited_cmd", msg), 30
    ))
    timer = node.create_timer(1.0 / hz, recorder.tick)
    node.get_logger().info(f"Salt-okunur karar gunlugu basladi: {out_dir}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        timer.cancel()
        recorder.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="IDA read-only decision telemetry logger")
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--render", type=Path)
    parser.add_argument("--last", type=int, default=30)
    args = parser.parse_args()
    if args.render is not None:
        if not 1 <= args.last <= 1000:
            raise ValueError("last must be within [1, 1000]")
        return render_file(args.render, args.last)
    if args.out_dir is None:
        parser.error("--out-dir is required while recording")
    if not math.isfinite(args.hz) or not 1.0 <= args.hz <= 10.0:
        raise ValueError("hz must be within [1, 10]")
    return run_ros(args.out_dir, args.hz)


if __name__ == "__main__":
    raise SystemExit(main())
