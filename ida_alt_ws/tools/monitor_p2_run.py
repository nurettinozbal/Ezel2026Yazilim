#!/usr/bin/env python3
"""Monitor a live P2 trial and emit one machine-readable verdict."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    radius = 6_371_000.0
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * radius * math.asin(min(1.0, math.sqrt(h)))


class Monitor(Node):
    def __init__(self, no_progress_s: float, no_progress_m: float) -> None:
        super().__init__("p2_trial_monitor")
        self.no_progress_s = no_progress_s
        self.no_progress_m = no_progress_m
        self.created_mono = time.monotonic()
        self.active_mono: float | None = None
        self.positions: deque[tuple[float, tuple[float, float]]] = deque()
        self.last_state: dict = {}
        self.last_telemetry: dict = {}
        self.state_samples = 0
        # /autonomy/state içindeki tarihsel ``penalty`` adı yanıltıcıdır:
        # score.py bu alana temas + parkur-içi güvenlik puanını (temizde 60)
        # yazar. Bu nedenle yüksek değer iyidir; hüküm son güvenlik puanından
        # verilir ve ilk rapor oluşmadan yayınlanan 0 değeri yok sayılır.
        self.minimum_safety_score = math.inf
        self.maximum_safety_score = 0.0
        self.final_safety_score: float | None = None
        self.max_course_distance_m = 0.0
        self.inside_seen = False
        self.outside_since: float | None = None
        self.longest_outside_s = 0.0
        self.result: dict | None = None
        self.create_subscription(String, "/autonomy/state", self.on_state, 20)
        self.create_subscription(String, "/telemetry/state", self.on_telemetry, 20)

    def on_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self.last_state = data
        state = str(data.get("state", ""))
        now = time.monotonic()
        if state == "PARKUR_2_AVOIDANCE" and self.active_mono is None:
            self.active_mono = now
        if state == "PARKUR_2_AVOIDANCE":
            self.state_samples += 1
            penalty = data.get("penalty")
            if isinstance(penalty, (int, float)) and not isinstance(penalty, bool):
                safety_score = float(penalty)
                if safety_score > 0.0:
                    self.minimum_safety_score = min(
                        self.minimum_safety_score, safety_score
                    )
                    self.maximum_safety_score = max(
                        self.maximum_safety_score, safety_score
                    )
                    self.final_safety_score = safety_score
            distance = data.get("distance_from_course_m")
            if isinstance(distance, (int, float)) and not isinstance(distance, bool):
                self.max_course_distance_m = max(
                    self.max_course_distance_m, float(distance)
                )
            inside = data.get("inside_course_geometry")
            if inside is True:
                self.inside_seen = True
                self.close_outside_interval(now)
            elif inside is False and self.inside_seen and self.outside_since is None:
                self.outside_since = now
        if self.active_mono is not None and state in {"PARKUR_3_TARGET_LOCK", "ENGAGE", "COMPLETE"}:
            self.close_outside_interval(now)
            # Tarihsel güvenlik puanı 1.6 m temas yarıçapıyla fiziksel gövdeden
            # daha muhafazakârdır. Nihai geçme/kalma kararı rosbag post-process
            # aşamasında fiziksel çarpışma ve parkur-dışı süreyle verilir.
            self.finish(True, "p2_completed")
        elif self.active_mono is not None and state == "FAILSAFE":
            self.finish(False, f"failsafe:{data.get('failsafe_reason', 'unknown')}")

    def on_telemetry(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            point = (float(data["lat"]), float(data["lon"]))
        except (KeyError, TypeError, ValueError):
            return
        self.last_telemetry = data
        if self.active_mono is None or self.result is not None:
            return
        now = time.monotonic()
        self.positions.append((now, point))
        while self.positions and now - self.positions[0][0] > self.no_progress_s:
            self.positions.popleft()
        if self.positions and now - self.positions[0][0] >= self.no_progress_s * 0.9:
            displacement = haversine_m(self.positions[0][1], point)
            speed = float(data.get("ground_speed", data.get("speed_mps", 0.0)) or 0.0)
            if displacement < self.no_progress_m and speed < 0.20:
                self.finish(False, f"no_progress:{displacement:.2f}m/{self.no_progress_s:.0f}s")

    def close_outside_interval(self, now: float) -> None:
        if self.outside_since is None:
            return
        self.longest_outside_s = max(
            self.longest_outside_s, now - self.outside_since
        )
        self.outside_since = None

    def finish(self, success: bool, reason: str) -> None:
        if self.result is not None:
            return
        now = time.monotonic()
        self.close_outside_interval(now)
        self.result = {
            "success": success,
            "reason": reason,
            "active_elapsed_s": None if self.active_mono is None else round(now - self.active_mono, 3),
            "wall_elapsed_s": round(now - self.created_mono, 3),
            "last_state": self.last_state,
            "last_telemetry": self.last_telemetry,
            "quality": {
                "state_samples": self.state_samples,
                "minimum_safety_score": (
                    None
                    if not math.isfinite(self.minimum_safety_score)
                    else round(self.minimum_safety_score, 3)
                ),
                "maximum_safety_score": round(self.maximum_safety_score, 3),
                "final_safety_score": self.final_safety_score,
                "max_course_distance_m": round(self.max_course_distance_m, 3),
                "inside_seen": self.inside_seen,
                "longest_outside_s": round(self.longest_outside_s, 3),
            },
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--start-timeout-s", type=float, default=60.0)
    parser.add_argument("--run-timeout-s", type=float, default=210.0)
    parser.add_argument("--no-progress-s", type=float, default=30.0)
    parser.add_argument("--no-progress-m", type=float, default=0.75)
    args = parser.parse_args()
    rclpy.init()
    node = Monitor(args.no_progress_s, args.no_progress_m)
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.25)
            now = time.monotonic()
            if node.active_mono is None and now - node.created_mono > args.start_timeout_s:
                node.finish(False, "mission_start_timeout")
            elif node.active_mono is not None and now - node.active_mono > args.run_timeout_s:
                node.finish(False, "p2_run_timeout")
        result = node.result or {"success": False, "reason": "monitor_stopped"}
        Path(args.output).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(result, sort_keys=True))
        return 0 if result.get("success") else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
