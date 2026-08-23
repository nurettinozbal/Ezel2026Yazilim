"""Thread-safe field logs and WebSocket log event queue."""

from __future__ import annotations

import csv
import json
import logging
import math
import queue
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


class BackendLogger:
    def __init__(self, logs_dir: Path | None = None) -> None:
        self.logs_dir = logs_dir or Path(__file__).resolve().parents[1] / "logs"
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.telemetry_path = self.logs_dir / f"telemetry_{stamp}.csv"
        self.commands_path = self.logs_dir / f"commands_{stamp}.jsonl"
        self.system_path = self.logs_dir / f"system_{stamp}.log"
        self.events: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=1000)
        self._lock = threading.Lock()
        self._telemetry_buffer: list[list[Any]] = []
        self._telemetry_flush_size = 10

        self._system_logger = logging.getLogger(f"ezel_gcs_backend_{stamp}")
        self._system_logger.setLevel(logging.INFO)
        self._system_logger.propagate = False
        handler = logging.FileHandler(self.system_path, encoding="utf-8")
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        self._system_logger.addHandler(handler)

        with self.telemetry_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "timestamp",
                    "ida_connected",
                    "ida_mode",
                    "ida_lat",
                    "ida_lon",
                    "ida_speed",
                    "ida_voltage",
                    "ida_heading_deg",
                    "ida_roll_deg",
                    "ida_pitch_deg",
                    "ida_target_speed_mps",
                    "ida_target_heading_deg",
                    "iha_connected",
                    "iha_lat",
                    "iha_lon",
                    "iha_alt",
                    "target_color",
                    "rssi",
                ]
            )

    @staticmethod
    def _timestamp() -> str:
        return datetime.now(timezone.utc).isoformat()

    def system(self, message: str, level: str = "INFO") -> None:
        level = level.upper()
        log_method = getattr(self._system_logger, level.lower(), self._system_logger.info)
        log_method(message)
        event = {
            "type": "log",
            "data": {
                "timestamp": self._timestamp(),
                "level": level,
                "message": message,
            },
        }
        try:
            self.events.put_nowait(event)
        except queue.Full:
            pass

    def command(self, command: str, payload: dict[str, Any], result: dict[str, Any]) -> None:
        record = {
            "timestamp": self._timestamp(),
            "command": command,
            "payload": payload,
            "result": result,
        }
        with self._lock, self.commands_path.open("a", encoding="utf-8") as file:
            file.write(json.dumps(record, ensure_ascii=False) + "\n")

    @staticmethod
    def _csv_value(value: Any) -> Any:
        return "" if value is None else value

    @staticmethod
    def _csv_degrees(radians: Any) -> Any:
        """MAVLink ATTITUDE roll/pitch radyan yayınlar; teslim CSV'si derece ister."""
        if radians is None:
            return ""
        try:
            return round(math.degrees(float(radians)), 2)
        except (TypeError, ValueError):
            return ""

    def telemetry(self, snapshot: dict[str, Any]) -> None:
        ida = snapshot["ida"]
        iha = snapshot["iha"]
        system = snapshot["system"]
        row = [
            self._timestamp(),
            ida["connected"],
            ida["mode"],
            ida["lat"],
            ida["lon"],
            ida["speed"],
            ida["voltage"],
            self._csv_value(ida.get("heading")),
            self._csv_degrees(ida.get("roll")),
            self._csv_degrees(ida.get("pitch")),
            self._csv_value(ida.get("target_speed")),
            self._csv_value(ida.get("target_heading")),
            iha["connected"],
            iha["lat"],
            iha["lon"],
            iha["alt"],
            iha["detected_color"],
            system["rssi"],
        ]
        with self._lock:
            self._telemetry_buffer.append(row)
            if len(self._telemetry_buffer) >= self._telemetry_flush_size:
                self._flush_telemetry_locked()

    def _flush_telemetry_locked(self) -> None:
        """Flush buffered telemetry rows to disk."""
        if not self._telemetry_buffer:
            return
        try:
            with self.telemetry_path.open("a", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerows(self._telemetry_buffer)
        except Exception as exc:
            self._system_logger.error("Telemetri CSV yazılamadı: %s", exc)
            try:
                self.events.put_nowait(
                    {
                        "type": "log",
                        "data": {
                            "timestamp": self._timestamp(),
                            "level": "ERROR",
                            "message": f"Telemetri CSV yazılamadı: {exc}",
                        },
                    }
                )
            except queue.Full:
                pass
            self._telemetry_buffer = self._telemetry_buffer[-self._telemetry_flush_size :]
            return
        self._telemetry_buffer.clear()

    def flush(self) -> None:
        """Force flush any remaining buffered data. Call on shutdown."""
        with self._lock:
            self._flush_telemetry_locked()

