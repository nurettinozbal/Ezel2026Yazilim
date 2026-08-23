#!/usr/bin/env python3
"""Static safety gate for the Jetson/YKI observe-only overlay."""

from __future__ import annotations

import ast
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
SOURCES = (
    ROOT / "src/ida_vehicle_test/ida_vehicle_test/monitor_node.py",
    ROOT / "src/ida_vehicle_test/ida_vehicle_test/producer_node.py",
)
LAUNCH = ROOT / "src/ida_bringup/launch/vehicle_test_lab.launch.py"
FORBIDDEN_IMPORT_ROOTS = {
    "pymavlink",
    "mavsdk",
    "ida_control",
    "subprocess",
}
FORBIDDEN_CALLS = {
    "create_client",
    "create_service",
    "create_action_client",
    "create_action_server",
    "Popen",
    "system",
}


def _call_name(node: ast.Call) -> str:
    function = node.func
    if isinstance(function, ast.Attribute):
        return function.attr
    if isinstance(function, ast.Name):
        return function.id
    return ""


def inspect_source(path: Path) -> dict[str, Any]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    errors: list[str] = []
    publishers: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots = {alias.name.split(".", 1)[0] for alias in node.names}
            blocked = roots & FORBIDDEN_IMPORT_ROOTS
            if blocked:
                errors.append(f"forbidden import: {sorted(blocked)}")
        elif isinstance(node, ast.ImportFrom) and node.module:
            root = node.module.split(".", 1)[0]
            if root in FORBIDDEN_IMPORT_ROOTS:
                errors.append(f"forbidden import: {node.module}")
        elif isinstance(node, ast.Call):
            name = _call_name(node)
            if name in FORBIDDEN_CALLS:
                errors.append(f"forbidden call: {name}")
            if name == "create_publisher":
                if len(node.args) < 2 or not isinstance(node.args[1], ast.Constant):
                    errors.append("publisher topic is not a static literal")
                    continue
                topic = node.args[1].value
                if not isinstance(topic, str):
                    errors.append("publisher topic is not a string")
                    continue
                publishers.append(topic)
                if not topic.startswith("/vehicle_test/"):
                    errors.append(f"production publisher forbidden: {topic}")
    return {"file": str(path), "publishers": sorted(publishers), "errors": errors}


def inspect_launch(path: Path) -> dict[str, Any]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    executables: list[str] = []
    errors: list[str] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = _call_name(node)
        if name == "ExecuteProcess":
            errors.append("ExecuteProcess is forbidden in passive overlay")
        if name != "Node":
            continue
        values = {
            keyword.arg: keyword.value.value
            for keyword in node.keywords
            if keyword.arg in {"package", "executable"}
            and isinstance(keyword.value, ast.Constant)
            and isinstance(keyword.value.value, str)
        }
        if values.get("package") != "ida_vehicle_test":
            errors.append(f"non-passive package in launch: {values.get('package')!r}")
        executable = values.get("executable")
        if executable:
            executables.append(executable)
    expected = ["vehicle_test_monitor", "vehicle_test_producer"]
    if sorted(executables) != sorted(expected):
        errors.append(f"launch executables differ from passive pair: {executables}")
    return {"file": str(path), "executables": executables, "errors": errors}


def compatibility_report() -> dict[str, Any]:
    source_reports = [inspect_source(path) for path in SOURCES]
    launch_report = inspect_launch(LAUNCH)
    errors = [error for report in source_reports for error in report["errors"]]
    errors.extend(launch_report["errors"])
    return {
        "schema_version": 1,
        "mode": "observe_only",
        "compatible": not errors,
        "actuation_enabled": False,
        "sources": source_reports,
        "launch": launch_report,
        "errors": errors,
    }


def main() -> int:
    report = compatibility_report()
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if report["compatible"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
