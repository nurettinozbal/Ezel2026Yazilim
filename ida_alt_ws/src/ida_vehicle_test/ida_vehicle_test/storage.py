"""Bounded filesystem helpers for passive evidence and logging checks."""

from __future__ import annotations

import hashlib
import os
import tempfile
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, MutableSet, Tuple

from .contracts import compact_json


EXPECTED_LOG_BASENAMES = frozenset(
    {"telemetry.csv", "processed_video.mp4", "map.mp4"}
)


def probe_log_files(
    root: Path,
    files: Any,
    previous: Mapping[str, int],
    grown: Iterable[str],
) -> Tuple[Dict[str, int], set[str], bool]:
    """Stat exactly three allow-listed regular files contained below ``root``.

    The bounded three-file probe assumes a normal local Jetson filesystem.
    Network/FUSE mounts with blocking ``stat`` are outside this passive harness
    profile and must not be configured as ``log_root``.
    """
    try:
        root = root.resolve(strict=True)
    except OSError:
        return {}, set(grown), False
    if (
        not isinstance(files, list)
        or len(files) != 3
        or any(not isinstance(item, str) or not item for item in files)
        or {Path(item).name for item in files} != EXPECTED_LOG_BASENAMES
    ):
        return {}, set(grown), False
    current: Dict[str, int] = {}
    for raw_path in files:
        try:
            lexical = Path(raw_path)
            if not lexical.is_absolute():
                return {}, set(grown), False
            normalized = Path(os.path.abspath(lexical))
            normalized.relative_to(root)
            relative = normalized.relative_to(root)
            cursor = root
            for part in relative.parts:
                cursor = cursor / part
                if cursor.is_symlink():
                    return {}, set(grown), False
            resolved = normalized.resolve(strict=True)
            resolved.relative_to(root)
            if not resolved.is_file() or resolved.name not in EXPECTED_LOG_BASENAMES:
                return {}, set(grown), False
            current[str(resolved)] = resolved.stat().st_size
        except (OSError, ValueError):
            return {}, set(grown), False
    if len(current) != 3:
        return {}, set(grown), False
    growth: MutableSet[str] = set(grown)
    for path, size in current.items():
        old_size = previous.get(path)
        if old_size is not None and size > old_size:
            growth.add(path)
    return current, set(growth).intersection(current), True


def atomic_persist(root: Path, filename: str, payload: Mapping[str, Any]) -> Dict[str, str]:
    root = root.resolve(strict=True)
    if not filename or Path(filename).name != filename:
        raise ValueError("evidence filename is invalid")
    target = root / filename
    target.relative_to(root)
    raw = (compact_json(payload) + "\n").encode("utf-8")
    digest = hashlib.sha256(raw).hexdigest()
    fd, temporary = tempfile.mkstemp(prefix=".vehicle-test-", dir=str(root))
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(raw)
            handle.flush()
            os.fsync(handle.fileno())
        os.link(temporary, target)
        os.unlink(temporary)
        try:
            directory_fd = os.open(str(root), os.O_RDONLY)
        except OSError:
            # Windows does not expose directory handles through os.open; the
            # file itself is still fsync'd. Jetson/Linux also fsyncs the dir.
            directory_fd = None
        if directory_fd is not None:
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise
    return {"path": str(target), "sha256": digest}
