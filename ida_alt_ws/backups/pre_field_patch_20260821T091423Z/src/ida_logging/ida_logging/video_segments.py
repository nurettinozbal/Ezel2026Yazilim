"""Crash-tolerant video segment journal and delivery finalizer.

An ordinary MP4 is finalized only when its writer is released. Keeping an
entire run in one writer therefore risks the whole recording after an abrupt
process or power loss. This module keeps short independently finalized MP4
segments and an fsync'ed JSONL journal.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path
from typing import Any, Callable, Iterable, Optional


SEGMENT_DIR_NAME = "processed_video_segments"
MANIFEST_NAME = "processed_video_segments.jsonl"


def _fsync_directory(path: Path) -> None:
    try:
        fd = os.open(str(path), os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError:
        # Some platforms/filesystems do not support directory fsync.
        pass


def _fsync_file(path: Path) -> None:
    # Windows requires a writable descriptor for fsync; Linux accepts this too.
    with path.open("r+b") as stream:
        os.fsync(stream.fileno())


class SegmentJournal:
    """Allocate collision-safe segment paths and durably journal finalization."""

    def __init__(self, run_dir: Path) -> None:
        self.run_dir = Path(run_dir)
        self.segment_dir = self.run_dir / SEGMENT_DIR_NAME
        self.manifest_path = self.run_dir / MANIFEST_NAME
        self.segment_dir.mkdir(parents=True, exist_ok=True)
        self._next_index = self._discover_next_index()

    def _discover_next_index(self) -> int:
        indexes = []
        for path in self.segment_dir.glob("processed_video_*.mp4"):
            try:
                indexes.append(int(path.stem.rsplit("_", 1)[-1]))
            except ValueError:
                continue
        return max(indexes, default=0) + 1

    def allocate(self) -> tuple[int, Path, Path]:
        index = self._next_index
        self._next_index += 1
        final_path = self.segment_dir / f"processed_video_{index:06d}.mp4"
        partial_path = self.segment_dir / f"processed_video_{index:06d}.partial.mp4"
        if final_path.exists() or partial_path.exists():
            raise FileExistsError(f"video segment collision: {final_path}")
        return index, partial_path, final_path

    def commit(self, partial_path: Path, final_path: Path, record: dict[str, Any]) -> Path:
        """Atomically expose a closed segment, then fsync its manifest record."""
        partial_path = Path(partial_path)
        final_path = Path(final_path)
        if not partial_path.is_file() or partial_path.stat().st_size <= 0:
            raise ValueError(f"empty or missing video segment: {partial_path}")
        if final_path.exists():
            raise FileExistsError(f"refusing to overwrite video segment: {final_path}")
        os.replace(partial_path, final_path)
        _fsync_file(final_path)
        _fsync_directory(final_path.parent)

        payload = dict(record)
        payload["file"] = str(final_path.relative_to(self.run_dir)).replace("\\", "/")
        payload["bytes"] = final_path.stat().st_size
        with open(self.manifest_path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        _fsync_directory(self.run_dir)
        return final_path

    def finalized_segments(self) -> list[Path]:
        pattern = "processed_video_[0-9][0-9][0-9][0-9][0-9][0-9].mp4"
        return sorted(self.segment_dir.glob(pattern))


def _concat_list_text(paths: Iterable[Path]) -> str:
    lines = []
    for path in paths:
        escaped = str(path.resolve()).replace("'", "'\\''")
        lines.append(f"file '{escaped}'")
    return "\n".join(lines) + "\n"


def finalize_delivery_video(
    run_dir: Path,
    *,
    runner: Callable[..., Any] = subprocess.run,
    ffmpeg_path: Optional[str] = None,
) -> Optional[Path]:
    """Stream-copy finalized segments into atomic ``processed_video.mp4``."""
    run_dir = Path(run_dir)
    segments = SegmentJournal(run_dir).finalized_segments()
    if not segments:
        return None
    executable = ffmpeg_path or shutil.which("ffmpeg")
    if not executable:
        return None

    concat_path = run_dir / ".processed_video.concat.txt"
    temp_path = run_dir / ".processed_video.finalizing.mp4"
    output_path = run_dir / "processed_video.mp4"
    concat_path.write_text(_concat_list_text(segments), encoding="utf-8")
    try:
        completed = runner(
            [
                executable, "-hide_banner", "-loglevel", "error", "-y",
                "-f", "concat", "-safe", "0", "-i", str(concat_path),
                "-c", "copy", "-movflags", "+faststart", str(temp_path),
            ],
            check=False,
            timeout=120.0,
        )
        if getattr(completed, "returncode", 1) != 0:
            return None
        if not temp_path.is_file() or temp_path.stat().st_size <= 0:
            return None
        _fsync_file(temp_path)
        os.replace(temp_path, output_path)
        _fsync_directory(run_dir)
        return output_path
    finally:
        concat_path.unlink(missing_ok=True)
        temp_path.unlink(missing_ok=True)
