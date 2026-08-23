"""Finalize one run's crash-safe camera segments into the delivery MP4."""

import argparse
from pathlib import Path

from ida_logging.video_segments import finalize_delivery_video


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", help="run_... evidence directory")
    args = parser.parse_args()
    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.is_dir() or not run_dir.name.startswith("run_"):
        raise SystemExit("run_dir must be an existing run_... directory")
    output = finalize_delivery_video(run_dir)
    if output is None:
        raise SystemExit("finalization failed: no segments or ffmpeg unavailable")
    print(output)


if __name__ == "__main__":
    main()
