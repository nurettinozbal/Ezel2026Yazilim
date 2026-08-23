"""Finalize one run's crash-safe camera and map segments."""

import argparse
from pathlib import Path

from ida_logging.video_segments import finalize_delivery_video


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", help="run_... evidence directory")
    parser.add_argument(
        "--kind", choices=("all", "processed_video", "map"), default="all"
    )
    args = parser.parse_args()
    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.is_dir() or not run_dir.name.startswith("run_"):
        raise SystemExit("run_dir must be an existing run_... directory")
    outputs = []
    if args.kind in ("all", "processed_video"):
        output = finalize_delivery_video(run_dir)
        if output is not None:
            outputs.append(output)
    if args.kind in ("all", "map"):
        output = finalize_delivery_video(run_dir, stem="map", output_name="map.mp4")
        if output is not None:
            outputs.append(output)
    if not outputs:
        raise SystemExit("finalization failed: no segments or ffmpeg unavailable")
    for output in outputs:
        print(output)


if __name__ == "__main__":
    main()
