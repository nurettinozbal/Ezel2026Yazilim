"""Local costmap recorder (``ida_map_logger``) — referee file 3.

The latest costmap, perception, autonomy decision and telemetry are rendered to
``map.mp4``. The canonical body frame is +X forward/up and +Y right.
"""

from __future__ import annotations

from typing import Any, Dict, List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from ida_logging.map_video_renderer import (
    FRAME_HEIGHT,
    FRAME_WIDTH,
    cv2,
    render_costmap_frame,
    renderer_available,
)
from ida_logging.run_dir import make_run_dir, make_run_dir_same
from ida_planning.contracts import loads


class MapLoggerNode(Node):
    """Render the bounded latest local-map state to an MP4 file."""

    def __init__(self) -> None:
        super().__init__("ida_map_logger")
        self.declare_parameter("log_dir", "./logs")
        self.declare_parameter("run_name", "")
        self.declare_parameter("log_hz", 5.0)
        # Backward-compatible with the old JSONL logger configuration.
        self.declare_parameter("flush_every_lines", 100)

        self.log_dir = str(self.get_parameter("log_dir").value)
        self.run_name = str(self.get_parameter("run_name").value).strip()
        self.log_hz = float(self.get_parameter("log_hz").value)
        if not (0.5 <= self.log_hz <= 30.0):
            raise ValueError("ida_map_logger log_hz must be in [0.5, 30]")

        self.telemetry: Dict[str, Any] = {}
        self.obstacles: List[Dict[str, Any]] = []
        self.buoys: List[Dict[str, Any]] = []
        self.costmap_cells: List[List[Any]] = []
        self.costmap_meta: Dict[str, Any] = {"size_m": 30.0, "cell_m": 0.25, "n": 120}
        self.score: Dict[str, Any] = {}
        self.autonomy: Dict[str, Any] = {}
        self.command: Dict[str, float] = {"vx": 0.0, "vy": 0.0, "yaw_rate": 0.0}

        self.create_subscription(String, "/perception/obstacles", self.on_obstacles, 30)
        self.create_subscription(String, "/perception/buoys", self.on_buoys, 30)
        self.create_subscription(String, "/telemetry/state", self.on_telemetry, 30)
        self.create_subscription(String, "/planning/costmap", self.on_costmap, 30)
        self.create_subscription(String, "/autonomy/score", self.on_score, 30)
        self.create_subscription(String, "/autonomy/state", self.on_autonomy, 30)
        self.create_subscription(Twist, "/control/cmd_vel_body", self.on_command, 30)

        run_dir = (
            make_run_dir_same(self.log_dir, self.run_name)
            if self.run_name
            else make_run_dir(self.log_dir)
        )
        self.video_path = run_dir / "map.mp4"
        self._writer = None
        if renderer_available():
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(
                str(self.video_path), fourcc, self.log_hz, (FRAME_WIDTH, FRAME_HEIGHT)
            )
            if not self._writer.isOpened():
                self._writer.release()
                self._writer = None
                self.get_logger().error(f"map.mp4 acilamadi: {self.video_path}")
        else:
            self.get_logger().error("map.mp4 icin python3-opencv/python3-numpy bulunamadi")

        self.create_timer(1.0 / self.log_hz, self.tick)
        self.get_logger().info(f"Costmap video logger started -> {self.video_path}")

    def on_obstacles(self, msg: String) -> None:
        payload = loads(msg.data, {})
        items = payload.get("obstacles", []) if isinstance(payload, dict) else []
        self.obstacles = items if isinstance(items, list) else []

    def on_buoys(self, msg: String) -> None:
        payload = loads(msg.data, {})
        items = payload.get("detections", []) if isinstance(payload, dict) else []
        self.buoys = items if isinstance(items, list) else []

    def on_telemetry(self, msg: String) -> None:
        payload = loads(msg.data, {})
        if isinstance(payload, dict):
            self.telemetry = payload

    def on_costmap(self, msg: String) -> None:
        payload = loads(msg.data, {})
        if not isinstance(payload, dict):
            return
        cells = payload.get("cells", [])
        if isinstance(cells, list):
            self.costmap_cells = cells[:20_000]
        self.costmap_meta = {
            "size_m": payload.get("size_m", 30.0),
            "cell_m": payload.get("cell_m", 0.25),
            "n": payload.get("n", 120),
        }

    def on_score(self, msg: String) -> None:
        payload = loads(msg.data, {})
        self.score = payload if isinstance(payload, dict) else {}

    def on_autonomy(self, msg: String) -> None:
        payload = loads(msg.data, {})
        self.autonomy = payload if isinstance(payload, dict) else {}

    def on_command(self, msg: Twist) -> None:
        self.command = {
            "vx": float(msg.linear.x),
            "vy": float(msg.linear.y),
            "yaw_rate": float(msg.angular.z),
        }

    def tick(self) -> None:
        if self._writer is None:
            return
        try:
            frame = render_costmap_frame(
                stamp=self.now_seconds(),
                telemetry=self.telemetry,
                autonomy=self.autonomy,
                command=self.command,
                cells=self.costmap_cells,
                costmap_meta=self.costmap_meta,
                obstacles=self.obstacles,
                buoys=self.buoys,
                score=self.score,
            )
            self._writer.write(frame)
        except Exception as exc:  # Keep the other two referee loggers alive.
            self.get_logger().error(f"map.mp4 kare yazimi basarisiz: {exc}")

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def destroy_node(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


