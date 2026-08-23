#!/usr/bin/env python3
"""Passive camera/lidar-fusion costmap and DWA decision viewer.

This diagnostic intentionally has no ROS publisher, service, action, MAVLink or
actuation dependency.  It subscribes to the *shadow* fusion outputs, runs the
same pure CostMap/DwaPlanner classes used by autonomy and exposes a local HTTP
page.  The displayed command is a prediction only; it is never sent to a
vehicle command topic.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional

from ida_planning.costmap import CostMap
from ida_planning.dwa import DwaPlanner


MAX_MESSAGE_BYTES = 1_000_000
FRESHNESS_S = 1.0


class PassiveShadowPlanner:
    """Thread-safe, ROS-independent shadow planner state."""

    def __init__(self, freshness_s: float = FRESHNESS_S) -> None:
        self.freshness_s = float(freshness_s)
        self._lock = threading.Lock()
        self._payloads: Dict[str, Dict[str, Any]] = {}
        self._received: Dict[str, float] = {}
        # Keep this diagnostic bit-for-bit aligned with the production values
        # in src/ida_bringup/config/autonomy.yaml.  A smaller diagnostic
        # footprint can falsely label a collision course as safe.
        self._costmap = CostMap(
            size_m=30.0,
            cell_m=0.25,
            bot_radius_m=0.6,
            safety_m=1.0,
            safety_base_m=0.5,
            safety_max_m=10.0,
            reaction_time_s=1.0,
            max_decel_mps2=1.5,
        )
        self._planner = DwaPlanner(
            max_speed_mps=0.6,
            max_yaw_rate_deg_s=50.0,
            vx_steps=7,
            yaw_steps=17,
            sim_time_s=3.5,
            sim_step_s=0.25,
            accel_limits={
                "max_accel_mps2": 0.8,
                "max_decel_mps2": 1.2,
                "max_yaw_accel_deg_s2": 180.0,
            },
            w={
                "obstacle": 0.7,
                "heading": 1.8,
                "progress": 1.2,
                "speed": 0.6,
                "smooth": 0.3,
                "unknown": 0.6,
                "corridor": 1.6,
                "avoid": 1.0,
            },
            recovery_vx=0.35,
            recovery_yaw_deg_s=25.0,
            unknown_slow_vx=0.4,
            bot_radius_m=1.6,
            cell_m=0.25,
            n_cells=120,
        )

    def ingest(self, channel: str, raw: str, received_monotonic: Optional[float] = None) -> bool:
        if channel not in {"buoys", "obstacles", "status", "camera_raw", "lidar_raw"}:
            return False
        if not isinstance(raw, str) or len(raw.encode("utf-8")) > MAX_MESSAGE_BYTES:
            return False
        try:
            payload = json.loads(raw)
        except (TypeError, ValueError, json.JSONDecodeError):
            return False
        if not isinstance(payload, dict):
            return False
        now = time.monotonic() if received_monotonic is None else float(received_monotonic)
        if not math.isfinite(now):
            return False
        with self._lock:
            self._payloads[channel] = payload
            self._received[channel] = now
        return True

    def snapshot(self, now_monotonic: Optional[float] = None) -> Dict[str, Any]:
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        with self._lock:
            payloads = {key: dict(value) for key, value in self._payloads.items()}
            received = dict(self._received)

        ages = {
            key: (now - received[key]) if key in received else None
            for key in ("buoys", "obstacles", "status")
        }
        fresh = all(age is not None and 0.0 <= age <= self.freshness_s for age in ages.values())
        status = payloads.get("status", {})
        accepted = bool(status.get("accepted", False)) and status.get("source_health") == "ok"
        camera_fresh = bool(status.get("camera_fresh", False))
        lidar_fresh = bool(status.get("lidar_fresh", False))

        buoy_payload = payloads.get("buoys", {})
        obstacle_payload = payloads.get("obstacles", {})
        buoys = buoy_payload.get("detections", []) if isinstance(buoy_payload.get("detections", []), list) else []
        obstacles = (
            obstacle_payload.get("obstacles", [])
            if isinstance(obstacle_payload.get("obstacles", []), list)
            else []
        )
        buoys = [item for item in buoys if isinstance(item, dict)]
        obstacles = [item for item in obstacles if isinstance(item, dict)]

        # The bottle remains a hard obstacle because the fused lidar object is
        # present.  strict_corridor additionally protects orange center cells.
        self._costmap.update(
            obstacles=obstacles if fresh and lidar_fresh else [],
            buoys=buoys if fresh and camera_fresh else [],
            target_color="",
            ground_speed=0.0,
            strict_corridor=True,
        )
        grid = json.loads(self._costmap.to_json())

        command: Optional[Dict[str, Any]] = None
        if fresh and accepted:
            planned = self._planner.plan((12.0, 0.0), self._costmap, None)
            command = {
                "vx": round(float(planned.vx), 4),
                "yaw_rate": round(float(planned.yaw_rate), 4),
                "yaw_rate_deg_s": round(math.degrees(float(planned.yaw_rate)), 2),
                "action": str(planned.action),
                "score": round(float(planned.score), 4),
                "advisory_only": True,
            }

        orange = [item for item in buoys if str(item.get("color", "")).lower() == "orange"]
        hard = [item for item in obstacles if bool(item.get("hard_obstacle", False))]
        raw_camera_rows = payloads.get("camera_raw", {}).get("detections", [])
        raw_lidar_rows = payloads.get("lidar_raw", {}).get("clusters", [])
        if not isinstance(raw_camera_rows, list):
            raw_camera_rows = []
        if not isinstance(raw_lidar_rows, list):
            raw_lidar_rows = []
        camera_bearing_deg: Optional[float] = None
        orange_candidates = [
            item for item in raw_camera_rows
            if isinstance(item, dict) and str(item.get("color", "")).lower() == "orange"
        ]
        if orange_candidates:
            try:
                candidate_bearing = float(max(
                    orange_candidates, key=lambda item: float(item.get("confidence", 0.0))
                ).get("bearing_deg"))
                if math.isfinite(candidate_bearing):
                    camera_bearing_deg = candidate_bearing
            except (TypeError, ValueError):
                pass
        nearest_lidar_delta_deg: Optional[float] = None
        if camera_bearing_deg is not None:
            lidar_bearings = []
            for item in obstacles:
                try:
                    bearing = float(item.get("bearing_deg"))
                except (TypeError, ValueError):
                    continue
                if math.isfinite(bearing):
                    delta = (bearing - camera_bearing_deg + 180.0) % 360.0 - 180.0
                    lidar_bearings.append(abs(delta))
            if lidar_bearings:
                nearest_lidar_delta_deg = round(min(lidar_bearings), 2)
        orange_geometry: Optional[Dict[str, float]] = None
        if orange:
            try:
                primary = max(orange, key=lambda item: float(item.get("confidence", 0.0)))
                geometry = {
                    "distance_m": round(float(primary.get("distance")), 3),
                    "forward_m": round(float(primary.get("forward_m")), 3),
                    "lateral_right_m": round(float(primary.get("lateral_m")), 3),
                }
                if all(math.isfinite(value) for value in geometry.values()):
                    orange_geometry = geometry
            except (TypeError, ValueError):
                pass
        return {
            "stamp": time.time(),
            "fresh": fresh,
            "accepted": accepted,
            "source_health": status.get("source_health", "missing"),
            "camera_fresh": camera_fresh,
            "lidar_fresh": lidar_fresh,
            "dt_ms": status.get("dt_ms"),
            "matched_count": int(status.get("matched_count", 0) or 0),
            "orange_count": len(orange),
            "hard_obstacle_count": len(hard),
            "camera_detection_count": sum(isinstance(item, dict) for item in raw_camera_rows),
            "camera_orange_count": sum(
                isinstance(item, dict) and str(item.get("color", "")).lower() == "orange"
                for item in raw_camera_rows
            ),
            "lidar_cluster_count": sum(isinstance(item, dict) for item in raw_lidar_rows),
            "camera_bearing_deg": camera_bearing_deg,
            "nearest_lidar_delta_deg": nearest_lidar_delta_deg,
            "orange_geometry": orange_geometry,
            "ages_s": ages,
            "costmap": grid,
            "command": command,
            "warning": "SHADOW ONLY - motor/ARM/cmd_vel output is not created",
        }


HTML = r"""<!doctype html>
<html lang="tr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>IDA Pasif Fusion Costmap</title>
<style>
body{margin:0;background:#0c1220;color:#edf2ff;font:15px system-ui}main{max-width:1150px;margin:auto;padding:16px}
.bar,.card{background:#151d2e;border:1px solid #2a3855;border-radius:10px;padding:12px;margin-bottom:12px}
.bar{display:flex;gap:18px;align-items:center;flex-wrap:wrap}.ok{color:#63e6a5}.bad{color:#ff7b7b}
canvas{display:block;width:100%;max-width:900px;aspect-ratio:1;background:#07101c;border:1px solid #33466b;border-radius:8px}
input{width:220px}.legend span{margin-right:16px}.orange{color:#ff971f}.red{color:#ff5c5c}.cyan{color:#54d7ff}
#decision{font-size:17px}small{color:#aebbd4}
</style></head><body><main>
<div class="bar"><strong>Pasif Fusion / Costmap</strong><label>Yakınlaştırma <input id="zoom" type="range" min="1" max="8" step="0.25" value="2"></label><b id="zv">2× (±7.5 m)</b></div>
<div class="card" id="health">Veri bekleniyor…</div><canvas id="map" width="900" height="900"></canvas>
<div class="card" id="decision">Planner kararı bekleniyor…</div>
<div class="card legend"><span class="orange">■ Turuncu eşleşmiş duba</span><span class="red">■ Sert lidar engeli/şişirme</span><span class="cyan">➜ Shadow DWA kararı</span><br><small>Araç ortadadır; yukarı ileri, sağ taraf aracın sağıdır. Bu sayfa hiçbir motor komutu göndermez.</small></div>
</main><script>
const c=document.getElementById('map'),x=c.getContext('2d'),zoom=document.getElementById('zoom');let state=null;
zoom.oninput=()=>{document.getElementById('zv').textContent=zoom.value+'× (±'+(15/(+zoom.value)).toFixed(1)+' m)';draw()};
function draw(){if(!state)return;const W=c.width,H=c.height,z=+zoom.value,size=state.costmap.size_m||30,ppm=W/size*z,cx=W/2,cy=H/2;
 x.fillStyle='#07101c';x.fillRect(0,0,W,H);x.strokeStyle='#17273c';x.lineWidth=1;
 for(let m=-15;m<=15;m++){let px=cx+m*ppm;x.beginPath();x.moveTo(px,0);x.lineTo(px,H);x.stroke();let py=cy+m*ppm;x.beginPath();x.moveTo(0,py);x.lineTo(W,py);x.stroke()}
 for(const cell of state.costmap.cells||[]){const [f,l,cost,tag]=cell,px=cx+l*ppm,py=cy-f*ppm,s=Math.max(3,(state.costmap.cell_m||.25)*ppm);if(tag==='orange'){x.fillStyle='#ff8c18';x.beginPath();x.arc(px,py,Math.max(8,s),0,Math.PI*2);x.fill();x.strokeStyle='#fff';x.lineWidth=2;x.stroke()}else{if(tag==='yellow')x.fillStyle='#ffe24a';else if(tag==='inflated')x.fillStyle='rgba(255,72,72,.34)';else x.fillStyle=cost>=8?'#ff4d5e':'#75849e';x.fillRect(px-s/2,py-s/2,s,s)}}
 if(state.camera_bearing_deg!==null){const a=state.camera_bearing_deg*Math.PI/180;x.save();x.setLineDash([12,8]);x.strokeStyle='#ff9b24';x.lineWidth=4;x.beginPath();x.moveTo(cx,cy);x.lineTo(cx+Math.sin(a)*220,cy-Math.cos(a)*220);x.stroke();x.restore()}
 x.save();x.translate(cx,cy);x.fillStyle='#eaf2ff';x.beginPath();x.moveTo(0,-17);x.lineTo(12,14);x.lineTo(-12,14);x.closePath();x.fill();x.restore();
 if(state.command){const yaw=state.command.yaw_rate*2,len=90;x.strokeStyle='#54d7ff';x.fillStyle='#54d7ff';x.lineWidth=7;x.beginPath();x.moveTo(cx,cy);x.lineTo(cx+Math.sin(yaw)*len,cy-Math.cos(yaw)*len);x.stroke()}
 x.fillStyle='#b8c7df';x.font='18px system-ui';x.fillText('İLERİ',cx-25,25);x.fillText('SOL',12,cy);x.fillText('SAĞ',W-55,cy);
}
async function tick(){try{const r=await fetch('/api/state',{cache:'no-store'});state=await r.json();const good=state.fresh&&state.accepted,g=state.orange_geometry,geom=g?` · orange konum: ileri ${g.forward_m} m / sağ ${g.lateral_right_m} m`:'';document.getElementById('health').innerHTML=`<b class="${good?'ok':'bad'}">${good?'FUSION KABUL':'FUSION BEKLE/RED'}</b> · kaynak: ${state.source_health} · kamera orange: ${state.camera_orange_count}/${state.camera_detection_count} · lidar küme: ${state.lidar_cluster_count} · en yakın açı farkı: ${state.nearest_lidar_delta_deg??'-'}° · eşleşme: ${state.matched_count} · fused orange: ${state.orange_count}${geom} · hard obstacle: ${state.hard_obstacle_count} · Δt: ${state.dt_ms??'-'} ms`;document.getElementById('decision').innerHTML=state.command?`<b>Shadow karar:</b> vx=${state.command.vx} m/s, dönüş=${state.command.yaw_rate_deg_s}°/s — ${state.command.action}<br><small>Üretim footprint yarıçapı 1.6 m ve 3.5 s öngörü kullanıldı. Yalnız hesaplandı; yayınlanmadı.</small>`:'Planner komutu üretilmiyor: fusion henüz taze ve kabul edilmiş değil.';draw()}catch(e){document.getElementById('health').innerHTML='<b class="bad">Bağlantı yok</b>'}}
setInterval(tick,250);tick();
</script></body></html>"""


def make_handler(state: PassiveShadowPlanner):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/api/state":
                body = json.dumps(state.snapshot(), separators=(",", ":")).encode()
                content_type = "application/json"
            elif self.path in {"/", "/index.html"}:
                body = HTML.encode()
                content_type = "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format: str, *_args: Any) -> None:
            return

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description="Passive shadow fusion costmap viewer")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8091)
    parser.add_argument("--buoys-topic", default="/perception/fusion/shadow/buoys")
    parser.add_argument("--obstacles-topic", default="/perception/fusion/shadow/obstacles")
    args = parser.parse_args()

    # ROS imports are deliberately local: pure state/unit tests do not need ROS.
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    state = PassiveShadowPlanner()

    class PassiveNode(Node):
        def __init__(self) -> None:
            super().__init__("ida_passive_costmap_viewer")
            self.create_subscription(String, args.buoys_topic, lambda m: state.ingest("buoys", m.data), 20)
            self.create_subscription(String, args.obstacles_topic, lambda m: state.ingest("obstacles", m.data), 20)
            self.create_subscription(String, "/perception/fusion/status", lambda m: state.ingest("status", m.data), 20)
            self.create_subscription(String, "/perception/camera/p1p2/raw", lambda m: state.ingest("camera_raw", m.data), 20)
            self.create_subscription(String, "/perception/lidar/raw_obstacles", lambda m: state.ingest("lidar_raw", m.data), 20)

    server = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"Passive viewer: http://{args.host}:{args.port} (no ROS publishers)", flush=True)
    rclpy.init()
    node = PassiveNode()
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
