#!/usr/bin/env python3
"""B0495 kamera canlı yayın + v4l2 ayar + costmap görselleştirme sunucusu.

Jetson Orin üzerinde çalışır:
  - GStreamer pipeline ile kameradan MJPEG alır (doğrudan geçiş, minimum gecikme).
  - Tarayıcıya multipart/x-mixed-replace HTTP yayını yapar (/video).
  - v4l2-ctl ile kamera kontrollerini listeler ve ayarlar (/api/ctrls, /api/ctrl).
  - ROS2 /planning/costmap topic'ini dinler ve /costmap endpoint'inden JSON
    verir (lokal maliyet haritası görselleştirmesi; index.html canvas çizer).
    rclpy yoksa (dev makinesi) endpoint hata mesajı döndürür, sunucu yine açar.

Yarışma DIŞI bir geliştirme aracıdır: yarışma anında WiFi yasak olduğundan
src/ (yarışma stack'i) ile karışmaz, tools/ altında ayrı yaşar.
"""

import argparse
import json
import re
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

try:  # rclpy opsiyonel: dev makinesinde (Windows) yok — endpoint hata döner.
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from std_msgs.msg import String  # type: ignore

    RCLPY_AVAILABLE = True
except Exception:  # pragma: no cover - dev makinesinde rclpy olmayabilir
    RCLPY_AVAILABLE = False

try:  # GStreamer Python bindings (Jetson JetPack'te varsayılan)
    import gi

    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
except Exception:  # pragma: no cover - Jetson'da kurulu olmalı
    Gst = None

BOUNDARY = "ida-frame"
MJPEG_CAPS = "image/jpeg,width={w},height={h},framerate={fps}/1"
FALLBACK_RAW_CAPS = "video/x-raw,width={w},height={h}"

# B0495 varsayılanları (kullanıcı bildirimi):
#   USB 3.0 -> 1920x1080@50, 960x600@80 ; USB 2.0 -> 960x600@10
DEFAULT_PRESETS = [
    {"label": "1920x1080 @50 (USB3)", "width": 1920, "height": 1080, "fps": 50},
    {"label": "960x600 @80 (USB3)", "width": 960, "height": 600, "fps": 80},
    {"label": "960x600 @10 (USB2)", "width": 960, "height": 600, "fps": 10},
]


class CameraStream:
    """GStreamer appsink'ten en güncel MJPEG karesini tutar; çoklu /video okuyucuya verir."""

    def __init__(self, device: str, width: int, height: int, fps: int, port: int,
                 camera_source: str = "csi", flip_method: int = 2) -> None:
        self.device = device
        self.port = port
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.camera_source = camera_source   # "csi" (Jetson CSI, varsayılan) veya "usb"
        self.flip_method = int(flip_method) # CSI flip (2 = normal düz görüntü)
        self.lock = threading.Lock()
        self.latest: bytes | None = None
        self.frame_times: deque[float] = deque(maxlen=90)
        self.connected = False
        self._use_fallback = False
        self.pipeline = None
        self.appsink = None
        self._stop = threading.Event()

    # ---- pipeline yönetimi ----

    def _make_pipeline(self, w: int, h: int, f: int, force_fallback: bool = False) -> None:
        """Pipeline'ı kurar ve PLAYING'e alır. Önceki pipeline'ı serbest bırakır."""
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self.appsink = None

        if force_fallback or self._use_fallback:
            # Kamera MJPEG vermiyorsa: ham görüntüyü yeniden kodla (CPU yükü daha yüksek).
            pipeline_str = (
                f"v4l2src device={self.device} ! videoconvert ! videoscale ! "
                f"{FALLBACK_RAW_CAPS.format(w=w, h=h)} ! jpegenc quality=85 ! "
                f"appsink name=sink max-buffers=2 drop=true sync=false"
            )
            self._use_fallback = True
        elif self.camera_source == "csi":
            # Jetson CSI kamera: nvarguscamerasrc (donanımsal hızlandırma).
            # nvvidconv ile flip-method (2 = düz). BGR'a çevir → jpegenc → appsink.
            # jpegenc ŞART: /video MJPEG olarak yayınlar; ham BGR gönderilirse
            # tarayıcı görüntüyü gösteremez (image/jpeg bekler).
            # emit-signals=true + new-sample callback: pull_sample bloklamaz.
            pipeline_str = (
                f"nvarguscamerasrc sensor-id=0 ! "
                f"video/x-raw(memory:NVMM), width={w}, height={h}, "
                f"format=NV12, framerate={f}/1 ! "
                f"nvvidconv flip-method={self.flip_method} ! "
                f"video/x-raw, format=BGRx ! videoconvert ! "
                f"video/x-raw, format=BGR ! jpegenc quality=60 ! "
                f"appsink name=sink max-buffers=1 drop=true sync=false emit-signals=true"
            )
            self._use_fallback = False
        else:
            # USB kamera (B0495 MJPEG): v4l2src doğrudan JPEG kare üretir -> gecikme en düşük.
            pipeline_str = (
                f"v4l2src device={self.device} ! "
                f"{MJPEG_CAPS.format(w=w, h=h, fps=f)} ! "
                f"appsink name=sink max-buffers=2 drop=true sync=false emit-signals=true"
            )
            self._use_fallback = False

        self.width, self.height, self.fps = w, h, f
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except Exception as exc:
            print(f"[stream] pipeline kurulamadı: {exc}")
            raise
        self.appsink = self.pipeline.get_by_name("sink")
        # Callback tabanlı: new-sample sinyali frame geldiğinde tetiklenir.
        # pull_sample bloklamaz; GStreamer main loop'u frame'i iletir.
        self.appsink.set_property("emit-signals", True)
        self.appsink.connect("new-sample", self._on_sample)
        self.pipeline.set_state(Gst.State.PLAYING)

    def rebuild(self, width: int | None = None, height: int | None = None, fps: int | None = None) -> None:
        """Harici çağrı (örn. /video çözünürlük değişimi): pipeline'ı yeniden kurar."""
        self._make_pipeline(
            int(width or self.width), int(height or self.height), int(fps or self.fps)
        )
        with self.lock:
            self.latest = None
            self.frame_times.clear()
            self.connected = False

    def _restart_pipeline_self(self) -> None:
        """Grab thread içinden çağrılır (kendi thread'ini join etmez)."""
        try:
            self._make_pipeline(self.width, self.height, self.fps, force_fallback=True)
        except Exception as exc:
            print(f"[stream] yedek pipeline da kurulamadı: {exc}")
            with self.lock:
                self.connected = False

    def stop(self) -> None:
        self._stop.set()
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self.appsink = None

    # ---- kare yakalama (callback tabanlı) ----

    def _on_sample(self, sink) -> Gst.FlowReturn:
        """appsink 'new-sample' sinyali: frame geldiğinde GStreamer thread'inde çağrılır.

        Bloklamaz — frame hazır olduğu anda tetiklenir; en güncel kare saklanır.
        """
        try:
            sample = sink.emit("pull-sample")
            if sample is None:
                return Gst.FlowReturn.OK
            buf = sample.get_buffer()
            ok, info = buf.map(Gst.MapFlags.READ)
            if ok:
                now = time.monotonic()
                with self.lock:
                    self.latest = bytes(info.data)
                    self.frame_times.append(now)
                    self.connected = True
                buf.unmap(info)
        except Exception as exc:
            print(f"[stream] sample hatası: {exc}")
        return Gst.FlowReturn.OK

    # ---- durum ----

    def status(self) -> dict:
        with self.lock:
            now = time.monotonic()
            recent = [t for t in self.frame_times if now - t < 2.0]
            measured = len(recent) / 2.0 if recent else 0.0
            return {
                "device": self.device,
                "width": self.width,
                "height": self.height,
                "fps_requested": self.fps,
                "fps_measured": round(measured, 1),
                "connected": self.connected,
                "fallback": self._use_fallback,
                "port": self.port,
            }


# ---- v4l2-ctl yardımcıları (saf Python, subprocess) ----

CTRL_RE = re.compile(r"^\s*([\w_]+)\s+0x[0-9a-f]+\s+\((\w+)\)\s*:\s*(.*)$")


def read_ctrls(device: str) -> dict:
    """v4l2-ctl --list-ctrls çıktısını JSON'a çevirir. ['controls': [{name,type,min,max,step,default,value}]]"""
    try:
        out = subprocess.run(
            ["v4l2-ctl", "-d", device, "--list-ctrls"],
            capture_output=True, text=True, timeout=3,
        ).stdout
    except Exception:
        return {"controls": [], "error": "v4l2-ctl çalışmadı (v4l-utils kurulu mu?)"}
    controls = []
    for line in out.splitlines():
        m = CTRL_RE.match(line)
        if not m:
            continue
        name, ctype, tail = m.group(1), m.group(2), m.group(3)
        ctrl: dict = {"name": name, "type": ctype}
        # Değer alanları: int/bool kontrollerde "min=.. max=.. step=.. default=.. value=.."
        for key in ("min", "max", "step", "default", "value"):
            mm = re.search(rf"{key}=([-\d.]+)", tail)
            if mm:
                try:
                    ctrl[key] = int(float(mm.group(1)))
                except ValueError:
                    ctrl[key] = mm.group(1)
        # Menu türü: "Menu Items:" listesi sonraki satırlarda — topla
        if ctype == "menu":
            items: list[str] = []
            for sub in out.splitlines()[out.splitlines().index(line) + 1:]:
                sm = re.match(r"^\s*(\d+)\s*:\s*(.+)$", sub)
                if sm:
                    items.append(f"{sm.group(1)}:{sm.group(2).strip()}")
                else:
                    break
            ctrl["items"] = items
        controls.append(ctrl)
    return {"controls": controls}


def set_ctrl(device: str, name: str, value) -> tuple[bool, str]:
    try:
        r = subprocess.run(
            ["v4l2-ctl", "-d", device, "--set-ctrl", f"{name}={value}"],
            capture_output=True, text=True, timeout=3,
        )
        if r.returncode != 0:
            return False, (r.stderr.strip() or r.stdout.strip() or "hata")
        return True, "ok"
    except Exception as exc:
        return False, str(exc)


def read_caps(device: str) -> dict:
    """v4l2-ctl --list-formats-ext çıktısını JSON'a çevirir."""
    try:
        out = subprocess.run(
            ["v4l2-ctl", "-d", device, "--list-formats-ext"],
            capture_output=True, text=True, timeout=3,
        ).stdout
    except Exception:
        return {"formats": [], "error": "v4l2-ctl çalışmadı"}
    formats: list = []
    cur: dict | None = None
    for line in out.splitlines():
        m = re.search(r"Pixel Format:\s*'(\w+)'", line)
        if m:
            cur = {"pixel_format": m.group(1), "sizes": []}
            formats.append(cur)
            continue
        if cur is None:
            continue
        m = re.search(r"Size: Discrete (\d+)x(\d+)", line)
        if m:
            cur["sizes"].append(
                {"width": int(m.group(1)), "height": int(m.group(2)), "fps_list": []}
            )
            continue
        m = re.search(r"Interval: Discrete [\d.]+s \(([\d.]+) fps\)", line)
        if m and cur["sizes"]:
            cur["sizes"][-1]["fps_list"].append(float(m.group(1)))
    return {"formats": formats}


# ---- costmap bridge (ROS2 /planning/costmap -> HTTP) ----

class CostmapBridge:
    """/planning/costmap topic'ini dinleyip en güncel JSON'u tutar.

    rclpy yoksa (dev makinesi) ``available=False`` olur ve /costmap endpoint'i
    hata mesajı döndürür; sunucunun geri kalanı (kamera, v4l2) çalışmaya devam
    eder. rclpy spin'i ayrı bir thread'de döner; en güncel mesaj JSON olarak
    saklanır (topic 10 Hz — anlık harita, tampon gerekmez).
    """

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.latest_json: str = ""
        self.available = RCLPY_AVAILABLE
        self.error = ""
        self._node = None
        self._spin_thread: threading.Thread | None = None
        if not RCLPY_AVAILABLE:
            self.error = "rclpy yok (dev makinesi) — /planning/costmap dinlenemiyor"
            return
        try:
            rclpy.init()
            self._node = Node("camera_stream_costmap_bridge")
            self._node.create_subscription(String, "/planning/costmap", self.on_costmap, 10)
            self._spin_thread = threading.Thread(target=self._spin, daemon=True)
            self._spin_thread.start()
        except Exception as exc:  # pragma: no cover
            self.available = False
            self.error = str(exc)

    def _spin(self) -> None:
        try:
            rclpy.spin(self._node)  # type: ignore
        except Exception as exc:  # pragma: no cover
            with self.lock:
                self.available = False
                self.error = str(exc)

    def on_costmap(self, msg) -> None:
        with self.lock:
            self.latest_json = msg.data

    def snapshot(self) -> dict:
        with self.lock:
            if not self.available:
                return {"available": False, "error": self.error, "cells": []}
            try:
                payload = json.loads(self.latest_json)
            except Exception:
                return {"available": True, "cells": [], "raw": self.latest_json}
            return {"available": True, **payload}

    def shutdown(self) -> None:
        if self._node is not None:
            self._node.destroy_node()


# ---- HTTP sunucu ----

class Handler(BaseHTTPRequestHandler):
    stream: CameraStream | None = None
    costmap: CostmapBridge | None = None

    def log_message(self, *args):  # sessiz tut
        pass

    def _serve_json(self, obj: dict) -> None:
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def _serve_index(self) -> None:
        idx = Path(__file__).parent / "index.html"
        if not idx.exists():
            self.send_error(500, "index.html bulunamadı")
            return
        data = idx.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_video(self, query: dict) -> None:
        s = self.stream
        if s is None:
            self.send_error(500)
            return
        try:
            w = int(query.get("width", [s.width])[0])
            h = int(query.get("height", [s.height])[0])
            f = int(query.get("fps", [s.fps])[0])
        except (TypeError, ValueError):
            w, h, f = s.width, s.height, s.fps
        if (w, h, f) != (s.width, s.height, s.fps):
            try:
                s.rebuild(w, h, f)
            except Exception:
                self.send_error(500, "pipeline kurulamadı")
                return
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        last = None
        try:
            while not s._stop.is_set():
                with s.lock:
                    cur = s.latest
                if cur is not None and cur is not last:
                    last = cur
                    self.wfile.write(b"--" + BOUNDARY.encode() + b"\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(b"Content-Length: " + str(len(cur)).encode() + b"\r\n\r\n")
                    self.wfile.write(cur)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                else:
                    time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            self._serve_index()
        elif parsed.path == "/video":
            self._serve_video(parse_qs(parsed.query))
        elif parsed.path == "/api/status":
            self._serve_json(self.stream.status() if self.stream else {"error": "no stream"})
        elif parsed.path == "/api/ctrls":
            self._serve_json(read_ctrls(self.stream.device) if self.stream else {"error": "no stream"})
        elif parsed.path == "/api/caps":
            self._serve_json(read_caps(self.stream.device) if self.stream else {"error": "no stream"})
        elif parsed.path == "/costmap":
            # Lokal maliyet haritası (autonomy /planning/costmap topic'i).
            if self.costmap is not None:
                self._serve_json(self.costmap.snapshot())
            else:
                self._serve_json({"available": False, "error": "costmap bridge yok", "cells": []})
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path != "/api/ctrl" or self.stream is None:
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length) or b"{}")
            name = str(body.get("name", ""))
            value = body.get("value")
        except Exception:
            self._serve_json({"ok": False, "message": "geçersiz istek"})
            return
        if not name or value is None:
            self._serve_json({"ok": False, "message": "name ve value gerekli"})
            return
        ok, msg = set_ctrl(self.stream.device, name, value)
        self._serve_json({"ok": ok, "message": msg, "name": name, "value": value})


def main() -> None:
    ap = argparse.ArgumentParser(description="İDA kamera yayın + ayar sunucusu")
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--camera-source", default="csi",
                    help="'csi' (Jetson CSI, varsayılan) veya 'usb' (v4l2)")
    ap.add_argument("--flip-method", type=int, default=2,
                    help="CSI nvvidconv flip-method (2 = düz)")
    args = ap.parse_args()

    if Gst is None:
        print("HATA: GStreamer Python bindings yok.")
        print("Kurulum: sudo apt install python3-gi gir1.2-gstreamer-1.0 "
              "gstreamer1.0-plugins-good gstreamer1.0-plugins-bad")
        raise SystemExit(1)
    Gst.init(None)

    stream = CameraStream(
        args.device, args.width, args.height, args.fps, args.port,
        camera_source=args.camera_source, flip_method=args.flip_method,
    )
    try:
        stream.rebuild()
    except Exception:
        print(f"Kamera açılamadı: {args.device} (lsusb / v4l2-ctl --list-devices ile kontrol)")
        raise SystemExit(1)

    Handler.stream = stream
    costmap_bridge = CostmapBridge()
    Handler.costmap = costmap_bridge
    httpd = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    httpd.daemon_threads = True
    print(f"Sunucu: http://0.0.0.0:{args.port}  (kaynak: {args.camera_source}, flip: {args.flip_method})")
    print(f"  /         -> arayüz")
    print(f"  /video    -> MJPEG canlı akış")
    print(f"  /costmap  -> ROS2 /planning/costmap (rclpy: {costmap_bridge.available})")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        costmap_bridge.shutdown()
        stream.stop()


if __name__ == "__main__":
    main()
