"""Async websocket runtime isolated from ROS callbacks and optional dependency."""

from __future__ import annotations

import asyncio
import json
import math
import re
import socket
import threading
import time
from collections import deque
from typing import Any, Callable, Dict

from .contracts import bounded_json_loads
from .producer_contract import BoundedOutbox, DebugSnapshotComposer, decode_vehicle_test_command, encode_envelope

TOKEN_HEADER = "x-ezel-jetson-debug-token"
MIN_WEBSOCKETS_VERSION = (9, 1)
MAX_WEBSOCKETS_VERSION = (14, 0)
DISCOVERY_MAGIC = "EZEL_YKI_DEBUG_V1"
DISCOVERY_PORT = 50555


async def discover_yki_url(timeout_s: float = 3.0) -> str:
    """Discover the YKI backend from its LAN broadcast without a fixed IP."""
    loop = asyncio.get_running_loop()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setblocking(False)
    try:
        sock.bind(("0.0.0.0", DISCOVERY_PORT))
        raw, source = await asyncio.wait_for(loop.sock_recvfrom(sock, 2048), timeout_s)
        payload = json.loads(raw.decode("utf-8"))
        if (
            not isinstance(payload, dict)
            or payload.get("magic") != DISCOVERY_MAGIC
            or payload.get("path") != "/ws/jetson-debug"
            or isinstance(payload.get("port"), bool)
            or not isinstance(payload.get("port"), int)
            or not 1 <= payload["port"] <= 65535
        ):
            raise ValueError("invalid YKI discovery beacon")
        return f"ws://{source[0]}:{payload['port']}{payload['path']}"
    finally:
        sock.close()


def supported_websockets_version(raw: Any) -> bool:
    """Return whether the installed API matches the tested producer range.

    Ubuntu 22.04 / JetPack provides websockets 9.1.  The producer deliberately
    uses only the stable ``connect``, async-iterator and ``send`` APIs shared by
    9.1 through 13.x, so the system package is sufficient and no field venv is
    required.
    """
    if not isinstance(raw, str):
        return False
    match = re.match(r"^(\d+)\.(\d+)(?:\.|$)", raw.strip())
    if match is None:
        return False
    version = (int(match.group(1)), int(match.group(2)))
    return MIN_WEBSOCKETS_VERSION <= version < MAX_WEBSOCKETS_VERSION


class ProducerRuntime:
    """Bounded cross-thread state. Only the backend may replay unacked commands."""

    def __init__(self, composer: DebugSnapshotComposer, outbox_limit: int = 128, inbox_limit: int = 32) -> None:
        if not 1 <= inbox_limit <= 256:
            raise ValueError("inbox limit out of range")
        self.composer = composer
        self.outbox = BoundedOutbox(outbox_limit)
        self._inbox: deque[tuple[Dict[str, Any], Dict[str, Any]]] = deque()
        self._inbox_limit = inbox_limit
        self._lock = threading.Lock()
        self._connected = False
        self._dependency_available = True
        self._last_error = ""
        self._reconnects = 0
        self._last_rx_mono: float | None = None
        self._last_tx_mono: float | None = None

    def receive_server_message(self, raw: str) -> bool:
        envelope = bounded_json_loads(raw, 16384, 256, 8)
        if isinstance(envelope, dict) and envelope.get("type") == "vehicle_test_upstream_ack":
            if set(envelope) != {"type", "data"} or not isinstance(envelope.get("data"), dict):
                raise ValueError("invalid upstream ACK envelope")
            data = envelope["data"]
            if set(data) != {"topic", "run_id", "seq", "accepted"}:
                raise ValueError("invalid upstream ACK fields")
            if data["topic"] != "/vehicle_test/result" or data["accepted"] is not True:
                raise ValueError("unsupported upstream ACK")
            if not isinstance(data["run_id"], str) or not data["run_id"] or isinstance(data["seq"], bool) or not isinstance(data["seq"], int) or data["seq"] < 0:
                raise ValueError("invalid upstream ACK identity")
            if not self.outbox.acknowledge_terminal(data["run_id"], data["seq"]):
                raise ValueError("upstream ACK does not match a retained terminal result")
            with self._lock:
                self._last_rx_mono = time.monotonic()
            return True
        request, ack = decode_vehicle_test_command(raw)
        with self._lock:
            if len(self._inbox) >= self._inbox_limit:
                ack = dict(ack)
                ack["data"] = {**ack["data"], "accepted": False}
                self.outbox.put(ack)
                return False
            self._inbox.append((request, ack))
            self._last_rx_mono = time.monotonic()
        return True

    def drain_commands(self, limit: int = 4) -> list[tuple[Dict[str, Any], Dict[str, Any]]]:
        result = []
        with self._lock:
            for _ in range(min(limit, len(self._inbox))):
                result.append(self._inbox.popleft())
        return result

    def set_connection(self, connected: bool, error: str = "") -> None:
        with self._lock:
            if connected and not self._connected:
                self._reconnects += 1
            self._connected = connected
            self._last_error = str(error)[:160]

    def set_dependency_missing(self, error: str) -> None:
        with self._lock:
            self._dependency_available = False
            self._connected = False
            self._last_error = str(error)[:160]

    def note_tx(self) -> None:
        with self._lock:
            self._last_tx_mono = time.monotonic()

    def health(self, now_mono: float) -> Dict[str, Any]:
        with self._lock:
            return {
                "schema_version": 1,
                "connected": self._connected,
                "dependency_available": self._dependency_available,
                "reconnect_count": self._reconnects,
                "pending_commands": len(self._inbox),
                "pending_upstream": len(self.outbox),
                "last_receive_age_s": None if self._last_rx_mono is None else max(0.0, now_mono - self._last_rx_mono),
                "last_send_age_s": None if self._last_tx_mono is None else max(0.0, now_mono - self._last_tx_mono),
                "error": self._last_error,
                "actuation_enabled": False,
            }


def deliver_pending_commands(
    runtime: ProducerRuntime, subscriber_count: int,
    publish: Callable[[Dict[str, Any]], None], limit: int = 4,
) -> int:
    """Publish-before-ACK handshake; no monitor means commands remain queued."""
    if subscriber_count <= 0:
        return 0
    delivered = 0
    for request, ack in runtime.drain_commands(limit):
        publish(request)
        runtime.outbox.put(ack)
        delivered += 1
    return delivered


async def connected_session(
    websocket: Any,
    runtime: ProducerRuntime,
    snapshot_hz: float,
    epoch: Callable[[], float] = time.time,
    monotonic: Callable[[], float] = time.monotonic,
) -> None:
    """Run exactly one receive loop and one sender for an established socket."""
    if not math.isfinite(snapshot_hz) or not 2.0 <= snapshot_hz <= 5.0:
        raise ValueError("snapshot_hz must be between 2 and 5")
    stopped = asyncio.Event()

    async def receive() -> None:
        try:
            async for raw in websocket:
                if not isinstance(raw, str) or len(raw.encode("utf-8")) > 16384:
                    continue
                try:
                    runtime.receive_server_message(raw)
                except (TypeError, ValueError, json.JSONDecodeError):
                    continue
        finally:
            stopped.set()

    async def send() -> None:
        next_snapshot = monotonic()
        period = 1.0 / snapshot_hz
        while not stopped.is_set():
            envelope = runtime.outbox.peek()
            if envelope is not None:
                await websocket.send(encode_envelope(envelope))
                runtime.outbox.mark_sent(envelope)
                runtime.note_tx()
            now = monotonic()
            if now >= next_snapshot:
                snapshot = {"type": "debug_snapshot", "data": runtime.composer.compose(epoch(), now)}
                await websocket.send(encode_envelope(snapshot))
                runtime.note_tx()
                next_snapshot = now + period
            try:
                await asyncio.wait_for(stopped.wait(), timeout=0.02)
            except asyncio.TimeoutError:
                pass

    receiver = asyncio.create_task(receive())
    sender = asyncio.create_task(send())
    tasks = {receiver, sender}
    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    except BaseException:
        stopped.set()
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        raise
    stopped.set()
    for task in tasks:
        if not task.done():
            task.cancel()
    results = await asyncio.gather(*tasks, return_exceptions=True)
    # Cancellation is expected cleanup; any real receiver/sender/composer error
    # belongs to the outer reconnect loop and must be visible in health.
    for result in results:
        if isinstance(result, BaseException) and not isinstance(result, asyncio.CancelledError):
            raise result


async def connection_loop(
    connect: Callable[..., Any], url: str, token: str, runtime: ProducerRuntime,
    snapshot_hz: float, initial_backoff_s: float, max_backoff_s: float,
) -> None:
    """Single-owner reconnect loop. Commands are never replayed locally."""
    backoff = initial_backoff_s
    while True:
        try:
            effective_url = (
                await discover_yki_url()
                if url == "auto://yki"
                else url
            )
            try:
                context = connect(effective_url, extra_headers={TOKEN_HEADER: token}, max_size=1048576)
            except TypeError:
                context = connect(effective_url, additional_headers={TOKEN_HEADER: token}, max_size=1048576)
            async with context as websocket:
                runtime.outbox.replay_unacked_terminals()
                runtime.set_connection(True)
                backoff = initial_backoff_s
                try:
                    await connected_session(websocket, runtime, snapshot_hz)
                finally:
                    runtime.set_connection(False, "disconnected")
        except asyncio.CancelledError:
            runtime.set_connection(False, "stopped")
            raise
        except Exception as exc:  # network boundary, health exposes bounded reason
            runtime.set_connection(False, f"{type(exc).__name__}:{exc}")
        await asyncio.sleep(backoff)
        backoff = min(max_backoff_s, backoff * 2.0)
