"""FastAPI entry point for EZEL GCS competition communications."""

from __future__ import annotations

import asyncio
import queue
import secrets
from contextlib import asynccontextmanager
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

import config
from links.mavlink_vehicle import MAVLinkVehicle
from services.command_gate import CommandGate
from services.logger import BackendLogger
from services.lab_debug import (
    InMemoryDebugAdapter, VehicleTestRelay, handle_producer_message,
    is_lab_authorized, producer_token_valid, validate_lab_config,
)
from services.mission_manager import MissionManager
from services.target_manager import VALID_COLORS, TargetManager
from services.telemetry_state import TelemetryState


logger = BackendLogger()
targets = TargetManager()
telemetry = TelemetryState(
    ida_sys_id=config.IDA_SYS_ID,
    iha_sys_id=config.IHA_SYS_ID,
    heartbeat_timeout=config.HEARTBEAT_TIMEOUT_SECONDS,
)

ida = MAVLinkVehicle(
    port=config.IDA_PORT,
    baudrate=config.BAUDRATE,
    expected_sys_id=config.IDA_SYS_ID,
    gcs_sys_id=config.GCS_SYS_ID,
    name="İDA",
    state_key="ida",
    telemetry_state=telemetry,
    logger=logger,
    emergency_action=config.EMERGENCY_IDA_ACTION,
    reconnect_delay=config.RECONNECT_DELAY_SECONDS,
    mission_timeout=config.MISSION_TIMEOUT_SECONDS,
    telemetry_rate_hz=config.TELEMETRY_RATE_HZ,
    command_ack_timeout=config.COMMAND_ACK_TIMEOUT_SECONDS,
    state_verify_timeout=config.STATE_VERIFY_TIMEOUT_SECONDS,
    auto_return_on_link_loss=config.AUTO_RETURN_ON_LINK_LOSS,
    return_home_mode=config.RETURN_HOME_MODE,
    companion_component_id=config.IDA_COMPANION_COMPONENT_ID,
)
iha = MAVLinkVehicle(
    port=config.IHA_PORT,
    baudrate=config.BAUDRATE,
    expected_sys_id=config.IHA_SYS_ID,
    gcs_sys_id=config.GCS_SYS_ID,
    name="İHA",
    state_key="iha",
    telemetry_state=telemetry,
    logger=logger,
    emergency_action=config.EMERGENCY_IHA_ACTION,
    reconnect_delay=config.RECONNECT_DELAY_SECONDS,
    mission_timeout=config.MISSION_TIMEOUT_SECONDS,
    telemetry_rate_hz=config.TELEMETRY_RATE_HZ,
    command_ack_timeout=config.COMMAND_ACK_TIMEOUT_SECONDS,
    state_verify_timeout=config.STATE_VERIFY_TIMEOUT_SECONDS,
    auto_return_on_link_loss=config.AUTO_RETURN_ON_LINK_LOSS,
    return_home_mode=config.RETURN_HOME_MODE,
)
missions = MissionManager(ida, logger)
command_gate = CommandGate(ida, iha, telemetry, missions, targets, logger)
lab_debug = InMemoryDebugAdapter()
vehicle_tests = VehicleTestRelay()


def handle_iha_target_detection(color: str) -> None:
    """İHA otonom hedef rengi bildirdiğinde çalışır (şartname §5.5.3.1).

    İHA'nın kendi üzerinde ürettiği tespit sonucu burada hedef olarak kilitlenir ve
    - görev henüz başlamadıysa - İDA'ya iletilir. Hedef zaten kilitliyse veya görev
    başladıysa TargetManager/CommandGate bu isteği zaten reddeder; şartname hedef
    bilgisinin İDA harekete başladıktan sonra aktarılmasını yasaklar.
    """
    ok, message = targets.lock_target({"color": color, "source": "IHA", "confidence": 1.0})
    if not ok:
        logger.system(f"İHA otonom hedef tespiti uygulanmadı: {message}", "WARNING")
        return

    logger.system(f"İHA otonom hedef tespiti kilitlendi: {color}", "SUCCESS")
    command_gate.execute("SEND_TARGET_TO_IDA")


iha.target_detection_handler = handle_iha_target_detection


def handle_ida_target_ack(color_code: int) -> None:
    ok, message = targets.confirm_vehicle_target(color_code)
    logger.system(message, "SUCCESS" if ok else "WARNING")


ida.target_ack_handler = handle_ida_target_ack


class WebSocketHub:
    def __init__(self) -> None:
        self.clients: set[WebSocket] = set()

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.clients.add(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        self.clients.discard(websocket)

    async def broadcast(self, message: dict[str, Any]) -> None:
        clients = list(self.clients)
        results = await asyncio.gather(
            *(self._send(client, message) for client in clients),
            return_exceptions=True,
        )
        dead_clients = [client for client, result in zip(clients, results) if result is False]
        for client in dead_clients:
            self.disconnect(client)

    @staticmethod
    async def _send(client: WebSocket, message: dict[str, Any]) -> bool:
        try:
            await asyncio.wait_for(client.send_json(message), timeout=0.25)
            return True
        except Exception:
            return False


hub = WebSocketHub()
jetson_debug_owner: WebSocket | None = None
jetson_debug_owner_lock = asyncio.Lock()


async def _deliver_vehicle_test_requests() -> None:
    """Deliver unacknowledged ROS requests to the sole producer, without claiming success."""
    owner = jetson_debug_owner
    if owner is None:
        return
    for request in vehicle_tests.pending_requests():
        vehicle_tests.mark_sent(request)
        try:
            await asyncio.wait_for(owner.send_json({"type": "vehicle_test_request", "data": request}), timeout=0.25)
        except Exception:
            vehicle_tests.release_inflight()
            return


def build_telemetry_snapshot() -> dict[str, Any]:
    target = targets.snapshot()
    mission = missions.snapshot()
    snapshot = telemetry.snapshot(target)
    snapshot["system"]["target_delivery_status"] = target["delivery_status"]
    snapshot["system"]["target_delivery_message"] = target["delivery_message"]
    snapshot["system"]["target_locked"] = target["is_locked"]
    snapshot["system"]["target_source"] = target["source"]
    snapshot["system"]["target_confidence"] = target["confidence"]
    snapshot["system"]["target_locked_at"] = target["locked_at"]
    snapshot["system"]["mission_started"] = target["mission_started"]
    snapshot["system"]["valid_target_colors"] = sorted(VALID_COLORS)
    snapshot["system"]["mission_uploaded"] = mission["has_uploaded_mission"]
    snapshot["system"]["mission_waypoint_count"] = mission["waypoint_count"]
    snapshot["system"]["command_auth_required"] = config.WS_AUTH_REQUIRED
    snapshot["system"]["command_auth_configured"] = bool(config.WS_AUTH_TOKEN)
    snapshot["system"]["auto_return_on_link_loss"] = config.AUTO_RETURN_ON_LINK_LOSS
    snapshot["system"]["return_home_mode"] = config.RETURN_HOME_MODE
    snapshot["system"]["lab_debug_enabled"] = config.LAB_DEBUG_ENABLED
    snapshot["system"]["jetson_debug_configured"] = bool(config.JETSON_DEBUG_TOKEN)
    return snapshot


async def telemetry_broadcast_loop() -> None:
    delay = 1 / max(config.TELEMETRY_RATE_HZ, 1)
    while True:
        snapshot = build_telemetry_snapshot()
        logger.telemetry(snapshot)
        await hub.broadcast({"type": "telemetry", "data": snapshot})
        await asyncio.sleep(delay)


async def log_broadcast_loop() -> None:
    while True:
        try:
            event = logger.events.get_nowait()
        except queue.Empty:
            await asyncio.sleep(0.1)
            continue
        await hub.broadcast(event)


@asynccontextmanager
async def lifespan(_: FastAPI):
    validate_lab_config(
        config.LAB_DEBUG_ENABLED, config.WS_AUTH_REQUIRED,
        config.WS_AUTH_TOKEN, config.JETSON_DEBUG_TOKEN,
    )
    ida.start()
    iha.start()
    logger.system("EZEL GCS backend başlatıldı", "SUCCESS")
    tasks = [
        asyncio.create_task(telemetry_broadcast_loop()),
        asyncio.create_task(log_broadcast_loop()),
    ]
    try:
        yield
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        ida.stop()
        iha.stop()
        logger.flush()
        logger.system("EZEL GCS backend durduruldu", "WARNING")


app = FastAPI(title="EZEL GCS Backend", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=config.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET"],
    allow_headers=["*"],
)


@app.get("/health")
def health() -> dict[str, Any]:
    snapshot = build_telemetry_snapshot()
    target = targets.snapshot()
    mission = missions.snapshot()
    return {
        "status": "ok",
        "ida_link": snapshot["system"]["ida_link"],
        "iha_link": snapshot["system"]["iha_link"],
        "target_locked": target["is_locked"],
        "target_color": target["target_color"],
        "mission_uploaded": mission["has_uploaded_mission"],
        "mission_waypoint_count": mission["waypoint_count"],
        "emergency_active": telemetry.emergency_active,
        "target_delivery_status": target["delivery_status"],
        "target_delivery_message": target["delivery_message"],
        "command_auth_required": config.WS_AUTH_REQUIRED,
        "command_auth_configured": bool(config.WS_AUTH_TOKEN),
        "auto_return_on_link_loss": config.AUTO_RETURN_ON_LINK_LOSS,
        "return_home_mode": config.RETURN_HOME_MODE,
        "lab_debug_enabled": config.LAB_DEBUG_ENABLED,
        "jetson_debug_configured": bool(config.JETSON_DEBUG_TOKEN),
    }


def _origin_allowed(websocket: WebSocket) -> bool:
    origin = websocket.headers.get("origin")
    return not origin or origin in config.ALLOWED_ORIGINS


def _token_valid(token: Any) -> bool:
    if not config.WS_AUTH_REQUIRED:
        return True
    if not config.WS_AUTH_TOKEN:
        return False
    return secrets.compare_digest(str(token or ""), config.WS_AUTH_TOKEN)


def _jetson_debug_token_valid(token: Any) -> bool:
    return producer_token_valid(token, config.JETSON_DEBUG_TOKEN, config.LAB_DEBUG_ENABLED)


async def _send_auth_status(websocket: WebSocket, authorized: bool, message: str) -> None:
    await websocket.send_json(
        {
            "type": "auth",
            "data": {
                "command_authorized": authorized,
                "auth_required": config.WS_AUTH_REQUIRED,
                "message": message,
            },
        }
    )


async def _handle_lab_message(
    websocket: WebSocket, message: dict[str, Any], command_authorized: bool
) -> bool:
    """Handle passive lab messages; return True when the type was consumed."""
    if message.get("type") != "vehicle_test":
        return False
    if not config.LAB_DEBUG_ENABLED:
        await websocket.send_json({
            "type": "vehicle_test",
            "data": {"schema_version": 1, "event": "rejected", "status": "debug_disabled"},
        })
        return True

    data = message.get("data", message.get("payload", {}))
    lab_authorized = is_lab_authorized(
        command_authorized, config.WS_AUTH_REQUIRED, config.WS_AUTH_TOKEN
    )
    action = data.get("action") if isinstance(data, dict) else None
    if not isinstance(action, str) or action not in {"start", "cancel"}:
        await websocket.send_json({
            "type": "vehicle_test",
            "data": {"schema_version": 1, "event": "rejected", "status": "invalid_action"},
        })
        return True
    expected_keys = {"action", "test_id", "timeout_s", "expectations"} if action == "start" else {"action", "run_id"}
    if not isinstance(data, dict) or set(data) != expected_keys:
        await websocket.send_json({
            "type": "vehicle_test",
            "data": {"schema_version": 1, "event": "rejected", "status": "invalid_request"},
        })
        return True
    if not lab_authorized:
        await websocket.send_json({
            "type": "vehicle_test",
            "data": {"schema_version": 1, "event": "rejected", "status": "unauthorized"},
        })
        return True
    try:
        if action == "start":
            event = vehicle_tests.start(
                str(data.get("test_id", "")), data.get("timeout_s"), data.get("expectations")
            )
            await websocket.send_json({"type": "vehicle_test", "data": event})
            await _deliver_vehicle_test_requests()
        else:
            run_id = str(data.get("run_id", ""))
            event = vehicle_tests.cancel(run_id)
            await websocket.send_json({"type": "vehicle_test", "data": event})
            await _deliver_vehicle_test_requests()
    except ValueError as exc:
        await websocket.send_json({"type": "vehicle_test", "data": {"schema_version": 1, "event": "rejected", "status": "invalid", "message": str(exc)}})
    return True


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    if not _origin_allowed(websocket):
        origin = websocket.headers.get("origin", "(none)")
        logger.system(f"WebSocket origin reddedildi: {origin}", "WARNING")
        await websocket.close(code=1008)
        return

    await hub.connect(websocket)
    logger.system("Frontend WebSocket bağlandı")
    command_authorized = not config.WS_AUTH_REQUIRED
    try:
        await websocket.send_json({"type": "telemetry", "data": build_telemetry_snapshot()})
        if config.WS_AUTH_REQUIRED and not config.WS_AUTH_TOKEN:
            await _send_auth_status(
                websocket,
                False,
                "Backend EZEL_WS_TOKEN ayarlı değil; araç komutları reddedilecek",
            )
            logger.system("WebSocket komut auth aktif fakat EZEL_WS_TOKEN ayarlı değil", "ERROR")
        else:
            await _send_auth_status(
                websocket,
                command_authorized,
                "Komut auth bekleniyor" if config.WS_AUTH_REQUIRED else "Komut auth devre dışı",
            )
        if config.LAB_DEBUG_ENABLED:
            await websocket.send_json({"type": "vehicle_test", "data": vehicle_tests.manifest()})
            latest_debug = lab_debug.display_snapshot()
            if latest_debug is not None:
                await websocket.send_json({"type": "debug_snapshot", "data": latest_debug})
        while True:
            message = await websocket.receive_json()
            if message.get("type") == "auth":
                command_authorized = _token_valid(message.get("token"))
                await _send_auth_status(
                    websocket,
                    command_authorized,
                    "Komut auth başarılı" if command_authorized else "Komut auth başarısız",
                )
                logger.system(
                    "Frontend WebSocket komut auth başarılı"
                    if command_authorized
                    else "Frontend WebSocket komut auth başarısız",
                    "SUCCESS" if command_authorized else "WARNING",
                )
                continue
            if await _handle_lab_message(websocket, message, command_authorized):
                continue
            if message.get("type") != "command":
                await websocket.send_json(
                    {
                        "type": "command_result",
                        "data": {"ok": False, "message": "Yalnız type=command kabul edilir", "status": "rejected"},
                    }
                )
                continue
            command = message.get("command", "")
            payload = message.get("payload", {})
            if not command_authorized:
                result = {
                    "ok": False,
                    "command": str(command or "").upper(),
                    "message": "WebSocket komut yetkisi yok; VITE_WS_TOKEN/EZEL_WS_TOKEN eşleşmeli",
                    "status": "unauthorized",
                }
                logger.command(result["command"], payload if isinstance(payload, dict) else {}, result)
                logger.system(f"KOMUT REDDEDİLDİ {result['command']}: {result['message']}", "WARNING")
                await websocket.send_json({"type": "command_result", "data": result})
                continue
            result = await asyncio.to_thread(command_gate.execute, command, payload)
            await websocket.send_json({"type": "command_result", "data": result})
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        logger.system(f"Frontend WebSocket hatası: {exc}", "ERROR")
    finally:
        hub.disconnect(websocket)
        logger.system("Frontend WebSocket ayrıldı", "WARNING")


@app.websocket("/ws/jetson-debug")
async def jetson_debug_endpoint(websocket: WebSocket) -> None:
    """Dedicated producer-only endpoint; browser command tokens are not accepted."""
    if not _jetson_debug_token_valid(websocket.headers.get("x-ezel-jetson-debug-token")):
        await websocket.close(code=1008)
        return
    global jetson_debug_owner
    async with jetson_debug_owner_lock:
        if jetson_debug_owner is not None:
            await websocket.close(code=1008)
            return
        await websocket.accept()
        jetson_debug_owner = websocket
    logger.system("Jetson debug monitor bağlandı")
    try:
        await _deliver_vehicle_test_requests()
        while True:
            message = await websocket.receive_json()
            try:
                message_type, data = await handle_producer_message(message, lab_debug, vehicle_tests)
                await hub.broadcast({"type": message_type, "data": data})
                if (
                    message.get("type") == "vehicle_test_ros"
                    and isinstance(message.get("data"), dict)
                    and message["data"].get("topic") == "/vehicle_test/result"
                ):
                    payload = message["data"].get("payload", {})
                    await websocket.send_json({
                        "type": "vehicle_test_upstream_ack",
                        "data": {
                            "topic": "/vehicle_test/result",
                            "run_id": payload.get("run_id"),
                            "seq": payload.get("seq"),
                            "accepted": True,
                        },
                    })
            except ValueError as exc:
                await websocket.send_json({"type": "debug_error", "data": {"status": "invalid", "message": str(exc)}})
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        logger.system(f"Jetson debug monitor hatası: {exc}", "ERROR")
    finally:
        vehicle_tests.release_inflight()
        async with jetson_debug_owner_lock:
            if jetson_debug_owner is websocket:
                jetson_debug_owner = None
        logger.system("Jetson debug monitor ayrıldı", "WARNING")
