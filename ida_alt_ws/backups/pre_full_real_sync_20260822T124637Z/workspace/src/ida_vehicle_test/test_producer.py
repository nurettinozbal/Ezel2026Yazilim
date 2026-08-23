import asyncio
import importlib.util
import json
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import AsyncMock, patch

PACKAGE = Path(__file__).resolve().parent
sys.path.insert(0, str(PACKAGE))

from ida_vehicle_test.contracts import ALLOWED_TESTS, compact_json
from ida_vehicle_test.producer_contract import (
    BoundedOutbox, DebugSnapshotComposer, decode_vehicle_test_command,
    vehicle_test_ros_envelope,
)
from ida_vehicle_test.producer_runtime import (
    ProducerRuntime, TOKEN_HEADER, connected_session, connection_loop,
    deliver_pending_commands, supported_websockets_version,
)


def _start_request(run_id="run-1", seq=1):
    return {
        "action": "start", "run_id": run_id, "seq": seq,
        "test": "telemetry", "timeout_s": 5.0,
    }


class ProducerContractTests(unittest.TestCase):
    def _composer(self, stamp=100.0):
        composer = DebugSnapshotComposer(freshness_s=1.0, max_lidar_points=2)
        values = {
            "camera_p1p2": {"acquisition_stamp": stamp, "detections": [
                {"id": "cam-1", "color": "orange", "bearing_deg": 30, "confidence": 0.9}
            ]},
            "camera_p3": {"acquisition_stamp": stamp, "detections": []},
            "lidar": {"acquisition_stamp": stamp, "clusters": [
                {"id": "lidar-1", "forward_m": 5, "lateral_left_m": 1},
                {"id": "lidar-2", "forward_m": 6, "lateral_left_m": -2},
                {"id": "lidar-3", "forward_m": 7, "lateral_left_m": 0},
            ], "points": [
                {"forward_m": 4.0, "lateral_left_m": -0.5},
                {"forward_m": 4.1, "lateral_left_m": 0.5},
                {"forward_m": 4.2, "lateral_left_m": 0.0},
            ]},
            "fusion_status": {
                "stamp": stamp, "accepted": True, "source_health": "ok",
                "camera_fresh": True, "lidar_fresh": True, "shadow_mode": False,
            },
            "canonical_buoys": {"stamp": stamp, "detections": [
                {"lidar_obstacle_id": "lidar-1", "forward_m": 5, "lateral_m": -1, "color": "orange"}
            ]},
            "canonical_obstacles": {"stamp": stamp, "obstacles": [
                {"id": "lidar-1", "forward_m": 5, "lateral_m": -1, "color": "unknown"}
            ]},
            "autonomy_state": {"stamp": stamp, "state": "PARKUR_1_NAV", "current_waypoint": 2, "failsafe_reason": ""},
            "autonomy_debug": {"stamp": stamp, "command": {"vx": 0.6, "yaw_rate": 0.2, "action": "dwa"}},
        }
        for channel, payload in values.items():
            self.assertTrue(composer.ingest_json(channel, compact_json(payload), 10.0))
        return composer

    def test_snapshot_strict_contract_sign_and_bounds(self):
        snapshot = self._composer().compose(100.2, 10.2)
        self.assertEqual(set(snapshot), {"schema_version", "source_stamp", "frame_id", "lidar", "camera", "fusion", "autonomy", "chosen_command"})
        self.assertEqual(snapshot["lidar"]["clusters"][0]["lateral_right_m"], -1.0)
        self.assertEqual(len(snapshot["lidar"]["points"]), 2)
        self.assertEqual(snapshot["lidar"]["points"][0]["lateral_right_m"], 0.5)
        self.assertAlmostEqual(snapshot["camera"]["bearing_rad"], math.pi / 6)
        self.assertNotIn("image", compact_json(snapshot).lower())
        self.assertNotIn("video", compact_json(snapshot).lower())

        backend = PACKAGE.parents[1] / "ezel-yazilim_yeni" / "ezel-yazilim" / "arayuz" / "backend" / "services" / "lab_debug.py"
        spec = importlib.util.spec_from_file_location("yki_lab_debug_contract", backend)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader
        spec.loader.exec_module(module)
        normalized = module.normalize_snapshot(snapshot, now_epoch=100.2, now_monotonic=10.2)
        self.assertEqual(normalized["frame_id"], "base_link")

    def test_source_and_receive_freshness_fail_closed(self):
        composer = self._composer(stamp=98.0)
        snapshot = composer.compose(100.2, 10.2)
        self.assertEqual(snapshot["lidar"]["status"], "stale")
        self.assertEqual(snapshot["fusion"]["status"], "stale")
        self.assertEqual(snapshot["autonomy"]["status"], "stale")
        composer.ingest_json("lidar", "not-json", 10.2)
        self.assertEqual(composer.compose(100.2, 10.2)["lidar"]["status"], "error")

    def test_fresh_malformed_channel_shapes_are_errors(self):
        composer = self._composer()
        composer.ingest_json("camera_p1p2", compact_json({
            "acquisition_stamp": 100.0, "detections": {"bearing_deg": 0, "confidence": 1}
        }), 10.1)
        self.assertEqual(composer.compose(100.2, 10.2)["camera"]["status"], "error")

        composer = self._composer()
        composer.ingest_json("lidar", compact_json({
            "acquisition_stamp": 100.0, "clusters": {"id": "bad"}
        }), 10.1)
        self.assertEqual(composer.compose(100.2, 10.2)["lidar"]["status"], "error")

        for channel, malformed in (
            ("autonomy_state", {"stamp": 100.0, "current_waypoint": 1}),
            ("autonomy_state", {"stamp": 100.0, "state": "PARKUR_1_NAV"}),
            ("autonomy_debug", {"stamp": 100.0, "command": {"vx": 0.5, "yaw_rate": 0.1}}),
            ("autonomy_debug", {"stamp": 100.0, "command": []}),
        ):
            composer = self._composer()
            composer.ingest_json(channel, compact_json(malformed), 10.1)
            snapshot = composer.compose(100.2, 10.2)
            self.assertEqual(snapshot["autonomy"]["status"], "error")
            if channel == "autonomy_debug":
                self.assertEqual(snapshot["chosen_command"]["status"], "error")

    def test_nonfinite_or_invalid_camera_rows_are_errors(self):
        for detection in (
            {"bearing_deg": "NaN", "confidence": 0.9},
            {"bearing_deg": 181, "confidence": 0.9},
            {"bearing_deg": 0, "confidence": 1.1},
            {"bearing_deg": 0},
        ):
            composer = self._composer()
            composer.ingest_json("camera_p1p2", compact_json({
                "acquisition_stamp": 100.0, "detections": [detection]
            }), 10.1)
            self.assertEqual(composer.compose(100.2, 10.2)["camera"]["status"], "error")

    def test_malformed_lidar_and_fusion_entries_are_errors(self):
        composer = self._composer()
        composer.ingest_json("lidar", compact_json({
            "acquisition_stamp": 100.0, "clusters": ["not-an-object"]
        }), 10.1)
        self.assertEqual(composer.compose(100.2, 10.2)["lidar"]["status"], "error")

        composer = self._composer()
        composer.ingest_json("canonical_buoys", compact_json({
            "stamp": 100.0, "detections": ["not-an-object"]
        }), 10.1)
        self.assertEqual(composer.compose(100.2, 10.2)["fusion"]["status"], "error")

        composer = self._composer()
        composer.ingest_json("canonical_buoys", compact_json({
            "stamp": 100.0, "detections": [
                {"id": "dup", "forward_m": 1, "lateral_m": 0, "color": "orange"},
                {"id": "dup", "forward_m": 2, "lateral_m": 0, "color": "orange"},
            ]
        }), 10.1)
        fusion = composer.compose(100.2, 10.2)["fusion"]
        self.assertEqual(fusion["status"], "error")
        self.assertFalse(any(item["id"] == "dup" for item in fusion["objects"]))

    def test_fusion_status_semantics_are_required(self):
        invalid = (
            {"stamp": 100.0, "accepted": True},
            {"stamp": 100.0, "accepted": False, "source_health": "ok", "camera_fresh": True, "lidar_fresh": True, "shadow_mode": False},
            {"stamp": 100.0, "accepted": True, "source_health": "camera_stale", "camera_fresh": False, "lidar_fresh": True, "shadow_mode": False},
        )
        for status in invalid:
            composer = self._composer()
            composer.ingest_json("fusion_status", compact_json(status), 10.1)
            self.assertNotEqual(composer.compose(100.2, 10.2)["fusion"]["status"], "ok")

    def test_duplicate_lidar_ids_suppressed(self):
        composer = DebugSnapshotComposer()
        raw = {"acquisition_stamp": 100, "clusters": [
            {"id": "x", "forward_m": 1, "lateral_left_m": 1},
            {"id": "x", "forward_m": 2, "lateral_left_m": 2},
        ]}
        composer.ingest_json("lidar", compact_json(raw), 1.0)
        result = composer.compose(100.1, 1.1)["lidar"]
        self.assertEqual(result["status"], "error")
        self.assertEqual(result["clusters"], [])

    def test_command_decoder_exact_and_passive(self):
        request = _start_request()
        canonical, ack = decode_vehicle_test_command({"type": "vehicle_test_request", "data": request})
        self.assertEqual(canonical, request)
        self.assertEqual(ack["data"], {"action": "start", "run_id": "run-1", "seq": 1, "accepted": True})
        with self.assertRaises(ValueError):
            decode_vehicle_test_command({"type": "vehicle_test_request", "data": {"action": "arm", "run_id": "x", "seq": 1}})
        with self.assertRaises(ValueError):
            decode_vehicle_test_command({"type": "vehicle_test_request", "data": request, "extra": 1})

    def test_ros_forwarding_topics_are_exact(self):
        payload = {"schema_version": 1, "status": "PASS"}
        envelope = vehicle_test_ros_envelope("/vehicle_test/result", payload)
        self.assertIs(envelope["data"]["payload"], payload)
        with self.assertRaises(ValueError):
            vehicle_test_ros_envelope("/vehicle_test/metrics", payload)

    def test_outbox_requires_explicit_send_commit(self):
        box = BoundedOutbox(8)
        result = {"type": "vehicle_test_ack", "data": {"x": 1}}
        box.put(result)
        self.assertIs(box.peek(), result)
        self.assertIs(box.peek(), result)
        box.mark_sent(result)
        self.assertIsNone(box.peek())
        old = vehicle_test_ros_envelope("/vehicle_test/status", {"schema_version": 1, "n": 1})
        new = vehicle_test_ros_envelope("/vehicle_test/status", {"schema_version": 1, "n": 2})
        box.put(old); box.put(new)
        self.assertIs(box.peek(), new)

    def test_critical_ack_and_result_survive_event_saturation(self):
        box = BoundedOutbox(8)
        for index in range(8):
            self.assertTrue(box.put(vehicle_test_ros_envelope(
                "/vehicle_test/events",
                {"schema_version": 1, "run_id": "r", "seq": 1, "event": str(index)},
            )))
        ack = {"type": "vehicle_test_ack", "data": {"action": "start", "run_id": "r", "seq": 1, "accepted": True}}
        result = vehicle_test_ros_envelope(
            "/vehicle_test/result",
            {"schema_version": 1, "run_id": "r", "seq": 1, "state": "PASS"},
        )
        self.assertTrue(box.put(ack))
        self.assertTrue(box.put(result))
        self.assertIs(box.peek(), ack)
        box.mark_sent(ack)
        self.assertIs(box.peek(), result)
        self.assertTrue(box.put(result))

    def test_terminal_result_retained_until_application_ack_and_replayed(self):
        box = BoundedOutbox(8)
        result = vehicle_test_ros_envelope(
            "/vehicle_test/result",
            {"schema_version": 1, "run_id": "r", "seq": 7, "state": "PASS"},
        )
        box.put(result)
        box.mark_sent(result)
        self.assertIsNone(box.peek())
        self.assertEqual(len(box), 1)
        box.replay_unacked_terminals()
        self.assertIs(box.peek(), result)
        self.assertTrue(box.acknowledge_terminal("r", 7))
        self.assertEqual(len(box), 0)

    def test_upstream_ack_removes_only_matching_terminal(self):
        runtime = ProducerRuntime(DebugSnapshotComposer())
        result = vehicle_test_ros_envelope(
            "/vehicle_test/result",
            {"schema_version": 1, "run_id": "r", "seq": 7, "state": "PASS"},
        )
        runtime.outbox.put(result); runtime.outbox.mark_sent(result)
        ack = {"type": "vehicle_test_upstream_ack", "data": {
            "topic": "/vehicle_test/result", "run_id": "r", "seq": 7, "accepted": True,
        }}
        self.assertTrue(runtime.receive_server_message(compact_json(ack)))
        self.assertEqual(len(runtime.outbox), 0)
        with self.assertRaises(ValueError):
            runtime.receive_server_message(compact_json({**ack, "data": {**ack["data"], "accepted": False}}))

    def test_manifest_drift_against_yki_fixture(self):
        path = PACKAGE.parents[1] / "ezel-yazilim_yeni" / "ezel-yazilim" / "arayuz" / "contracts" / "ida_vehicle_test.v1.json"
        manifest = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual({item["id"] for item in manifest["tests"]}, set(ALLOWED_TESTS))
        self.assertEqual(manifest["actions"], ["start", "cancel"])


class FakeWebsocket:
    def __init__(self, messages):
        self.messages = list(messages)
        self.sent = []

    def __aiter__(self):
        return self

    async def __anext__(self):
        if self.messages:
            return self.messages.pop(0)
        await asyncio.sleep(0.06)
        raise StopAsyncIteration

    async def send(self, raw):
        self.sent.append(json.loads(raw))


class RuntimeTests(unittest.TestCase):
    def test_websockets_version_gate_is_fail_closed(self):
        for supported in ("9.1", "9.1.1", "10.4", "10.4.1", "12.0", "13.1"):
            with self.subTest(supported=supported):
                self.assertTrue(supported_websockets_version(supported))
        for unsupported in (None, "", "bad", "8.1", "9.0.9", "14.0", "15.0"):
            with self.subTest(unsupported=unsupported):
                self.assertFalse(supported_websockets_version(unsupported))

    def test_fake_websocket_receive_snapshot_and_ack_lifecycle(self):
        async def scenario():
            runtime = ProducerRuntime(DebugSnapshotComposer())
            ws = FakeWebsocket([compact_json({"type": "vehicle_test_request", "data": _start_request()})])
            await connected_session(ws, runtime, 2.0, epoch=lambda: 100.0, monotonic=lambda: 10.0)
            commands = runtime.drain_commands()
            self.assertEqual(commands[0][0], _start_request())
            # ACK is deliberately absent until ROS publishes the exact request.
            self.assertFalse(any(item["type"] == "vehicle_test_ack" for item in ws.sent))
            self.assertTrue(any(item["type"] == "debug_snapshot" for item in ws.sent))
        asyncio.run(scenario())

    def test_inbox_overflow_nacks_without_unbounded_growth(self):
        runtime = ProducerRuntime(DebugSnapshotComposer(), inbox_limit=1)
        self.assertTrue(runtime.receive_server_message(compact_json({"type": "vehicle_test_request", "data": _start_request("a", 1)})))
        self.assertFalse(runtime.receive_server_message(compact_json({"type": "vehicle_test_request", "data": _start_request("b", 2)})))
        self.assertEqual(runtime.health(100)["pending_commands"], 1)
        self.assertFalse(runtime.outbox.peek()["data"]["accepted"])

    def test_monitor_handshake_gates_publish_and_ack(self):
        runtime = ProducerRuntime(DebugSnapshotComposer())
        runtime.receive_server_message(compact_json({"type": "vehicle_test_request", "data": _start_request()}))
        published = []
        self.assertEqual(deliver_pending_commands(runtime, 0, published.append), 0)
        self.assertEqual(runtime.health(0)["pending_commands"], 1)
        self.assertIsNone(runtime.outbox.peek())
        self.assertEqual(deliver_pending_commands(runtime, 1, published.append), 1)
        self.assertEqual(published, [_start_request()])
        self.assertEqual(runtime.outbox.peek()["type"], "vehicle_test_ack")

    def test_external_session_cancel_cleans_child_tasks(self):
        class HangingSocket:
            def __aiter__(self): return self
            async def __anext__(self): await asyncio.Event().wait()
            async def send(self, _raw): await asyncio.Event().wait()
        async def scenario():
            task = asyncio.create_task(connected_session(HangingSocket(), ProducerRuntime(DebugSnapshotComposer()), 2.0))
            await asyncio.sleep(0)
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            pending = [item for item in asyncio.all_tasks() if item is not asyncio.current_task() and not item.done()]
            self.assertEqual(pending, [])
        asyncio.run(scenario())

    def test_connection_loop_uses_exact_token_header_and_single_owner(self):
        class HoldingSocket(FakeWebsocket):
            async def __anext__(self):
                if self.messages:
                    return self.messages.pop(0)
                await asyncio.Event().wait()

        class Context:
            active = 0
            maximum = 0
            def __init__(self, websocket):
                self.websocket = websocket
            async def __aenter__(self):
                Context.active += 1
                Context.maximum = max(Context.maximum, Context.active)
                return self.websocket
            async def __aexit__(self, *_):
                Context.active -= 1

        async def scenario():
            calls = []
            websocket = HoldingSocket([compact_json({"type": "vehicle_test_request", "data": _start_request()})])
            def connect(url, **kwargs):
                calls.append((url, kwargs))
                return Context(websocket)
            runtime = ProducerRuntime(DebugSnapshotComposer())
            task = asyncio.create_task(connection_loop(connect, "ws://yki/ws/jetson-debug", "secret", runtime, 2.0, 0.01, 0.02))
            for _ in range(100):
                if runtime.health(0)["pending_commands"]:
                    break
                await asyncio.sleep(0.001)
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            self.assertEqual(calls[0][1]["extra_headers"], {TOKEN_HEADER: "secret"})
            self.assertEqual(Context.maximum, 1)
            self.assertFalse(runtime.health(0)["connected"])
        asyncio.run(scenario())

    def test_connection_loop_auto_discovers_yki_without_fixed_ip(self):
        class HoldingSocket(FakeWebsocket):
            async def __anext__(self):
                await asyncio.Event().wait()

        class Context:
            async def __aenter__(self): return HoldingSocket([])
            async def __aexit__(self, *_): return None

        async def scenario():
            calls = []
            def connect(url, **kwargs):
                calls.append((url, kwargs))
                return Context()
            runtime = ProducerRuntime(DebugSnapshotComposer())
            with patch(
                "ida_vehicle_test.producer_runtime.discover_yki_url",
                new=AsyncMock(return_value="ws://192.168.1.10:5000/ws/jetson-debug"),
            ) as discover:
                task = asyncio.create_task(connection_loop(
                    connect, "auto://yki", "secret", runtime, 2.0, 0.1, 0.1,
                ))
                for _ in range(100):
                    if calls:
                        break
                    await asyncio.sleep(0.001)
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
                discover.assert_awaited()
            self.assertEqual(
                calls[0][0], "ws://192.168.1.10:5000/ws/jetson-debug"
            )
        asyncio.run(scenario())

    def test_graceful_close_marks_disconnected_then_reconnects(self):
        class GracefulSocket:
            def __aiter__(self):
                return self
            async def __anext__(self):
                raise StopAsyncIteration
            async def send(self, _raw):
                return None

        class Context:
            async def __aenter__(self):
                return GracefulSocket()
            async def __aexit__(self, *_):
                return None

        async def scenario():
            runtime = ProducerRuntime(DebugSnapshotComposer())
            task = asyncio.create_task(connection_loop(
                lambda *_args, **_kwargs: Context(), "ws://yki/ws/jetson-debug",
                "secret", runtime, 2.0, 0.03, 0.03,
            ))
            for _ in range(100):
                health = runtime.health(0)
                if health["reconnect_count"] == 1 and not health["connected"]:
                    break
                await asyncio.sleep(0.001)
            self.assertEqual(runtime.health(0)["reconnect_count"], 1)
            self.assertFalse(runtime.health(0)["connected"])
            self.assertEqual(runtime.health(0)["error"], "disconnected")
            for _ in range(100):
                if runtime.health(0)["reconnect_count"] >= 2:
                    break
                await asyncio.sleep(0.001)
            self.assertGreaterEqual(runtime.health(0)["reconnect_count"], 2)
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        asyncio.run(scenario())

    def test_send_failure_propagates_to_outer_health(self):
        class SendFailureSocket:
            def __aiter__(self):
                return self
            async def __anext__(self):
                await asyncio.Event().wait()
            async def send(self, _raw):
                raise RuntimeError("send_broken")

        class Context:
            async def __aenter__(self):
                return SendFailureSocket()
            async def __aexit__(self, *_):
                return None

        async def scenario():
            runtime = ProducerRuntime(DebugSnapshotComposer())
            with self.assertRaisesRegex(RuntimeError, "send_broken"):
                await connected_session(SendFailureSocket(), runtime, 2.0, epoch=lambda: 100.0, monotonic=lambda: 10.0)
            task = asyncio.create_task(connection_loop(
                lambda *_args, **_kwargs: Context(), "ws://yki/ws/jetson-debug",
                "secret", runtime, 2.0, 0.1, 0.1,
            ))
            for _ in range(100):
                health = runtime.health(0)
                if "send_broken" in health["error"]:
                    break
                await asyncio.sleep(0.001)
            health = runtime.health(0)
            self.assertFalse(health["connected"])
            self.assertIn("RuntimeError:send_broken", health["error"])
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        asyncio.run(scenario())

    def test_header_and_no_actuation_static_contract(self):
        producer = (PACKAGE / "ida_vehicle_test" / "producer_node.py").read_text(encoding="utf-8")
        runtime = (PACKAGE / "ida_vehicle_test" / "producer_runtime.py").read_text(encoding="utf-8")
        self.assertIn(TOKEN_HEADER, runtime)
        for forbidden in ("/cmd_vel", "arm_vehicle", "set_mode", "motor_pub"):
            self.assertNotIn(forbidden, producer)
        self.assertNotIn("import websockets\n", producer.split("def load_websockets", 1)[0])
        self.assertIn('qos = 20 if topic == "/vehicle_test/events" else _RETAINED_QOS', producer)
        monitor = (PACKAGE / "ida_vehicle_test" / "monitor_node.py").read_text(encoding="utf-8")
        self.assertIn('"/vehicle_test/events", 20', monitor)
        self.assertIn('"/vehicle_test/result", _STATUS_QOS', monitor)


if __name__ == "__main__":
    unittest.main()
