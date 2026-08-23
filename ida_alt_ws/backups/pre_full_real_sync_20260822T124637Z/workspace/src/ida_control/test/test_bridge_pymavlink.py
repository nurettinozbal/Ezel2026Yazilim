"""pymavlink komut yolu mock testi: mavsdk_bridge_node._command_loop_pymavlink.

Bu test, node'un rclpy gerçek bağımlılığını sys.modules ile taklit eder
(mock deseni — ws_gateway DryRunLink gibi). Amacı:

1. ``_command_loop_pymavlink`` 1 iterasyon çalıştığında
   ``set_position_target_local_ned_send`` çağrıldığını,
   ``type_mask=0x05E7``, signed forward ``vx``, ``vy=0`` ve ``yaw_rate``
   değerlerini doğrulamak.
2. ``_set_guided_mode`` ``set_mode_apm("GUIDED")`` çağrısını doğrulamak.
3. Gönderim hatasında (bağlantı kopması) hatanın loglanıp bağlantının
   yeniden kurulduğunu doğrulamak (B1 — ``command_retry_s`` retry).
4. pymavlink bağlantısı başarısız olduğunda (port yok/pymavlink yok) telemetri
   task'larının YİNE başlatıldığını doğrulamak (B2 — mavsdk_task sıralaması).

rclpy kurulu olmayan dev makinesinde de çalışır (import mock'lanır).
"""

import asyncio
import json
import sys
import types
import unittest
from unittest import mock

# --- rclpy + geometry_msgs + std_msgs + pymavlink mock'ları (sys.modules) ------
# mavsdk_bridge_node import'u rclpy, geometry_msgs, std_msgs ister; bunlar gerçekte
# ROS2 paketleridir. _command_loop_pymavlink ayrıca `from pymavlink import mavutil`
# yapar (frame sabiti için) — pymavlink kurulu olmayabilir, sahte modül enjekte edilir.

_rclpy = types.ModuleType("rclpy")
_rclpy.ok = lambda: True  # noqa: E731 (sabit True — döngü 1 iterasyon)
_rclpy.init = lambda *a, **k: None
_rclpy.shutdown = lambda *a, **k: None

# rclpy.qos sahte modülü: waypoint_pub TRANSIENT_LOCAL (latch) testi için.
_rclpy_qos = types.ModuleType("rclpy.qos")


class _DurabilityPolicy:
    VOLATILE = 1
    TRANSIENT_LOCAL = 2


class _QoSProfile:
    def __init__(self, *args, **kwargs):
        self.depth = kwargs.get("depth")
        self.durability = kwargs.get("durability")


_rclpy_qos.DurabilityPolicy = _DurabilityPolicy
_rclpy_qos.QoSProfile = _QoSProfile
sys.modules["rclpy.qos"] = _rclpy_qos

_geometry_msgs = types.ModuleType("geometry_msgs")
_geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")


class _Twist:
    def __init__(self):
        self.linear = _Vector3()
        self.angular = _Vector3()


class _Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


_geometry_msgs_msg.Twist = _Twist
_geometry_msgs.msg = _geometry_msgs_msg

_std_msgs = types.ModuleType("std_msgs")
_std_msgs_msg = types.ModuleType("std_msgs.msg")


class _String:
    def __init__(self, data=""):
        self.data = data


_std_msgs_msg.String = _String
_std_msgs.msg = _std_msgs_msg

# pymavlink.mavutil sahte modülü (frame sabitleri + mavlink_connection).
# reconnect testlerinde gerçek _connect_pymavlink bunları kullanır;
# _FakeConn.mavlink_connection aynı imzayı karşılar.
_pymavlink = types.ModuleType("pymavlink")
_mavutil = types.ModuleType("pymavlink.mavutil")
_mavutil.MAV_FRAME_BODY_NED = 9
_mavutil.MAV_FRAME_LOCAL_NED = 8
# _set_guided_mode mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED kullanır.
_mavlink_mod = types.ModuleType("pymavlink.dialects.v20.common")
_mavlink_mod.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
_mavlink_mod.MAV_PARAM_TYPE_INT8 = 1  # RCMAP param_set_send için
_mavlink_mod.MAV_FRAME_LOCAL_NED = 1  # SET_POSITION_TARGET frame
_mavutil.mavlink = _mavlink_mod


def _fake_mavlink_connection(*args, **kwargs):
    return _FakeConn()


_mavutil.mavlink_connection = _fake_mavlink_connection
_pymavlink.mavutil = _mavutil
sys.modules["pymavlink"] = _pymavlink
sys.modules["pymavlink.mavutil"] = _mavutil

# mavsdk sahte modülü: mavsdk_task testi (B2) System() kurar, connection_state()'e
# `async for` yapar, sonra telemetri/poll task'larını başlatır. Dev makinesinde
# mavsdk kurulu değil; bu sahte System yalnız başlatma sırasını doğrulamak içindir.
# Task'lar asyncio.run sonunda hata üretmesin diye telemetri/param stream'leri
# anında döner (async generator, yield yok) — gerçek veri simülasyonu gerekmez.
_mavsdk = types.ModuleType("mavsdk")


class _Core:
    async def connection_state(self):
        # Bağlı durum tek sefer yayınla; async generator (async for uyumlu).
        yield types.SimpleNamespace(is_connected=True)


class _Telemetry:
    @staticmethod
    async def position():
        if False:
            yield  # pragma: no cover — async generator gövdesi
        return

    @staticmethod
    async def attitude_euler():
        if False:
            yield
        return

    @staticmethod
    async def velocity_ned():
        if False:
            yield
        return

    @staticmethod
    async def armed():
        if False:
            yield
        return

    @staticmethod
    async def health():
        if False:
            yield
        return

    @staticmethod
    async def flight_mode():
        if False:
            yield
        return


class _Param:
    async def get_param_float(self, name):
        raise RuntimeError("param not available in mock")

    async def get_param_int(self, name):
        raise RuntimeError("param not available in mock")


class _MissionRaw:
    async def download_mission(self):
        return []


class _System:
    def __init__(self):
        self.core = _Core()
        self.telemetry = _Telemetry()
        self.param = _Param()
        self.mission_raw = _MissionRaw()

    async def connect(self, **kwargs):
        return None


_mavsdk.System = _System
sys.modules["mavsdk"] = _mavsdk

_node_module = types.ModuleType("rclpy.node")


class _Node:
    def __init__(self, name):
        pass

    def create_publisher(self, *a, **k):
        return mock.MagicMock()

    def create_subscription(self, *a, **k):
        return mock.MagicMock()

    def create_timer(self, *a, **k):
        return mock.MagicMock()

    def declare_parameter(self, *a, **k):
        return mock.MagicMock()

    def get_parameter(self, name):
        # Varsayılan değerlerle dolu sahte parametre nesnesi.
        defaults = {
            "dry_run": True,
            "system_address": "udp://:14540",
            "command_timeout_s": 0.6,
            "send_hz": 10.0,
            "health_source_timeout_s": 1.5,
            "pymavlink_address": "udp://:14540",
            "pymavlink_baud": 115200,
            "guided_retry_s": 1.0,
            "guided_timeout_s": 5.0,
            "command_retry_s": 0.5,
            "guided_health_timeout_s": 2.5,
            "guided_max_forward_mps": 0.6,
            "guided_max_reverse_mps": 0.6,
            "guided_max_yaw_rate_rad_s": 0.785398163,
            "target_color_param": "SCR_USER4",
            "target_color_poll_hz": 1.0,
            "mission_download_hz": 0.2,
            "mission_raw_enabled": True,
            "mission_counts_param": "SCR_USER5",
            "mission_control_param": "SCR_USER6",
            "mission_control_poll_hz": 2.0,
            "mission_control_ros_ack_timeout_s": 1.0,
            "param_call_timeout_s": 2.0,
            "param_max_retries": 2,
            "param_consecutive_fail_limit": 30,
            "companion_system_id": 1,
            "companion_component_id": 191,
            "yki_status_heartbeat_s": 2.0,
            "yki_status_udp_host": "",
            "yki_status_udp_port": 14550,
            "vehicle_setup_enabled": False,
            "guided_mode_enabled": False,
            "motor_command_enabled": False,
            "motor_output_limits_check_enabled": False,
            "expected_mot_thr_min_pct": 0,
            "expected_mot_thr_max_pct": 30,
            "expected_mot_slewrate_pct_s": 20,
            "motor_output_limits_poll_s": 5.0,
            "left_motor_servo_channel": 9,
            "right_motor_servo_channel": 11,
            "vehicle_frame_class": 2,
            "actuation_source_system_id": 255,
            "actuation_backend": "guided_velocity",
            "bench_rc_safety_ack": "",
            "bench_rc_steering_channel": 1,
            "bench_rc_throttle_channel": 2,
            "bench_rc_neutral_pwm": 1500,
            "bench_rc_max_delta_pwm": 100,
            "bench_rc_max_forward_mps": 0.30,
            "bench_rc_source_system_id": 255,
        }
        return mock.MagicMock(value=defaults.get(name, None))

    def destroy_node(self):
        return None

    def get_logger(self):
        return mock.MagicMock()

    def get_clock(self):
        return _FakeClock()


class _FakeClock:
    """rclpy Clock taklidi: now() -> _FakeTime (nanoseconds + __sub__)."""

    def now(self):
        return _FakeTime()


class _FakeTime:
    """rclpy Time taklidi: (now - old).nanoseconds = 0 (age <= timeout)."""

    nanoseconds = 0

    def __sub__(self, other):
        return _FakeTime()


class _FakeTwist:

    def destroy_node(self):
        pass


_node_module.Node = _Node

_rclpy.node = _node_module
sys.modules["rclpy"] = _rclpy
sys.modules["rclpy.node"] = _node_module
sys.modules["geometry_msgs"] = _geometry_msgs
sys.modules["geometry_msgs.msg"] = _geometry_msgs_msg
sys.modules["std_msgs"] = _std_msgs
sys.modules["std_msgs.msg"] = _std_msgs_msg

# --- Gerçek node import'u (mock'lardan sonra) ---
_PKG_ROOT = __import__("os").path.abspath(__import__("os").path.join(__import__("os").path.dirname(__file__), ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from ida_control import mavsdk_bridge_node  # noqa: E402
from ida_control.bridge_health import BridgeHealth  # noqa: E402


class TestStructuredBridgeHealth(unittest.TestCase):
    def test_dry_status_is_json_and_never_connected(self):
        node = mavsdk_bridge_node.MavsdkBridgeNode()
        node.publish_dry_status()
        payload = json.loads(node.status_pub.publish.call_args.args[0].data)
        self.assertTrue(payload["dry_run"])
        self.assertFalse(payload["connected"])
        self.assertFalse(payload["heartbeat_fresh"])
        self.assertEqual(
            payload["guided_actuation"]["reason"], "vehicle_disarmed"
        )
        self.assertFalse(payload["guided_actuation"]["allowed"])

    def test_default_telemetry_is_not_published_before_real_acquisition(self):
        node = mavsdk_bridge_node.MavsdkBridgeNode()
        self.assertEqual(node._health.timeout_s, 1.5)
        node.publish_telemetry()
        node.telemetry_pub.publish.assert_not_called()

    def test_real_complete_acquisition_publishes_acquisition_stamp(self):
        node = mavsdk_bridge_node.MavsdkBridgeNode()
        node._health = BridgeHealth(False, 0.5)
        node._health.mark_connection()
        node._update_telemetry("position", lat=41.0, lon=29.0)
        node._update_telemetry("attitude", heading_deg=1.0, roll_deg=0.0, pitch_deg=0.0)
        node._update_telemetry("velocity", ground_speed=0.0)
        node._update_telemetry("armed", mode="DISARMED")
        node.publish_telemetry()
        payload = json.loads(node.telemetry_pub.publish.call_args.args[0].data)
        self.assertIn("acquisition_stamp", payload)
        self.assertEqual(payload["stamp"], payload["acquisition_stamp"])
        node.publish_status()
        status = json.loads(node.status_pub.publish.call_args.args[0].data)
        self.assertTrue(status["connected"])
        self.assertFalse(status["dry_run"])

    def test_target_ack_uses_same_gateway_free_mavlink_connection(self):
        node = mavsdk_bridge_node.MavsdkBridgeNode()
        conn = _FakeConn()
        node._set_pymavlink_conn(conn)

        node._handle_target_color(2)

        payload = json.loads(node.target_pub.publish.call_args.args[0].data)
        self.assertEqual(payload["target_color"], "green")
        calls = conn.mav.named_value_int_send.call_args_list
        self.assertTrue(calls)
        self.assertEqual(calls[-1].args[1:], (b"TGT_ACK", 2))

    def test_direct_yki_status_connection_takes_priority_over_vehicle_link(self):
        node = mavsdk_bridge_node.MavsdkBridgeNode()
        vehicle_conn = _FakeConn()
        direct_conn = _FakeConn()
        node._set_pymavlink_conn(vehicle_conn)
        node._yki_status_conn = direct_conn

        node.on_autonomy_state(
            _String(json.dumps({"state": "PARKUR_1_NAV", "action": "corridor"}))
        )

        direct_names = {
            call.args[1] for call in direct_conn.mav.named_value_int_send.call_args_list
        }
        vehicle_names = {
            call.args[1] for call in vehicle_conn.mav.named_value_int_send.call_args_list
        }
        self.assertIn(b"AUTO_ST", direct_names)
        self.assertIn(b"PARKUR", direct_names)
        self.assertNotIn(b"AUTO_ST", vehicle_names)

    def test_pymavlink_companion_identity_is_explicit(self):
        node = mavsdk_bridge_node.MavsdkBridgeNode()
        with mock.patch.object(
            _mavutil, "mavlink_connection", return_value=_FakeConn()
        ) as factory:
            node._connect_pymavlink()
        kwargs = factory.call_args.kwargs
        self.assertEqual(kwargs["source_system"], 1)
        self.assertEqual(kwargs["source_component"], 191)
        self.assertTrue(kwargs["autoreconnect"])

    def test_router_gcs_heartbeat_cannot_be_selected_as_vehicle(self):
        node = mavsdk_bridge_node.MavsdkBridgeNode()

        class RouterConn(_FakeConn):
            def __init__(self):
                super().__init__()
                self.target_system = 0
                self.target_component = 0

            def wait_heartbeat(self, timeout=0.0):
                # Router/GCS heartbeat arrived first; it is not a command target.
                return types.SimpleNamespace(autopilot=8)

            def recv_match(self, **_kwargs):
                return types.SimpleNamespace(
                    autopilot=3,
                    get_srcSystem=lambda: 1,
                    get_srcComponent=lambda: 1,
                )

        conn = RouterConn()
        with mock.patch.object(_mavutil, "mavlink_connection", return_value=conn):
            result = node._connect_pymavlink()

        self.assertIs(result, conn)
        self.assertEqual((conn.target_system, conn.target_component), (1, 1))


class TestBenchRcConfiguration(unittest.TestCase):
    @staticmethod
    def _parameter_override(overrides):
        original = _Node.get_parameter

        def get_parameter(_self, name):
            if name in overrides:
                return mock.MagicMock(value=overrides[name])
            return original(None, name)

        return get_parameter

    def test_motor_enable_requires_exact_physical_safety_ack(self):
        overrides = {
            "actuation_backend": "bench_rc_override",
            "motor_command_enabled": True,
            "guided_mode_enabled": False,
            "vehicle_setup_enabled": False,
            "bench_rc_safety_ack": "",
        }
        with mock.patch.object(
            _Node, "get_parameter", new=self._parameter_override(overrides)
        ):
            with self.assertRaisesRegex(ValueError, "PROPELLER_AREA_CLEAR"):
                mavsdk_bridge_node.MavsdkBridgeNode()

    def test_bench_rc_and_guided_are_mutually_exclusive(self):
        overrides = {
            "actuation_backend": "bench_rc_override",
            "motor_command_enabled": True,
            "guided_mode_enabled": True,
            "vehicle_setup_enabled": False,
            "bench_rc_safety_ack": "PROPELLER_AREA_CLEAR",
        }
        with mock.patch.object(
            _Node, "get_parameter", new=self._parameter_override(overrides)
        ):
            with self.assertRaisesRegex(ValueError, "cannot be enabled together"):
                mavsdk_bridge_node.MavsdkBridgeNode()


class TestMissionPayload(unittest.TestCase):
    """Mission messages must be parseable by JSON-only ROS consumers."""

    def test_waypoints_payload_is_strict_json(self) -> None:
        waypoints = [
            {"lat": 41.0, "lon": 29.0, "parkur": 1},
            {"lat": 41.0001, "lon": 29.0002, "parkur": 2},
        ]
        payload = mavsdk_bridge_node.MavsdkBridgeNode._serialize_waypoints(
            waypoints, 123.456
        )
        self.assertEqual(json.loads(payload), {"stamp": 123.456, "waypoints": waypoints})
        self.assertNotIn("'", payload)

    def test_waypoints_payload_rejects_nonfinite_numbers(self) -> None:
        with self.assertRaises(ValueError):
            mavsdk_bridge_node.MavsdkBridgeNode._serialize_waypoints(
                [{"lat": float("nan"), "lon": 29.0, "parkur": 1}], 123.0
            )

    def test_exact_mission_boundary_count_validation(self) -> None:
        helper = mavsdk_bridge_node.MavsdkBridgeNode._exact_nonnegative_count
        self.assertEqual(helper(4.0, require_positive=True), 4)
        self.assertEqual(helper(0.0, require_positive=False), 0)
        self.assertIsNone(helper(0.0, require_positive=True))
        self.assertIsNone(helper(1.2, require_positive=False))
        self.assertIsNone(helper(float("nan"), require_positive=False))

    def test_poll_mission_publishes_json_not_python_repr(self) -> None:
        node = mavsdk_bridge_node.MavsdkBridgeNode()
        waypoints = [{"lat": 41.0, "lon": 29.0, "parkur": 1}]
        node._mission_items_to_waypoints = (
            lambda _items, p1_count, p2_count: waypoints
            if (p1_count, p2_count) == (1, 0) else []
        )
        async def _get_param(_name):
            return 1001.0  # packed P1=1, P2=0
        drone = types.SimpleNamespace(
            mission_raw=types.SimpleNamespace(download_mission=mock.AsyncMock(return_value=[object()])),
            param=types.SimpleNamespace(
                get_param_float=_get_param,
                get_param_int=_get_param,
            ),
        )

        with mock.patch.object(mavsdk_bridge_node.rclpy, "ok", side_effect=[True, False]), \
                mock.patch.object(mavsdk_bridge_node.asyncio, "sleep", new=mock.AsyncMock()):
            asyncio.run(node._poll_mission(drone))

        published = node.waypoint_pub.publish.call_args.args[0].data
        self.assertEqual(json.loads(published)["waypoints"], waypoints)
        self.assertNotIn("'lat'", published)

    def test_pending_start_reaches_autonomy_then_persists_ack(self) -> None:
        node = mavsdk_bridge_node.MavsdkBridgeNode()
        node._last_published_waypoints = [{"lat": 41.0, "lon": 29.0, "parkur": 1}]
        values = {"SCR_USER6": 8_000_001.0}

        async def get_param(name):
            return values[name]

        async def set_param(name, value):
            values[name] = float(value)

        drone = types.SimpleNamespace(param=types.SimpleNamespace(
            get_param_float=get_param,
            get_param_int=get_param,
            set_param_float=set_param,
        ))

        def acknowledge(message):
            payload = json.loads(message.data)
            ack = _String()
            ack.data = json.dumps({"token": payload["token"], "applied": True})
            node.on_mission_control_ack(ack)

        node.mission_control_pub.publish.side_effect = acknowledge
        with mock.patch.object(mavsdk_bridge_node.rclpy, "ok", side_effect=[True, False]):
            asyncio.run(node._poll_mission_control(drone))

        self.assertEqual(values["SCR_USER6"], 8_000_003.0)
        node.mission_control_pub.publish.assert_called_once()

    def test_pending_stop_reaches_autonomy_then_persists_ack(self) -> None:
        node = mavsdk_bridge_node.MavsdkBridgeNode()
        values = {"SCR_USER6": 8_000_006.0}

        async def get_param(name):
            return values[name]

        async def set_param(name, value):
            values[name] = float(value)

        drone = types.SimpleNamespace(param=types.SimpleNamespace(
            get_param_float=get_param,
            get_param_int=get_param,
            set_param_float=set_param,
        ))

        def acknowledge(message):
            payload = json.loads(message.data)
            self.assertFalse(payload["start"])
            ack = _String()
            ack.data = json.dumps({"token": payload["token"], "applied": True})
            node.on_mission_control_ack(ack)

        node.mission_control_pub.publish.side_effect = acknowledge
        with mock.patch.object(mavsdk_bridge_node.rclpy, "ok", side_effect=[True, False]):
            asyncio.run(node._poll_mission_control(drone))

        self.assertEqual(values["SCR_USER6"], 8_000_008.0)
        node.mission_control_pub.publish.assert_called_once()

    def test_out_of_order_control_token_is_not_published(self) -> None:
        node = mavsdk_bridge_node.MavsdkBridgeNode()
        node._last_mission_mailbox_sequence = 1
        values = {"SCR_USER6": 8_000_009.0}  # START command seq=3; seq=2 expected

        async def get_param(name):
            return values[name]

        drone = types.SimpleNamespace(param=types.SimpleNamespace(
            get_param_float=get_param,
            get_param_int=get_param,
        ))
        with mock.patch.object(mavsdk_bridge_node.rclpy, "ok", side_effect=[True, False]), \
                mock.patch.object(mavsdk_bridge_node.asyncio, "sleep", new=mock.AsyncMock()):
            asyncio.run(node._poll_mission_control(drone))

        node.mission_control_pub.publish.assert_not_called()

    def test_acknowledged_token_is_not_replayed_after_restart(self) -> None:
        node = mavsdk_bridge_node.MavsdkBridgeNode()

        async def get_param(_name):
            return 8_000_003.0

        drone = types.SimpleNamespace(param=types.SimpleNamespace(
            get_param_float=get_param,
            get_param_int=get_param,
        ))
        with mock.patch.object(mavsdk_bridge_node.rclpy, "ok", side_effect=[True, False]), \
                mock.patch.object(mavsdk_bridge_node.asyncio, "sleep", new=mock.AsyncMock()):
            asyncio.run(node._poll_mission_control(drone))
        node.mission_control_pub.publish.assert_not_called()


class TestWaypointLatchedQos(unittest.TestCase):
    """Geç abone mission mesajını kaçırmasın diye waypoint_pub TRANSIENT_LOCAL."""

    def test_waypoint_qos_profile_is_transient_local(self) -> None:
        calls = []

        def _create_publisher(*args, **kwargs):
            calls.append((args, kwargs))
            return mock.MagicMock()

        with mock.patch.object(_Node, "create_publisher", new=_create_publisher):
            mavsdk_bridge_node.MavsdkBridgeNode()

        # Bağlı metot self'i ilk argüman olarak alır: (self, msg_type, topic, qos).
        waypoint_call = [c for c in calls if c[0][2] == "/mission/waypoints"]
        self.assertTrue(waypoint_call, "/mission/waypoints publisher'ı kurulmalı")
        profile = waypoint_call[0][0][3] if len(waypoint_call[0][0]) > 3 else waypoint_call[0][1].get("qos_profile")
        self.assertIsNotNone(profile, "waypoint publisher QoS profili verilmeli")
        self.assertEqual(profile.durability, _rclpy_qos.DurabilityPolicy.TRANSIENT_LOCAL)


class _FakeConn:
    """Sahte pymavlink bağlantısı — komut çağrılarını kaydeder."""

    def __init__(self):
        self.calls = []
        self.mode_calls = []
        self.target_system = 1
        self.target_component = 1
        self.mav = mock.MagicMock()
        self.mav.srcSystem = 1
        self.mavlink = mock.MagicMock()
        # MAV_FRAME_BODY_NED sabiti (gerçek değer 9).
        type(self.mavlink).MAV_FRAME_BODY_NED = 9

        def _send(*args, **kwargs):
            self.calls.append(kwargs)
            return None

        self.mav.set_position_target_local_ned_send.side_effect = _send

        def _pos(*args, **kwargs):
            # set_position_target_local_ned_send(time_boot_ms, sys, comp, frame, mask,
            #    x,y,z, vx,vy,vz, afx,afy,afz, yaw, yaw_rate) — vx=index 8, yaw_rate=15.
            self.calls.append({
                "cmd": "set_position_target",
                "source_system": self.mav.srcSystem,
                "frame": args[3] if len(args) > 3 else kwargs.get("coordinate_frame"),
                "type_mask": args[4] if len(args) > 4 else kwargs.get("type_mask"),
                "vx": args[8] if len(args) > 8 else kwargs.get("vx"),
                "vy": args[9] if len(args) > 9 else kwargs.get("vy"),
                "yaw": args[14] if len(args) > 14 else kwargs.get("yaw"),
                "yaw_rate": args[15] if len(args) > 15 else kwargs.get("yaw_rate"),
            })
            return None

        self.mav.set_position_target_local_ned_send.side_effect = _pos

        def _rc(*args, **kwargs):
            # rc_channels_override_send(sys, comp, ch1, ch2, ch3, ch4, ch5..ch8)
            self.calls.append({
                "cmd": "rc_override",
                "source_system": self.mav.srcSystem,
                "ch1": args[2] if len(args) > 2 else kwargs.get("ch1"),
                "ch2": args[3] if len(args) > 3 else kwargs.get("ch2"),
                "ch3": args[4] if len(args) > 4 else kwargs.get("ch3"),
            })
            return None

        self.mav.rc_channels_override_send.side_effect = _rc

    def set_mode_apm(self, mode):
        self.mode_calls.append(mode)

    def set_mode_send(self, target_system, mode_base, custom_mode):
        self.mode_calls.append(f"AUTO:{custom_mode}")
        self.calls.append({"cmd": "set_mode_send", "mode_base": mode_base, "custom_mode": custom_mode})

    def _register_mav_set_mode(self):
        """conn.mav.set_mode_send'i kaydeder (gerçek _set_guided_mode onu çağırır)."""

        def _ms(*args, **kwargs):
            self.mode_calls.append(f"AUTO_MAV:{args[2] if len(args) > 2 else kwargs.get('custom_mode')}")
            self.calls.append({
                "cmd": "set_mode_send",
                "source_system": self.mav.srcSystem,
                "custom_mode": args[2] if len(args) > 2 else None,
            })

        self.mav.set_mode_send = _ms

    # Gerçek _connect_pymavlink (reconnect testlerinde) mavlink_connection +
    # wait_heartbeat çağırır; burada sahte modüle karşılık üretilir.
    @classmethod
    def mavlink_connection(cls, *args, **kwargs):
        conn = cls()
        return conn

    def wait_heartbeat(self, timeout=0.0):
        return True


class TestServoOutputRawReader(unittest.TestCase):
    """SERVO_OUTPUT_RAW pasif okuyucusu: yalnız OKUR, komut göndermez."""

    def test_motor_pwm_lands_in_telemetry_state(self) -> None:
        node = mavsdk_bridge_node.MavsdkBridgeNode()
        msg = types.SimpleNamespace(servo9_raw=1620, servo11_raw=1500)
        node._update_motor_pwm(msg, None)
        with node._telemetry_lock:
            self.assertEqual(node._telemetry_state["motor_left_pwm"], 1620)
            self.assertEqual(node._telemetry_state["motor_right_pwm"], 1500)

    def test_out_of_range_pwm_is_rejected(self) -> None:
        node = mavsdk_bridge_node.MavsdkBridgeNode()
        node._update_motor_pwm(types.SimpleNamespace(servo9_raw=9999, servo11_raw=1500), None)
        with node._telemetry_lock:
            self.assertNotIn("motor_left_pwm", node._telemetry_state)

    def test_reader_thread_never_sends_and_writes_state(self) -> None:
        node = mavsdk_bridge_node.MavsdkBridgeNode()
        node.dry_run = False
        msg = types.SimpleNamespace(servo9_raw=1610, servo11_raw=1540)
        conn = types.SimpleNamespace(
            recv_match=mock.MagicMock(return_value=msg),
            mav=mock.MagicMock(),
        )
        node._set_pymavlink_conn(conn)
        node._start_servo_reader()
        deadline = mavsdk_bridge_node.time.monotonic() + 1.5
        while mavsdk_bridge_node.time.monotonic() < deadline:
            with node._telemetry_lock:
                if node._telemetry_state.get("motor_left_pwm") == 1610:
                    break
            mavsdk_bridge_node.time.sleep(0.02)
        with node._telemetry_lock:
            self.assertEqual(node._telemetry_state["motor_left_pwm"], 1610)
            self.assertEqual(node._telemetry_state["motor_right_pwm"], 1540)
        conn.mav.assert_not_called()  # thread hiçbir *_send çağrısı yapmaz
        node.destroy_node()

    def test_reader_stays_quiet_in_dry_run(self) -> None:
        node = mavsdk_bridge_node.MavsdkBridgeNode()  # dry_run=True default
        conn = types.SimpleNamespace(recv_match=mock.MagicMock(), mav=mock.MagicMock())
        node._set_pymavlink_conn(conn)
        node._start_servo_reader()
        self.assertIsNone(node._servo_reader_thread)
        conn.recv_match.assert_not_called()


class TestMavsdkTaskTelemetryFirst(unittest.TestCase):
    """B2: mavsdk_task pymavlink bağlantısından ÖNCE telemetri/poll task'larını başlatır.

    pymavlink bağlantısı başarısız olduğunda (port yok, pymavlink eksik) telemetri
    task'ları HİÇ başlamamalı değil — komut yolu düşer, /telemetry/state akışı devam.
    """

    def setUp(self):
        self.node = mavsdk_bridge_node.MavsdkBridgeNode()
        self.node.guided_timeout = 0.05  # test hızı
        # mavsdk_task modül seviyesinde asyncio.create_task çağırır. Test yalnız
        # SIRALAMAYI doğrular: telemetri/poll task'ları bağlantıdan ÖNCE başlatılır.
        # Telemetri task'ları sonsuz döngüdür — planlanırsa asyncio.run kapanışında
        # takılır (Windows'ta to_thread + loop kapatma çakışması). Bu yüzden
        # create_task PATCH'lenir ve task'lar HİÇ schedule edilmez; yalnızca toplanır.
        # Coroutine'ler test sonunda .close() ile kapatılır (never-awaited uyarısı).
        self.created_tasks = []

        def _capture(coro, *a, **k):
            self.created_tasks.append(coro)
            return mock.MagicMock()  # Task nesnesi gerekmez — asla beklenmez

        self._patch = mock.patch.object(asyncio, "create_task", side_effect=_capture)
        self._patch.start()
        self.addCleanup(self._patch.stop)
        self.addCleanup(self._close_pending_coroutines)

    def _close_pending_coroutines(self) -> None:
        for c in self.created_tasks:
            try:
                c.close()
            except Exception:
                pass

    def test_connect_failure_still_starts_telemetry_tasks(self) -> None:
        """Bağlantı hatası -> telemetri task'ları yine create_task ile başlar."""
        # to_thread ile çalışan _connect_pymavlink her zaman başarısız olsun.
        def _fail_connect():
            raise ConnectionError("Simulated link failure")

        self.node._connect_pymavlink = _fail_connect

        # mavsdk_task'ı başlatıp bağlantı hatasına kadar ilerlet. create_task
        # patch'li olduğundan arka planda hiçbir task çalışmaz (takılma yok).
        # mavsdk_task bağlantı hatası sonrası return eder -> coroutine tamamlanır.
        asyncio.run(self.node.mavsdk_task())

        # Telemetri/poll task'ları bağlantı hatasından bağımsız başladı:
        # position, attitude, velocity, armed, EKF health, flight mode,
        # target_color (+mission raw default).
        names = [getattr(c, "__name__", "") for c in self.created_tasks]
        self.assertIn("_telemetry_position", names)
        self.assertIn("_telemetry_attitude", names)
        self.assertIn("_telemetry_velocity", names)
        self.assertIn("_telemetry_armed", names)
        self.assertIn("_telemetry_health", names)
        self.assertIn("_telemetry_flight_mode", names)
        self.assertIn("_poll_target_color", names)
        self.assertIn("_poll_mission", names)
        # Komut yolu bağlanmadı (create_task hiç _command_loop çağrılmadı).
        self.assertNotIn("_command_loop_pymavlink", names)

    def test_connect_success_starts_command_loop(self) -> None:
        """Bağlantı başarılı -> telemetri task'ları + _command_loop_pymavlink başlar."""
        self.node.motor_command_enabled = True
        self.node.guided_mode_enabled = True
        self.node._connect_pymavlink = lambda: self._fake_conn

        self._fake_conn = types.SimpleNamespace(
            target_system=1, target_component=1, mav=mock.MagicMock()
        )

        asyncio.run(self.node.mavsdk_task())

        names = [getattr(c, "__name__", "") for c in self.created_tasks]
        self.assertIn("_command_loop_pymavlink", names)
        # Komut yolu telemetri task'larından SONRA başlatılır.
        self.assertGreater(names.index("_command_loop_pymavlink"), names.index("_telemetry_position"))

    def test_motor_permission_without_guided_permission_is_inhibited(self) -> None:
        self.node.motor_command_enabled = True
        self.node.guided_mode_enabled = False
        self.node._connect_pymavlink = lambda: self._fake_conn
        self._fake_conn = types.SimpleNamespace(
            target_system=1, target_component=1, mav=mock.MagicMock()
        )

        asyncio.run(self.node.mavsdk_task())

        names = [getattr(c, "__name__", "") for c in self.created_tasks]
        self.assertNotIn("_command_loop_pymavlink", names)
        self._fake_conn.mav.set_mode_send.assert_not_called()

    def test_connect_success_is_status_only_by_default(self) -> None:
        """Live telemetry does not imply permission to configure or move."""
        self.node._connect_pymavlink = lambda: self._fake_conn
        self._fake_conn = types.SimpleNamespace(
            target_system=1, target_component=1, mav=mock.MagicMock()
        )

        asyncio.run(self.node.mavsdk_task())

        names = [getattr(c, "__name__", "") for c in self.created_tasks]
        self.assertNotIn("_command_loop_pymavlink", names)
        self._fake_conn.mav.param_set_send.assert_not_called()
        self._fake_conn.mav.set_mode_send.assert_not_called()


class TestRunMavsdkKeepsTasksAlive(unittest.TestCase):
    """B2 (üretim): _run_mavsdk telemetri task'larını asyncio.run iptalinden korur.

    mavsdk_task telemetri task'larını create_task ile başlatıp DÖNER. asyncio.run
    kullanılsaydı loop kapanışında _cancel_all_tasks telemetriyi iptal ederdi
    (bağlantı hatası yolunda ölçüldü). _run_mavsdk bunu elle yönetilen loop ile
    çözer: task'lar process ömrü boyunca yaşar.
    """

    def test_run_mavsdk_keeps_background_tasks_alive(self) -> None:
        import threading
        import time

        node = mavsdk_bridge_node.MavsdkBridgeNode()
        node.mission_raw_enabled = True  # _poll_mission da başlatılsın (8 measured task)
        node.guided_timeout = 0.05

        cancelled = []
        ran = []

        def make_task(name):
            async def task(self, drone):
                ran.append(name)
                try:
                    while True:
                        await asyncio.sleep(0.001)
                except asyncio.CancelledError:
                    cancelled.append(name)
                    raise

            return task

        # Gerçek telemetri/poll task'larını ölçülebilir sahtelerle değiştir
        # (aslen sonsuz MAVSDK stream'leri — bu test yalnız yaşam süresini ölçer).
        for name in ("position", "attitude", "velocity", "armed", "health", "flight_mode"):
            setattr(
                node,
                "_telemetry_" + name,
                make_task(name).__get__(node, mavsdk_bridge_node.MavsdkBridgeNode),
            )
        node._poll_target_color = make_task("target_color").__get__(
            node, mavsdk_bridge_node.MavsdkBridgeNode
        )
        node._poll_mission = make_task("mission").__get__(
            node, mavsdk_bridge_node.MavsdkBridgeNode
        )

        def fail():
            raise ConnectionError("simulated connect failure")

        node._connect_pymavlink = fail

        thread = threading.Thread(target=node._run_mavsdk, daemon=True)
        thread.start()
        time.sleep(1.2)  # task'ların çalışması ve loop'un run_forever'da beklemesi için

        self.assertTrue(thread.is_alive(), "_run_mavsdk thread'i run_forever'da olmalı")
        self.assertEqual(cancelled, [], "hiçbir telemetri task'ı iptal edilmemeli")
        self.assertEqual(len(ran), 8, "8 telemetri/poll task'ı çalışmaya başlamalı")
        # Bağlantı hatası yalnız komut yolunu düşürür; telemetri task'ları canlı.
        self.assertNotIn("_command_loop_pymavlink", ran)


class TestCommandLoopPymavlink(unittest.TestCase):
    def setUp(self):
        self.node = mavsdk_bridge_node.MavsdkBridgeNode()
        self.node.vehicle_setup_enabled = True
        self.node.guided_mode_enabled = True
        self.conn = _FakeConn()
        self.conn._register_mav_set_mode()
        # Komut: ileri 0.8 m/s, yaw 0.5 rad/s.
        cmd = _Twist()
        cmd.linear.x = 0.8
        cmd.angular.z = 0.5
        self.node.latest_cmd = cmd
        self.node.latest_cmd_time = self.node.get_clock().now()  # age <= timeout
        self.node._telemetry_state = {"heading_deg": 0.0}
        self.node.command_timeout = 10.0  # timeout'u aşmasın
        self.node.guided_max_forward = 1.0
        # A real live cycle must observe DISARM before ARM, healthy EKF/global
        # position and the actual GUIDED mode.  Unit command tests establish
        # that precondition explicitly.
        now = mavsdk_bridge_node.time.monotonic()
        self.node._guided_gate.mark_armed(False, now)
        self.node._guided_gate.mark_position_health(True, True, now)
        self.node._guided_gate.mark_flight_mode("GUIDED", now)
        self.node._guided_gate.mark_armed(True, now)

    def test_send_position_target_velocity(self) -> None:
        # Rover 0x05E7: x=0.8 -> signed forward speed, z=0.5 -> yaw_rate=0.5.
        asyncio.run(self.node._command_loop_pymavlink(self.conn, max_iterations=1))
        pos = [c for c in self.conn.calls if c.get("cmd") == "set_position_target"]
        self.assertTrue(pos, "set_position_target_local_ned_send çağrılmalı")
        self.assertEqual(pos[0]["frame"], _mavlink_mod.MAV_FRAME_LOCAL_NED)
        self.assertEqual(pos[0]["type_mask"], 0x05E7)
        self.assertEqual(pos[0]["source_system"], 255)
        self.assertEqual(self.conn.mav.srcSystem, 1)
        self.assertAlmostEqual(pos[0]["vx"], 0.8, places=6)
        self.assertAlmostEqual(pos[0]["vy"], 0.0, places=6)
        self.assertAlmostEqual(pos[0]["yaw"], 0.0, places=6)
        self.assertAlmostEqual(pos[0]["yaw_rate"], 0.5, places=6)

    def test_forward_speed_does_not_flip_at_heading_90(self) -> None:
        """0x05E7 vx is signed speed and must not be rotated into NED."""
        self.node._telemetry_state = {"heading_deg": 90.0}
        asyncio.run(self.node._command_loop_pymavlink(self.conn, max_iterations=1))
        pos = [c for c in self.conn.calls if c.get("cmd") == "set_position_target"]
        self.assertAlmostEqual(pos[0]["vx"], 0.8, places=6)
        self.assertAlmostEqual(pos[0]["vy"], 0.0, places=6)

    def test_forward_speed_does_not_reverse_at_heading_180(self) -> None:
        """Regression: body-to-NED rotation used to turn +0.8 into -0.8."""
        self.node._telemetry_state = {"heading_deg": 180.0}
        asyncio.run(self.node._command_loop_pymavlink(self.conn, max_iterations=1))
        pos = [c for c in self.conn.calls if c.get("cmd") == "set_position_target"]
        self.assertAlmostEqual(pos[0]["vx"], 0.8, places=6)
        self.assertAlmostEqual(pos[0]["vy"], 0.0, places=6)

    def test_send_position_target_turn(self) -> None:
        # Saf dönüş: x=0, z=0.5 -> vx=0, yaw_rate=0.5.
        cmd = _Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
        self.node.latest_cmd = cmd
        asyncio.run(self.node._command_loop_pymavlink(self.conn, max_iterations=1))
        pos = [c for c in self.conn.calls if c.get("cmd") == "set_position_target"]
        self.assertTrue(pos, "set_position_target_local_ned_send çağrılmalı")
        self.assertAlmostEqual(pos[0]["vx"], 0.0, places=6)
        self.assertAlmostEqual(pos[0]["yaw_rate"], 0.5, places=6)

    def test_guided_health_loss_neutralizes_and_latches_output(self) -> None:
        now = mavsdk_bridge_node.time.monotonic()
        self.assertTrue(self.node._guided_gate.decision(now).allowed)
        self.node._guided_gate.mark_flight_mode("HOLD", now)
        asyncio.run(self.node._command_loop_pymavlink(self.conn, max_iterations=1))
        pos = [c for c in self.conn.calls if c.get("cmd") == "set_position_target"]
        self.assertEqual((pos[-1]["vx"], pos[-1]["yaw_rate"]), (0.0, 0.0))
        self.assertTrue(
            self.node._guided_gate.decision(mavsdk_bridge_node.time.monotonic()).latched
        )

    def test_guided_command_is_bounded_by_bridge_envelope(self) -> None:
        self.node.guided_max_forward = 0.25
        self.node.guided_max_reverse = 0.0
        self.node.guided_max_yaw_rate = 0.10
        asyncio.run(self.node._command_loop_pymavlink(self.conn, max_iterations=1))
        pos = [c for c in self.conn.calls if c.get("cmd") == "set_position_target"]
        self.assertAlmostEqual(pos[-1]["vx"], 0.25)
        self.assertAlmostEqual(pos[-1]["yaw_rate"], 0.10)

        cmd = _Twist()
        cmd.linear.x = -0.5
        cmd.angular.z = -1.0
        self.node.latest_cmd = cmd
        asyncio.run(self.node._command_loop_pymavlink(self.conn, max_iterations=1))
        pos = [c for c in self.conn.calls if c.get("cmd") == "set_position_target"]
        self.assertAlmostEqual(pos[-1]["vx"], 0.0)
        self.assertAlmostEqual(pos[-1]["yaw_rate"], -0.10)

        cmd.linear.x = float("nan")
        forward, yaw, reason = self.node._guided_command(
            cmd, mavsdk_bridge_node.time.monotonic()
        )
        self.assertEqual((forward, yaw, reason), (0.0, 0.0, "command_nonfinite"))

    def test_unverified_motor_limits_force_guided_neutral(self) -> None:
        self.node.motor_output_limits_check_enabled = True
        self.node._motor_output_limits_verified = False
        cmd = _Twist()
        cmd.linear.x = 0.8
        forward, yaw, reason = self.node._guided_command(
            cmd, mavsdk_bridge_node.time.monotonic()
        )
        self.assertEqual(
            (forward, yaw, reason),
            (0.0, 0.0, "motor_output_limits_unverified"),
        )

    def test_motor_limit_verifier_is_read_only_and_exact(self) -> None:
        self.node.expected_mot_thr_min_pct = 0
        self.node.expected_mot_thr_max_pct = 30
        self.node.expected_mot_slewrate_pct_s = 20
        good = {
            "MOT_THR_MIN": 0.0,
            "MOT_THR_MAX": 30.0,
            "MOT_SLEWRATE": 20.0,
        }

        async def read_good(_drone, name):
            return good[name]

        with mock.patch.object(self.node, "_read_mavsdk_param", side_effect=read_good):
            self.assertTrue(asyncio.run(self.node._verify_motor_output_limits(object())))

        bad = dict(good, MOT_THR_MAX=50.0)

        async def read_bad(_drone, name):
            return bad[name]

        with mock.patch.object(self.node, "_read_mavsdk_param", side_effect=read_bad):
            self.assertFalse(asyncio.run(self.node._verify_motor_output_limits(object())))

    def _enable_bench_rc(self) -> None:
        self.node.actuation_backend = "bench_rc_override"
        self.node.vehicle_setup_enabled = False
        self.node.guided_mode_enabled = False
        self.node.motor_command_enabled = True
        self.node.bench_rc_steering_channel = 1
        self.node.bench_rc_throttle_channel = 2
        self.node.bench_rc_neutral_pwm = 1500
        self.node.bench_rc_max_delta_pwm = 100
        self.node.bench_rc_max_forward_mps = 0.30
        self.node.bench_rc_source_system_id = 255

    def test_bench_rc_maps_bounded_forward_to_ch2_only(self) -> None:
        self._enable_bench_rc()
        cmd = _Twist()
        cmd.linear.x = 0.30
        cmd.angular.z = 0.0
        self.node.latest_cmd = cmd

        asyncio.run(self.node._command_loop_pymavlink(self.conn, max_iterations=1))

        rc = [c for c in self.conn.calls if c.get("cmd") == "rc_override"]
        self.assertEqual(len(rc), 1)
        self.assertEqual(rc[0]["source_system"], 255)
        self.assertEqual((rc[0]["ch1"], rc[0]["ch2"]), (1500, 1600))
        self.assertEqual(self.conn.mav.srcSystem, 1)
        self.assertFalse(any(c.get("cmd") == "set_position_target" for c in self.conn.calls))

    def test_bench_rc_reverse_yaw_and_nonfinite_fail_to_neutral(self) -> None:
        self._enable_bench_rc()
        for forward, yaw in [(-0.1, 0.0), (0.1, 0.2), (float("nan"), 0.0)]:
            cmd = _Twist()
            cmd.linear.x = forward
            cmd.angular.z = yaw
            self.assertEqual(self.node._bench_rc_pwm(cmd), (1500, 1500))

    def test_bench_rc_timeout_and_destroy_are_neutral(self) -> None:
        self._enable_bench_rc()
        self.node.timed_command = lambda: _Twist()
        self.node._set_pymavlink_conn(self.conn)

        asyncio.run(self.node._command_loop_pymavlink(self.conn, max_iterations=1))
        self.node.destroy_node()

        rc = [c for c in self.conn.calls if c.get("cmd") == "rc_override"]
        self.assertGreaterEqual(len(rc), 2)
        self.assertTrue(all((c["ch1"], c["ch2"]) == (1500, 1500) for c in rc))

    def test_explicit_vehicle_setup_uses_real_aux_channels(self) -> None:
        self.node._configure_vehicle(self.conn)
        names = [call.args[2] for call in self.conn.mav.param_set_send.call_args_list]
        self.assertEqual(names, [b"SERVO9_FUNCTION", b"SERVO11_FUNCTION", b"FRAME_CLASS"])

    def test_send_error_triggers_reconnect(self) -> None:
        """B1: gönderim hatası (bağlantı kopması) -> hata loglanır + yeniden bağlanılır.

        İlk ``set_position_target_local_ned_send`` OSError fırlatır (seri hat
        kopması); döngü bunu yakalamalı, ``_connect_pymavlink``'i yeniden çağırıp
        bağlantıyı kurmalı ve döngüye devam etmelidir. İkinci iterasyonda gönderim
        başarılı olur.
        """
        # _FakeConn._send tekrar sayısı; ilk çağrıda OSError.
        self.conn.send_attempts = 0

        def _flaky_send(*args, **kwargs):
            self.conn.send_attempts += 1
            if self.conn.send_attempts == 1:
                raise OSError("Simulated serial link loss")
            self.conn.calls.append({"cmd": "set_position_target", "vx": 0.8, "yaw_rate": 0.5})
            return None

        self.conn.mav.set_position_target_local_ned_send.side_effect = _flaky_send

        # _connect_pymavlink + _set_guided_mode yeniden bağlanma için çağrılmalı.
        reconnects = []
        original_connect = self.node._connect_pymavlink

        def _fake_reconnect():
            reconnects.append(True)
            return self.conn  # AYNI bağlantı döner — gönderimler self.conn'a yazılır

        self.node._connect_pymavlink = _fake_reconnect

        asyncio.run(self.node._command_loop_pymavlink(self.conn, max_iterations=2))

        # 2 iterasyon: ilki hata verdi (1. gönderim), yeniden bağlanıldı, ikincisi başarılı.
        self.assertEqual(len(reconnects), 1)
        # Sadece RC override gönderimleri say (set_mode_send ayrı kaydedilir).
        sends = [c for c in self.conn.calls if c.get("cmd") == "set_position_target"]
        self.assertEqual(len(sends), 1)  # sadece başarılı gönderim kaydedildi
        self.assertEqual(sends[0]["vx"], 0.8)
        # Reconnect HOLD/emergency durumunu bozup kendiliğinden GUIDED'a geçmez;
        # mode authority yalnız YKİ START handshake'indedir.
        self.assertEqual(self.conn.mode_calls, [])

    def test_send_error_reconnect_failure_keeps_loop_alive(self) -> None:
        """B1: gönderim hatası + yeniden bağlanma BAŞARISIZ -> döngü ölmez.

        ``_connect_pymavlink`` tekrar tekrar başarısız olsa da ``_command_loop_pymavlink``
        iptal olmaz — döngü sonraki turda yeniden dener (canlı kalır, araç ArduPilot
        timeout'uyla durur).
        """
        self.conn.send_attempts = 0

        def _always_fail(*args, **kwargs):
            self.conn.send_attempts += 1
            raise OSError("Simulated serial link loss")

        self.conn.mav.set_position_target_local_ned_send.side_effect = _always_fail

        def _fail_reconnect():
            raise ConnectionError("Simulated reconnect failure")

        self.node._connect_pymavlink = _fail_reconnect

        # 3 iterasyon: her biri gönderim hatası + başarısız yeniden bağlanma.
        asyncio.run(self.node._command_loop_pymavlink(self.conn, max_iterations=3))

        self.assertEqual(self.conn.send_attempts, 3)  # döngü 3 tur da çalıştı
        self.assertEqual(len(self.conn.calls), 0)  # başarılı gönderim yok


if __name__ == "__main__":
    unittest.main()
