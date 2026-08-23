"""Gazebo launch ve mission-helper regresyonları."""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "src" / "ida_bringup" / "launch" / "sim_gazebo.launch.py"
REAL_LAUNCH = ROOT / "src" / "ida_bringup" / "launch" / "real_vehicle.launch.py"
SIM_FULL_LAUNCH = ROOT / "src" / "ida_bringup" / "launch" / "sim_full_mission.launch.py"
AUTONOMY_CONFIG = ROOT / "src" / "ida_bringup" / "config" / "autonomy.yaml"
BRINGUP_SETUP = ROOT / "src" / "ida_bringup" / "setup.py"
MISSION_UPLOAD = ROOT / "src" / "ida_bringup" / "scripts" / "sim_upload_mission.py"
MISSION_SENDER = ROOT / "src" / "ida_bringup" / "scripts" / "send_mission.py"
FULL_SCENARIO = ROOT / "src" / "ida_bringup" / "scenarios" / "full_mission.yaml"
P3_SCENARIO = ROOT / "src" / "ida_bringup" / "scenarios" / "parkur3_target_color.yaml"
REAL_START_SCRIPT = ROOT / "scripts" / "start_real.sh"
CONTROL_PACKAGE = ROOT / "src" / "ida_control" / "package.xml"
CONTROL_SETUP = ROOT / "src" / "ida_control" / "setup.py"
CONTROL_BRIDGE = ROOT / "src" / "ida_control" / "ida_control" / "mavsdk_bridge_node.py"
LEGACY_GATEWAY_IGNORE = ROOT / "src" / "ida_ws_gateway" / "COLCON_IGNORE"
LEGACY_GATEWAY_CONFIG = ROOT / "src" / "ida_bringup" / "config" / "gateway.yaml"

PLANNING_ROOT = str(ROOT / "src" / "ida_planning")
if PLANNING_ROOT not in sys.path:
    sys.path.insert(0, PLANNING_ROOT)


def _load_mission_sender():
    if "rclpy" not in sys.modules:
        rclpy = types.ModuleType("rclpy")
        rclpy.init = lambda **_kwargs: None
        rclpy.shutdown = lambda: None
        rclpy.ok = lambda: False
        rclpy_node = types.ModuleType("rclpy.node")
        rclpy_node.Node = type("Node", (), {})
        rclpy.node = rclpy_node
        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")
        std_msgs_msg.String = type("String", (), {})
        std_msgs.msg = std_msgs_msg
        sys.modules.update({
            "rclpy": rclpy,
            "rclpy.node": rclpy_node,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
        })
    spec = importlib.util.spec_from_file_location("send_mission_test", MISSION_SENDER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_mission_chain_uses_fourth_argument_as_mavlink_address() -> None:
    text = LAUNCH.read_text(encoding="utf-8")

    assert 'python3 \\"$1\\" --address \\"$4\\"' in text
    assert 'python3 \\"$2\\" --address \\"$4\\"' in text
    assert 'python3 \\"$3\\" --rate 1.0 --scenario-file \\"$5\\"' in text
    assert '--address \\"$3\\"' not in text
    assert "source ~/ida_ws/install/setup.bash" not in text
    assert '--out=udp:127.0.0.1:14542' in text
    assert 'DeclareLaunchArgument("mission_address"' in text
    assert 'mission_address,' in text
    assert 'scenario_file,' in text


def test_full_scenario_matches_legacy_waypoint_reference() -> None:
    module = _load_mission_sender()
    converted = module.waypoints_from_scenario(str(FULL_SCENARIO))
    assert len(converted) == len(module.WAYPOINTS)
    for actual, expected in zip(converted, module.WAYPOINTS):
        assert actual["parkur"] == expected["parkur"]
        assert abs(actual["lat"] - expected["lat"]) <= 1e-7
        assert abs(actual["lon"] - expected["lon"]) <= 1e-7


def test_parkur3_scenario_produces_exactly_one_parkur3_waypoint() -> None:
    module = _load_mission_sender()
    converted = module.waypoints_from_scenario(str(P3_SCENARIO))
    assert len(converted) == 1
    assert converted[0]["parkur"] == 3
    published = []

    class Publisher:
        @staticmethod
        def publish(msg):
            published.append(json.loads(msg.data))

    class Logger:
        @staticmethod
        def info(_message):
            pass

    sender = module.MissionSender.__new__(module.MissionSender)
    sender.waypoints = converted
    sender.waypoints_pub = Publisher()
    sender._sent = 0
    sender.get_logger = lambda: Logger()
    assert sender.send()
    assert published == [{"waypoints": converted}]


def test_malformed_scenario_fails_before_ros_or_publisher_creation() -> None:
    module = _load_mission_sender()
    calls = []
    module.rclpy.init = lambda **_kwargs: calls.append("ros_init")
    module.MissionSender = lambda *_args, **_kwargs: calls.append("publisher")
    with tempfile.TemporaryDirectory() as directory:
        malformed = Path(directory) / "bad.json"
        malformed.write_text(json.dumps({
            "origin": {"lat": 40.0, "lon": 29.0},
            "waypoints": [{"x": "nan", "y": 0.0, "parkur": 3}],
        }), encoding="utf-8")
        try:
            module.main(["--scenario-file", str(malformed)])
        except ValueError:
            pass
        else:
            raise AssertionError("malformed scenario must fail")
    assert calls == []


def test_huge_waypoints_fail_geographic_validation_before_ros_init() -> None:
    for value in (1e20, "1e10000"):
        module = _load_mission_sender()
        calls = []
        module.rclpy.init = lambda **_kwargs: calls.append("ros_init")
        module.MissionSender = lambda *_args, **_kwargs: calls.append("publisher")
        with tempfile.TemporaryDirectory() as directory:
            scenario = Path(directory) / "huge.json"
            scenario.write_text(json.dumps({
                "origin": {"lat": 40.0, "lon": 29.0},
                "waypoints": [{"x": value, "y": value, "parkur": 3}],
            }), encoding="utf-8")
            try:
                module.main(["--scenario-file", str(scenario)])
            except ValueError:
                pass
            else:
                raise AssertionError("huge waypoint must fail")
        assert calls == []


def test_bringup_installs_mission_helper_scripts() -> None:
    text = BRINGUP_SETUP.read_text(encoding="utf-8")
    assert 'glob("scripts/*.py")' in text


def test_mission_upload_waits_past_stale_manual_heartbeat() -> None:
    text = MISSION_UPLOAD.read_text(encoding="utf-8")
    assert "def wait_for_mode" in text
    assert 'wait_for_mode(conn, mavutil, "GUIDED"' in text

    spec = importlib.util.spec_from_file_location("sim_upload_mission_test", MISSION_UPLOAD)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class FakeConnection:
        def __init__(self) -> None:
            self.messages = ["MANUAL", "GUIDED"]

        def recv_match(self, **_kwargs):
            return self.messages.pop(0) if self.messages else None

    class FakeMavutil:
        @staticmethod
        def mode_string_v10(message):
            return message

    assert module.wait_for_mode(FakeConnection(), FakeMavutil, "GUIDED", 0.1) == "GUIDED"


def test_sim_has_single_authoritative_mission_topic_publisher() -> None:
    text = LAUNCH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("mission_raw_enabled", default_value="false")' in text
    assert 'LaunchConfiguration("mission_raw_enabled"), value_type=bool' in text
    assert '"send_mission.py"' in text


def test_yki_sim_has_dedicated_mavlink_out_and_field_speed() -> None:
    text = LAUNCH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("speedup", default_value="1")' in text
    assert 'DeclareLaunchArgument("yki_mavlink_host", default_value="127.0.0.1")' in text
    assert 'DeclareLaunchArgument("yki_mavlink_port", default_value="14550")' in text
    assert '--out=\\"udp:$4:$5\\"' in text
    assert 'home, speedup, yki_mavlink_host, yki_mavlink_port' in text


def test_gate_truth_is_sim_opt_in_and_production_default_off() -> None:
    sim_text = LAUNCH.read_text(encoding="utf-8")
    real_text = REAL_LAUNCH.read_text(encoding="utf-8")
    config_text = AUTONOMY_CONFIG.read_text(encoding="utf-8")

    assert '"sim_gate_truth_enabled": True' in sim_text
    assert "sim_gate_truth_enabled: false" in config_text
    assert '"sim_gate_truth_enabled": True' not in real_text


def test_real_launch_has_one_serial_owner_and_opt_in_actuation() -> None:
    text = REAL_LAUNCH.read_text(encoding="utf-8")

    assert 'package="ida_ws_gateway"' not in text
    assert 'executable="ws_gateway_node"' not in text
    assert '"mavlink-routerd"' in text
    assert 'DeclareLaunchArgument("mavlink_router_enabled", default_value="false")' in text
    assert 'DeclareLaunchArgument("canonical_takeover_enabled", default_value="false")' in text
    assert 'DeclareLaunchArgument("vehicle_setup_enabled", default_value="false")' in text
    assert 'DeclareLaunchArgument("guided_mode_enabled", default_value="false")' in text
    assert 'DeclareLaunchArgument("motor_command_enabled", default_value="false")' in text
    assert 'DeclareLaunchArgument("left_motor_servo_channel", default_value="9")' in text
    assert 'DeclareLaunchArgument("right_motor_servo_channel", default_value="11")' in text
    assert 'DeclareLaunchArgument("target_color_param", default_value="SCR_USER4")' in text
    assert 'DeclareLaunchArgument("mission_counts_param", default_value="SCR_USER5")' in text
    assert 'DeclareLaunchArgument("mission_control_param", default_value="SCR_USER6")' in text
    assert '"mission_counts_param": LaunchConfiguration("mission_counts_param")' in text
    assert '"mission_control_param": LaunchConfiguration("mission_control_param")' in text
    assert 'DeclareLaunchArgument("pymavlink_address", default_value="udpin:0.0.0.0:14541")' in text
    assert '"vehicle_setup_enabled": vehicle_setup_enabled' in text
    assert '"guided_mode_enabled": guided_mode_enabled' in text
    assert '"motor_command_enabled": motor_command_enabled' in text
    assert text.count('"buoy_output_topic": "/perception/buoys"') == 1
    assert text.count('"obstacle_output_topic": "/perception/obstacles"') == 1
    assert '"output_topic": "/perception/camera/p1p2/raw"' in text
    assert '"output_topic": "/perception/camera/p3/raw"' in text
    assert '"output_topic": "/perception/lidar/raw_obstacles"' in text
    assert '"raw_contract": True' in text


def test_bridge_never_changes_vehicle_mode_on_startup_or_reconnect() -> None:
    text = CONTROL_BRIDGE.read_text(encoding="utf-8")

    assert "def _set_guided_mode" not in text
    assert ".set_mode_send(" not in text
    assert "MAV_CMD_DO_SET_MODE" not in text


def test_legacy_gateway_is_not_buildable_or_installed_by_bringup() -> None:
    assert LEGACY_GATEWAY_IGNORE.is_file()
    assert not LEGACY_GATEWAY_CONFIG.exists()


def test_real_start_script_requires_takeover_and_physical_motor_ack() -> None:
    text = REAL_START_SCRIPT.read_text(encoding="utf-8")

    assert 'IDA_CANONICAL_TAKEOVER:-false' in text
    assert 'IDA_PHYSICAL_SAFETY_ACK:-' in text
    assert 'PROPELLER_AREA_CLEAR' in text
    assert 'command -v mavlink-routerd' in text
    assert "import mavsdk, pymavlink" in text
    assert "import ultralytics" in text
    assert '[[ -e "$LIDAR_PORT" ]]' in text
    assert '[[ -e "$PIXHAWK_PORT" ]]' in text
    assert 'lsof "$PIXHAWK_PORT"' in text
    assert 'canonical_takeover_enabled:=true' in text
    assert 'mavlink_router_enabled:=true' in text
    assert 'vehicle_setup_enabled:="$SETUP_ENABLED"' in text
    assert 'guided_mode_enabled:="$GUIDED_ENABLED"' in text
    assert 'motor_command_enabled:="$MOTOR_ENABLED"' in text
    assert "gateway_dry_run" not in text
    assert "use_sllidar" not in text


def test_control_declares_the_official_pymavlink_rosdep_and_mavsdk_runtime() -> None:
    package_text = CONTROL_PACKAGE.read_text(encoding="utf-8")
    setup_text = CONTROL_SETUP.read_text(encoding="utf-8")

    assert "<exec_depend>python3-pymavlink-pip</exec_depend>" in package_text
    assert "<exec_depend>python-pymavlink</exec_depend>" not in package_text
    assert '"mavsdk"' in setup_text
    assert '"pymavlink"' in setup_text


def test_sim_explicitly_enables_vehicle_setup_and_motor_path() -> None:
    text = LAUNCH.read_text(encoding="utf-8")

    assert '"vehicle_setup_enabled": True' in text
    assert '"guided_mode_enabled": True' in text
    assert '"motor_command_enabled": True' in text
    assert '"left_motor_servo_channel": 1' in text
    assert '"right_motor_servo_channel": 3' in text


def test_all_loggers_share_one_explicit_run_name() -> None:
    for path, expected_count in (
        (LAUNCH, 4),
        (SIM_FULL_LAUNCH, 5),
        (REAL_LAUNCH, 4),
    ):
        text = path.read_text(encoding="utf-8")
        assert 'DeclareLaunchArgument(\n                "log_run_name"' in text
        assert text.count('"run_name": log_run_name') == expected_count
        assert 'strftime("run_%Y%m%d_%H%M%S_%f_") + uuid4().hex[:8]' in text
