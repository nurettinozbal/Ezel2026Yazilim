"""Manually started, full canonical field stack.

This wrapper owns the camera source and includes ``real_vehicle.launch.py``.
It never arms, changes mode, uploads a mission or starts autonomy by itself.
All live gates default closed and are validated before any node is created.
"""

from __future__ import annotations

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ida_bringup.field_stack_contract import (
    validate_field_drive_envelope,
    validate_thruster_hardware_profile,
    validate_field_launch_tuning,
    validate_field_tuning_coverage,
    validate_field_profile,
)
from ida_bringup.camera_profile import (
    controls_to_ros_parameters,
    load_camera_control_profile,
)


def _load_field_profile():
    path = (
        Path(get_package_share_directory("ida_bringup"))
        / "config"
        / "field_profile.yaml"
    )
    profile = yaml.safe_load(path.read_text(encoding="utf-8"))
    validate_field_drive_envelope(profile)
    validate_thruster_hardware_profile(profile)
    validate_field_tuning_coverage(profile)
    return path, profile


def _profile_params(profile, node_name):
    return profile[node_name]["ros__parameters"]


def _validate(context):
    names = (
        "canonical_takeover_enabled",
        "dry_run",
        "mavlink_router_enabled",
        "vehicle_setup_enabled",
        "guided_mode_enabled",
        "motor_command_enabled",
        "motor_output_limits_check_enabled",
        "expected_mot_thr_min_pct",
        "expected_mot_thr_max_pct",
        "expected_mot_slewrate_pct_s",
        "motor_output_limits_poll_s",
        "fusion_model_loaded",
        "camera_calibrated",
        "lidar_calibrated",
        "extrinsics_calibrated",
        "field_camera_driver_enabled",
        "lidar_enabled",
        "single_general_model_enabled",
        "yki_debug_enabled",
        "yki_debug_websocket_url",
        "max_speed_mps",
        "max_yaw_rate_rad_s",
        "autonomy_stuck_timeout_s",
        "field_camera_width",
        "field_camera_height",
        "field_camera_fps",
        "camera_fov_deg",
        "camera_focal_length_px",
        "lidar_serial_baudrate",
        "lidar_scan_mode",
    )
    values = {name: LaunchConfiguration(name).perform(context) for name in names}
    validate_field_profile(values, os.environ)
    _, profile = _load_field_profile()
    validate_field_launch_tuning(values, profile)
    return []


def _resolve_serial_devices(context):
    if LaunchConfiguration("canonical_takeover_enabled").perform(context) != "true":
        return []
    router_enabled = LaunchConfiguration("mavlink_router_enabled").perform(context) == "true"
    actions = []
    definitions = {
        "pixhawk_serial_port": (
            Path("/dev/idaws_pixhawk"),
            ("ArduPilot", "Pixhawk", "Cube", "PX4", "FMU", "Holybro"),
        ),
        "lidar_serial_port": (
            Path("/dev/idaws_lidar"),
            ("SLAMTEC", "Slamtec", "CP210", "Silicon_Labs"),
        ),
    }
    for name, (stable_alias, markers) in definitions.items():
        if name == "pixhawk_serial_port" and not router_enabled:
            continue
        if (
            name == "lidar_serial_port"
            and LaunchConfiguration("lidar_enabled").perform(context) != "true"
        ):
            continue
        requested = LaunchConfiguration(name).perform(context)
        if requested != "auto":
            continue
        if stable_alias.exists():
            resolved = str(stable_alias)
        else:
            candidates = [
                path for path in sorted(Path("/dev/serial/by-id").glob("*"))
                if any(marker in path.name for marker in markers)
            ]
            if len(candidates) != 1:
                raise ValueError(
                    f"{name} auto-discovery requires exactly one matching device"
                )
            resolved = str(candidates[0])
        actions.append(SetLaunchConfiguration(name, resolved))
    return actions


def _camera_nodes(context):
    takeover = LaunchConfiguration("canonical_takeover_enabled").perform(context)
    enabled = LaunchConfiguration("field_camera_driver_enabled").perform(context)
    if takeover != "true" or enabled != "true":
        return []
    camera_device = LaunchConfiguration("field_camera_device").perform(context)
    if camera_device == "auto":
        candidates = sorted(Path("/dev/v4l/by-id").glob("*Arducam*video-index0"))
        if len(candidates) != 1:
            raise ValueError(
                "field camera auto-discovery requires exactly one Arducam device"
            )
        camera_device = str(candidates[0])
    _, profile = _load_field_profile()
    camera = dict(_profile_params(profile, "ida_field_camera_driver"))
    control_profile_path = os.environ.get("IDA_CAMERA_CONTROL_PROFILE", "")
    if control_profile_path:
        persisted = load_camera_control_profile(control_profile_path)
        if persisted is not None:
            camera.update(controls_to_ros_parameters(persisted["controls"]))
            print(
                "IDA camera persistent field controls loaded: "
                f"{control_profile_path} ({persisted['saved_at_utc']})",
                flush=True,
            )
    width = int(LaunchConfiguration("field_camera_width").perform(context))
    height = int(LaunchConfiguration("field_camera_height").perform(context))
    fps = int(LaunchConfiguration("field_camera_fps").perform(context))
    if width <= 0 or height <= 0 or not 1 <= fps <= 80:
        raise ValueError("field camera width/height/fps are outside safe bounds")
    return [
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            namespace="camera",
            name="ida_field_camera",
            output="screen",
            parameters=[{
                "video_device": camera_device,
                "pixel_format": str(camera["pixel_format"]),
                "output_encoding": str(camera["output_encoding"]),
                "image_size": [width, height],
                "time_per_frame": [1, fps],
                "camera_frame_id": str(camera["camera_frame_id"]),
                "auto_exposure": int(camera["auto_exposure"]),
                "exposure_time_absolute": int(camera["exposure_time_absolute"]),
                "white_balance_automatic": bool(camera["white_balance_automatic"]),
                "white_balance_temperature": int(camera["white_balance_temperature"]),
                "gain": int(camera["gain"]),
                "saturation": int(camera["saturation"]),
                "brightness": int(camera["brightness"]),
                "contrast": int(camera["contrast"]),
                "power_line_frequency": int(camera["power_line_frequency"]),
            }],
        )
    ]


def generate_launch_description():
    profile_path, profile = _load_field_profile()
    autonomy_profile = profile["ida_autonomy"]["ros__parameters"]
    thruster_profile = _profile_params(profile, "ida_thruster_envelope")
    camera_profile = _profile_params(profile, "ida_field_camera_driver")
    lidar_profile = _profile_params(profile, "sllidar_node")
    profile_max_speed = str(float(autonomy_profile["max_speed_mps"]))
    profile_max_yaw_rad = str(
        float(autonomy_profile["max_yaw_rate_deg_s"]) * 3.141592653589793 / 180.0
    )
    profile_stuck_timeout = str(float(autonomy_profile["stuck_timeout_s"]))
    defaults = {
        "canonical_takeover_enabled": "false",
        "dry_run": "true",
        "mavlink_router_enabled": "false",
        "vehicle_setup_enabled": "false",
        "guided_mode_enabled": "false",
        "motor_command_enabled": "false",
        "motor_output_limits_check_enabled": "true",
        "expected_mot_thr_min_pct": str(int(thruster_profile["pixhawk_mot_thr_min_pct"])),
        "expected_mot_thr_max_pct": str(int(thruster_profile["pixhawk_mot_thr_max_pct"])),
        "expected_mot_slewrate_pct_s": str(
            int(thruster_profile["pixhawk_mot_slewrate_pct_s"])
        ),
        "motor_output_limits_poll_s": str(float(thruster_profile["verification_poll_s"])),
        "fusion_model_loaded": "false",
        "camera_calibrated": "false",
        "lidar_calibrated": "false",
        "extrinsics_calibrated": "false",
        "field_camera_driver_enabled": "true",
        "lidar_enabled": "true",
        "field_camera_device": "auto",
        "field_camera_width": str(int(camera_profile["image_width"])),
        "field_camera_height": str(int(camera_profile["image_height"])),
        "field_camera_fps": str(int(camera_profile["camera_fps"])),
        "camera_topic": "/camera/image_raw",
        "camera_topic_type": "raw",
        # Keep runtime bearing/distance geometry aligned with the bench viewer.
        "camera_fov_deg": str(float(camera_profile["fov_deg"])),
        "camera_focal_length_px": str(float(camera_profile["focal_length_px"])),
        "pixhawk_serial_port": "auto",
        "pixhawk_baud": "115200",
        "lidar_serial_port": "auto",
        "lidar_serial_baudrate": str(int(lidar_profile["serial_baudrate"])),
        "lidar_scan_mode": str(lidar_profile["scan_mode"]),
        "model_path": "",
        "class_names": "black,green,orange,red,yellow",
        "class_name_map": "",
        "allowed_colors": "orange,yellow",
        "model_path_p3": "",
        "class_names_p3": "red,green,black",
        "class_name_map_p3": "",
        "allowed_colors_p3": "red,green,black",
        "single_general_model_enabled": "true",
        "yki_debug_enabled": "false",
        "yki_debug_websocket_url": "auto://yki",
        "max_speed_mps": profile_max_speed,
        "max_yaw_rate_rad_s": profile_max_yaw_rad,
        "autonomy_stuck_timeout_s": profile_stuck_timeout,
        "field_profile_path": str(profile_path),
        "left_motor_servo_channel": "9",
        "right_motor_servo_channel": "11",
        "target_color_param": "SCR_USER4",
        "mission_counts_param": "SCR_USER5",
        "mission_control_param": "SCR_USER6",
        "log_dir": "./logs",
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()
    ]
    forwarded_names = (
        "canonical_takeover_enabled", "dry_run", "mavlink_router_enabled",
        "vehicle_setup_enabled", "guided_mode_enabled", "motor_command_enabled",
        "motor_output_limits_check_enabled", "expected_mot_thr_min_pct",
        "expected_mot_thr_max_pct", "expected_mot_slewrate_pct_s",
        "motor_output_limits_poll_s",
        "fusion_model_loaded", "camera_calibrated", "lidar_calibrated",
        "extrinsics_calibrated", "camera_topic", "camera_topic_type",
        "lidar_enabled",
        "camera_fov_deg", "camera_focal_length_px",
        "pixhawk_serial_port", "pixhawk_baud", "lidar_serial_port",
        "lidar_serial_baudrate", "lidar_scan_mode", "model_path", "class_names",
        "class_name_map", "allowed_colors", "model_path_p3", "class_names_p3",
        "class_name_map_p3", "allowed_colors_p3",
        "single_general_model_enabled",
        "yki_debug_enabled", "yki_debug_websocket_url",
        "field_profile_path",
        "left_motor_servo_channel", "right_motor_servo_channel",
        "target_color_param", "mission_counts_param", "mission_control_param", "log_dir",
    )
    forwarded = {name: LaunchConfiguration(name) for name in forwarded_names}
    forwarded.update({
        "camera_enabled": LaunchConfiguration("field_camera_driver_enabled"),
        "autonomy_max_speed_mps": LaunchConfiguration("max_speed_mps"),
        "autonomy_stuck_timeout_s": LaunchConfiguration("autonomy_stuck_timeout_s"),
        "guided_max_forward_mps": LaunchConfiguration("max_speed_mps"),
        "guided_max_reverse_mps": LaunchConfiguration("max_speed_mps"),
        "guided_max_yaw_rate_rad_s": LaunchConfiguration("max_yaw_rate_rad_s"),
    })
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ida_bringup"), "launch", "real_vehicle.launch.py"]
            )
        ),
        launch_arguments=forwarded.items(),
    )
    return LaunchDescription([
        *arguments,
        OpaqueFunction(function=_resolve_serial_devices),
        OpaqueFunction(function=_validate),
        OpaqueFunction(function=_camera_nodes),
        include,
    ])
