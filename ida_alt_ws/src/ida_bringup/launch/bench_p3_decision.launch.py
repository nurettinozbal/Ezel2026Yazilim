"""Direct-start P3 decision bench profile.

Decision/telemetry-only session: the vehicle starts at parkur 3 without first
satisfying P1/P2 acceptance (bench_p3_only_enabled=true), but the motor path is
closed — dry_run=true, guided_mode_enabled=false, motor_command_enabled=false,
canonical_takeover_enabled=false.  This wrapper never arms the vehicle and never
touches the normal field profile.  A live motor path is rejected outright by
``validate_p3_bench_profile``; the P3 bench is only accepted with the motor path
closed.
"""

from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ida_bringup.bench_contract import (
    BENCH_MAX_SPEED_MPS,
    BENCH_MAX_YAW_RATE_RAD_S,
    BENCH_STUCK_TIMEOUT_S,
    validate_p3_bench_profile,
)


def _validate(context):
    names = (
        "canonical_takeover_enabled",
        "dry_run",
        "mavlink_router_enabled",
        "vehicle_setup_enabled",
        "guided_mode_enabled",
        "motor_command_enabled",
        "fusion_model_loaded",
        "camera_calibrated",
        "lidar_calibrated",
        "extrinsics_calibrated",
        "bench_max_speed_mps",
        "bench_max_yaw_rate_rad_s",
        "bench_stuck_timeout_s",
        "bench_p3_only_enabled",
        "bench_p3_start_parkur",
    )
    values = {name: LaunchConfiguration(name).perform(context) for name in names}
    validate_p3_bench_profile(values, os.environ)
    return []


def generate_launch_description():
    args = [
        # Motor/ownership/mode gates: ALL closed. The P3 bench is a
        # decision/telemetry session; validate_p3_bench_profile rejects any
        # live motor path outright, so these defaults cannot be flipped into a
        # motor-enabled P3 bench.
        DeclareLaunchArgument("canonical_takeover_enabled", default_value="false"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("mavlink_router_enabled", default_value="false"),
        DeclareLaunchArgument("vehicle_setup_enabled", default_value="false"),
        DeclareLaunchArgument("guided_mode_enabled", default_value="false"),
        DeclareLaunchArgument("motor_command_enabled", default_value="false"),
        DeclareLaunchArgument(
            "bench_max_speed_mps", default_value=str(BENCH_MAX_SPEED_MPS)
        ),
        DeclareLaunchArgument(
            "bench_max_yaw_rate_rad_s",
            default_value=str(BENCH_MAX_YAW_RATE_RAD_S),
        ),
        DeclareLaunchArgument(
            "bench_stuck_timeout_s", default_value=str(BENCH_STUCK_TIMEOUT_S)
        ),
        # P3 direct-start deployment flags (default fail-closed: disabled).
        DeclareLaunchArgument("bench_p3_only_enabled", default_value="true"),
        DeclareLaunchArgument("bench_p3_start_parkur", default_value="3"),
        DeclareLaunchArgument("pixhawk_serial_port", default_value=""),
        DeclareLaunchArgument("pixhawk_baud", default_value="115200"),
        DeclareLaunchArgument("lidar_serial_port", default_value=""),
        DeclareLaunchArgument("lidar_serial_baudrate", default_value="1000000"),
        DeclareLaunchArgument("lidar_scan_mode", default_value="DenseBoost"),
        DeclareLaunchArgument(
            "camera_topic", default_value="/camera/image_raw"
        ),
        DeclareLaunchArgument("camera_topic_type", default_value="raw"),
        # Bench-only camera source. The field launch remains unchanged and an
        # external camera publisher can be used by setting this false.
        DeclareLaunchArgument("bench_camera_driver_enabled", default_value="true"),
        DeclareLaunchArgument("bench_camera_device", default_value=""),
        DeclareLaunchArgument("bench_camera_width", default_value="960"),
        DeclareLaunchArgument("bench_camera_height", default_value="600"),
        DeclareLaunchArgument("bench_camera_fps", default_value="30"),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("model_path_p3", default_value=""),
        DeclareLaunchArgument("fusion_model_loaded", default_value="false"),
        DeclareLaunchArgument("camera_calibrated", default_value="false"),
        DeclareLaunchArgument("lidar_calibrated", default_value="false"),
        DeclareLaunchArgument("extrinsics_calibrated", default_value="false"),
        DeclareLaunchArgument("left_motor_servo_channel", default_value="9"),
        DeclareLaunchArgument("right_motor_servo_channel", default_value="11"),
        DeclareLaunchArgument("target_color_param", default_value="SCR_USER4"),
        DeclareLaunchArgument("mission_counts_param", default_value="SCR_USER5"),
        DeclareLaunchArgument("mission_control_param", default_value="SCR_USER6"),
        DeclareLaunchArgument("log_dir", default_value="./logs"),
    ]

    forwarded = {
        name: LaunchConfiguration(name)
        for name in (
            "canonical_takeover_enabled",
            "dry_run",
            "mavlink_router_enabled",
            "vehicle_setup_enabled",
            "guided_mode_enabled",
            "motor_command_enabled",
            "pixhawk_serial_port",
            "pixhawk_baud",
            "lidar_serial_port",
            "lidar_serial_baudrate",
            "lidar_scan_mode",
            "camera_topic",
            "camera_topic_type",
            "model_path",
            "model_path_p3",
            "fusion_model_loaded",
            "camera_calibrated",
            "lidar_calibrated",
            "extrinsics_calibrated",
            "left_motor_servo_channel",
            "right_motor_servo_channel",
            "target_color_param",
            "mission_counts_param",
            "mission_control_param",
            "log_dir",
        )
    }
    forwarded.update(
        {
            # Deployment parameters (never field_profile.yaml tuning): the
            # direct-start P3 override is handed to the autonomy node here.
            "bench_p3_only_enabled": LaunchConfiguration("bench_p3_only_enabled"),
            "bench_p3_start_parkur": LaunchConfiguration("bench_p3_start_parkur"),
            "autonomy_max_speed_mps": LaunchConfiguration("bench_max_speed_mps"),
            # This is a second, bridge-side physical envelope.  Even if DWA
            # requests an aggressive recovery pivot, the restrained bench
            # cannot exceed the measured conservative yaw rate.
            "guided_max_forward_mps": LaunchConfiguration("bench_max_speed_mps"),
            "guided_max_reverse_mps": "0.0",
            "guided_max_yaw_rate_rad_s": LaunchConfiguration(
                "bench_max_yaw_rate_rad_s"
            ),
            "autonomy_stuck_timeout_s": LaunchConfiguration(
                "bench_stuck_timeout_s"
            ),
        }
    )
    real_launch = PathJoinSubstitution(
        [FindPackageShare("ida_bringup"), "launch", "real_vehicle.launch.py"]
    )
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_launch),
        launch_arguments=forwarded.items(),
    )
    return LaunchDescription(
        [
            *args,
            OpaqueFunction(function=_validate),
            include,
        ]
    )
