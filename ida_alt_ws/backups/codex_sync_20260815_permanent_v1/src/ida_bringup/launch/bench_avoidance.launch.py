"""Restrained yellow-buoy avoidance bench profile.

Defaults are observe/dry-run only.  This wrapper never arms the vehicle and
keeps the normal field profile untouched.  A live motor path is rejected unless
all explicit ownership/mode gates and the session-local physical acknowledgement
are present.
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ida_bringup.bench_contract import (
    BENCH_MAX_SPEED_MPS,
    BENCH_STUCK_TIMEOUT_S,
    validate_bench_profile,
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
        "bench_stuck_timeout_s",
    )
    values = {name: LaunchConfiguration(name).perform(context) for name in names}
    validate_bench_profile(values, os.environ)
    return []


def _camera_nodes(context):
    takeover = LaunchConfiguration("canonical_takeover_enabled").perform(context)
    enabled = LaunchConfiguration("bench_camera_driver_enabled").perform(context)
    if takeover not in ("true", "false") or enabled not in ("true", "false"):
        raise ValueError("camera/takeover flags must be exactly true or false")
    if takeover != "true" or enabled != "true":
        return []
    width = int(LaunchConfiguration("bench_camera_width").perform(context))
    height = int(LaunchConfiguration("bench_camera_height").perform(context))
    fps = int(LaunchConfiguration("bench_camera_fps").perform(context))
    if width <= 0 or height <= 0 or not 1 <= fps <= 80:
        raise ValueError("bench camera width/height/fps are outside safe bounds")
    return [
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            namespace="camera",
            name="bench_v4l2_camera",
            output="screen",
            parameters=[
                {
                    "video_device": LaunchConfiguration("bench_camera_device"),
                    "pixel_format": "YUYV",
                    "output_encoding": "rgb8",
                    "image_size": [width, height],
                    "time_per_frame": [1, fps],
                    "camera_frame_id": "camera_optical",
                }
            ],
        ),
    ]


def generate_launch_description():
    args = [
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
            "bench_stuck_timeout_s", default_value=str(BENCH_STUCK_TIMEOUT_S)
        ),
        DeclareLaunchArgument("pixhawk_serial_port", default_value="/dev/pixhawk"),
        DeclareLaunchArgument("pixhawk_baud", default_value="115200"),
        DeclareLaunchArgument("lidar_serial_port", default_value="/dev/lidar"),
        DeclareLaunchArgument(
            "camera_topic", default_value="/camera/image_raw"
        ),
        DeclareLaunchArgument("camera_topic_type", default_value="raw"),
        # Bench-only camera source. The field launch remains unchanged and an
        # external camera publisher can be used by setting this false.
        DeclareLaunchArgument("bench_camera_driver_enabled", default_value="true"),
        DeclareLaunchArgument("bench_camera_device", default_value="/dev/video0"),
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
            "autonomy_max_speed_mps": LaunchConfiguration("bench_max_speed_mps"),
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
            OpaqueFunction(function=_camera_nodes),
            include,
        ]
    )
