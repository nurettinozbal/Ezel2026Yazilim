"""Manually started, full canonical field stack.

This wrapper owns the camera source and includes ``real_vehicle.launch.py``.
It never arms, changes mode, uploads a mission or starts autonomy by itself.
All live gates default closed and are validated before any node is created.
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ida_bringup.field_stack_contract import validate_field_profile


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
        "field_camera_driver_enabled",
        "single_general_model_enabled",
        "max_speed_mps",
        "max_yaw_rate_rad_s",
    )
    values = {name: LaunchConfiguration(name).perform(context) for name in names}
    validate_field_profile(values, os.environ)
    return []


def _camera_nodes(context):
    takeover = LaunchConfiguration("canonical_takeover_enabled").perform(context)
    enabled = LaunchConfiguration("field_camera_driver_enabled").perform(context)
    if takeover != "true" or enabled != "true":
        return []
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
                "video_device": LaunchConfiguration("field_camera_device"),
                "pixel_format": "YUYV",
                "output_encoding": "rgb8",
                "image_size": [width, height],
                "time_per_frame": [1, fps],
                "camera_frame_id": "camera_optical",
            }],
        )
    ]


def generate_launch_description():
    defaults = {
        "canonical_takeover_enabled": "false",
        "dry_run": "true",
        "mavlink_router_enabled": "false",
        "vehicle_setup_enabled": "false",
        "guided_mode_enabled": "false",
        "motor_command_enabled": "false",
        "fusion_model_loaded": "false",
        "camera_calibrated": "false",
        "lidar_calibrated": "false",
        "extrinsics_calibrated": "false",
        "field_camera_driver_enabled": "true",
        "field_camera_device": "/dev/video0",
        "field_camera_width": "960",
        "field_camera_height": "600",
        "field_camera_fps": "30",
        "camera_topic": "/camera/image_raw",
        "camera_topic_type": "raw",
        "pixhawk_serial_port": "/dev/ttyACM0",
        "pixhawk_baud": "115200",
        "lidar_serial_port": "/dev/ttyUSB0",
        "lidar_serial_baudrate": "1000000",
        "lidar_scan_mode": "DenseBoost",
        "model_path": "",
        "class_names": "orange,yellow",
        "allowed_colors": "orange,yellow",
        "model_path_p3": "",
        "class_names_p3": "red,green,black",
        "allowed_colors_p3": "red,green,black",
        "single_general_model_enabled": "true",
        "max_speed_mps": "0.6",
        "max_yaw_rate_rad_s": "0.785398163",
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
        "fusion_model_loaded", "camera_calibrated", "lidar_calibrated",
        "extrinsics_calibrated", "camera_topic", "camera_topic_type",
        "pixhawk_serial_port", "pixhawk_baud", "lidar_serial_port",
        "lidar_serial_baudrate", "lidar_scan_mode", "model_path", "class_names",
        "allowed_colors", "model_path_p3", "class_names_p3", "allowed_colors_p3",
        "single_general_model_enabled",
        "left_motor_servo_channel", "right_motor_servo_channel",
        "target_color_param", "mission_counts_param", "mission_control_param", "log_dir",
    )
    forwarded = {name: LaunchConfiguration(name) for name in forwarded_names}
    forwarded.update({
        "autonomy_max_speed_mps": LaunchConfiguration("max_speed_mps"),
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
        OpaqueFunction(function=_validate),
        OpaqueFunction(function=_camera_nodes),
        include,
    ])
