"""Isolated rosbag-replay alternative for Jetson camera/lidar fusion.

The launch refuses to start on the declared production ROS domain. It opens no
sensor device, control bridge, autonomy node, gateway or logger. Inputs must be
provided by rosbag playback in the isolated domain.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ALT = "/ida_alt"


def _require_isolated_domain(_context):
    if os.environ.get("IDA_OFFLINE_REPLAY", "") != "1":
        raise RuntimeError("set IDA_OFFLINE_REPLAY=1 for the isolated replay launch")
    try:
        domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
        production = int(os.environ.get("IDA_PRODUCTION_ROS_DOMAIN_ID", "0"))
    except ValueError as exc:
        raise RuntimeError("ROS domain IDs must be integers") from exc
    if not 0 <= domain <= 232 or not 0 <= production <= 232:
        raise RuntimeError("ROS domain IDs must be in [0,232]")
    if domain == production:
        raise RuntimeError("offline replay ROS_DOMAIN_ID equals production domain")
    return []


def generate_launch_description():
    camera_topic = LaunchConfiguration("camera_topic")
    model_path_p1p2 = LaunchConfiguration("model_path_p1p2")
    model_path_p3 = LaunchConfiguration("model_path_p3")
    calibration_ready = ParameterValue(
        LaunchConfiguration("calibration_ready"), value_type=bool
    )

    return LaunchDescription(
        [
            OpaqueFunction(function=_require_isolated_domain),
            DeclareLaunchArgument(
                "camera_topic", default_value="/camera/image_raw/compressed"
            ),
            DeclareLaunchArgument("model_path_p1p2", default_value=""),
            DeclareLaunchArgument("model_path_p3", default_value=""),
            DeclareLaunchArgument("calibration_ready", default_value="false"),
            Node(
                package="ida_perception",
                executable="yolo_camera_node",
                name="ida_yolo_camera",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "model_path": model_path_p1p2,
                    "model_type": "auto",
                    "class_names": "orange,yellow",
                    "camera_index": -1,
                    "camera_topic": camera_topic,
                    "output_topic": f"{ALT}/perception/camera/p1p2/raw",
                    "processed_image_topic": f"{ALT}/perception/processed_image/p1p2",
                }],
            ),
            Node(
                package="ida_perception",
                executable="yolo_camera_node",
                name="ida_yolo_camera_p3",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "model_path": model_path_p3,
                    "model_type": "auto",
                    "class_names": "red,green,black",
                    "camera_index": -1,
                    "camera_topic": camera_topic,
                    "output_topic": f"{ALT}/perception/camera/p3/raw",
                    "processed_image_topic": f"{ALT}/perception/processed_image/p3",
                }],
            ),
            Node(
                package="ida_perception",
                executable="sllidar_bridge_node",
                name="ida_sllidar_bridge",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "scan_topic": "/scan",
                    "output_topic": f"{ALT}/perception/lidar/raw_obstacles",
                    "raw_contract": True,
                }],
            ),
            Node(
                package="ida_sensor_fusion",
                executable="sensor_fusion_node",
                name="ida_sensor_fusion",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "camera_topic_p1p2": f"{ALT}/perception/camera/p1p2/raw",
                    "camera_topic_p3": f"{ALT}/perception/camera/p3/raw",
                    "lidar_topic": f"{ALT}/perception/lidar/raw_obstacles",
                    "shadow_mode": True,
                    "buoy_output_topic": f"{ALT}/perception/fusion/shadow/buoys",
                    "obstacle_output_topic": f"{ALT}/perception/fusion/shadow/obstacles",
                    "status_output_topic": f"{ALT}/perception/fusion/status",
                    "model_loaded": calibration_ready,
                    "camera_calibrated": calibration_ready,
                    "lidar_calibrated": calibration_ready,
                    "extrinsics_calibrated": calibration_ready,
                }],
            ),
        ]
    )
