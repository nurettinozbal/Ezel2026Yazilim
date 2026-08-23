"""Explicit lab-only passive test console overlay.

This launch never arms the vehicle and creates no vehicle-motion publisher.  The
Jetson producer token is read by ida_vehicle_test from the
EZEL_JETSON_DEBUG_TOKEN environment variable; it is intentionally not a ROS
parameter or launch argument.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    websocket_url = LaunchConfiguration("websocket_url")
    log_root = LaunchConfiguration("log_root")
    evidence_root = LaunchConfiguration("evidence_root")
    snapshot_hz = LaunchConfiguration("snapshot_hz")
    simulation_sources = LaunchConfiguration("simulation_sources")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "websocket_url",
                default_value="ws://127.0.0.1:5000/ws/jetson-debug",
            ),
            DeclareLaunchArgument("log_root", default_value="./logs"),
            DeclareLaunchArgument(
                "evidence_root", default_value="./vehicle_test_evidence"
            ),
            DeclareLaunchArgument("snapshot_hz", default_value="3.0"),
            DeclareLaunchArgument("simulation_sources", default_value="false"),
            Node(
                package="ida_vehicle_test",
                executable="vehicle_test_monitor",
                name="ida_vehicle_test_monitor",
                output="screen",
                parameters=[
                    {
                        "log_root": log_root,
                        "evidence_root": evidence_root,
                        "allow_sim_sources": simulation_sources,
                    }
                ],
            ),
            Node(
                package="ida_vehicle_test",
                executable="vehicle_test_producer",
                name="ida_vehicle_test_producer",
                output="screen",
                parameters=[
                    {
                        "websocket_url": websocket_url,
                        "snapshot_hz": snapshot_hz,
                    }
                ],
            ),
        ]
    )
