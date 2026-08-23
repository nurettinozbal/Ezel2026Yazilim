from datetime import datetime
from uuid import uuid4

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ida_bringup.sim_field_fidelity import load_sim_field_fidelity


def _resolve_sim_field_fidelity(context):
    enabled_text = LaunchConfiguration("sim_field_fidelity").perform(context).strip().lower()
    if enabled_text not in {"true", "false"}:
        raise ValueError("sim_field_fidelity tam olarak true veya false olmali")
    if enabled_text == "false":
        fallback = LaunchConfiguration("sim_max_speed_mps").perform(context)
        return [
            SetLaunchConfiguration("sim_effective_max_speed_mps", fallback),
            LogInfo(msg=["SIM profil=KNOWN_GOOD, hiz_tavani=", fallback, " m/s"]),
        ]
    values = load_sim_field_fidelity(
        LaunchConfiguration("field_profile_path").perform(context)
    )
    return [
        SetLaunchConfiguration(
            "sim_effective_max_speed_mps", str(values["max_speed_mps"])
        ),
        SetLaunchConfiguration(
            "sim_effective_max_yaw_rate_rad_s",
            str(values["max_yaw_rate_rad_s"]),
        ),
        LogInfo(
            msg=(
                "SIM profil=FIELD_FIDELITY "
                f"hiz_tavani={values['max_speed_mps']:.2f}m/s "
                f"seyir={values['cruise_speed_mps']:.2f}m/s"
            )
        ),
    ]


def generate_launch_description():
    default_log_run_name = (
        datetime.now().strftime("run_%Y%m%d_%H%M%S_%f_") + uuid4().hex[:8]
    )
    default_scenario = PathJoinSubstitution(
        [FindPackageShare("ida_bringup"), "scenarios", "full_mission.yaml"]
    )
    scenario_file = LaunchConfiguration("scenario_file")
    target_color = LaunchConfiguration("target_color")
    log_dir = LaunchConfiguration("log_dir")
    log_run_name = LaunchConfiguration("log_run_name")
    sensor_fusion_enabled = LaunchConfiguration("sensor_fusion_enabled")
    sensor_fusion_shadow_mode = LaunchConfiguration("sensor_fusion_shadow_mode")
    perception_sim_publish_canonical = LaunchConfiguration(
        "perception_sim_publish_canonical"
    )
    perception_sim_lidar_range_m = LaunchConfiguration(
        "perception_sim_lidar_range_m"
    )
    # Simülasyon hız kısması: gazebo'da deniz simülasyonu olmadığından araç
    # gerçek saha zarfından (1.6 m/s) çok daha hızlı hissettirebilir. Bu
    # argüman YALNIZCA simülasyon launch'ında otonomi + limiter tavanını
    # düşürür; gerçek araç (field_profile.yaml) ve bench launch'ları
    # etkilenmez. Algoritma koduna dokunulmaz.
    sim_max_speed_mps = LaunchConfiguration("sim_max_speed_mps")
    sim_effective_max_speed_mps = LaunchConfiguration(
        "sim_effective_max_speed_mps"
    )
    sim_effective_max_yaw_rate_rad_s = LaunchConfiguration(
        "sim_effective_max_yaw_rate_rad_s", default="0.785398163"
    )
    sim_field_fidelity = LaunchConfiguration("sim_field_fidelity")
    field_profile_path = LaunchConfiguration("field_profile_path")

    perception_config = PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "perception.yaml"])
    logging_config = PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "logging.yaml"])
    fusion_config = PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "fusion.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario_file", default_value=default_scenario),
            DeclareLaunchArgument("target_color", default_value="green"),
            DeclareLaunchArgument("log_dir", default_value="./logs"),
            DeclareLaunchArgument(
                "log_run_name",
                default_value=default_log_run_name,
            ),
            DeclareLaunchArgument("sensor_fusion_enabled", default_value="false"),
            DeclareLaunchArgument("sensor_fusion_shadow_mode", default_value="true"),
            DeclareLaunchArgument("perception_sim_publish_canonical", default_value="true"),
            DeclareLaunchArgument("perception_sim_lidar_range_m", default_value="35.0"),
            DeclareLaunchArgument(
                "sim_max_speed_mps",
                default_value="0.6",
                description=(
                    "Simülasyon hız tavanı (m/s). Üretim autonomy ve limiter "
                    "tavanıyla aynı güvenli 0.6 varsayılanını kullanır."
                ),
            ),
            DeclareLaunchArgument("sim_field_fidelity", default_value="false"),
            DeclareLaunchArgument(
                "field_profile_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ida_bringup"), "config", "field_profile.yaml"]
                ),
            ),
            OpaqueFunction(function=_resolve_sim_field_fidelity),
            Node(
                package="ida_autonomy",
                executable="autonomy_node",
                name="ida_autonomy",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "autonomy.yaml"]),
                    {
                        "sim_gate_truth_enabled": True,
                        # Simülasyon hız kısması: autonomy.yaml max_speed_mps=0.6
                        # yerine sim_max_speed_mps (varsayılan 0.6) kullanılır.
                        "max_speed_mps": sim_effective_max_speed_mps,
                    },
                ],
                condition=UnlessCondition(sim_field_fidelity),
            ),
            Node(
                package="ida_autonomy",
                executable="autonomy_node",
                name="ida_autonomy",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "autonomy.yaml"]),
                    field_profile_path,
                    {
                        "sim_gate_truth_enabled": True,
                        "max_speed_mps": sim_effective_max_speed_mps,
                    },
                ],
                condition=IfCondition(sim_field_fidelity),
            ),
            Node(
                package="ida_control",
                executable="command_limiter_node",
                name="ida_command_limiter",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "command_limits.yaml"]),
                    {
                        # Simülasyon hız kısması: limiter tavanı otonomi tavanıyla
                        # hizalı tutulur (sim_max_speed_mps).
                        "max_vx_mps": sim_effective_max_speed_mps,
                    },
                ],
                condition=UnlessCondition(sim_field_fidelity),
            ),
            Node(
                package="ida_control",
                executable="command_limiter_node",
                name="ida_command_limiter",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "command_limits.yaml"]),
                    field_profile_path,
                    {
                        "max_vx_mps": sim_effective_max_speed_mps,
                        "max_yaw_rate_rad_s": sim_effective_max_yaw_rate_rad_s,
                    },
                ],
                condition=IfCondition(sim_field_fidelity),
            ),
            Node(
                package="ida_telemetry_sim",
                executable="telemetry_sim_node",
                name="ida_telemetry_sim",
                output="screen",
                parameters=[{"scenario_file": scenario_file, "auto_start": True, "start_delay_s": 1.5}],
            ),
            Node(
                package="ida_perception_sim",
                executable="perception_sim_node",
                name="ida_perception_sim",
                output="screen",
                parameters=[{
                    "scenario_file": scenario_file,
                    "publish_raw": sensor_fusion_enabled,
                    "publish_canonical": perception_sim_publish_canonical,
                    "lidar_range_m": perception_sim_lidar_range_m,
                }],
            ),
            Node(
                package="ida_sensor_fusion",
                executable="sensor_fusion_node",
                name="ida_sensor_fusion",
                output="screen",
                condition=IfCondition(sensor_fusion_enabled),
                parameters=[
                    fusion_config,
                    {
                        "shadow_mode": sensor_fusion_shadow_mode,
                        "model_loaded": True,
                        "camera_calibrated": True,
                        "lidar_calibrated": True,
                        "extrinsics_calibrated": True,
                    },
                ],
            ),
            Node(
                package="ida_uav_target",
                executable="uav_target_node",
                name="ida_uav_target",
                output="screen",
                parameters=[{"sim_target_color": target_color, "stable_frames": 1}],
            ),
            Node(
                package="ida_perception",
                executable="yolo_camera_node",
                name="ida_yolo_camera",
                output="screen",
                parameters=[perception_config],  # model_path boş -> boş detection
            ),
            # P3 algı akışı (sim: model_path boş -> boş detection; eşleme config'ten).
            Node(
                package="ida_perception",
                executable="yolo_camera_node",
                name="ida_yolo_camera_p3",
                output="screen",
                parameters=[perception_config],
            ),
            Node(
                package="ida_perception",
                executable="lidar_node",
                name="ida_rplidar",
                output="screen",
                parameters=[perception_config],  # dry_run true -> örnek nokta bulutu
            ),
            # NOT: sllidar_bridge_node burada EKLEME — sim'de /scan yayını yok;
            # engel kaynağı ida_perception_sim (aynı /perception/obstacles kontratı).
            Node(
                package="ida_logging",
                executable="telemetry_logger_node",
                name="ida_telemetry_logger",
                output="screen",
                parameters=[logging_config, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            Node(
                package="ida_logging",
                executable="video_logger_node",
                name="ida_video_logger",
                output="screen",
                parameters=[logging_config, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            Node(
                package="ida_logging",
                executable="map_logger_node",
                name="ida_map_logger",
                output="screen",
                parameters=[logging_config, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            Node(
                package="ida_logging",
                executable="events_logger_node",
                name="ida_events_logger",
                output="screen",
                parameters=[logging_config, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            Node(
                package="ida_logging",
                executable="logging_status_node",
                name="ida_logging_status",
                output="screen",
                parameters=[logging_config, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
        ]
    )
