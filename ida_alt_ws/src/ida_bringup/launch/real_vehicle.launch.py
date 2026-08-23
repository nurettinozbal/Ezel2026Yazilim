from datetime import datetime
from uuid import uuid4

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    default_log_run_name = (
        datetime.now().strftime("run_%Y%m%d_%H%M%S_%f_") + uuid4().hex[:8]
    )
    dry_run = LaunchConfiguration("dry_run")
    canonical_takeover_enabled = LaunchConfiguration("canonical_takeover_enabled")
    vehicle_setup_enabled = LaunchConfiguration("vehicle_setup_enabled")
    guided_mode_enabled = LaunchConfiguration("guided_mode_enabled")
    motor_command_enabled = LaunchConfiguration("motor_command_enabled")
    motor_output_limits_check_enabled = LaunchConfiguration(
        "motor_output_limits_check_enabled"
    )
    mavlink_router_enabled = LaunchConfiguration("mavlink_router_enabled")
    pixhawk_serial_port = LaunchConfiguration("pixhawk_serial_port")
    pixhawk_baud = LaunchConfiguration("pixhawk_baud")
    model_path = LaunchConfiguration("model_path")
    class_names = LaunchConfiguration("class_names")
    class_name_map = LaunchConfiguration("class_name_map")
    allowed_colors = LaunchConfiguration("allowed_colors")
    model_path_p3 = LaunchConfiguration("model_path_p3")
    class_names_p3 = LaunchConfiguration("class_names_p3")
    class_name_map_p3 = LaunchConfiguration("class_name_map_p3")
    allowed_colors_p3 = LaunchConfiguration("allowed_colors_p3")
    single_general_model_enabled = LaunchConfiguration("single_general_model_enabled")
    camera_topic = LaunchConfiguration("camera_topic")
    camera_topic_type = LaunchConfiguration("camera_topic_type")
    camera_fov_deg = LaunchConfiguration("camera_fov_deg")
    camera_focal_length_px = LaunchConfiguration("camera_focal_length_px")
    camera_enabled = LaunchConfiguration("camera_enabled")
    lidar_enabled = LaunchConfiguration("lidar_enabled")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    system_address = LaunchConfiguration("system_address")
    # Repo içindeki tty_mavlink_router fiziksel UART'ın tek sahibidir. MAVSDK ve
    # pymavlink yalnız Jetson loopback endpointlerini kullanır; dış ağdan UDP yoktur.
    pymavlink_address = LaunchConfiguration("pymavlink_address")
    log_dir = LaunchConfiguration("log_dir")
    log_run_name = LaunchConfiguration("log_run_name")
    autonomy_max_speed_mps = LaunchConfiguration("autonomy_max_speed_mps")
    autonomy_stuck_timeout_s = LaunchConfiguration("autonomy_stuck_timeout_s")
    guided_max_forward_mps = LaunchConfiguration("guided_max_forward_mps")
    guided_max_reverse_mps = LaunchConfiguration("guided_max_reverse_mps")
    guided_max_yaw_rate_rad_s = LaunchConfiguration("guided_max_yaw_rate_rad_s")
    yki_debug_enabled = LaunchConfiguration("yki_debug_enabled")
    yki_debug_websocket_url = LaunchConfiguration("yki_debug_websocket_url")
    field_profile_path = LaunchConfiguration("field_profile_path")

    perception_config = PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "perception.yaml"])
    fusion_config = PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "fusion.yaml"])
    logging_config = PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "logging.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="true"),
            # Takımın çalışan Jetson stack'iyle aynı ROS domaininde yanlışlıkla
            # ikinci canonical publisher/seri-port sahibi açılmasın. Tam gerçek
            # stack yalnız bilinçli takeover ile başlar.
            DeclareLaunchArgument("canonical_takeover_enabled", default_value="false"),
            # Bağlantı, araç kurulumu ve motor çıkışı bağımsız güvenlik kapılarıdır.
            # Varsayılan launch takımın mevcut Jetson/araç durumuna dokunmaz.
            DeclareLaunchArgument("vehicle_setup_enabled", default_value="false"),
            DeclareLaunchArgument("guided_mode_enabled", default_value="false"),
            DeclareLaunchArgument("motor_command_enabled", default_value="false"),
            DeclareLaunchArgument(
                "motor_output_limits_check_enabled", default_value="false"
            ),
            DeclareLaunchArgument("expected_mot_thr_min_pct", default_value="0"),
            DeclareLaunchArgument("expected_mot_thr_max_pct", default_value="30"),
            DeclareLaunchArgument(
                "expected_mot_slewrate_pct_s", default_value="20"
            ),
            DeclareLaunchArgument("motor_output_limits_poll_s", default_value="5.0"),
            # The real-vehicle entry point must consume the central field
            # profile by default.  Bench wrappers may still pass restrained
            # deployment envelopes explicitly; simulation launches continue
            # to use autonomy.yaml independently.
            DeclareLaunchArgument("autonomy_max_speed_mps", default_value="1.6"),
            DeclareLaunchArgument("autonomy_stuck_timeout_s", default_value="3.0"),
            DeclareLaunchArgument("guided_max_forward_mps", default_value="1.6"),
            DeclareLaunchArgument("guided_max_reverse_mps", default_value="1.6"),
            DeclareLaunchArgument("guided_max_yaw_rate_rad_s", default_value="0.872664626"),
            DeclareLaunchArgument(
                "field_profile_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ida_bringup"), "config", "field_profile.yaml"]
                ),
            ),
            DeclareLaunchArgument("left_motor_servo_channel", default_value="9"),
            DeclareLaunchArgument("right_motor_servo_channel", default_value="11"),
            DeclareLaunchArgument("target_color_param", default_value="SCR_USER4"),
            DeclareLaunchArgument("mission_counts_param", default_value="SCR_USER5"),
            DeclareLaunchArgument("mission_control_param", default_value="SCR_USER6"),
            DeclareLaunchArgument("yki_debug_enabled", default_value="false"),
            DeclareLaunchArgument(
                "yki_debug_websocket_url",
                default_value="auto://yki",
            ),
            # The in-process TTY router sends Pixhawk frames to this loopback
            # port, so MAVSDK must explicitly bind/listen here.  ``udp://host``
            # is an outbound endpoint and silently leaves the router with no
            # consumer.
            DeclareLaunchArgument("system_address", default_value="udpin://127.0.0.1:14540"),
            DeclareLaunchArgument("pymavlink_address", default_value="udpin:127.0.0.1:14541"),
            # Varsayılan kapalı: takımın mevcut Jetson sürecine dokunmaz. Canlı alternatif
            # stack için dry_run:=false ile birlikte açılır. Router yoksa bridge UDP'de
            # heartbeat alamaz ve fail-closed kalır.
            DeclareLaunchArgument("mavlink_router_enabled", default_value="false"),
            DeclareLaunchArgument("pixhawk_serial_port", default_value=""),
            DeclareLaunchArgument("pixhawk_baud", default_value="115200"),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("class_names", default_value="orange,yellow"),
            DeclareLaunchArgument("class_name_map", default_value=""),
            DeclareLaunchArgument("allowed_colors", default_value="orange,yellow"),
            DeclareLaunchArgument("model_path_p3", default_value=""),
            DeclareLaunchArgument("class_names_p3", default_value="red,green,black"),
            DeclareLaunchArgument("class_name_map_p3", default_value=""),
            DeclareLaunchArgument("allowed_colors_p3", default_value="red,green,black"),
            DeclareLaunchArgument("single_general_model_enabled", default_value="false"),
            DeclareLaunchArgument("camera_topic", default_value="/camera/image_raw/compressed"),
            DeclareLaunchArgument("camera_topic_type", default_value="compressed"),
            DeclareLaunchArgument("camera_fov_deg", default_value="90.0"),
            DeclareLaunchArgument("camera_focal_length_px", default_value="600.0"),
            DeclareLaunchArgument("camera_enabled", default_value="true"),
            DeclareLaunchArgument("lidar_enabled", default_value="true"),
            DeclareLaunchArgument("lidar_serial_port", default_value=""),
            DeclareLaunchArgument("lidar_serial_baudrate", default_value="1000000"),
            DeclareLaunchArgument("lidar_scan_mode", default_value="DenseBoost"),
            DeclareLaunchArgument("fusion_model_loaded", default_value="false"),
            DeclareLaunchArgument("camera_calibrated", default_value="false"),
            DeclareLaunchArgument("lidar_calibrated", default_value="false"),
            DeclareLaunchArgument("extrinsics_calibrated", default_value="false"),
            DeclareLaunchArgument("log_dir", default_value="./logs"),
            DeclareLaunchArgument(
                "log_run_name",
                default_value=default_log_run_name,
            ),
            ExecuteProcess(
                cmd=[
                    PathJoinSubstitution(
                        [
                            FindPackagePrefix("ida_control"),
                            "lib",
                            "ida_control",
                            "tty_mavlink_router",
                        ]
                    ),
                    "--device", pixhawk_serial_port,
                    "--baud", pixhawk_baud,
                    "--endpoint-port", "14540",
                    "--endpoint-port", "14541",
                    "--endpoint-port", "14542",
                ],
                name="ida_tty_mavlink_router",
                output="screen",
                condition=IfCondition(PythonExpression([
                    "'", canonical_takeover_enabled, "' == 'true' and '",
                    mavlink_router_enabled, "' == 'true'",
                ])),
                on_exit=[Shutdown(reason="Pixhawk TTY router sonlandı; araç stack'i fail-closed kapatılıyor")],
            ),
            Node(
                package="ida_autonomy",
                executable="autonomy_node",
                name="ida_autonomy",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("ida_bringup"), "config", "autonomy.yaml"]
                    ),
                    field_profile_path,
                    {
                        "max_speed_mps": ParameterValue(
                            autonomy_max_speed_mps, value_type=float
                        ),
                        "stuck_timeout_s": ParameterValue(
                            autonomy_stuck_timeout_s, value_type=float
                        ),
                    },
                ],
            ),
            # Salt-okunur ROS -> YKİ teşhis köprüsü. Aktüatör topic'i üretmez;
            # ayrı EZEL_JETSON_DEBUG_TOKEN ortam değişkeni yoksa fail-closed kalır.
            Node(
                package="ida_vehicle_test",
                executable="vehicle_test_producer",
                name="ida_vehicle_test_producer",
                output="screen",
                condition=IfCondition(PythonExpression([
                    "'", canonical_takeover_enabled, "' == 'true' and '",
                    yki_debug_enabled, "' == 'true'",
                ])),
                parameters=[{"websocket_url": yki_debug_websocket_url}],
            ),
            Node(
                package="ida_control",
                executable="command_limiter_node",
                name="ida_command_limiter",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("ida_bringup"), "config", "command_limits.yaml"]
                    ),
                    field_profile_path,
                    # Otonomi, limiter ve GUIDED bridge aynı saha hız tavanını
                    # kullanmalı. Field-stack wrapper ve bu entry point varsayılan
                    # olarak aynı saha profilinden türetir; bench wrapper'ları
                    # yalnızca açıkça restrained deployment değeri geçirir.
                    {
                        "max_vx_mps": ParameterValue(
                            autonomy_max_speed_mps, value_type=float
                        )
                    },
                ],
            ),
            Node(
                package="ida_control",
                executable="mavsdk_bridge_node",
                name="ida_mavsdk_bridge",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[
                    field_profile_path,
                    {
                        "dry_run": dry_run,
                        "system_address": system_address,
                        # pymavlink komut yolu (GUIDED + SET_POSITION_TARGET):
                        # MAVSDK system_address'ten AYRI port (çift istemci koruması).
                        "pymavlink_address": pymavlink_address,
                        # Takım stack'i SCR_USER1/2/3 kullanır; canonical alanlar ayrıdır.
                        # Varsayılanlar mission_contract/param kontratıyla uyumludur.
                        "target_color_param": LaunchConfiguration("target_color_param"),
                        "mission_counts_param": LaunchConfiguration("mission_counts_param"),
                        "mission_control_param": LaunchConfiguration("mission_control_param"),
                        "companion_system_id": 1,
                        "companion_component_id": 191,
                        "vehicle_setup_enabled": vehicle_setup_enabled,
                        "guided_mode_enabled": guided_mode_enabled,
                        "motor_command_enabled": motor_command_enabled,
                        "motor_output_limits_check_enabled": (
                            motor_output_limits_check_enabled
                        ),
                        "expected_mot_thr_min_pct": ParameterValue(
                            LaunchConfiguration("expected_mot_thr_min_pct"), value_type=int
                        ),
                        "expected_mot_thr_max_pct": ParameterValue(
                            LaunchConfiguration("expected_mot_thr_max_pct"), value_type=int
                        ),
                        "expected_mot_slewrate_pct_s": ParameterValue(
                            LaunchConfiguration("expected_mot_slewrate_pct_s"), value_type=int
                        ),
                        "motor_output_limits_poll_s": ParameterValue(
                            LaunchConfiguration("motor_output_limits_poll_s"), value_type=float
                        ),
                        "guided_max_forward_mps": ParameterValue(
                            guided_max_forward_mps, value_type=float
                        ),
                        "guided_max_reverse_mps": ParameterValue(
                            guided_max_reverse_mps, value_type=float
                        ),
                        "guided_max_yaw_rate_rad_s": ParameterValue(
                            guided_max_yaw_rate_rad_s, value_type=float
                        ),
                        "left_motor_servo_channel": ParameterValue(
                            LaunchConfiguration("left_motor_servo_channel"), value_type=int
                        ),
                        "right_motor_servo_channel": ParameterValue(
                            LaunchConfiguration("right_motor_servo_channel"), value_type=int
                        ),
                    }
                ],
            ),
            Node(
                package="ida_perception",
                executable="yolo_camera_node",
                name="ida_yolo_camera",
                output="screen",
                condition=IfCondition(PythonExpression([
                    "'", canonical_takeover_enabled, "' == 'true' and '",
                    camera_enabled, "' == 'true'",
                ])),
                parameters=[
                    perception_config,
                    field_profile_path,
                    {
                        "model_path": model_path,
                        "class_names": class_names,
                        "class_name_map": ParameterValue(
                            class_name_map, value_type=str
                        ),
                        "allowed_colors": allowed_colors,
                        "secondary_output_topic": ParameterValue(
                            PythonExpression([
                                "'/perception/camera/p3/raw' if '",
                                single_general_model_enabled,
                                "' == 'true' else ''",
                            ]),
                            value_type=str,
                        ),
                        "secondary_allowed_colors": allowed_colors_p3,
                        "camera_topic": camera_topic,
                        "camera_topic_type": camera_topic_type,
                        "output_topic": "/perception/camera/p1p2/raw",
                        "processed_image_topic": "/perception/processed_image/p1p2",
                        "fov_deg": ParameterValue(camera_fov_deg, value_type=float),
                        "focal_length_px": ParameterValue(
                            camera_focal_length_px, value_type=float
                        ),
                    },
                ],
            ),
            # Ayrı P3 modeli yalnız single-general modu kapalıyken açılır. Saha
            # profilinde tek general inference iki role fan-out edildiği için bu
            # ikinci GPU yükü oluşturulmaz.
            Node(
                package="ida_perception",
                executable="yolo_camera_node",
                name="ida_yolo_camera_p3",
                output="screen",
                condition=IfCondition(PythonExpression([
                    "'", canonical_takeover_enabled, "' == 'true' and '",
                    camera_enabled, "' == 'true' and '",
                    single_general_model_enabled, "' != 'true'",
                ])),
                parameters=[
                    perception_config,
                    field_profile_path,
                    {
                        "model_path": model_path_p3,
                        "class_names": class_names_p3,
                        "class_name_map": ParameterValue(
                            class_name_map_p3, value_type=str
                        ),
                        "allowed_colors": allowed_colors_p3,
                        "camera_topic": camera_topic,
                        "camera_topic_type": camera_topic_type,
                        "output_topic": "/perception/camera/p3/raw",
                        "processed_image_topic": "/perception/processed_image/p3",
                        "fov_deg": ParameterValue(camera_fov_deg, value_type=float),
                        "focal_length_px": ParameterValue(
                            camera_focal_length_px, value_type=float
                        ),
                    },
                ],
            ),
            # sllidar_ros2 sürücüsü: seri porttan /scan (sensor_msgs/LaserScan) yayınlar.
            # S2 resmi profil: keşfedilmiş kalıcı seri kimlik, DenseBoost, 1 Mbps.
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_node",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                condition=IfCondition(PythonExpression([
                    "'", canonical_takeover_enabled, "' == 'true' and '",
                    lidar_enabled, "' == 'true'",
                ])),
                parameters=[
                    field_profile_path,
                    {
                        "channel_type": "serial",
                        "serial_port": lidar_serial_port,
                        "serial_baudrate": ParameterValue(
                            LaunchConfiguration("lidar_serial_baudrate"), value_type=int
                        ),
                        "scan_mode": LaunchConfiguration("lidar_scan_mode"),
                    }
                ],
            ),
            # sllidar köprüsü: /scan -> /perception/obstacles (JSON kontratı).
            Node(
                package="ida_perception",
                executable="sllidar_bridge_node",
                name="ida_sllidar_bridge",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                condition=IfCondition(PythonExpression([
                    "'", canonical_takeover_enabled, "' == 'true' and '",
                    lidar_enabled, "' == 'true'",
                ])),
                parameters=[perception_config, field_profile_path, {
                    "output_topic": "/perception/lidar/raw_obstacles",
                    "raw_contract": True,
                }],
            ),
            Node(
                package="ida_sensor_fusion",
                executable="sensor_fusion_node",
                name="ida_sensor_fusion",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[fusion_config, field_profile_path, {
                    "shadow_mode": False,
                    "buoy_output_topic": "/perception/buoys",
                    "obstacle_output_topic": "/perception/obstacles",
                    "model_loaded": ParameterValue(
                        LaunchConfiguration("fusion_model_loaded"), value_type=bool
                    ),
                    "camera_calibrated": ParameterValue(
                        LaunchConfiguration("camera_calibrated"), value_type=bool
                    ),
                    "lidar_calibrated": ParameterValue(
                        LaunchConfiguration("lidar_calibrated"), value_type=bool
                    ),
                    "extrinsics_calibrated": ParameterValue(
                        LaunchConfiguration("extrinsics_calibrated"), value_type=bool
                    ),
                }],
            ),
            Node(
                package="ida_logging",
                executable="telemetry_logger_node",
                name="ida_telemetry_logger",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[logging_config, field_profile_path, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            Node(
                package="ida_logging",
                executable="video_logger_node",
                name="ida_video_logger",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[logging_config, field_profile_path, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            Node(
                package="ida_logging",
                executable="map_logger_node",
                name="ida_map_logger",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[logging_config, field_profile_path, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            Node(
                package="ida_logging",
                executable="logging_status_node",
                name="ida_logging_status",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[logging_config, field_profile_path, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
        ]
    )
