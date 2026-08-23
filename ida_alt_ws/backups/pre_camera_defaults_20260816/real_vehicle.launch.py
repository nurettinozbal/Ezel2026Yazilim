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
    mavlink_router_enabled = LaunchConfiguration("mavlink_router_enabled")
    pixhawk_serial_port = LaunchConfiguration("pixhawk_serial_port")
    pixhawk_baud = LaunchConfiguration("pixhawk_baud")
    model_path = LaunchConfiguration("model_path")
    class_names = LaunchConfiguration("class_names")
    allowed_colors = LaunchConfiguration("allowed_colors")
    model_path_p3 = LaunchConfiguration("model_path_p3")
    class_names_p3 = LaunchConfiguration("class_names_p3")
    allowed_colors_p3 = LaunchConfiguration("allowed_colors_p3")
    single_general_model_enabled = LaunchConfiguration("single_general_model_enabled")
    camera_topic = LaunchConfiguration("camera_topic")
    camera_topic_type = LaunchConfiguration("camera_topic_type")
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
            # Field defaults remain unchanged.  The restrained bench wrapper
            # overrides only these two values without editing autonomy.yaml.
            DeclareLaunchArgument("autonomy_max_speed_mps", default_value="0.6"),
            DeclareLaunchArgument("autonomy_stuck_timeout_s", default_value="3.0"),
            DeclareLaunchArgument("guided_max_forward_mps", default_value="0.6"),
            DeclareLaunchArgument("guided_max_reverse_mps", default_value="0.6"),
            DeclareLaunchArgument("guided_max_yaw_rate_rad_s", default_value="0.785398163"),
            DeclareLaunchArgument("left_motor_servo_channel", default_value="9"),
            DeclareLaunchArgument("right_motor_servo_channel", default_value="11"),
            DeclareLaunchArgument("target_color_param", default_value="SCR_USER4"),
            DeclareLaunchArgument("mission_counts_param", default_value="SCR_USER5"),
            DeclareLaunchArgument("mission_control_param", default_value="SCR_USER6"),
            DeclareLaunchArgument("system_address", default_value="udp://127.0.0.1:14540"),
            DeclareLaunchArgument("pymavlink_address", default_value="udpin:127.0.0.1:14541"),
            # Varsayılan kapalı: takımın mevcut Jetson sürecine dokunmaz. Canlı alternatif
            # stack için dry_run:=false ile birlikte açılır. Router yoksa bridge UDP'de
            # heartbeat alamaz ve fail-closed kalır.
            DeclareLaunchArgument("mavlink_router_enabled", default_value="false"),
            DeclareLaunchArgument("pixhawk_serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("pixhawk_baud", default_value="115200"),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("class_names", default_value="orange,yellow"),
            DeclareLaunchArgument("allowed_colors", default_value="orange,yellow"),
            DeclareLaunchArgument("model_path_p3", default_value=""),
            DeclareLaunchArgument("class_names_p3", default_value="red,green,black"),
            DeclareLaunchArgument("allowed_colors_p3", default_value="red,green,black"),
            DeclareLaunchArgument("single_general_model_enabled", default_value="false"),
            DeclareLaunchArgument("camera_topic", default_value="/camera/image_raw/compressed"),
            DeclareLaunchArgument("camera_topic_type", default_value="compressed"),
            DeclareLaunchArgument("lidar_serial_port", default_value="/dev/ttyUSB0"),
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
            Node(
                package="ida_control",
                executable="command_limiter_node",
                name="ida_command_limiter",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "command_limits.yaml"])],
            ),
            Node(
                package="ida_control",
                executable="mavsdk_bridge_node",
                name="ida_mavsdk_bridge",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[
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
                        "mission_control_poll_hz": 2.0,
                        "mission_control_ros_ack_timeout_s": 1.0,
                        "target_color_poll_hz": 1.0,
                        "mission_download_hz": 0.2,
                        "mission_raw_enabled": True,
                        # YKİ + MAVSDK aynı anda açıkken param poll'unu koru
                        # (Pixhawk portlar arası ACK forward'ı zaman aşımı üretebilir).
                        "param_call_timeout_s": 2.0,
                        "param_max_retries": 2,
                        "param_consecutive_fail_limit": 30,
                        "companion_system_id": 1,
                        "companion_component_id": 191,
                        "yki_status_heartbeat_s": 2.0,
                        "vehicle_setup_enabled": vehicle_setup_enabled,
                        "guided_mode_enabled": guided_mode_enabled,
                        "motor_command_enabled": motor_command_enabled,
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
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[
                    perception_config,
                    {
                        "model_path": model_path,
                        "class_names": class_names,
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
                    single_general_model_enabled, "' != 'true'",
                ])),
                parameters=[
                    perception_config,
                    {
                        "model_path": model_path_p3,
                        "class_names": class_names_p3,
                        "allowed_colors": allowed_colors_p3,
                        "camera_topic": camera_topic,
                        "camera_topic_type": camera_topic_type,
                        "output_topic": "/perception/camera/p3/raw",
                        "processed_image_topic": "/perception/processed_image/p3",
                    },
                ],
            ),
            # sllidar_ros2 sürücüsü: seri porttan /scan (sensor_msgs/LaserScan) yayınlar.
            # S2 resmi profil: /dev/ttyUSB0, DenseBoost, baudrate 1 Mbps.
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar_node",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": lidar_serial_port,
                        "serial_baudrate": ParameterValue(
                            LaunchConfiguration("lidar_serial_baudrate"), value_type=int
                        ),
                        "frame_id": "laser",
                        "inverted": False,
                        "angle_compensate": True,
                        "scan_mode": LaunchConfiguration("lidar_scan_mode"),
                        "scan_frequency": 10.0,
                    }
                ],
            ),
            # sllidar köprüsü: /scan -> /perception/obstacles (JSON kontratı).
            Node(
                package="ida_perception",
                executable="sllidar_bridge_node",
                name="ida_sllidar_bridge",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[perception_config, {
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
                parameters=[fusion_config, {
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
                parameters=[logging_config, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            Node(
                package="ida_logging",
                executable="video_logger_node",
                name="ida_video_logger",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[logging_config, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            Node(
                package="ida_logging",
                executable="map_logger_node",
                name="ida_map_logger",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[logging_config, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            Node(
                package="ida_logging",
                executable="logging_status_node",
                name="ida_logging_status",
                output="screen",
                condition=IfCondition(canonical_takeover_enabled),
                parameters=[logging_config, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
        ]
    )
