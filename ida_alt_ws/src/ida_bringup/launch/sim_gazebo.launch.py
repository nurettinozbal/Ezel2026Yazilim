"""sim_gazebo.launch.py — ArduRover SITL + Gazebo Classic + İDA stack birlikte.

Fizik ArduRover SITL'de koşar (sim_vehicle.py); Gazebo'daki `ida_boat` modeli
SITL konumunu İZLER (gazebo_pose_sync_node /telemetry/state'i dinler, pose'u
gazebo_msgs SetEntityState ile yazar). Kamera/lidar plugin'leri modele bağlı
olduğundan SITL ile senkron hareket eder.

ÖNEMLİ NOTLAR:
1. telemetry_sim_node BİLEREK BU LAUNCH'TA YOK — mavsdk_bridge (dry_run=false,
   system_address=udp://:14540) zaten /telemetry/state yayınlar; ikisi çakışır.
2. KOMUT YOLU: ArduRover'da MAVSDK offboard YOKTUR; gövde hızı komutları
   pymavlink ile GUIDED + SET_POSITION_TARGET_LOCAL_NED (velocity + yaw_rate)
   üzerinden gönderilir (mavsdk_bridge._command_loop_pymavlink). pymavlink
   bağlantısı MAVSDK system_address'ten AYRI port kullanmalıdır (çift istemci
   çakışması — COZUM_MAVLINK_CAKISMASI): SITL'de UDP 14541 (MAVSDK 14540).
   Telemetri/poll task'ları pymavlink bağlantısından bağımsızdır — komut yolu
   ölse de /telemetry/state akışı devam eder (Gazebo senkronu korunur).
3. SITL home: --home=40.8630501,29.2599517,0,0 (İstanbul origin). Aksi halde
   sim_vehicle.py default Canberra'da başlar ve perception_sim yanlış local
   konum hesaplar (scenario origin'iyle uyuşmaz).
"""

import os
from datetime import datetime
from uuid import uuid4

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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

    profile_path = LaunchConfiguration("field_profile_path").perform(context)
    values = load_sim_field_fidelity(profile_path)
    summary = (
        "SIM profil=FIELD_FIDELITY "
        f"hiz_tavani={values['max_speed_mps']:.2f}m/s "
        f"seyir={values['cruise_speed_mps']:.2f}m/s "
        f"MOT_THR={values['mot_thr_min_pct']:.0f}-{values['mot_thr_max_pct']:.0f}% "
        f"slew={values['mot_slewrate_pct_s']:.0f}%/s "
        "(MOT degerleri bu asamada bilgi; SITL/Pixhawk'a yazilmaz)"
    )
    return [
        SetLaunchConfiguration(
            "sim_effective_max_speed_mps", str(values["max_speed_mps"])
        ),
        SetLaunchConfiguration(
            "sim_effective_max_yaw_rate_rad_s",
            str(values["max_yaw_rate_rad_s"]),
        ),
        LogInfo(msg=summary),
    ]


def generate_launch_description():
    default_log_run_name = (
        datetime.now().strftime("run_%Y%m%d_%H%M%S_%f_") + uuid4().hex[:8]
    )
    default_scenario = PathJoinSubstitution(
        [FindPackageShare("ida_bringup"), "scenarios", "full_mission.yaml"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("ida_bringup"), "worlds", "sim_gazebo.world"]
    )
    # Launch argümanlarını tek noktadan oku (varsayılanlar kontratı korur).
    scenario_file = LaunchConfiguration("scenario_file")
    target_color = LaunchConfiguration("target_color")
    log_dir = LaunchConfiguration("log_dir")
    log_run_name = LaunchConfiguration("log_run_name")
    home = LaunchConfiguration("home")
    speedup = LaunchConfiguration("speedup")
    yki_mavlink_host = LaunchConfiguration("yki_mavlink_host")
    yki_mavlink_port = LaunchConfiguration("yki_mavlink_port")
    sitl_wipe = LaunchConfiguration("sitl_wipe")
    system_address = LaunchConfiguration("system_address")
    pymavlink_address = LaunchConfiguration("pymavlink_address")
    mission_address = LaunchConfiguration("mission_address")
    sim_vehicle_path = LaunchConfiguration("sim_vehicle_path")
    world_file = LaunchConfiguration("world_file")
    model_name = LaunchConfiguration("model_name")
    sensor_fusion_enabled = LaunchConfiguration("sensor_fusion_enabled")
    sensor_fusion_shadow_mode = LaunchConfiguration("sensor_fusion_shadow_mode")
    perception_sim_publish_canonical = LaunchConfiguration(
        "perception_sim_publish_canonical"
    )
    perception_sim_lidar_range_m = LaunchConfiguration(
        "perception_sim_lidar_range_m"
    )
    # Simülasyon hız kısması: Gazebo'da deniz direnci yok; araç gerçek saha
    # zarfından çok daha hızlı hissettirebilir. Yalnızca sim launch'ında
    # otonomi + limiter tavanını düşürür; gerçek araç (field_profile.yaml)
    # ve bench launch'ları etkilenmez.
    sim_max_speed_mps = LaunchConfiguration("sim_max_speed_mps")
    sim_effective_max_speed_mps = LaunchConfiguration(
        "sim_effective_max_speed_mps"
    )
    sim_effective_max_yaw_rate_rad_s = LaunchConfiguration(
        "sim_effective_max_yaw_rate_rad_s", default="0.785398163"
    )
    sim_field_fidelity = LaunchConfiguration("sim_field_fidelity")
    field_profile_path = LaunchConfiguration("field_profile_path")

    # SITL script yolu: ~/ardupilot/Tools/autotest/sim_vehicle.py ($HOME env expand).
    default_sim_vehicle = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), "ardupilot", "Tools", "autotest", "sim_vehicle.py"]
    )

    logging_config = PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "logging.yaml"])
    fusion_config = PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "fusion.yaml"])

    # DİKKAT: komutları f-string ile LaunchConfiguration içine GÖMME — launch
    # substitution'ları ($(var ...)) ancak ExecuteProcess cmd listesinin ayrı
    # öğeleri olarak çözülür; string içine gömülünce literal metin olarak kalır
    # ve bash `python3 '$(var sim_vehicle_path)'` hatası verir (review B1/B2).
    # bash -lc sarmalama: ROS2 Humble ortamını (setup.bash) source et — böylece
    # gazebo_ros plugin'leri (/opt/ros/humble/lib/*.so) ve sistem python'ı
    # (pymavlink) doğru ortamda yüklenir. Komut argümanlarını `$1` üzerinden
    # ilet: bash içine gömülen substitution'lar çözülmez, bu yüzden path/home
    # değerleri cmd listesinin ayrı öğelerinden geçirilir.
    ros_env_cmd = "source /opt/ros/humble/setup.bash && "

    # SITL üç ayrı istemciye yayın yapar: MAVSDK telemetri (14540),
    # pymavlink komut yolu (14541), sim ARM/upload zinciri (14542). Adres
    # parametreleri değiştirilirse --out portları da birlikte güncellenmeli;
    # her consumer ayrı kalmalıdır (COZUM_MAVLINK_CAKISMASI).
    # mavproxy SITL SERIAL0'a (TCP 5760) bağlanır ve MAVLink trafiğini UDP
    # 14540/14541/14542'ye köprüler — mavsdk_bridge telemetri, pymavlink
    # GUIDED komutu ve mission araçları ayrı istemcilerdir. mavproxy yoksa SITL yalnız TCP 5760
    # dinler, UDP istemciler "waiting for connection"da kalırdı.
    # motorboat-skid: SITL su fiziği (dalga/direnç) + SKID/diferansiyel motor
    # modeli. ArduRover'da skid, SERVO1_FUNCTION=73 (sol motor) + SERVO3_FUNCTION=74
    # (sağ motor) ile otomatik devreye girer (have_skid_steering). Araba modeli
    # (rover) ch1/ch3'ü steering/throttle sanıp araç düz gidiyordu.
    # HOME: --custom-location motorboat-skid ile uygulanmıyor (araç Canberra'da
    # başlıyordu). -A "--home=..." arudover'a doğrudan home geçirir (güvenilir).
    sitl_cmd = (
        ros_env_cmd
        + "WIPE=; if [[ \"$6\" == true ]]; then WIPE=-w; fi; "
        + "python3 \"$1\" -v Rover -f motorboat-skid -N $WIPE "
        + "-A \"--home=$2\" --speedup=\"$3\" "
        + "--out=udp:127.0.0.1:14540 --out=udp:127.0.0.1:14541 "
        + "--out=udp:127.0.0.1:14542 "
        + "--out=\"udp:$4:$5\" "
        + "--aircraft=ida_sim"
    )

    gazebo_cmd = (
        ros_env_cmd
        + "export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-} && "
        + "gazebo --verbose \"$1\""
    )

    def _node(pkg, exe, name, params=None, condition=None):
        """Kısa Node üretici — satır uzunluğunu ve tekrarı azaltır."""
        return Node(
            package=pkg,
            executable=exe,
            name=name,
            output="screen",
            parameters=params or [],
            condition=condition,
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario_file", default_value=default_scenario),
            DeclareLaunchArgument("target_color", default_value="green"),
            DeclareLaunchArgument("log_dir", default_value="./logs"),
            DeclareLaunchArgument(
                "log_run_name",
                default_value=default_log_run_name,
            ),
            DeclareLaunchArgument("home", default_value="40.8630501,29.2599517,0,0"),
            # SITL hızlandırma: speedup:=5 ile simülasyon 5 kat hızlı koşar
            # (fizik SITL'de — Gazebo hız kontrolü SITL'i etkilemez).
            DeclareLaunchArgument("speedup", default_value="1"),
            DeclareLaunchArgument("yki_mavlink_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("yki_mavlink_port", default_value="14550"),
            DeclareLaunchArgument(
                "sitl_wipe",
                default_value="false",
                description="true: her launch'ta SITL EEPROM/mission durumunu temizler",
            ),
            # OTOMATİK MISSION ZİNCİRİ: auto_mission:=true (varsayılan) ->
            # sim_arm + sim_upload_mission + send_mission OTOMATİK çalışır.
            # auto_mission:=false -> zincir DEVRE DIŞI; arm/waypoint/mission
            # start MANUEL yapılır (saha/hata ayıklama akışı). Koşulsuz
            # çalışan zincir bazı kurulumlarda takılıyordu — manuel akış için.
            DeclareLaunchArgument("auto_mission", default_value="true"),
            # mavproxy SITL TCP 5760'a bağlanır, UDP 14540'a köprüler (--out).
            # mavsdk_bridge bu UDP'den alır (telemetri + mission). pymavlink 14541.
            DeclareLaunchArgument("system_address", default_value="udp://:14540"),
            # pymavlink komut yolu: MAVSDK system_address'ten AYRI port (çift
            # istemci). mavproxy ikinci köprü UDP 14541 — pymavlink oradan alır.
            # NOT: pymavlink boş host çözemez ("Name or service not known") — host
            # açık yazılır: udp:127.0.0.1:14541 (MAVSDK udp://:port kabul eder).
            DeclareLaunchArgument("pymavlink_address", default_value="udp:127.0.0.1:14541"),
            # ARM/upload yardımcıları bridge komut döngüsüyle aynı UDP
            # socket'i tüketmemeli; aksi halde ACK/HEARTBEAT paketleri iki consumer
            # arasında kaybolur ve kabul edilen ARM işlemi scriptte fail görünür.
            DeclareLaunchArgument("mission_address", default_value="udp:127.0.0.1:14542"),
            DeclareLaunchArgument("mission_raw_enabled", default_value="false"),
            DeclareLaunchArgument("target_color_param", default_value="SCR_USER1"),
            DeclareLaunchArgument("sim_vehicle_path", default_value=default_sim_vehicle),
            DeclareLaunchArgument("world_file", default_value=default_world),
            DeclareLaunchArgument("model_name", default_value="ida_boat"),
            DeclareLaunchArgument("sensor_fusion_enabled", default_value="false"),
            DeclareLaunchArgument("sensor_fusion_shadow_mode", default_value="true"),
            DeclareLaunchArgument("perception_sim_publish_canonical", default_value="true"),
            # Full-mission P3 targets are 23-29 m from the P2 exit. This is a
            # sim-only explicit override; perception_sim's production/default
            # node contract remains 18 m.
            DeclareLaunchArgument("perception_sim_lidar_range_m", default_value="35.0"),
            DeclareLaunchArgument(
                "sim_max_speed_mps",
                default_value="0.6",
                description=(
                    "Simülasyon hız tavanı (m/s). Üretim autonomy ve limiter "
                    "tavanıyla aynı güvenli 0.6 varsayılanını kullanır."
                ),
            ),
            DeclareLaunchArgument(
                "sim_field_fidelity",
                default_value="false",
                description=(
                    "true: merkezi field_profile.yaml davranis ve olculmus hedef "
                    "hiz zarfini simde kullanir; false: bilinen iyi sim profili"
                ),
            ),
            DeclareLaunchArgument(
                "field_profile_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ida_bringup"), "config", "field_profile.yaml"]
                ),
            ),
            OpaqueFunction(function=_resolve_sim_field_fidelity),
            # [1] ArduRover SITL: fizik + MAVLink telemetri (UDP 14540).
            # -N: rebuild etme; --no-mavproxy: MAVProxy gereksiz (mavsdk direkt bağlanır).
            # --home: İstanbul origin (default Canberra'dır — origin'e çekilmeli).
            ExecuteProcess(
                cmd=[
                    "bash", "-lc", sitl_cmd, "sim_gazebo", sim_vehicle_path,
                    home, speedup, yki_mavlink_host, yki_mavlink_port, sitl_wipe,
                ],
                name="ardupilot_sitl",
                output="screen",
                cwd=os.path.join(os.path.expanduser("~"), "ardupilot"),
                on_exit=[LogInfo(msg="SITL (ardupilot_sitl) sonlandı")],
            ),
            # [2] Gazebo Classic: world'i doğrudan binary ile başlat (gazebo_ros
            # wrapper yerine — daha az kırılgan). Plugin'ler world içinden yüklenir.
            ExecuteProcess(
                cmd=["bash", "-lc", gazebo_cmd, "sim_gazebo", world_file],
                name="gazebo_classic",
                output="screen",
                cwd=os.path.expanduser("~"),
                on_exit=[LogInfo(msg="Gazebo (gazebo_classic) sonlandı")],
            ),
            # [3] SITL konumunu Gazebo model pose'una senkronlar (ENU dönüşümü
            # node içinde; origin parametreleri scenario origin'iyle birebir aynı).
            _node(
                "ida_control",
                "gazebo_pose_sync_node",
                "ida_gazebo_pose_sync",
                [
                    {
                        "model_name": model_name,
                        "origin_lat": 40.8630501,
                        "origin_lon": 29.2599517,
                        "publish_hz": 20.0,
                    }
                ],
            ),
            # [4] İDA stack (sim_full_mission.launch.py'den kopyalandı;
            # telemetry_sim_node ÇIKARILDI — mavsdk_bridge /telemetry/state üretir).
            _node(
                "ida_autonomy",
                "autonomy_node",
                "ida_autonomy",
                [
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
            _node(
                "ida_autonomy",
                "autonomy_node",
                "ida_autonomy",
                [
                    PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "autonomy.yaml"]),
                    field_profile_path,
                    {
                        "sim_gate_truth_enabled": True,
                        "max_speed_mps": sim_effective_max_speed_mps,
                    },
                ],
                condition=IfCondition(sim_field_fidelity),
            ),
            _node(
                "ida_control",
                "command_limiter_node",
                "ida_command_limiter",
                [
                    PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "command_limits.yaml"]),
                    {
                        # Simülasyon hız kısması: limiter tavanı otonomi
                        # tavanıyla hizalı tutulur (sim_max_speed_mps).
                        "max_vx_mps": sim_effective_max_speed_mps,
                    },
                ],
                condition=UnlessCondition(sim_field_fidelity),
            ),
            _node(
                "ida_control",
                "command_limiter_node",
                "ida_command_limiter",
                [
                    PathJoinSubstitution([FindPackageShare("ida_bringup"), "config", "command_limits.yaml"]),
                    field_profile_path,
                    {
                        "max_vx_mps": sim_effective_max_speed_mps,
                        "max_yaw_rate_rad_s": sim_effective_max_yaw_rate_rad_s,
                    },
                ],
                condition=IfCondition(sim_field_fidelity),
            ),
            _node(
                "ida_perception_sim",
                "perception_sim_node",
                "ida_perception_sim",
                [{
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
            _node(
                "ida_uav_target",
                "uav_target_node",
                "ida_uav_target",
                [{"sim_target_color": target_color, "stable_frames": 1}],
            ),
            # MAVSDK bridge: SITL'e gerçek bağlantı (telemetri + SCR_USER1 poll).
            # KOMUT YOLU: pymavlink GUIDED + SET_POSITION_TARGET_LOCAL_NED
            # (ArduRover'da offboard yok) — pymavlink_address AYRI port (14541).
            _node(
                "ida_control",
                "mavsdk_bridge_node",
                "ida_mavsdk_bridge",
                [
                    {
                        "dry_run": False,
                        "system_address": system_address,
                        "pymavlink_address": pymavlink_address,
                        "target_color_param": LaunchConfiguration("target_color_param"),
                        "target_color_poll_hz": 1.0,
                        "mission_download_hz": 0.2,
                        # Sim'de tek authoritative mission kaynağı aşağıdaki
                        # send_mission.py'dir (4xP1+1xP2). SITL mission_raw tüm
                        # NAV item'larını P1 diye yayıp atomic reset yarışı yapmasın.
                        "mission_raw_enabled": ParameterValue(
                            LaunchConfiguration("mission_raw_enabled"), value_type=bool
                        ),
                        "param_call_timeout_s": 2.0,
                        "param_max_retries": 2,
                        "param_consecutive_fail_limit": 30,
                        "companion_system_id": 1,
                        "companion_component_id": 191,
                        "yki_status_udp_host": yki_mavlink_host,
                        "yki_status_udp_port": ParameterValue(
                            yki_mavlink_port, value_type=int
                        ),
                        "vehicle_setup_enabled": True,
                        "guided_mode_enabled": True,
                        "motor_command_enabled": True,
                        "left_motor_servo_channel": 1,
                        "right_motor_servo_channel": 3,
                    }
                ],
            ),
            # [5] Veri kaydı (log_dir parametresi).
            _node(
                "ida_logging",
                "telemetry_logger_node",
                "ida_telemetry_logger",
                [logging_config, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            _node(
                "ida_logging",
                "video_logger_node",
                "ida_video_logger",
                [logging_config, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            _node(
                "ida_logging",
                "map_logger_node",
                "ida_map_logger",
                [logging_config, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            _node(
                "ida_logging",
                "logging_status_node",
                "ida_logging_status",
                [logging_config, {"log_dir": log_dir, "run_name": log_run_name}],
            ),
            # [6] SITL ARM + mission + GUIDED + /mission/start (tek sıralı zincir).
            # Sıralama kritik: (1) sim_arm.py — ArduRover disarmed araçta GUIDED
            # velocity komutunu UYGULAMAZ (motorlar dönmez); pre-arm kapatır + ARM.
            # (2) sim_upload_mission.py — hakem rotasını (hakem_mission.txt) SITL'e
            # yükler + GUIDED'a geçirir.
            # (3) send_mission.py — aynı scenario_file'daki waypoint'leri ve
            # /mission/start yayınlar. ros2 topic pub escape karmaşası
            # waypoint'leri bozuyordu; Python JSON temiz üretir.
            # Tek bash zinciri: ayrı ExecuteProcess'ler PARALEL başlardı.
            ExecuteProcess(
                cmd=[
                    "bash", "-lc",
                    # Launch'u başlatan overlay ortamı ExecuteProcess'e miras
                    # kalır. Sabit ~/ida_ws overlay'ini yeniden source etmek,
                    # geçici/test workspace'inde eski kurulu kodu öne alıyordu.
                    ros_env_cmd + "python3 \"$1\" --address \"$4\" && "
                    + "python3 \"$2\" --address \"$4\" && "
                    + "python3 \"$3\" --rate 1.0 --scenario-file \"$5\"",
                    "sim_mission_chain",
                    PathJoinSubstitution(
                        [FindPackageShare("ida_bringup"), "scripts", "sim_arm.py"]
                    ),
                    PathJoinSubstitution(
                        [FindPackageShare("ida_bringup"), "scripts", "sim_upload_mission.py"]
                    ),
                    PathJoinSubstitution(
                        [FindPackageShare("ida_bringup"), "scripts", "send_mission.py"]
                    ),
                    mission_address,
                    scenario_file,
                ],
                name="sim_mission_chain",
                output="screen",
                cwd=os.path.expanduser("~"),
                condition=IfCondition(LaunchConfiguration("auto_mission")),
            ),
        ]
    )
