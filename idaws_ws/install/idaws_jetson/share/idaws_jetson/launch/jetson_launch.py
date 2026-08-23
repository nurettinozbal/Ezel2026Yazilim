"""
IDAWS — Jetson (gerçek araç) launch dosyası
───────────────────────────────────────────
Gerçek donanım zinciri:

  Slamtec LiDAR ─► sllidar_node ─► /scan_raw ─► scan_filter_node ─► /scan
                                                                     │
  Pixhawk ──────► pymavlink_controller_node ◄──────────────────┐     │
  Kamera ───────► yolo_vision_node                             │     │
                                        collision_avoidance / kamikaze
                                                               │
                                                      mission_manager
                                                               │
                                                  mission/lidar logger

Görev seçimi otopilot parametresi SCR_USER1 iledir (0=IDLE, 5=DRIVETEST,
10=WAYPOINT, 15=KAMIKAZE). KAMIKAZE fazını kamikaze_node yürütür. Web paneli
kamera kalibrasyonu içindir (sadece görüntü + filtre + lens + ROS log).

Node listesi idaws deposundaki idaws_launch.py ile aynıdır; iki fark var:

  1. sllidar_node modele göre doğru baud/scan_mode ile başlatılır.
     idaws_launch.py 115200'ü sabit yazıyor — A3/S1'de (256000) ve
     S2/S3'te (1000000) LiDAR hiç açılmaz.
  2. sllidar çıktısı /scan_raw'a yönlendirilip araya scan_filter_node
     girer. Aşağı akış (collision_avoidance, lidar_logger) değişmeden
     /scan dinlemeye devam eder.

Kullanım:
  ros2 launch idaws_jetson jetson_launch.py
  ros2 launch idaws_jetson jetson_launch.py lidar_model:=s1 use_yolo:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


# Slamtec resmi launch dosyalarından alınan model tablosu.
# (sllidar_ros2 @ 3430009 — launch/sllidar_*_launch.py)
LIDAR_MODELS = {
    'a1':    {'baud': 115200,  'mode': 'Sensitivity', 'range': 12.0},
    'a2m7':  {'baud': 256000,  'mode': 'Sensitivity', 'range': 12.0},
    'a2m8':  {'baud': 115200,  'mode': 'Sensitivity', 'range': 12.0},
    'a2m12': {'baud': 256000,  'mode': 'Sensitivity', 'range': 12.0},
    'a3':    {'baud': 256000,  'mode': 'Sensitivity', 'range': 25.0},
    'c1':    {'baud': 460800,  'mode': 'Standard',    'range': 12.0},
    's1':    {'baud': 256000,  'mode': '',            'range': 40.0},
    's2':    {'baud': 1000000, 'mode': 'DenseBoost',  'range': 30.0},
    's3':    {'baud': 1000000, 'mode': 'DenseBoost',  'range': 40.0},
    't1':    {'baud': 1000000, 'mode': 'DenseBoost',  'range': 40.0},
}


def _build(context, *args, **kwargs):
    model = LaunchConfiguration('lidar_model').perform(context).lower()
    if model not in LIDAR_MODELS:
        raise RuntimeError(
            f"Bilinmeyen lidar_model: '{model}'. "
            f"Seçenekler: {', '.join(sorted(LIDAR_MODELS))}"
        )
    spec = LIDAR_MODELS[model]

    share = get_package_share_directory('idaws_jetson')
    params_file = os.path.join(share, 'config', 'jetson_params.yaml')

    # value_type=bool şart: çıplak LaunchConfiguration string üretir, node ise
    # use_yolo'yu bool olarak declare ediyor — tür uyuşmazlığı node'u düşürür.
    use_yolo = ParameterValue(LaunchConfiguration('use_yolo'), value_type=bool)
    lidar_port = LaunchConfiguration('lidar_port')
    pixhawk_port = LaunchConfiguration('pixhawk_port')

    # Kayıt dizini dışarıdan yönlendirilebilsin (idaws ile aynı sözleşme).
    log_dir = os.environ.get('IDAWS_LOG_DIR', '')
    log_override = [{'log_dir': log_dir}] if log_dir else []
    rec_override = [{'record_dir': log_dir}] if log_dir else []

    # ─── 1. Slamtec LiDAR sürücüsü ───
    # scan_mode boş bırakılırsa sürücü cihazın varsayılanını kullanır; S1'de
    # adlandırılmış mod yok, boş geçmek zorunlu.
    lidar_params = {
        'channel_type': 'serial',
        'serial_port': lidar_port,
        'serial_baudrate': spec['baud'],
        'frame_id': 'laser',
        'inverted': False,
        'angle_compensate': True,
    }
    if spec['mode']:
        lidar_params['scan_mode'] = spec['mode']

    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        respawn=True,             # USB düşerse sürücü kendini toparlasın
        respawn_delay=3.0,
        parameters=[lidar_params],
        remappings=[('/scan', '/scan_raw')],
    )

    # ─── 2. Tarama hizalama + kör sektör maskesi ───
    scan_filter = Node(
        package='idaws_jetson',
        executable='scan_filter_node',
        name='scan_filter_node',
        output='screen',
        parameters=[params_file, {'range_max': spec['range']}],
    )

    # ─── 3. Otopilot köprüsü ───
    pymavlink_node = Node(
        package='idaws_nodes',
        executable='pymavlink_controller_node',
        name='pymavlink_controller_node',
        output='screen',
        # Otopilot heartbeat'i 30 sn gelmezse node RuntimeError ile cikiyor.
        # Respawn olmadan MAVLink koprusu KALICI olarak kayboluyor: sim'de
        # Gazebo/SITL gec kalkarsa, sahada USB bir an duserse tum telemetri ve
        # komut yolu bir daha geri gelmiyordu. sllidar_node'da da ayni desen var.
        respawn=True,
        respawn_delay=5.0,
        parameters=[params_file, {'serial_port': pixhawk_port}],
    )

    # ─── 4. Görüntü işleme ───
    yolo_node = Node(
        package='idaws_nodes',
        executable='yolo_vision_node',
        name='yolo_vision_node',
        output='screen',
        parameters=[params_file, {'use_yolo': use_yolo}] + rec_override,
    )

    # ─── 5–8. Otonomi ve kayıt ───
    collision_node = Node(
        package='idaws_nodes',
        executable='collision_avoidance_node',
        name='collision_avoidance_node',
        output='screen',
        parameters=[params_file],
    )

    # ─── 5b. kamikaze_node ───
    # KAMIKAZE fazinda (SCR_USER1=15) kamerayla hedef renkteki dubaya surer.
    # collision_avoidance_node bu fazda sessiz oldugu icin cmd/velocity
    # cakismasi yok; LiDAR kacisi bu fazda KASITLI olarak devre disi.
    kamikaze_node = Node(
        package='idaws_nodes',
        executable='kamikaze_node',
        name='kamikaze_node',
        output='screen',
        parameters=[params_file],
    )

    mission_node = Node(
        package='idaws_nodes',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen',
        parameters=[params_file],
    )

    logger_node = Node(
        package='idaws_nodes',
        executable='mission_logger_node',
        name='mission_logger_node',
        output='screen',
        parameters=[params_file] + log_override,
    )

    lidar_logger_node = Node(
        package='idaws_nodes',
        executable='lidar_logger_node',
        name='lidar_logger_node',
        output='screen',
        parameters=[params_file] + log_override,
    )

    # ─── 8. Kamera kalibrasyon paneli ───
    # Görev kontrolü DEĞİL (o SCR_USER1/2/3 ile): panelde yalnızca kamera
    # görüntüsü, güneş filtresi, lens ayarları ve ROS log var. Hotspot elle
    # açılıyor. Yarışma turunda görüntü aktarımı yasak (şartname 4.1) —
    # o turda panel kapatılmalı.
    webui_node = Node(
        package='idaws_nodes',
        executable='web_ui_node',
        name='web_ui_node',
        output='screen',
        parameters=[params_file],
    )

    # ─── 9. Canlı durum özeti ───
    # Diğer node'ların "başlatıldı" logu bir kere yazılıp geçer; bu, veri
    # akmaya DEVAM ettiği anlamına gelmez. system_check_node Pixhawk/LiDAR/
    # kamera/görev alt sistemlerini periyodik olarak tek satırda özetler,
    # böylece "çalışıyor" demekle gerçekten çalıştığını görmek arasındaki
    # farkı terminalden takip edebilirsin.
    system_check_node = Node(
        package='idaws_jetson',
        executable='system_check_node',
        name='system_check_node',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
    )

    return [
        LogInfo(msg=f'═══ IDAWS gerçek araç — LiDAR {model.upper()} '
                    f'@ {spec["baud"]} baud, menzil {spec["range"]} m ═══'),
        sllidar_node,
        scan_filter,
        pymavlink_node,
        yolo_node,
        collision_node,
        kamikaze_node,
        mission_node,
        logger_node,
        lidar_logger_node,
        webui_node,
        system_check_node,
        LogInfo(msg='═══ Tüm node\'lar başlatıldı — görev seçimi SCR_USER1 ile '
                    '(0=IDLE 5=DRIVETEST 10=WAYPOINT 15=KAMIKAZE) ═══'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_model', default_value='s2',
            description='Slamtec model: ' + ', '.join(sorted(LIDAR_MODELS))),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/idaws_lidar',
            description='LiDAR seri portu (udev kuralının kurduğu sabit ad)'),
        DeclareLaunchArgument(
            'pixhawk_port', default_value='/dev/idaws_pixhawk',
            description='Pixhawk seri portu (udev kuralının kurduğu sabit ad)'),
        DeclareLaunchArgument(
            'use_yolo', default_value='true',
            description='YOLO duba tespitini etkinleştir (her zaman aktif — arayüzden kapatılamaz)'),

        OpaqueFunction(function=_build),
    ])
