# Çalıştırma ve kayıt alma

## Ortamı yükleme

Her yeni terminalde:

```bash
cd ~/ida_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Kod değiştiyse:

```bash
cd ~/ida_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Tam simülasyon

```bash
ros2 launch ida_bringup sim_gazebo.launch.py \
  speedup:=1 \
  auto_mission:=true \
  target_color:=green
```

Kamera-lidar fusion canonical kabul koşusu açıkça şu üçlüyle başlatılır:

```bash
ros2 launch ida_bringup sim_gazebo.launch.py \
  speedup:=1 \
  auto_mission:=true \
  target_color:=green \
  sensor_fusion_enabled:=true \
  sensor_fusion_shadow_mode:=false \
  perception_sim_publish_canonical:=false \
  perception_sim_lidar_range_m:=35.0
```

Odaklı Parkur 3 testi:

```bash
ros2 launch ida_bringup sim_gazebo.launch.py \
  scenario_file:="$(ros2 pkg prefix ida_bringup)/share/ida_bringup/scenarios/parkur3_target_color.yaml" \
  sensor_fusion_enabled:=true \
  sensor_fusion_shadow_mode:=false \
  perception_sim_publish_canonical:=false \
  perception_sim_lidar_range_m:=35.0 \
  target_color:=green
```

Video için Gazebo penceresi Windows ekran kaydıyla alınabilir. Rosbag video değildir;
ROS mesajlarını kaydeder. İkisini birlikte almak incelemeyi kolaylaştırır.

## Gerçek araç — önce motor kapalı canlı bağlantı

Model yolları ve seri portlar araca göre değiştirilmelidir:

```bash
ros2 launch ida_bringup real_vehicle.launch.py \
  canonical_takeover_enabled:=true \
  mavlink_router_enabled:=true \
  dry_run:=false \
  vehicle_setup_enabled:=false \
  guided_mode_enabled:=false \
  motor_command_enabled:=false \
  pixhawk_serial_port:=/dev/pixhawk \
  lidar_serial_port:=/dev/lidar \
  camera_topic:=/camera/image_raw/compressed \
  model_path:=/home/ezel/models/parkur12_best.pt \
  model_path_p3:=/home/ezel/models/parkur3_best.pt \
  fusion_model_loaded:=false \
  camera_calibrated:=false \
  lidar_calibrated:=false \
  extrinsics_calibrated:=false
```

Kamera sürücüsü ayrıca başlatılmalı; gerçek sıkıştırılmış kamera topic’i
`camera_topic` launch argümanıyla verilmelidir.

`.pt` ve `.engine` artık aynı fail-closed Ultralytics adapterını kullanır; ancak
gerçek engine henüz hedef Jetson'da üretilip parity testinden geçirilmemiştir.
Bu test tamamlanana kadar `.pt` kullanılır. Ayrıntı için
[Kamera modelleri ve TensorRT](KAMERA_MODELLERI_VE_TENSORRT.md) okunmalıdır.

## Gerçek motor çıkışı

Aşağıdaki değişiklik yalnız saha kontrol listesi tamamlandıktan, pervane alanı
boşaltıldıktan ve acil durdurma hazırlandıktan sonra yapılır:

```text
vehicle_setup_enabled:=true
guided_mode_enabled:=true
motor_command_enabled:=true
```

`guided_mode_enabled` bridge'in hız paketi yoluna izin verir; bridge açılışta
veya reconnectte kendi kendine GUIDED moda geçmez. Mod yalnız YKİ'deki
doğrulanmış Görev Başlat handshake'iyle değiştirilir.

Bu iki değer `scripts/start_real.sh` kullanılırken ayrıca
`IDA_PHYSICAL_SAFETY_ACK=PROPELLER_AREA_CLEAR` ister. `dry_run:=false` tek başına
motoru veya araç parametre yazımını açmaz.

## Hızlı sağlık kontrolü

```bash
ros2 node list
ros2 topic hz /telemetry/state
ros2 topic hz /scan
ros2 topic echo --once /control/mavsdk_status --full-length
ros2 topic echo --once /autonomy/state --full-length
ros2 topic echo --once /autonomy/debug --full-length
```

Beklenen kamera topic adı sistemden sisteme değişebilir:

```bash
ros2 topic list -t | grep -E 'camera|image|scan|telemetry|autonomy'
```

## Rosbag kaydı

Klasör adı tarih ve test aşamasını içermelidir:

```bash
ros2 bag record -o saha_p1_YYYYMMDD_HHMM \
  /scan \
  /camera/image_raw/compressed \
  /camera/camera_info \
  /tf \
  /tf_static \
  /telemetry/state \
  /mission/waypoints \
  /mission/target_color \
  /perception/buoys \
  /perception/obstacles \
  /perception/camera/p1p2/raw \
  /perception/camera/p3/raw \
  /perception/lidar/raw_obstacles \
  /perception/fusion/status \
  /autonomy/state \
  /autonomy/debug \
  /autonomy/score \
  /planning/costmap \
  /autonomy/cmd_vel_body \
  /control/cmd_vel_body \
  /control/mavsdk_status
```

Kamera topic adı farklıysa komuttaki ad değiştirilmelidir. Kayıt `Ctrl+C` ile
düzgün kapatılır. Oluşan klasör doğrudan ziplenebilir; `tar.gz` şart değildir.

## Kayıtla birlikte tutulacak bilgiler

- ekran videosu,
- test düzeninin bir fotoğrafı,
- başlangıç/bitiş saati,
- hedef rengi ve hız ayarı,
- test sırasında söylenen önemli zaman notları,
- `ros2 topic list -t` çıktısı.

## Testleri çalıştırma

```bash
cd ~/ida_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --event-handlers console_direct+
python3 tools/verify_course.py --seed 2026
```

Kaydedilmiş canonical fusion baginde Parkur 2 rota sapmasını ROS kurulumu
olmadan ölçmek için:

```bash
python3 tools/analyze_p2_rosbag.py \
  debug_logs/server_fusion_20260814/fusion_full_bag_1
```

Optimizasyon sonrası regresyon kapısı:

```bash
python3 tools/analyze_p2_rosbag.py <yeni_bag_klasoru> \
  --require-max-distance-m 5.5 \
  --output <yeni_bag_klasoru>/p2_course_report.json
```

Komut, maksimum rota uzaklığı 5.5 m'yi geçerse `2` koduyla başarısız olur.

Test çıktısında bir paketin `stderr` kullanması tek başına hata değildir; Python
test runner’ı başarılı test satırlarını stderr’e yazabilir. Asıl ölçüt `FAILED`,
`ERROR` veya sıfırdan farklı çıkış kodu olmamasıdır.
