# Hotspot üzerinden Jetson: TTY, S2 lidar ve bench başlatma

Bu yönerge ekransız Jetson içindir. İlk çalıştırmada motor/ESC gücü kapalı
olmalıdır. `idaws` takım servisiyle canonical stack aynı anda çalıştırılmaz.

## 1. Ağ erişimini geri getirme

Jetson ağı görünmüyorsa, klavye/ekran olmadığı için uzaktan düzeltme mümkün
değildir. Jetson'u kontrollü biçimde yeniden başlatın ve `idaws-hotspot`
ağının gelmesini bekleyin. Bilgisayarı bu ağa bağladıktan sonra:

```bash
ssh ezelproject@ezelproject-desktop.local
```

Hostname çözülmezse Jetson hotspot gateway'i çoğunlukla `10.42.0.1` olur:

```bash
ssh ezelproject@10.42.0.1
```

Bu test boyunca telefon Wi-Fi geçiş scriptini yeniden çalıştırmayın. Kod
aktarımı ve tüm ROS komutları Jetson'un kendi hotspotu üzerinden yapılabilir;
internet gerekmez.

## 2. Çalışan takım servisini durdurma

```bash
sudo systemctl stop idaws
systemctl is-active idaws
pgrep -af 'idaws|mavsdk|mavproxy|mavlink|sllidar|rplidar|ros2|python3'
```

`systemctl is-active` çıktısı `inactive` olmalı. Listede takımın Pixhawk/lidar
süreçleri kalmışsa canonical launch başlatılmaz.

## 3. Workspace güncelleme ve build

Bilgisayardan, bu reponun kökünde:

```bash
scp ida_tty_lidar_hotspot_patch.zip \
  ezelproject@ezelproject-desktop.local:/tmp/
```

Jetson SSH oturumunda:

```bash
cd ~/ida_alt_ws
unzip -o /tmp/ida_tty_lidar_hotspot_patch.zip
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select ida_control ida_bringup
source install/setup.bash
command -v tty_mavlink_router
```

Son komut `~/ida_alt_ws/install/.../tty_mavlink_router` yolunu göstermelidir.
Repo içindeki yönlendirici kullanıldığı için `mavlink-routerd` kurulmaz.

## 4. Seri cihazları kesin doğrulama

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
udevadm info -q property -n /dev/ttyACM0 | grep -E 'ID_VENDOR|ID_MODEL|ID_SERIAL'
udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_VENDOR|ID_MODEL|ID_SERIAL'
sudo lsof /dev/ttyACM0 /dev/ttyUSB0
groups
```

Beklenen varsayım:

- Pixhawk: `/dev/ttyACM0`, 115200 baud,
- Slamtec S2: `/dev/ttyUSB0`, 1000000 baud.

Kimlikler ters veya farklıysa yalnız doğrulanmış cihaz yolunu launch argümanında
değiştirin. İki port da boş olmalı. Kullanıcı `dialout` grubunda değilse:

```bash
sudo usermod -aG dialout ezelproject
```

Bu değişiklik yeni oturum gerektirir; Jetson'u yeniden başlatıp hotspot üzerinden
tekrar bağlanın. Geçici `chmod 777` kullanmayın.

## 5. S2 lidarın tek başına kabulü

Önce full stack değil, resmi S2 profili denenir:

```bash
source /opt/ros/humble/setup.bash
source ~/ida_alt_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_s2_launch.py \
  serial_port:=/dev/ttyUSB0 \
  serial_baudrate:=1000000 \
  scan_mode:=DenseBoost
```

İkinci SSH oturumunda:

```bash
source /opt/ros/humble/setup.bash
source ~/ida_alt_ws/install/setup.bash
ros2 topic hz /scan
ros2 topic echo --once /scan --field header
```

`/scan` yaklaşık 10 Hz olmalıdır. `80008004` tekrar görülürse launch'ı kapatın ve:

```bash
sudo lsof /dev/ttyUSB0
dmesg --ctime | tail -n 60
```

Port başka süreçteyse o süreç durdurulur. Port doğru ve boşsa yalnız teşhis için
`scan_mode:=Standard` bir kez denenebilir. Mod değişikliği port/izin hatasını
çözmez. S2 kabulünden sonra `Ctrl+C` ile standalone lidar kapatılır.

## 6. Pixhawk TTY bağlantısı — motor kapalı

```bash
cd ~/ida_alt_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ida_bringup real_vehicle.launch.py \
  canonical_takeover_enabled:=true \
  mavlink_router_enabled:=true \
  pixhawk_serial_port:=/dev/ttyACM0 \
  pixhawk_baud:=115200 \
  lidar_serial_port:=/dev/ttyUSB0 \
  lidar_serial_baudrate:=1000000 \
  lidar_scan_mode:=DenseBoost \
  dry_run:=false \
  vehicle_setup_enabled:=false \
  guided_mode_enabled:=false \
  motor_command_enabled:=false \
  fusion_model_loaded:=false \
  camera_calibrated:=false \
  lidar_calibrated:=false \
  extrinsics_calibrated:=false
```

Bu komut ARM, mod değişikliği, servo parametre yazımı veya motor komutu üretmez.
Fiziksel `/dev/ttyACM0` yalnız `tty_mavlink_router` tarafından açılır. MAVSDK ve
pymavlink ayrımı sadece `127.0.0.1` üzerindeki Jetson içi portlardır; araç ağına
UDP açılmaz.

İkinci SSH oturumunda kabul:

```bash
sudo lsof /dev/ttyACM0
ss -lunp | grep -E '127.0.0.1:14540|127.0.0.1:14541'
ros2 topic echo --once /control/mavsdk_status --full-length
ros2 topic echo --once /telemetry/state --full-length
ros2 topic hz /telemetry/state
ros2 topic hz /scan
```

Beklenen:

- TTY sahibinde yalnız `tty_mavlink_router`,
- `connected=true`, `heartbeat_fresh=true`, `telemetry_ready=true`,
- `/telemetry/state` ve `/scan` düzenli,
- `guided_actuation.allowed=false`, motor çıkışı yok.

## 7. Kamera + lidar + füzyon bench

Model ve kalibrasyon dosyaları gerçekten doğrulanmadan readiness alanlarını
`true` yapmayın. Genel orange/yellow model yolu örnek olarak verilmiştir; gerçek
dosya yolunu `find ~/ -name 'last.pt' -o -name 'best.pt'` ile doğrulayın.

```bash
export IDA_CANONICAL_TAKEOVER=true
export IDA_BENCH_DRY_RUN=false
export IDA_MAVLINK_ROUTER_ENABLED=true
export IDA_VEHICLE_SETUP_ENABLED=false
export IDA_GUIDED_MODE_ENABLED=false
export IDA_MOTOR_COMMAND_ENABLED=false
export IDA_PIXHAWK_PORT=/dev/ttyACM0
export IDA_LIDAR_PORT=/dev/ttyUSB0
export IDA_LIDAR_BAUDRATE=1000000
export IDA_LIDAR_SCAN_MODE=DenseBoost
export IDA_MODEL_P1P2=/home/ezelproject/models/last.pt
export IDA_MODEL_CLASS_NAMES=orange,yellow
export IDA_BENCH_CAMERA_DEVICE=/dev/video0
export IDA_BENCH_CAMERA_WIDTH=960
export IDA_BENCH_CAMERA_HEIGHT=600
export IDA_BENCH_CAMERA_FPS=30
export IDA_FUSION_MODEL_LOADED=true
export IDA_CAMERA_CALIBRATED=false
export IDA_LIDAR_CALIBRATED=true
export IDA_EXTRINSICS_CALIBRATED=false

cd ~/ida_alt_ws
./scripts/start_bench_avoidance.sh
```

Bu profil sensörleri ve kararı gözlemletir fakat motor yolunu açmaz. Kalibrasyon
bayrakları false iken füzyon fail-closed kalmalıdır; logda `ready=false` doğru
sonuçtur. Kalibrasyon dosyaları oluşturulup ölçümle doğrulandıktan sonra ilgili
bayraklar true yapılır.

## 8. Kayıt

Başka bir SSH oturumunda:

```bash
cd ~/ida_alt_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
RUN="bench_$(date +%Y%m%d_%H%M%S)"
ros2 bag record -o "$RUN" \
  /scan /telemetry/state /control/mavsdk_status \
  /perception/camera/p1p2/raw /perception/lidar/raw_obstacles \
  /perception/fusion/status /perception/buoys /perception/obstacles \
  /planning/costmap /autonomy/state /autonomy/debug \
  /autonomy/cmd_vel_body /control/cmd_vel_body
```

## 9. Kapatma ve takım servisine dönme

Launch ve rosbag terminallerinde `Ctrl+C` kullanın. Sonra:

```bash
pkill -INT -f 'ros2 launch ida_bringup' 2>/dev/null || true
sleep 3
sudo lsof /dev/ttyACM0 /dev/ttyUSB0
sudo systemctl start idaws
systemctl is-active idaws
```

Motorlu test ayrı güvenlik onayı ve saha prosedürü ister; bu belgedeki komutlar
motor yolunu açmaz.
