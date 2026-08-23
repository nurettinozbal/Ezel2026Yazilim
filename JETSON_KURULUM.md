# Jetson Kurulum ve İlk Setup Yönergesi

Kodları Jetson'a aktarma + ROS2 + bağımlılıklar + ilk çalıştırma.
**Lidar USB ile bağlanacak** (TTL-UART değil, USB) — buna göre ayarlanmıştır.

> Hedef: Jetson Orin + JetPack (ROS2 Humble). Windows dev makinesinden kod aktarımı.

---

## 1. Kodları Jetson'a Aktarma

### 1.1 Ne aktarılacak (proje kökü)
```
teknofest_2025_kodlar/
├── src/                  → ROS2 workspace (10 paket) — AKTAR
├── ezel-yazilim/yolo/    → model .pt dosyaları — AKTAR
├── tools/                → camera_stream (lens/costmap aracı) — AKTAR (isteğe bağlı)
├── *.md                  → dokümanlar (SAHA_EL_KITABI, SAHA_TEST_PLANI...) — AKTAR
├── *.py (kök)            → geçen senenin referans kodları — AKTARMA (gerek yok)
└── ezel-yazilim/arayuz/  → YKİ arayüzü — AKTARMA (Windows'ta kalır, Jetson'da gerekmez)
```

**Önerilen dizin:** `~/ida_ws/` (Jetson'da)
```bash
mkdir -p ~/ida_ws
```

### 1.2 Aktarma yöntemleri (Jetson'da)

**A) USB bellek (en basit, offline):**
- Kodu USB'ye kopyala → Jetson'a tak → `cp -r` ile `~/ida_ws/`
- YKİ arayüzünü (`ezel-yazilim/arayuz`) **aktarma** — Windows'ta kalır

**B) Ağ üzerinden (scp/git):**
```bash
# Windows'tan (Jetson SSH'ı açıksa, aynı ağda):
scp -r "teknofest_2025_kodlar/src" kullanici@jetson-ip:~/ida_ws/
scp -r "teknofest_2025_kodlar/ezel-yazilim/yolo" kullanici@jetson-ip:~/ida_ws/
```
- Şu an telefon WiFi paylaşımıyla bağlıysan → aynı ağdaki IP'yi kullan

### 1.3 Aktarım sonrası kontrol
```bash
ls ~/ida_ws/src/ida_*        # 10 klasör
ls ~/ida_ws/yolo/yolo_8_m/   # best.pt
du -sh ~/ida_ws              # boyut (src küçük, modeller ~67MB)
```

---

## 2. ROS2 + Bağımlılık Kurulumu (ilk sefer)

### 2.1 Temel paketler
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
    python3-pip v4l-utils network-manager git
```

### 2.2 ROS2 Humble (JetPack 5.x / Ubuntu 20.04)
JetPack 6 (Ubuntu 22.04) kullanıyorsan aşağıdaki "ubuntu-22.04" yerine geçerli sürümü koy:
```bash
sudo apt install -y ros-humble-desktop ros-humble-rmw-fastrtps-cpp
```
> ROS2 Humble kurulu değilse kur; kuruluysa atla (`ros2 --version` ile kontrol).

### 2.3 Python bağımlılıkları (venv ile — önerilir)
Sistem pip'ini bozmamak için **venv** kullan (önceki sohbetteki not — paket yükseltme riskine karşı):
```bash
cd ~/ida_ws
python3 -m venv --system-site-packages .venv    # ROS2'yi sistemden görsün
source .venv/bin/activate
pip install mavsdk pyserial pymavlink numpy opencv-python ultralytics
pip install rplidar-roboticia    # USB lidar için (RPLidar kütüphanesi)
```
- `--system-site-packages` → ROS2 (rclpy) sistemde, diğer paketler venv'de
- **Her terminal açılışında:** `source ~/ida_ws/.venv/bin/activate`
- `rplidar-roboticia` RPLidar S2'yi seri porttan okur (USB → seri görünür)

### 2.4 Rosdep (paket bağımlılıkları)
```bash
cd ~/ida_ws/src
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

---

## 3. Build (colcon)

```bash
cd ~/ida_ws/src
source /opt/ros/humble/setup.bash
source ~/ida_ws/.venv/bin/activate
colcon build --symlink-install
source install/setup.bash
```
> **NOT:** `colcon build` sırasında venv aktif olmalı (mavsdk vb. import eden paketler için).

### Build sonrası kontrol
```bash
ros2 pkg list | grep ida_     # 10 paket
```
```
ida_autonomy  ida_bringup  ida_control  ida_logging  ida_perception
ida_perception_sim  ida_planning  ida_telemetry_sim  ida_uav_target  ida_ws_gateway
```

---

## 4. Birim Testleri (lidar bağlı değil — yine de çalışır)

Testler donanım gerektirmez (sim/dry-run). Lidar olmadan da **hepsi yeşil olmalı**:
```bash
cd ~/ida_ws/src
python3 -m unittest discover -s ida_planning        # 53 test
python3 -m unittest discover -s ida_perception/test # 23 test
python3 -m unittest discover -s ida_logging/test    # 12 test
python3 -m unittest discover -s ida_ws_gateway/test # 26 test
```
- ✅ Hepsi yeşil → kurulum doğru
- ❌ import hatası → venv aktif mi? (`source .venv/bin/activate`)

---

## 5. İlk Çalıştırma (lidar OLMADAN)

### 5.1 Port sabitleme (udev) — Pixhawk + USB lidar
```bash
# Pixhawk + lidar için udev kuralları:
sudo tee /etc/udev/rules.d/99-ida.rules > /dev/null <<'EOF'
# Pixhawk (İDA autopilot) — VENDOR/PRODUCT'i lsusb'dan güncelle
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="pixhawk"
# Lidar (USB seri çevirici — CP210x/CH340 olabilir) — lsusb'dan güncelle
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar"
EOF
sudo udevadm control --reload-rules
sudo usermod -aG dialout $USER    # seri portlara erişim
```
> **ÖNEMLİ:** `lsusb` çalıştır → Pixhawk'ın ve lidar'ın gerçek `idVendor:idProduct` değerlerini bul (örn. `10c4:ea60` = CP210x, `1a86:7523` = CH340, `0483:5740` = Pixhawk STM32). Bu değerleri kurallara yaz.

### 5.2 Model yollarını ayarla
`~/ida_ws/src/ida_bringup/config/perception.yaml`'da:
```yaml
ida_yolo_camera:
  model_path: "/home/<kullanici>/ida_ws/models/p1p2.engine"   # P1/P2 (TensorRT gelince)
ida_yolo_camera_p3:
  model_path: "/home/<kullanici>/ida_ws/models/p3.engine"      # P3
```
> **Şimdilik model yok** (`.pt`'ler var, TensorRT `.engine` bekleniyor). Model yolu boş bırakılırsa node boş tespit yayınlar (dry-run gibi) — **test için sorun değil**.

### 5.3 Dry-run benchtop testi (S1 — çalışır)
```bash
cd ~/ida_ws/src
source install/setup.bash
ros2 launch ida_bringup real_vehicle.launch.py dry_run:=true
```
- **Lidar USB bağlı değilse:** `ida_rplidar` dry_run=true → örnek nokta bulutu (sorun değil)
- **Lidar USB bağlıysa:** `perception.yaml` `dry_run: false` yap → gerçek veri
- Ayrı terminalde:
```bash
source install/setup.bash
ros2 topic echo /autonomy/state
ros2 topic echo /perception/obstacles    # lidar bağlıysa gerçek, değilse dry-run örnek
ros2 topic echo /planning/costmap        # costmap yayınlanıyor mu
```
- ✅ Topic'ler veri yayınlıyor → kurulum OK

### 5.4 Costmap görselleştirme (lidar'sız)
```bash
# Jetson'da (aynı terminal, ROS2 çalışırken):
cd ~/ida_ws/tools/camera_stream && ./start.sh
```
- Windows PC → Jetson WiFi'sına bağlan → `http://<jetson-ip>:8080`
- Costmap panelinde **dry-run örnek engeller** görünür (lidar olmadığı için gerçek değil)

---

## 6. Lidar USB Bağlantısı

Lidar **USB ile** bağlanır (TTL-UART değil). USB'den seri port görünür.

1. **USB tak** → port gör:
```bash
lsusb                       # çeviriciyi bul (CP210x: 10c4:ea60, CH340: 1a86:7523, veya RPLidar'ın kendi)
ls /dev/ttyUSB*             # genelde ttyUSB0
```
2. **udev kuralını güncelle** (`/etc/udev/rules.d/99-ida.rules`): `lsusb`'dan alınan `idVendor/idProduct` ile `SYMLINK+=lidar` → `sudo udevadm control --reload-rules`
3. **perception.yaml** `ida_rplidar`:
```yaml
serial_port: "/dev/lidar"   # udev ile sabitlenmiş
dry_run: false
```
4. **Test:**
```bash
ros2 run ida_perception lidar_node --ros-args -p serial_port:=/dev/lidar -p dry_run:=false
ros2 topic echo /perception/obstacles   # gerçek engel verisi (el/duvar göster)
```
- ✅ mesafe değişiyor → OK
- ❌ veri yok → `baudrate` (RPLidar S2: 115200), port adı, `rplidar-roboticia` kurulu mu

---

## 7. Sorun Giderme (ilk setup)

| Belirti | Çözüm |
|---|---|
| `colcon build` import hatası | venv aktif mi? `source ~/ida_ws/.venv/bin/activate` |
| `ros2` komutu bulunamıyor | `source /opt/ros/humble/setup.bash` (her terminal) |
| `mavsdk` yok | venv'de `pip install mavsdk` |
| `/dev/pixhawk` yok | `lsusb` → udev kuralı güncelle |
| Node "dry_run" diyor | normal — model yok / port yok → güvenli mod |
| Kamera açılmıyor | `lsusb | grep -i cam`, `v4l2-ctl --list-devices` |
| Lidar veri yok | `dry_run: false` mı, port doğru mu, baud 115200 |

---

## 8. Terminal Açılış Şablonu (Jetson'da her oturum)

```bash
source /opt/ros/humble/setup.bash
source ~/ida_ws/.venv/bin/activate
cd ~/ida_ws/src && source install/setup.bash
```

---

*Bu yönerge Jetson ilk kurulumu içindir. Saha testi adımları: `SAHA_EL_KITABI.md`. Mimari: `PROJE_DURUMU.md`.*
