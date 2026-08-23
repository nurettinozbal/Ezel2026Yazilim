# Jetson'da mevcut sistemi bozmadan alternatif test

## Değişmez kural

Takım arkadaşlarının çalışan workspace'i, launch dosyaları, model dosyaları,
Python ortamı, servisleri ve cihaz ayarları yerinde değiştirilmez. Bizim kodumuz
ayrı bir workspace'e kopyalanır; mevcut workspace'e `rsync --delete`, paket
üzerine yazma veya sistem geneli `pip install` uygulanmaz.

Gerçek motor/ARM yolu bu testin parçası değildir.

## Akşam uygulanacak iki mod

### Mod A — Canlı observe-only

Takımın mevcut sistemi aynen çalışır. Bizim tarafta yalnız iki pasif node
başlatılır:

- `ida_vehicle_test_monitor`
- `ida_vehicle_test_producer`

Bu node'lar mevcut topic'leri okur. Yalnız `/vehicle_test/*` tanılama topic'lerine
yazar; sensör, fusion, otonomi, kontrol, mission veya MAVLink topic'ine yazmaz.

Başlatmadan önce:

```bash
cd ~/ida_alt_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/check_passive_compatibility.py
```

Sonuçta `compatible=true` ve `actuation_enabled=false` görülmeden devam edilmez.

Mevcut ROS domain'i korunarak ayrı terminalde:

```bash
export EZEL_JETSON_DEBUG_TOKEN='<jetson-icin-ayri-token>'
ros2 launch ida_bringup vehicle_test_lab.launch.py \
  websocket_url:=ws://<YKI-IP>:5000/ws/jetson-debug \
  simulation_sources:=false \
  evidence_root:=/home/ezel/ida_alt_evidence \
  log_root:=<takimin-mevcut-log-koku>
```

Topic şeması bizim kontratla uyuşmazsa test FAIL/STALE gösterir; mevcut sistemi
değiştirmeye veya otomatik uyarlamaya çalışmaz.

### Mod B — Ayrı domain'de rosbag replay

Bizim YOLO/lidar bridge/fusion alternatifimiz canlı production DDS graph'inde
başlatılmaz. Önce mevcut sistemden yalnız okuma yapan rosbag kaydı alınır:

```bash
ros2 bag record -o ~/ida_bags/pet_replay_YYYYMMDD_HHMM \
  /scan \
  /camera/image_raw/compressed \
  /camera/camera_info \
  /tf /tf_static \
  /telemetry/state
```

Gerçek kamera topic'i farklıysa komutta düzeltilir. Kayıt işlemi publisher veya
araç komutu oluşturmaz.

Replay, production'dan farklı bir ROS domain'inde yapılır. Aşağıdaki örnekte
production domain `0`, alternatif domain `42`:

```bash
export IDA_PRODUCTION_ROS_DOMAIN_ID=0
export ROS_DOMAIN_ID=42
export IDA_OFFLINE_REPLAY=1
source /opt/ros/humble/setup.bash
source ~/ida_alt_ws/install/setup.bash

ros2 launch ida_bringup jetson_replay_fusion.launch.py \
  model_path_p1p2:=/home/ezel/ida_alt_models/parkur12_best.pt \
  model_path_p3:=/home/ezel/ida_alt_models/parkur3_best.pt \
  camera_topic:=/camera/image_raw/compressed \
  calibration_ready:=false
```

İkinci terminalde aynı üç environment değişkeni ve aynı overlay yüklenerek:

```bash
ros2 bag play ~/ida_bags/pet_replay_YYYYMMDD_HHMM --clock
```

Launch yalnız `/ida_alt/*` çıktıları üretir ve şunları başlatmaz:

- `ida_control`
- `ida_autonomy`
- `mavsdk_bridge`
- `ida_ws_gateway`
- fiziksel kamera/lidar sürücüleri

`calibration_ready` varsayılan olarak false'tur. İlk replay'de readiness/fail-closed
davranışı kontrol edilir. Kamera-lidar yönü ve zaman kalibrasyonu gerçekten
doğrulandıktan sonra yalnız izole replay domain'inde true yapılır.

Alternatif replay aynı Jetson GPU/CPU'sunu kullanır. Takımın canlı stack'iyle
eşzamanlı çalıştırılmaz; DDS izolasyonu kaynak tüketimi izolasyonu sağlamaz.

`ida_ws_gateway` adı yukarıdaki listede yalnız “bu testte kesinlikle başlamayan
eski bileşen” anlamındadır. Paket kökünde `COLCON_IGNORE` vardır; canonical canlı
mimaride hiçbir rolü yoktur.

## Ayrı workspace

Önerilen dizinler:

```text
~/takim_ws      # dokunulmaz, mevcut çalışan sistem
~/ida_alt_ws    # bizim alternatif kaynak/build/install
~/ida_bags      # salt okunur replay kayıtları
~/ida_alt_evidence
~/ida_alt_models
```

Takım launch'ını başlatan terminalde `~/ida_alt_ws/install/setup.bash` source
edilmez. Alternatif terminalde de takım prosesleri yeniden başlatılmaz.

## Değişiklik öncesi/sonrası kanıt

Observe-only overlay başlamadan önce ve sonra:

```bash
ros2 node list | sort
ros2 topic info -v /perception/buoys
ros2 topic info -v /perception/obstacles
ros2 topic info -v /control/cmd_vel_body
ros2 topic info -v /autonomy/cmd_vel_body
```

Production topic'lerindeki publisher sayısı ve node kimlikleri değişmemelidir.
Yeni görülmesi gereken node'lar yalnız iki `ida_vehicle_test_*` node'udur; yeni
publisher topic'leri yalnız `/vehicle_test/*` olmalıdır.

Tam `real_vehicle.launch.py` takımın çalışan stack'i yanında açılmaz. Launch
varsayılan olarak `canonical_takeover_enabled:=false` olduğundan hiçbir canonical
ROS düğümü başlatmaz. Tam sahiplik devri ancak eski stack tamamen durdurulup seri
portun boş olduğu kanıtlandıktan sonra açıkça yapılır.

## Geri alma

Observe-only overlay `Ctrl+C` ile kapatılır. Takımın proseslerine sinyal
gönderilmez. Alternatif workspace'i kaldırmak gerekirse test sonrası ayrı dizin
arşivlenebilir; mevcut workspace'ten dosya silinmez.
