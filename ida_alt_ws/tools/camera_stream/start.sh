#!/usr/bin/env bash
# Jetson hotspot + kamera yayın sunucusu başlatıcı (atölye/lens ayarı).
# Kullanım: ./start.sh   (Jetson'da; root gerekmez ama nmcli için izin isteyebilir)
set -euo pipefail

SSID="${IDA_CAM_SSID:-IDA-CAM}"
PASS="${IDA_CAM_PASS:-idacamera1}"
IFACE="${IDA_CAM_IFACE:-wlP1p1s0}"
PORT="${IDA_CAM_PORT:-8080}"
DEVICE="${IDA_CAM_DEVICE:-/dev/video0}"

echo "== İDA Kamera Lens Ayarı =="

# 1) GStreamer/v4l bağımlılıkları (yoksa kur)
for pkg in gstreamer1.0-plugins-good gstreamer1.0-plugins-bad v4l-utils; do
  if ! dpkg -s "$pkg" >/dev/null 2>&1; then
    echo "Kuruluyor: $pkg"
    sudo apt-get install -y "$pkg"
  fi
done

# 2) Hotspot (nmcli) — zaten aktifse atla
if ! nmcli -t -f active,ssid dev wifi | grep -q ":${SSID}"; then
  echo "Hotspot açılıyor: SSID=$SSID"
  sudo nmcli device wifi hotspot ifname "$IFACE" ssid "$SSID" password "$PASS" || {
    echo "nmcli hotspot başarısız; manuel kurulum gerekebilir."
    echo "  sudo nmcli device wifi hotspot ifname $IFACE ssid $SSID password $PASS"
    exit 1
  }
fi

# 3) IP'yi bul ve göster (hotspot arayüzünün IP'sini nmcli'den al, sonra fallback)
IP=$(nmcli -g IP4.ADDRESS device show "$IFACE" 2>/dev/null | head -1 | cut -d/ -f1)
[ -z "$IP" ] && IP=$(ip -4 addr show "$IFACE" 2>/dev/null | grep -oP 'inet \K[\d.]+' | head -1 || true)
[ -z "$IP" ] && IP=$(hostname -I 2>/dev/null | awk '{print $1}' || true)
[ -z "$IP" ] && IP="0.0.0.0"
echo "Jetson IP: $IP   (Windows PC bu IP'ye bağlanacak)"
echo "PC'de tarayıcı: http://$IP:$PORT"
echo "(IP doğru görünmüyorsa: nmcli device show $IFACE | grep IP4)"

# 4) Sunucuyu başlat (port/cihaz/çözünürlük ayarlanabilir)
# Varsayılan 640x480@15 — MJPEG akışı tarayıcıyı boğmasın (lens ayarı için yeterli).
WIDTH="${IDA_CAM_WIDTH:-640}"
HEIGHT="${IDA_CAM_HEIGHT:-480}"
FPS="${IDA_CAM_FPS:-15}"
echo "Kamera yayını başlatılıyor: $DEVICE @ :$PORT (${WIDTH}x${HEIGHT}@${FPS})"
exec python3 "$(dirname "$0")/stream.py" --device "$DEVICE" --port "$PORT" \
  --width "$WIDTH" --height "$HEIGHT" --fps "$FPS"
