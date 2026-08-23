#!/usr/bin/env bash
set -euo pipefail

IDA_WS="${IDA_WS:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
CLI_SOURCE="$IDA_WS/scripts/ida_alt_ws"
LIVE_ENV="$IDA_WS/scripts/field_test.live.env"
SERVICE_TEMPLATE="$IDA_WS/systemd/ida-canonical-field.service.in"
FAKE_GPS_TEMPLATE="$IDA_WS/systemd/ida-fake-gps.service.in"
IDA_USER="${SUDO_USER:-$(id -un)}"
[[ "$IDA_USER" != "root" ]] || {
  echo "Servis kullanicisi root olamaz; kurulumu arac kullanicisi oturumundan calistirin" >&2
  exit 1
}
IDA_GROUP="$(id -gn "$IDA_USER")"
escape_sed() { printf '%s' "$1" | sed 's/[&|\\]/\\&/g'; }
WS_ESC="$(escape_sed "$IDA_WS")"
USER_ESC="$(escape_sed "$IDA_USER")"
GROUP_ESC="$(escape_sed "$IDA_GROUP")"

[[ -f "$CLI_SOURCE" ]] || { echo "CLI bulunamadi: $CLI_SOURCE" >&2; exit 1; }
[[ -f "$LIVE_ENV" ]] || { echo "Canli profil bulunamadi: $LIVE_ENV" >&2; exit 1; }
[[ -f "$SERVICE_TEMPLATE" ]] || { echo "Service template bulunamadi" >&2; exit 1; }
[[ -f "$FAKE_GPS_TEMPLATE" ]] || { echo "Fake GPS service template bulunamadi" >&2; exit 1; }

sudo install -m 0755 "$CLI_SOURCE" /usr/local/bin/ida_alt_ws
sudo ln -sfn /usr/local/bin/ida_alt_ws /usr/local/bin/ida_cli
sudo install -d -m 0755 /etc/ida
LIVE_ENV_TMP="$(mktemp)"
trap 'rm -f "$LIVE_ENV_TMP"' EXIT
sed "s|@IDA_WS@|$WS_ESC|g" "$LIVE_ENV" >"$LIVE_ENV_TMP"
sudo install -m 0640 "$LIVE_ENV_TMP" /etc/ida/field-test.env
sed -e "s|@IDA_WS@|$WS_ESC|g" -e "s|@IDA_USER@|$USER_ESC|g" -e "s|@IDA_GROUP@|$GROUP_ESC|g" "$SERVICE_TEMPLATE" | \
  sudo tee /etc/systemd/system/ida-canonical-field.service >/dev/null
sudo chmod 0644 /etc/systemd/system/ida-canonical-field.service
sed -e "s|@IDA_WS@|$WS_ESC|g" -e "s|@IDA_USER@|$USER_ESC|g" -e "s|@IDA_GROUP@|$GROUP_ESC|g" "$FAKE_GPS_TEMPLATE" | \
  sudo tee /etc/systemd/system/ida-fake-gps.service >/dev/null
sudo chmod 0644 /etc/systemd/system/ida-fake-gps.service
# Eski Wi-Fi/WebSocket lidar aktarımını yükseltmelerde de kalıcı kapat.
sudo systemctl disable --now ida-yki-perception.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/ida-yki-perception.service
sudo systemctl daemon-reload

echo "Kuruldu: /usr/local/bin/ida_alt_ws"
echo "Kisa ad: /usr/local/bin/ida_cli"
echo "Servis boot-capable kuruldu; bu betik guvenlik geregi kendisi baslatmaz."
echo "Saha Jetson'unda: sudo systemctl enable --now ida-canonical-field.service"
echo "Baslangic: ida_alt_ws start"
echo "Ham lidar/costmap YKI Wi-Fi aktarimi kaldirildi; yerel algi ve kayit etkindir."


