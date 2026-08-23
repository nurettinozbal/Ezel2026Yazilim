#!/usr/bin/env bash
# Installs files only.  It never enables, starts or stops any service.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
IDA_WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
UNIT_TEMPLATE="$IDA_WS/systemd/ida-canonical-field.service.in"
FAKE_GPS_TEMPLATE="$IDA_WS/systemd/ida-fake-gps.service.in"
ENV_EXAMPLE="$IDA_WS/scripts/field_test.env.example"
IDA_USER="${SUDO_USER:-$(id -un)}"
[[ "$IDA_USER" != "root" ]] || {
  echo "Servis kullanicisi root olamaz; kurulumu arac kullanicisi oturumundan calistirin" >&2
  exit 2
}
IDA_GROUP="$(id -gn "$IDA_USER")"
escape_sed() { printf '%s' "$1" | sed 's/[&|\\]/\\&/g'; }
WS_ESC="$(escape_sed "$IDA_WS")"
USER_ESC="$(escape_sed "$IDA_USER")"
GROUP_ESC="$(escape_sed "$IDA_GROUP")"

[[ -f "$UNIT_TEMPLATE" && -f "$FAKE_GPS_TEMPLATE" && -f "$ENV_EXAMPLE" ]] || {
  echo "Kurulum dosyalari eksik." >&2
  exit 2
}

# Windows/ZIP tabanli workspace aktarimlari POSIX executable bitini tasimaz.
# Systemd 203/EXEC ile dusmesin ve ida_cli her kopyada calissin.
chmod 0755 \
  "$IDA_WS/scripts/start_field_stack.sh" \
  "$IDA_WS/scripts/ida_alt_ws" \
  "$IDA_WS/scripts/install_field_service.sh" \
  "$IDA_WS/scripts/field_device_discovery.sh"
if systemctl is-active --quiet ida-canonical-field.service; then
  echo "Servis aktifken kurulum reddedildi; once operator kontrollu durdurun." >&2
  exit 3
fi

# The field service runs as IDA_USER and resolves the active model from
# /etc/ida/model-selection at runtime. Keep machine configuration private,
# while allowing the service account to traverse this directory.
sudo install -d -m 0750 -o root -g "$IDA_GROUP" /etc/ida
if [[ ! -e /etc/ida/field-test.env ]]; then
  sed "s|@IDA_WS@|$WS_ESC|g" "$ENV_EXAMPLE" | sudo tee /etc/ida/field-test.env >/dev/null
  sudo chmod 0640 /etc/ida/field-test.env
else
  echo "/etc/ida/field-test.env korundu; uzerine yazilmadi."
fi
sed -e "s|@IDA_WS@|$WS_ESC|g" -e "s|@IDA_USER@|$USER_ESC|g" -e "s|@IDA_GROUP@|$GROUP_ESC|g" "$UNIT_TEMPLATE" | \
  sudo tee /etc/systemd/system/ida-canonical-field.service >/dev/null
sudo chmod 0644 /etc/systemd/system/ida-canonical-field.service
sed -e "s|@IDA_WS@|$WS_ESC|g" -e "s|@IDA_USER@|$USER_ESC|g" -e "s|@IDA_GROUP@|$GROUP_ESC|g" "$FAKE_GPS_TEMPLATE" | \
  sudo tee /etc/systemd/system/ida-fake-gps.service >/dev/null
sudo chmod 0644 /etc/systemd/system/ida-fake-gps.service
sudo systemctl disable --now ida-yki-perception.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/ida-yki-perception.service
sudo systemctl daemon-reload

echo "Kurulum tamam. Servis BASLATILMADI; boot icin etkinlestirme ayri ve acik bir adimdir."
echo "Saha Jetson'unda: sudo systemctl enable --now ida-canonical-field.service"
echo "Once /etc/ida/field-test.env dosyasini dogrulayin."

