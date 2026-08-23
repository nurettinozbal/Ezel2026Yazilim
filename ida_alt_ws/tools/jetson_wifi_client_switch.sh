#!/usr/bin/env bash
# Jetson'u mevcut Wi-Fi hotspot/AP profilinden telefon Wi-Fi'sine geçirir.
#
# Ana mod bağlantı profilini hazırlar ve geçişi systemd transient service olarak
# zamanlar. Böylece hotspot kapanınca SSH oturumu düşse bile işlem tamamlanır.
# Parola loglara veya komut çıktısına yazılmaz.

set -Eeuo pipefail

usage() {
  cat <<'EOF'
Kullanım:
  sudo ./jetson_wifi_client_switch.sh --ssid "Telefon WiFi Adı" [--interface wlan0]

Geçiş sırasında mevcut SSH bağlantısı kesilir. Yeni IP'yi telefonun bağlı
cihazlar listesinden bulun ve tekrar bağlanın.
EOF
}

log() {
  printf '[ida-wifi] %s\n' "$*"
}

require_root() {
  if [[ ${EUID} -ne 0 ]]; then
    echo "Bu script NetworkManager profili değiştirdiği için sudo ile çalıştırılmalı." >&2
    exit 2
  fi
}

require_tools() {
  local tool
  for tool in nmcli systemd-run; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
      echo "Gerekli komut bulunamadı: ${tool}" >&2
      exit 3
    fi
  done
}

wifi_interface() {
  nmcli -t -f DEVICE,TYPE device status |
    awk -F: '$2 == "wifi" { print $1; exit }'
}

active_wifi_profile() {
  local interface=$1
  nmcli -t -f NAME,TYPE,DEVICE connection show --active |
    awk -F: -v dev="${interface}" '$2 == "802-11-wireless" && $3 == dev { print $1; exit }'
}

worker() {
  local interface=$1
  local target_profile=$2
  local previous_profile=$3
  local attempt
  local target_ssid

  log "Hotspot profili kapatılıyor: ${previous_profile:-yok}"
  if [[ -n ${previous_profile} && ${previous_profile} != "${target_profile}" ]]; then
    nmcli connection down "${previous_profile}" >/dev/null 2>&1 || true
  fi

  # AP -> client geçişinde bazı Jetson sürücüleri birkaç saniye AP kanalında
  # kalabiliyor. Cihazı açık/managed istemci durumuna getirip hedef SSID gerçekten
  # görülene kadar sınırlı tarama yap; parola/activation denemesine körlemesine
  # geçme.
  nmcli radio wifi on
  nmcli device set "${interface}" managed yes
  nmcli device disconnect "${interface}" >/dev/null 2>&1 || true
  sleep 2
  target_ssid=$(nmcli -g 802-11-wireless.ssid connection show "${target_profile}")
  for attempt in $(seq 1 10); do
    nmcli device wifi rescan ifname "${interface}" >/dev/null 2>&1 || true
    sleep 2
    if nmcli -t -f SSID device wifi list ifname "${interface}" |
      sed 's/\\:/\x00/g' | grep -Fqx "${target_ssid}"; then
      break
    fi
  done
  if [[ ${attempt} -ge 10 ]] && ! nmcli -t -f SSID device wifi list ifname "${interface}" |
    sed 's/\\:/\x00/g' | grep -Fqx "${target_ssid}"; then
    log "Hedef SSID taramada bulunamadı: ${target_ssid}. Önceki hotspot geri açılıyor."
    if [[ -n ${previous_profile} && ${previous_profile} != "${target_profile}" ]]; then
      nmcli connection up "${previous_profile}" ifname "${interface}" || true
    fi
    exit 9
  fi

  log "Telefon Wi-Fi profiline bağlanılıyor: ${target_profile} (${interface})"
  if ! nmcli connection up "${target_profile}" ifname "${interface}"; then
    log "Telefon Wi-Fi bağlantısı kurulamadı. Önceki hotspot geri açılıyor."
    if [[ -n ${previous_profile} && ${previous_profile} != "${target_profile}" ]]; then
      nmcli connection up "${previous_profile}" ifname "${interface}" || true
    fi
    exit 10
  fi

  # DHCP ve telefon internet paylaşımının hazır olması için sınırlı bekleme.
  for attempt in $(seq 1 15); do
    if nmcli -g IP4.ADDRESS device show "${interface}" | grep -qE '^[0-9]'; then
      if ping -c 1 -W 2 1.1.1.1 >/dev/null 2>&1; then
        log "Bağlantı ve internet doğrulandı."
        nmcli -g IP4.ADDRESS device show "${interface}"
        exit 0
      fi
    fi
    sleep 2
  done

  log "Wi-Fi IP aldı fakat internet 30 saniyede doğrulanamadı. Güvenli geri dönüş yapılıyor."
  nmcli connection down "${target_profile}" >/dev/null 2>&1 || true
  if [[ -n ${previous_profile} && ${previous_profile} != "${target_profile}" ]]; then
    nmcli connection up "${previous_profile}" ifname "${interface}" || true
  fi
  exit 11
}

if [[ ${1:-} == "--worker" ]]; then
  require_root
  require_tools
  [[ $# -eq 4 ]] || exit 2
  worker "$2" "$3" "$4"
  exit $?
fi

require_root
require_tools

ssid=""
interface=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ssid)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      ssid=$2
      shift 2
      ;;
    --interface)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      interface=$2
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Bilinmeyen argüman: $1" >&2
      usage
      exit 2
      ;;
  esac
done

[[ -n ${ssid} ]] || { echo "--ssid zorunlu." >&2; usage; exit 2; }
if [[ ${#ssid} -gt 32 || ${ssid} == *$'\n'* || ${ssid} == *$'\r'* ]]; then
  echo "SSID geçersiz veya 32 karakterden uzun." >&2
  exit 2
fi

if [[ -z ${interface} ]]; then
  interface=$(wifi_interface)
fi
[[ -n ${interface} ]] || { echo "Wi-Fi arayüzü bulunamadı." >&2; exit 4; }
nmcli -t -f DEVICE,TYPE device status | grep -Fqx "${interface}:wifi" || {
  echo "Seçilen cihaz Wi-Fi arayüzü değil: ${interface}" >&2
  exit 4
}

previous_profile=$(active_wifi_profile "${interface}" || true)
profile_suffix=$(printf '%s' "${ssid}" | sha256sum | cut -c1-10)
target_profile="ida-phone-${profile_suffix}"

printf "Telefon Wi-Fi parolası (ekranda görünmez): "
IFS= read -rs wifi_password
printf '\n'
if [[ ${#wifi_password} -lt 8 || ${#wifi_password} -gt 63 ]]; then
  unset wifi_password
  echo "WPA/WPA2 parolası 8–63 karakter olmalı." >&2
  exit 2
fi

# Profil hazırlanırken mevcut hotspot bağlantısı açık kalır.
if nmcli -t -f NAME connection show | grep -Fqx "${target_profile}"; then
  nmcli connection modify "${target_profile}" \
    connection.interface-name "${interface}" \
    connection.autoconnect yes \
    802-11-wireless.ssid "${ssid}" \
    802-11-wireless.mode infrastructure \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.psk "${wifi_password}"
else
  nmcli connection add type wifi ifname "${interface}" con-name "${target_profile}" \
    ssid "${ssid}" \
    802-11-wireless.mode infrastructure \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.psk "${wifi_password}" \
    connection.autoconnect yes
fi
unset wifi_password

script_path=$(readlink -f "$0")
unit="ida-wifi-switch-$(date +%s)"

log "Geçiş 3 saniye sonra başlayacak. SSH bağlantısının kesilmesi normaldir."
log "Başarısız olursa önceki profil geri açılacak: ${previous_profile:-yok}"
systemd-run \
  --unit "${unit}" \
  --collect \
  --on-active=3s \
  "${script_path}" --worker "${interface}" "${target_profile}" "${previous_profile:-}"

log "Yeni IP'yi telefonun bağlı cihazlar listesinden bulun."
log "Jetson ekranından durum: journalctl -u ${unit} --no-pager"
