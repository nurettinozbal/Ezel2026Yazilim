#!/usr/bin/env bash
# Canonical gerçek stack sahiplik devri. Takımın mevcut stack'i çalışırken bu
# script kullanılmaz; pasif vehicle_test_lab.launch.py kullanılır.
set -euo pipefail

# Ortamı yükle
source "$(dirname "$0")/jetson_env.sh"
# shellcheck disable=SC1091
source "$IDA_WS/scripts/field_device_discovery.sh"

LIDAR_PORT="$(resolve_field_serial_device lidar "${IDA_LIDAR_PORT:-auto}")" || exit 4
PIXHAWK_PORT="$(resolve_field_serial_device pixhawk "${IDA_PIXHAWK_PORT:-auto}")" || exit 4
CAMERA_TOPIC="${IDA_CAMERA_TOPIC:-/camera/image_raw/compressed}"
SETUP_ENABLED="${IDA_VEHICLE_SETUP_ENABLED:-false}"
GUIDED_ENABLED="${IDA_GUIDED_MODE_ENABLED:-false}"
MOTOR_ENABLED="${IDA_MOTOR_COMMAND_ENABLED:-false}"
MODEL_P1P2="${IDA_MODEL_P1P2:-}"
MODEL_P3="${IDA_MODEL_P3:-}"
MISSION_COUNTS_PARAM="${IDA_MISSION_COUNTS_PARAM:-${IDA_MISSION_P1_COUNT_PARAM:-SCR_USER5}}"
MISSION_CONTROL_PARAM="${IDA_MISSION_CONTROL_PARAM:-${IDA_MISSION_P2_COUNT_PARAM:-SCR_USER6}}"

require_bool() {
  local name="$1"
  local value="$2"
  if [[ "$value" != "true" && "$value" != "false" ]]; then
    echo "RED: $name yalnizca true veya false olabilir (gelen: $value)."
    exit 3
  fi
}

require_bool IDA_VEHICLE_SETUP_ENABLED "$SETUP_ENABLED"
require_bool IDA_GUIDED_MODE_ENABLED "$GUIDED_ENABLED"
require_bool IDA_MOTOR_COMMAND_ENABLED "$MOTOR_ENABLED"
require_bool IDA_FUSION_MODEL_LOADED "${IDA_FUSION_MODEL_LOADED:-false}"
require_bool IDA_CAMERA_CALIBRATED "${IDA_CAMERA_CALIBRATED:-false}"
require_bool IDA_LIDAR_CALIBRATED "${IDA_LIDAR_CALIBRATED:-false}"
require_bool IDA_EXTRINSICS_CALIBRATED "${IDA_EXTRINSICS_CALIBRATED:-false}"

if [[ "${IDA_CANONICAL_TAKEOVER:-false}" != "true" ]]; then
  echo "RED: canonical takeover onayi yok; mevcut takim stack'ine dokunulmadi."
  echo "Pasif test: ros2 launch ida_bringup vehicle_test_lab.launch.py ..."
  echo "Tam devir icin once mevcut stack'i durdurup IDA_CANONICAL_TAKEOVER=true ayarlayin."
  exit 2
fi

if [[ "$SETUP_ENABLED" == "true" || "$GUIDED_ENABLED" == "true" || "$MOTOR_ENABLED" == "true" ]]; then
  if [[ "${IDA_PHYSICAL_SAFETY_ACK:-}" != "PROPELLER_AREA_CLEAR" ]]; then
    echo "RED: setup/motor icin IDA_PHYSICAL_SAFETY_ACK=PROPELLER_AREA_CLEAR gerekli."
    exit 3
  fi
fi

command -v ros2 >/dev/null || { echo "RED: ros2 bulunamadi."; exit 4; }
command -v python3 >/dev/null || { echo "RED: python3 bulunamadi."; exit 4; }
command -v tty_mavlink_router >/dev/null || {
  echo "RED: tty_mavlink_router bulunamadi; workspace build/source edilmemis."
  exit 4
}
command -v lsof >/dev/null || { echo "RED: lsof bulunamadi; seri sahipligi dogrulanamadi."; exit 4; }
python3 -c 'import mavsdk, pymavlink' >/dev/null 2>&1 || {
  echo "RED: Jetson Python ortaminda mavsdk veya pymavlink import edilemiyor."
  exit 4
}
[[ -e "$PIXHAWK_PORT" ]] || { echo "RED: $PIXHAWK_PORT yok."; exit 4; }
[[ -e "$LIDAR_PORT" ]] || { echo "RED: $LIDAR_PORT yok."; exit 4; }

if [[ -n "$MODEL_P1P2" || -n "$MODEL_P3" ]]; then
  python3 -c 'import ultralytics' >/dev/null 2>&1 || {
    echo "RED: model yolu verildi ama ultralytics import edilemiyor."
    exit 4
  }
fi
if [[ -n "$MODEL_P1P2" && ! -f "$MODEL_P1P2" ]]; then
  echo "RED: P1/P2 model dosyasi bulunamadi: $MODEL_P1P2"
  exit 4
fi
if [[ -n "$MODEL_P3" && ! -f "$MODEL_P3" ]]; then
  echo "RED: P3 model dosyasi bulunamadi: $MODEL_P3"
  exit 4
fi
if lsof "$PIXHAWK_PORT" >/dev/null 2>&1; then
  echo "RED: $PIXHAWK_PORT baska bir process tarafindan acik:"
  lsof "$PIXHAWK_PORT" || true
  exit 5
fi

echo "== İDA GERÇEK SİSTEM =="
echo "Pixhawk: $PIXHAWK_PORT (tek sahip: repo ici tty_mavlink_router)"
echo "S2 lidar: $LIDAR_PORT"
echo "Setup: $SETUP_ENABLED, motor: $MOTOR_ENABLED"

echo "== Launch başlatılıyor (Ctrl+C ile durdur) =="
ros2 launch ida_bringup real_vehicle.launch.py \
  canonical_takeover_enabled:=true \
  mavlink_router_enabled:=true \
  pixhawk_serial_port:="$PIXHAWK_PORT" \
  lidar_serial_port:="$LIDAR_PORT" \
  lidar_serial_baudrate:="${IDA_LIDAR_BAUDRATE:-1000000}" \
  lidar_scan_mode:="${IDA_LIDAR_SCAN_MODE:-DenseBoost}" \
  camera_topic:="$CAMERA_TOPIC" \
  dry_run:=false \
  vehicle_setup_enabled:="$SETUP_ENABLED" \
  guided_mode_enabled:="$GUIDED_ENABLED" \
  motor_command_enabled:="$MOTOR_ENABLED" \
  left_motor_servo_channel:="${IDA_LEFT_MOTOR_SERVO_CHANNEL:-9}" \
  right_motor_servo_channel:="${IDA_RIGHT_MOTOR_SERVO_CHANNEL:-11}" \
  target_color_param:="${IDA_TARGET_COLOR_PARAM:-SCR_USER4}" \
  mission_counts_param:="$MISSION_COUNTS_PARAM" \
  mission_control_param:="$MISSION_CONTROL_PARAM" \
  model_path:="$MODEL_P1P2" \
  model_path_p3:="$MODEL_P3" \
  fusion_model_loaded:="${IDA_FUSION_MODEL_LOADED:-false}" \
  camera_calibrated:="${IDA_CAMERA_CALIBRATED:-false}" \
  lidar_calibrated:="${IDA_LIDAR_CALIBRATED:-false}" \
  extrinsics_calibrated:="${IDA_EXTRINSICS_CALIBRATED:-false}"
