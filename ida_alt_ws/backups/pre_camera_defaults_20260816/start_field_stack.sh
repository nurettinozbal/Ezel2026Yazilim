#!/usr/bin/env bash
# Manually-started canonical stack.  This script never arms, changes mode,
# uploads a mission or starts a mission.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/jetson_env.sh"
set -u

bool_value() {
  local name="$1" value="$2"
  if [[ "$value" != "true" && "$value" != "false" ]]; then
    echo "RED: $name yalnizca true/false olabilir (gelen: $value)." >&2
    exit 3
  fi
}

TAKEOVER="${IDA_CANONICAL_TAKEOVER:-false}"
DRY_RUN="${IDA_FIELD_DRY_RUN:-true}"
ROUTER="${IDA_MAVLINK_ROUTER_ENABLED:-false}"
SETUP="${IDA_VEHICLE_SETUP_ENABLED:-false}"
GUIDED="${IDA_GUIDED_MODE_ENABLED:-false}"
MOTOR="${IDA_MOTOR_COMMAND_ENABLED:-false}"
CAMERA_DRIVER="${IDA_FIELD_CAMERA_DRIVER_ENABLED:-true}"
READY_REQUIRED="${IDA_FIELD_REQUIRE_FUSION_READY:-true}"
SINGLE_GENERAL="${IDA_SINGLE_GENERAL_MODEL_ENABLED:-true}"

for item in \
  "IDA_CANONICAL_TAKEOVER:$TAKEOVER" \
  "IDA_FIELD_DRY_RUN:$DRY_RUN" \
  "IDA_MAVLINK_ROUTER_ENABLED:$ROUTER" \
  "IDA_VEHICLE_SETUP_ENABLED:$SETUP" \
  "IDA_GUIDED_MODE_ENABLED:$GUIDED" \
  "IDA_MOTOR_COMMAND_ENABLED:$MOTOR" \
  "IDA_FIELD_CAMERA_DRIVER_ENABLED:$CAMERA_DRIVER" \
  "IDA_FIELD_REQUIRE_FUSION_READY:$READY_REQUIRED" \
  "IDA_SINGLE_GENERAL_MODEL_ENABLED:$SINGLE_GENERAL" \
  "IDA_FUSION_MODEL_LOADED:${IDA_FUSION_MODEL_LOADED:-false}" \
  "IDA_CAMERA_CALIBRATED:${IDA_CAMERA_CALIBRATED:-false}" \
  "IDA_LIDAR_CALIBRATED:${IDA_LIDAR_CALIBRATED:-false}" \
  "IDA_EXTRINSICS_CALIBRATED:${IDA_EXTRINSICS_CALIBRATED:-false}"
do
  bool_value "${item%%:*}" "${item#*:}"
done

if [[ "$TAKEOVER" != "true" ]]; then
  echo "RED: canonical takeover kapali; hicbir node baslatilmadi." >&2
  exit 2
fi
if [[ "$SETUP" != "false" ]]; then
  echo "RED: saha servisi Pixhawk parametre kurulumu yapmayi reddeder." >&2
  exit 2
fi
if command -v systemctl >/dev/null && systemctl is-active --quiet idaws.service; then
  echo "RED: idaws.service aktif. Bu servis onu otomatik durdurmaz." >&2
  exit 5
fi

command -v ros2 >/dev/null || { echo "RED: ros2 bulunamadi." >&2; exit 4; }
command -v lsof >/dev/null || { echo "RED: lsof bulunamadi." >&2; exit 4; }

PIXHAWK_PORT="${IDA_PIXHAWK_PORT:-/dev/idaws_pixhawk}"
LIDAR_PORT="${IDA_LIDAR_PORT:-/dev/idaws_lidar}"
CAMERA_DEVICE="${IDA_FIELD_CAMERA_DEVICE:-/dev/video0}"
MODEL_P1P2="${IDA_MODEL_P1P2:-}"
MODEL_P3="${IDA_MODEL_P3:-$MODEL_P1P2}"

[[ -e "$LIDAR_PORT" ]] || { echo "RED: lidar aygiti yok: $LIDAR_PORT" >&2; exit 4; }
[[ -e "$CAMERA_DEVICE" ]] || { echo "RED: kamera aygiti yok: $CAMERA_DEVICE" >&2; exit 4; }
[[ -f "$MODEL_P1P2" ]] || { echo "RED: P1/P2 model yok: $MODEL_P1P2" >&2; exit 4; }
[[ -f "$MODEL_P3" ]] || { echo "RED: P3 model yok: $MODEL_P3" >&2; exit 4; }

if lsof "$LIDAR_PORT" >/dev/null 2>&1; then
  echo "RED: lidar baska bir process tarafindan kullaniliyor:" >&2
  lsof "$LIDAR_PORT" >&2 || true
  exit 5
fi
if lsof "$CAMERA_DEVICE" >/dev/null 2>&1; then
  echo "RED: kamera baska bir process tarafindan kullaniliyor:" >&2
  lsof "$CAMERA_DEVICE" >&2 || true
  exit 5
fi
if [[ "$ROUTER" == "true" ]]; then
  ROUTER_BIN="$(ros2 pkg prefix ida_control)/lib/ida_control/tty_mavlink_router"
  [[ -x "$ROUTER_BIN" ]] || {
    echo "RED: kurulu tty_mavlink_router bulunamadi; colcon build/source gerekli." >&2
    exit 4
  }
  [[ -e "$PIXHAWK_PORT" ]] || { echo "RED: Pixhawk aygiti yok: $PIXHAWK_PORT" >&2; exit 4; }
  if lsof "$PIXHAWK_PORT" >/dev/null 2>&1; then
    echo "RED: Pixhawk baska bir process tarafindan kullaniliyor:" >&2
    lsof "$PIXHAWK_PORT" >&2 || true
    exit 5
  fi
fi

if [[ "$READY_REQUIRED" == "true" ]]; then
  for item in \
    "IDA_FUSION_MODEL_LOADED:${IDA_FUSION_MODEL_LOADED:-false}" \
    "IDA_CAMERA_CALIBRATED:${IDA_CAMERA_CALIBRATED:-false}" \
    "IDA_LIDAR_CALIBRATED:${IDA_LIDAR_CALIBRATED:-false}" \
    "IDA_EXTRINSICS_CALIBRATED:${IDA_EXTRINSICS_CALIBRATED:-false}"
  do
    if [[ "${item#*:}" != "true" ]]; then
      echo "RED: full fusion icin ${item%%:*}=true kaniti gerekli." >&2
      exit 6
    fi
  done
fi

if [[ "$MOTOR" == "true" ]]; then
  if [[ "$DRY_RUN" != "false" || "$ROUTER" != "true" || "$GUIDED" != "true" ]]; then
    echo "RED: motor yolu dry_run=false, router=true, guided=true ister." >&2
    exit 6
  fi
  if [[ "${IDA_FIELD_PHYSICAL_SAFETY_ACK:-}" != "FIELD_OPERATOR_READY_NO_AUTO_ARM" ]]; then
    echo "RED: exact saha operator onayi eksik." >&2
    exit 6
  fi
fi

echo "== IDA CANONICAL FIELD STACK =="
echo "Bu servis ARM/GUIDED/mission upload/START gondermez. Tum hareket YKI komut zincirindedir."
echo "takeover=$TAKEOVER dry_run=$DRY_RUN router=$ROUTER guided_gate=$GUIDED motor_gate=$MOTOR"

exec ros2 launch ida_bringup field_stack.launch.py \
  canonical_takeover_enabled:="$TAKEOVER" \
  dry_run:="$DRY_RUN" \
  mavlink_router_enabled:="$ROUTER" \
  vehicle_setup_enabled:="$SETUP" \
  guided_mode_enabled:="$GUIDED" \
  motor_command_enabled:="$MOTOR" \
  pixhawk_serial_port:="$PIXHAWK_PORT" \
  pixhawk_baud:="${IDA_PIXHAWK_BAUD:-115200}" \
  lidar_serial_port:="$LIDAR_PORT" \
  lidar_serial_baudrate:="${IDA_LIDAR_BAUDRATE:-1000000}" \
  lidar_scan_mode:="${IDA_LIDAR_SCAN_MODE:-DenseBoost}" \
  field_camera_driver_enabled:="$CAMERA_DRIVER" \
  field_camera_device:="$CAMERA_DEVICE" \
  field_camera_width:="${IDA_FIELD_CAMERA_WIDTH:-960}" \
  field_camera_height:="${IDA_FIELD_CAMERA_HEIGHT:-600}" \
  field_camera_fps:="${IDA_FIELD_CAMERA_FPS:-30}" \
  camera_topic:="${IDA_CAMERA_TOPIC:-/camera/image_raw}" \
  camera_topic_type:="${IDA_CAMERA_TOPIC_TYPE:-raw}" \
  model_path:="$MODEL_P1P2" \
  class_names:="${IDA_MODEL_P1P2_CLASS_NAMES:-orange,yellow}" \
  allowed_colors:="${IDA_MODEL_P1P2_ALLOWED_COLORS:-orange,yellow}" \
  model_path_p3:="$MODEL_P3" \
  class_names_p3:="${IDA_MODEL_P3_CLASS_NAMES:-red,green,black}" \
  allowed_colors_p3:="${IDA_MODEL_P3_ALLOWED_COLORS:-red,green,black}" \
  single_general_model_enabled:="$SINGLE_GENERAL" \
  fusion_model_loaded:="${IDA_FUSION_MODEL_LOADED:-false}" \
  camera_calibrated:="${IDA_CAMERA_CALIBRATED:-false}" \
  lidar_calibrated:="${IDA_LIDAR_CALIBRATED:-false}" \
  extrinsics_calibrated:="${IDA_EXTRINSICS_CALIBRATED:-false}" \
  max_speed_mps:="${IDA_MAX_SPEED_MPS:-0.6}" \
  max_yaw_rate_rad_s:="${IDA_MAX_YAW_RATE_RAD_S:-0.785398163}" \
  left_motor_servo_channel:="${IDA_LEFT_MOTOR_SERVO_CHANNEL:-9}" \
  right_motor_servo_channel:="${IDA_RIGHT_MOTOR_SERVO_CHANNEL:-11}" \
  target_color_param:="${IDA_TARGET_COLOR_PARAM:-SCR_USER4}" \
  mission_counts_param:="${IDA_MISSION_COUNTS_PARAM:-SCR_USER5}" \
  mission_control_param:="${IDA_MISSION_CONTROL_PARAM:-SCR_USER6}" \
  log_dir:="${IDA_LOG_DIR:-$IDA_WS/logs}"
