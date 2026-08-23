#!/usr/bin/env bash
# Restrained yellow-buoy avoidance bench launcher.
#
# This script never arms by itself.  Defaults fail closed and the normal field
# autonomy.yaml remains unchanged.  Run only after the team stack is stopped and
# the Pixhawk port is proven free.
set -eo pipefail

# ROS Humble setup scripts legitimately probe unset AMENT variables.  Load the
# environment before enabling nounset; all bench variables below remain strict.
source "$(dirname "$0")/jetson_env.sh"
set -u

TAKEOVER="${IDA_CANONICAL_TAKEOVER:-false}"
DRY_RUN="${IDA_BENCH_DRY_RUN:-true}"
ROUTER="${IDA_MAVLINK_ROUTER_ENABLED:-false}"
SETUP="${IDA_VEHICLE_SETUP_ENABLED:-false}"
GUIDED="${IDA_GUIDED_MODE_ENABLED:-false}"
MOTOR="${IDA_MOTOR_COMMAND_ENABLED:-false}"
PIXHAWK_PORT="${IDA_PIXHAWK_PORT:-/dev/pixhawk}"
LIDAR_PORT="${IDA_LIDAR_PORT:-/dev/lidar}"
MODEL_P1P2="${IDA_MODEL_P1P2:-}"
MODEL_P3="${IDA_MODEL_P3:-}"
MAX_SPEED="${IDA_BENCH_MAX_SPEED_MPS:-0.25}"
STUCK_TIMEOUT="${IDA_BENCH_STUCK_TIMEOUT_S:-30.0}"

require_bool() {
  local name="$1"
  local value="$2"
  if [[ "$value" != "true" && "$value" != "false" ]]; then
    echo "RED: $name yalnizca true veya false olabilir (gelen: $value)."
    exit 3
  fi
}

for pair in \
  "IDA_CANONICAL_TAKEOVER:$TAKEOVER" \
  "IDA_BENCH_DRY_RUN:$DRY_RUN" \
  "IDA_MAVLINK_ROUTER_ENABLED:$ROUTER" \
  "IDA_VEHICLE_SETUP_ENABLED:$SETUP" \
  "IDA_GUIDED_MODE_ENABLED:$GUIDED" \
  "IDA_MOTOR_COMMAND_ENABLED:$MOTOR" \
  "IDA_FUSION_MODEL_LOADED:${IDA_FUSION_MODEL_LOADED:-false}" \
  "IDA_CAMERA_CALIBRATED:${IDA_CAMERA_CALIBRATED:-false}" \
  "IDA_LIDAR_CALIBRATED:${IDA_LIDAR_CALIBRATED:-false}" \
  "IDA_EXTRINSICS_CALIBRATED:${IDA_EXTRINSICS_CALIBRATED:-false}"
do
  require_bool "${pair%%:*}" "${pair#*:}"
done

if [[ "$TAKEOVER" != "true" ]]; then
  echo "RED: bench canonical takeover onayi yok; hicbir ROS node baslatilmadi."
  exit 2
fi

command -v ros2 >/dev/null || { echo "RED: ros2 bulunamadi."; exit 4; }
command -v lsof >/dev/null || { echo "RED: lsof bulunamadi."; exit 4; }
[[ -e "$LIDAR_PORT" ]] || { echo "RED: lidar portu yok: $LIDAR_PORT"; exit 4; }
[[ -n "$MODEL_P1P2" && -f "$MODEL_P1P2" ]] || {
  echo "RED: genel orange/yellow model yolu bulunamadi: $MODEL_P1P2"
  exit 4
}

if [[ "$ROUTER" == "true" ]]; then
  command -v mavlink-routerd >/dev/null || {
    echo "RED: mavlink-routerd bulunamadi; Pixhawk seri portu acilmayacak."
    exit 4
  }
  [[ -e "$PIXHAWK_PORT" ]] || { echo "RED: Pixhawk portu yok: $PIXHAWK_PORT"; exit 4; }
  if lsof "$PIXHAWK_PORT" >/dev/null 2>&1; then
    echo "RED: $PIXHAWK_PORT baska bir process tarafindan acik:"
    lsof "$PIXHAWK_PORT" || true
    exit 5
  fi
fi

if [[ "$MOTOR" == "true" ]]; then
  if [[ "$DRY_RUN" != "false" || "$GUIDED" != "true" || "$ROUTER" != "true" ]]; then
    echo "RED: motor bench yolu dry_run=false, guided=true ve router=true ister."
    exit 6
  fi
  if [[ "${IDA_BENCH_PHYSICAL_SAFETY_ACK:-}" != "VEHICLE_RESTRAINED_MOTOR_AREA_CLEAR" ]]; then
    echo "RED: restrained-bench fiziksel guvenlik onayi yok."
    exit 6
  fi
  for ready in \
    "IDA_FUSION_MODEL_LOADED:${IDA_FUSION_MODEL_LOADED:-false}" \
    "IDA_CAMERA_CALIBRATED:${IDA_CAMERA_CALIBRATED:-false}" \
    "IDA_LIDAR_CALIBRATED:${IDA_LIDAR_CALIBRATED:-false}" \
    "IDA_EXTRINSICS_CALIBRATED:${IDA_EXTRINSICS_CALIBRATED:-false}"
  do
    if [[ "${ready#*:}" != "true" ]]; then
      echo "RED: motorlu sensor-fusion bench testi icin ${ready%%:*}=true gerekli."
      exit 6
    fi
  done
fi

echo "== SARI DUBA KACINMA BENCH PROFILI =="
echo "Takeover=$TAKEOVER dry_run=$DRY_RUN router=$ROUTER guided=$GUIDED motor=$MOTOR"
echo "max_speed=$MAX_SPEED m/s stuck_timeout=$STUCK_TIMEOUT s"
echo "model=$MODEL_P1P2"
echo "Bu script ARM veya mission START gondermez."

ros2 launch ida_bringup bench_avoidance.launch.py \
  canonical_takeover_enabled:="$TAKEOVER" \
  dry_run:="$DRY_RUN" \
  mavlink_router_enabled:="$ROUTER" \
  vehicle_setup_enabled:="$SETUP" \
  guided_mode_enabled:="$GUIDED" \
  motor_command_enabled:="$MOTOR" \
  bench_max_speed_mps:="$MAX_SPEED" \
  bench_stuck_timeout_s:="$STUCK_TIMEOUT" \
  pixhawk_serial_port:="$PIXHAWK_PORT" \
  lidar_serial_port:="$LIDAR_PORT" \
  camera_topic:="${IDA_CAMERA_TOPIC:-/camera/image_raw}" \
  camera_topic_type:="${IDA_CAMERA_TOPIC_TYPE:-raw}" \
  bench_camera_driver_enabled:="${IDA_BENCH_CAMERA_DRIVER_ENABLED:-true}" \
  bench_camera_device:="${IDA_BENCH_CAMERA_DEVICE:-/dev/video0}" \
  bench_camera_width:="${IDA_BENCH_CAMERA_WIDTH:-960}" \
  bench_camera_height:="${IDA_BENCH_CAMERA_HEIGHT:-600}" \
  bench_camera_fps:="${IDA_BENCH_CAMERA_FPS:-30}" \
  model_path:="$MODEL_P1P2" \
  class_names:="${IDA_MODEL_CLASS_NAMES:-orange,yellow}" \
  model_path_p3:="$MODEL_P3" \
  fusion_model_loaded:="${IDA_FUSION_MODEL_LOADED:-false}" \
  camera_calibrated:="${IDA_CAMERA_CALIBRATED:-false}" \
  lidar_calibrated:="${IDA_LIDAR_CALIBRATED:-false}" \
  extrinsics_calibrated:="${IDA_EXTRINSICS_CALIBRATED:-false}" \
  left_motor_servo_channel:="${IDA_LEFT_MOTOR_SERVO_CHANNEL:-9}" \
  right_motor_servo_channel:="${IDA_RIGHT_MOTOR_SERVO_CHANNEL:-11}" \
  target_color_param:="${IDA_TARGET_COLOR_PARAM:-SCR_USER4}" \
  mission_counts_param:="${IDA_MISSION_COUNTS_PARAM:-SCR_USER5}" \
  mission_control_param:="${IDA_MISSION_CONTROL_PARAM:-SCR_USER6}" \
  log_dir:="${IDA_LOG_DIR:-./logs}"
