#!/usr/bin/env bash
# Manually-started canonical stack.  This script never arms, changes mode,
# uploads a mission or starts a mission.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/jetson_env.sh"
set -u
source "$SCRIPT_DIR/field_device_discovery.sh"

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
LIDAR_ENABLED="${IDA_FIELD_LIDAR_ENABLED:-true}"
READY_REQUIRED="${IDA_FIELD_REQUIRE_FUSION_READY:-true}"
REQUIRE_CAMERA="${IDA_FIELD_REQUIRE_CAMERA:-false}"
REQUIRE_LIDAR="${IDA_FIELD_REQUIRE_LIDAR:-false}"
SENSOR_WAIT_TIMEOUT_SEC="${IDA_FIELD_SENSOR_WAIT_TIMEOUT_SEC:-60}"
SENSOR_WAIT_INTERVAL_SEC="${IDA_FIELD_SENSOR_WAIT_INTERVAL_SEC:-2}"
export IDA_CAMERA_CONTROL_PROFILE="${IDA_CAMERA_CONTROL_PROFILE:-$IDA_WS/state/camera_controls.json}"

for item in \
  "IDA_CANONICAL_TAKEOVER:$TAKEOVER" \
  "IDA_FIELD_DRY_RUN:$DRY_RUN" \
  "IDA_MAVLINK_ROUTER_ENABLED:$ROUTER" \
  "IDA_VEHICLE_SETUP_ENABLED:$SETUP" \
  "IDA_GUIDED_MODE_ENABLED:$GUIDED" \
  "IDA_MOTOR_COMMAND_ENABLED:$MOTOR" \
  "IDA_FIELD_CAMERA_DRIVER_ENABLED:$CAMERA_DRIVER" \
  "IDA_FIELD_LIDAR_ENABLED:$LIDAR_ENABLED" \
  "IDA_FIELD_REQUIRE_FUSION_READY:$READY_REQUIRED" \
  "IDA_FIELD_REQUIRE_CAMERA:$REQUIRE_CAMERA" \
  "IDA_FIELD_REQUIRE_LIDAR:$REQUIRE_LIDAR" \
  "IDA_FUSION_MODEL_LOADED:${IDA_FUSION_MODEL_LOADED:-false}" \
  "IDA_CAMERA_CALIBRATED:${IDA_CAMERA_CALIBRATED:-false}" \
  "IDA_LIDAR_CALIBRATED:${IDA_LIDAR_CALIBRATED:-false}" \
  "IDA_EXTRINSICS_CALIBRATED:${IDA_EXTRINSICS_CALIBRATED:-false}"
do
  bool_value "${item%%:*}" "${item#*:}"
done

[[ "$SENSOR_WAIT_TIMEOUT_SEC" =~ ^[1-9][0-9]*$ && "$SENSOR_WAIT_TIMEOUT_SEC" -le 600 ]] || {
  echo "RED: IDA_FIELD_SENSOR_WAIT_TIMEOUT_SEC 1..600 tam sayi olmali." >&2
  exit 3
}
[[ "$SENSOR_WAIT_INTERVAL_SEC" =~ ^[1-9][0-9]*$ && "$SENSOR_WAIT_INTERVAL_SEC" -le 30 ]] || {
  echo "RED: IDA_FIELD_SENSOR_WAIT_INTERVAL_SEC 1..30 tam sayi olmali." >&2
  exit 3
}
if [[ "$REQUIRE_CAMERA" == "true" && "$CAMERA_DRIVER" != "true" ]]; then
  echo "RED: zorunlu kamera, kamera surucusu kapaliyken kullanilamaz." >&2
  exit 3
fi
if [[ "$REQUIRE_LIDAR" == "true" && "$LIDAR_ENABLED" != "true" ]]; then
  echo "RED: zorunlu lidar, lidar surucusu kapaliyken kullanilamaz." >&2
  exit 3
fi

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

PIXHAWK_PORT=""
LIDAR_PORT="/dev/ida-lidar-not-present"
CAMERA_DEVICE="/dev/ida-camera-not-present"
WAIT_DEADLINE=$((SECONDS + SENSOR_WAIT_TIMEOUT_SEC))
echo "Donanim hazirligi bekleniyor: Pixhawk + zorunlu saha sensorleri (${SENSOR_WAIT_TIMEOUT_SEC}s)."
while true; do
  PIXHAWK_PORT="$(resolve_field_serial_device pixhawk "${IDA_PIXHAWK_PORT:-auto}" 2>/dev/null)" || PIXHAWK_PORT=""
  if [[ -n "$PIXHAWK_PORT" && ! -e "$PIXHAWK_PORT" ]]; then PIXHAWK_PORT=""; fi

  if [[ "$LIDAR_ENABLED" == "true" ]]; then
    LIDAR_PORT="$(resolve_field_serial_device lidar "${IDA_LIDAR_PORT:-auto}" 2>/dev/null)" || LIDAR_PORT=""
    if [[ -n "$LIDAR_PORT" && ! -e "$LIDAR_PORT" ]]; then LIDAR_PORT=""; fi
  fi
  if [[ "$CAMERA_DRIVER" == "true" ]]; then
    CAMERA_DEVICE="$(resolve_field_camera_device "${IDA_FIELD_CAMERA_DEVICE:-auto}" 2>/dev/null)" || CAMERA_DEVICE=""
    if [[ -n "$CAMERA_DEVICE" && ! -e "$CAMERA_DEVICE" ]]; then CAMERA_DEVICE=""; fi
  fi

  MISSING=()
  [[ -n "$PIXHAWK_PORT" ]] || MISSING+=("Pixhawk")
  if [[ "$REQUIRE_LIDAR" == "true" && -z "$LIDAR_PORT" ]]; then MISSING+=("lidar"); fi
  if [[ "$REQUIRE_CAMERA" == "true" && -z "$CAMERA_DEVICE" ]]; then MISSING+=("kamera"); fi
  if [[ ${#MISSING[@]} -eq 0 ]]; then break; fi
  if (( SECONDS >= WAIT_DEADLINE )); then
    echo "RED: donanim hazirlik suresi doldu; eksik: ${MISSING[*]}. Otonomi baslatilmadi." >&2
    exit 4
  fi
  sleep "$SENSOR_WAIT_INTERVAL_SEC"
done

if [[ "$LIDAR_ENABLED" == "true" && -z "$LIDAR_PORT" ]]; then
  echo "UYARI: lidar bulunamadi; stack lidar olmadan baslatilacak." >&2
  LIDAR_ENABLED="false"
  LIDAR_PORT="/dev/ida-lidar-not-present"
fi
if [[ "$CAMERA_DRIVER" == "true" && -z "$CAMERA_DEVICE" ]]; then
  echo "UYARI: kamera bulunamadi; stack kamera olmadan baslatilacak." >&2
  CAMERA_DRIVER="false"
  CAMERA_DEVICE="/dev/ida-camera-not-present"
fi
echo "Donanim hazir: Pixhawk=$PIXHAWK_PORT lidar=$LIDAR_PORT kamera=$CAMERA_DEVICE"
MODEL_CATALOG="$IDA_WS/models/model_catalog.json"
MODEL_SELECTION="${IDA_MODEL_SELECTION_FILE:-/etc/ida/model-selection}"
MODEL_TOOL="$IDA_WS/tools/model_catalog.py"
[[ -f "$MODEL_CATALOG" && -f "$MODEL_TOOL" ]] || {
  echo "RED: model katalogu/cozumleyicisi eksik." >&2
  exit 4
}
mapfile -t MODEL_FIELDS < <(
  python3 "$MODEL_TOOL" --catalog "$MODEL_CATALOG" \
    --selection "$MODEL_SELECTION" resolve --profile "${IDA_MODEL_PROFILE:-}"
) || { echo "RED: model profili cozumlenemedi." >&2; exit 4; }
[[ ${#MODEL_FIELDS[@]} -eq 12 ]] || {
  echo "RED: model katalogu eksik alan dondurdu." >&2
  exit 4
}
MODEL_PROFILE="${MODEL_FIELDS[0]}"
MODEL_P1P2="${MODEL_FIELDS[1]}"
MODEL_P1P2_SHA256="${MODEL_FIELDS[2]}"
MODEL_P1P2_CLASS_NAMES="${MODEL_FIELDS[3]}"
MODEL_P1P2_CLASS_MAP="${MODEL_FIELDS[4]}"
MODEL_P1P2_ALLOWED_COLORS="${MODEL_FIELDS[5]}"
MODEL_P3="${MODEL_FIELDS[6]}"
MODEL_P3_SHA256="${MODEL_FIELDS[7]}"
MODEL_P3_CLASS_NAMES="${MODEL_FIELDS[8]}"
MODEL_P3_CLASS_MAP="${MODEL_FIELDS[9]}"
MODEL_P3_ALLOWED_COLORS="${MODEL_FIELDS[10]}"
SINGLE_GENERAL="${MODEL_FIELDS[11]}"

# ``general`` is deliberately a single five-colour model for both camera
# roles.  A damaged/old machine-local selection file must not turn that into
# a fictitious separate P1/P2 model requirement.  Only this named profile is
# allowed to recover to the deployed general artifact; every other profile
# remains fail-closed below.
if [[ "$MODEL_PROFILE" == "general" && -z "$MODEL_P1P2" ]]; then
  GENERAL_MODEL="${IDA_GENERAL_MODEL:-$IDA_WS/models/p3_candidates/190826_brogy.pt}"
  [[ -f "$GENERAL_MODEL" ]] || {
    echo "RED: genel model yok: $GENERAL_MODEL" >&2
    exit 4
  }
  MODEL_P1P2="$GENERAL_MODEL"
  MODEL_P1P2_SHA256="$(sha256sum "$GENERAL_MODEL" | awk '{print $1}')"
  MODEL_P1P2_CLASS_NAMES="black_buoy,red_buoy,green_buoy,orange,yellow"
  MODEL_P1P2_CLASS_MAP='{"black_buoy":"black","red_buoy":"red","green_buoy":"green","orange":"orange","yellow":"yellow"}'
  MODEL_P1P2_ALLOWED_COLORS="orange,yellow"
  SINGLE_GENERAL="true"
  echo "UYARI: general profil P1/P2 alani bos; ayni genel model kullaniliyor." >&2
fi
if [[ "$MODEL_PROFILE" == "general" && -z "$MODEL_P3" ]]; then
  MODEL_P3="$MODEL_P1P2"
  MODEL_P3_SHA256="$MODEL_P1P2_SHA256"
  MODEL_P3_CLASS_NAMES="$MODEL_P1P2_CLASS_NAMES"
  MODEL_P3_CLASS_MAP="$MODEL_P1P2_CLASS_MAP"
  MODEL_P3_ALLOWED_COLORS="red,green,black"
  SINGLE_GENERAL="true"
fi

if [[ "$CAMERA_DRIVER" == "true" ]]; then
  [[ -f "$MODEL_P1P2" ]] || { echo "RED: P1/P2 model yok: $MODEL_P1P2" >&2; exit 4; }
  [[ -f "$MODEL_P3" ]] || { echo "RED: P3 model yok: $MODEL_P3" >&2; exit 4; }
fi
verify_model_hash() {
  local label="$1" path="$2" expected="$3" actual
  [[ "$expected" =~ ^[0-9a-fA-F]{64}$ ]] || {
    echo "RED: $label model SHA256 kontrati eksik/gecersiz." >&2
    exit 4
  }
  command -v sha256sum >/dev/null || { echo "RED: sha256sum bulunamadi." >&2; exit 4; }
  actual="$(sha256sum "$path" | awk '{print $1}')"
  [[ "${actual,,}" == "${expected,,}" ]] || {
    echo "RED: $label model hash uyusmazligi; yanlis/eksik model." >&2
    exit 4
  }
}
if [[ "$CAMERA_DRIVER" == "true" ]]; then
  verify_model_hash P1P2 "$MODEL_P1P2" "$MODEL_P1P2_SHA256"
  verify_model_hash P3 "$MODEL_P3" "$MODEL_P3_SHA256"
fi

if [[ "$LIDAR_ENABLED" == "true" ]] && lsof "$LIDAR_PORT" >/dev/null 2>&1; then
  echo "RED: lidar baska bir process tarafindan kullaniliyor:" >&2
  lsof "$LIDAR_PORT" >&2 || true
  exit 5
fi
if [[ "$CAMERA_DRIVER" == "true" ]] && lsof "$CAMERA_DEVICE" >/dev/null 2>&1; then
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
echo "sensors: camera=$CAMERA_DRIVER lidar=$LIDAR_ENABLED"
if [[ -f "$IDA_CAMERA_CONTROL_PROFILE" ]]; then
  echo "camera_control_profile=$(basename "$IDA_CAMERA_CONTROL_PROFILE") (persistent)"
else
  echo "camera_control_profile=field_profile.yaml defaults"
fi
echo "model_profile=$MODEL_PROFILE p1p2=$(basename "$MODEL_P1P2") p3=$(basename "$MODEL_P3")"

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
  lidar_enabled:="$LIDAR_ENABLED" \
  field_camera_driver_enabled:="$CAMERA_DRIVER" \
  field_camera_device:="$CAMERA_DEVICE" \
  camera_topic:="${IDA_CAMERA_TOPIC:-/camera/image_raw}" \
  camera_topic_type:="${IDA_CAMERA_TOPIC_TYPE:-raw}" \
  model_path:="$MODEL_P1P2" \
  class_names:="$MODEL_P1P2_CLASS_NAMES" \
  class_name_map:="$MODEL_P1P2_CLASS_MAP" \
  allowed_colors:="$MODEL_P1P2_ALLOWED_COLORS" \
  model_path_p3:="$MODEL_P3" \
  class_names_p3:="$MODEL_P3_CLASS_NAMES" \
  class_name_map_p3:="$MODEL_P3_CLASS_MAP" \
  allowed_colors_p3:="$MODEL_P3_ALLOWED_COLORS" \
  single_general_model_enabled:="$SINGLE_GENERAL" \
  yki_debug_enabled:="${IDA_YKI_DEBUG_ENABLED:-false}" \
  yki_debug_websocket_url:="${IDA_YKI_DEBUG_WEBSOCKET_URL:-auto://yki}" \
  fusion_model_loaded:="${IDA_FUSION_MODEL_LOADED:-false}" \
  camera_calibrated:="${IDA_CAMERA_CALIBRATED:-false}" \
  lidar_calibrated:="${IDA_LIDAR_CALIBRATED:-false}" \
  extrinsics_calibrated:="${IDA_EXTRINSICS_CALIBRATED:-false}" \
  left_motor_servo_channel:="${IDA_LEFT_MOTOR_SERVO_CHANNEL:-9}" \
  right_motor_servo_channel:="${IDA_RIGHT_MOTOR_SERVO_CHANNEL:-11}" \
  target_color_param:="${IDA_TARGET_COLOR_PARAM:-SCR_USER4}" \
  mission_counts_param:="${IDA_MISSION_COUNTS_PARAM:-SCR_USER5}" \
  mission_control_param:="${IDA_MISSION_CONTROL_PARAM:-SCR_USER6}" \
  log_dir:="${IDA_LOG_DIR:-$IDA_WS/logs}"
