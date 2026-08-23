#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-status}"
YKI_HOST="${2:-}"
YKI_PORT="${3:-14550}"
WORKSPACE="${IDA_SIM_WS:-$HOME/ida_sim_ws}"
UNIT_DIR="$HOME/.config/systemd/user"
UNIT_FILE="$UNIT_DIR/ida-yki-sim.service"
ENV_FILE="$WORKSPACE/.sim-yki.env"

validate_endpoint() {
  if [[ ! "$YKI_HOST" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
    echo "HATA: YKI host gecerli bir IPv4 olmali" >&2
    exit 2
  fi
  local octet
  IFS=. read -r -a octets <<< "$YKI_HOST"
  for octet in "${octets[@]}"; do
    if (( 10#$octet > 255 )); then
      echo "HATA: YKI IPv4 aralik disi" >&2
      exit 2
    fi
  done
  if [[ ! "$YKI_PORT" =~ ^[0-9]+$ ]] || (( YKI_PORT < 1 || YKI_PORT > 65535 )); then
    echo "HATA: YKI UDP portu [1,65535] araliginda olmali" >&2
    exit 2
  fi
}

install_unit() {
  mkdir -p "$UNIT_DIR" "$WORKSPACE/logs"
  cat > "$UNIT_FILE" <<'UNIT'
[Unit]
Description=IDA isolated YKI Gazebo/SITL simulation
After=network.target

[Service]
Type=simple
EnvironmentFile=%h/ida_sim_ws/.sim-yki.env
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash; source "$HOME/ida_sim_ws/install/setup.bash"; exec xvfb-run -a ros2 launch ida_bringup sim_gazebo.launch.py speedup:=1 sitl_wipe:=true sim_field_fidelity:=true auto_mission:=false mission_raw_enabled:=true target_color_param:=SCR_USER4 sensor_fusion_enabled:=true sensor_fusion_shadow_mode:=false perception_sim_publish_canonical:=false perception_sim_lidar_range_m:=35.0 yki_mavlink_host:=${YKI_HOST} yki_mavlink_port:=${YKI_PORT}'
WorkingDirectory=%h/ida_sim_ws
KillMode=mixed
TimeoutStopSec=15
Restart=no
StandardOutput=append:%h/ida_sim_ws/logs/yki_sim.log
StandardError=append:%h/ida_sim_ws/logs/yki_sim.log

[Install]
WantedBy=default.target
UNIT
  systemctl --user daemon-reload
}

case "$ACTION" in
  start)
    validate_endpoint
    [[ -f "$WORKSPACE/install/setup.bash" ]] || {
      echo "HATA: $WORKSPACE/install/setup.bash yok; workspace derlenmemis" >&2
      exit 3
    }
    install_unit
    printf 'YKI_HOST=%s\nYKI_PORT=%s\n' "$YKI_HOST" "$YKI_PORT" > "$ENV_FILE"
    systemctl --user stop ida-yki-sim.service >/dev/null 2>&1 || true
    systemctl --user reset-failed ida-yki-sim.service >/dev/null 2>&1 || true
    systemctl --user start ida-yki-sim.service
    sleep 2
    systemctl --user --no-pager --full status ida-yki-sim.service | sed -n '1,12p'
    ;;
  stop)
    systemctl --user stop ida-yki-sim.service
    echo "IDA SIM durduruldu"
    ;;
  status)
    systemctl --user --no-pager --full status ida-yki-sim.service | sed -n '1,18p'
    ;;
  logs)
    tail -n "${2:-80}" "$WORKSPACE/logs/yki_sim.log"
    ;;
  *)
    echo "Kullanim: $0 {start YKI_IPV4 [UDP_PORT]|stop|status|logs [SATIR]}" >&2
    exit 2
    ;;
esac
