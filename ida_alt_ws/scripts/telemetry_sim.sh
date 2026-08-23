#!/usr/bin/env bash
# Telemetri simülatörü — gerçek Pixhawk telemetrisi yokken failsafe'i önler.
# Kullanım: ./telemetry_sim.sh   (ayrı terminal)
source "$(dirname "$0")/jetson_env.sh"

echo "== Telemetri Sim =="
ros2 run ida_telemetry_sim telemetry_sim_node
