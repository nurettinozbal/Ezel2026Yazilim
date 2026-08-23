#!/usr/bin/env bash
# Source-only deployment helper for the 2026-08-17 ROS-side batch fix (Jetson).
# Covers: near-field safety + DWA hysteresis, waypoint threshold 4.0/6.0,
# decision logger QoS + SERVO_OUTPUT_RAW reader, P3 bench mode (autonomy).
# Never starts services, launches ROS, opens a serial device, arms a vehicle
# or sends a Pixhawk command.
set -euo pipefail

archive="/tmp/ida_jetson_20260817.zip"
workspace="/home/ezelproject/ida_alt_ws"
expected="2569ef71c344bbf60004eabeaaab44c5d37a88aa834cad059daf9967989e0bc8"

if [[ ! -f "$archive" ]]; then
  echo "RED: arsiv yok: $archive" >&2
  echo "Zip dosyasini once Jetson'a kopyalayin:" >&2
  echo "  scp ida_jetson_20260817.zip ezelproject@<jetson-ip>:/tmp/" >&2
  exit 2
fi
actual="$(sha256sum "$archive" | awk '{print $1}')"
if [[ "${actual,,}" != "$expected" ]]; then
  echo "RED: Archive SHA-256 uyusmazligi" >&2
  echo "  beklenen: $expected" >&2
  echo "  gelen   : $actual" >&2
  exit 2
fi
test -d "$workspace/src"
cd "$workspace"
mkdir -p backups

# Yalniz gercekten var olan dosyalari yedekle.
existing=()
while IFS= read -r line; do
  existing+=("$line")
done < <(unzip -Z1 "$archive" | grep -v '/$')
if ((${#existing[@]})); then
  tar -czf "backups/pre_ros_fix_20260817_$(date +%H%M%S).tar.gz" "${existing[@]}"
fi

unzip -q -o "$archive" -d "$workspace"

echo "SYNC_OK: 2026-08-17 ROS batch fix (near-field/waypoint/logger/P3-bench)"
echo "Sonraki adimlar:"
echo "  cd $workspace && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash"
echo "  ida_cli unit"
echo "  P3 bench icin: ros2 launch ida_bringup bench_p3_decision.launch.py"
