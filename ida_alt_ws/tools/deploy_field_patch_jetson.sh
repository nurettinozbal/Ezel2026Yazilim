#!/usr/bin/env bash
# Source-only deployment helper.  Never starts services, launches ROS, opens a
# serial device, arms a vehicle or sends a Pixhawk command.
set -euo pipefail

archive="/tmp/field_stack_yki_patch_20260815.zip"
workspace="/home/ezelproject/ida_alt_ws"
expected="ca57b1ee077b18bc355b624c29d28c26f876e970e8553d420b22946234c5952e"

actual="$(sha256sum "$archive" | awk '{print $1}')"
if [[ "$actual" != "$expected" ]]; then
  echo "Archive SHA-256 mismatch" >&2
  exit 2
fi
test -d "$workspace/src"
cd "$workspace"
mkdir -p backups

files=(
  src/ida_perception/ida_perception/yolo_camera_node.py
  src/ida_perception/test/test_raw_contract_nodes.py
  src/ida_bringup/config/perception.yaml
  src/ida_bringup/launch/real_vehicle.launch.py
  scripts/field_test.env.example
  scripts/start_field_stack.sh
  scripts/install_field_service.sh
  tools/test_field_stack_service.py
  docs/SAHA_SERVISI_VE_YKI_GOREV_AKISI.md
  ezel-yazilim_yeni/ezel-yazilim/arayuz/src/context/VehicleContext.jsx
  ezel-yazilim_yeni/ezel-yazilim/arayuz/src/features/MapSystem/components/MapView.jsx
  ezel-yazilim_yeni/ezel-yazilim/arayuz/src/features/MissionControl/components/MissionPlanner.jsx
)
existing=()
for file in "${files[@]}"; do
  [[ -e "$file" ]] && existing+=("$file")
done
if ((${#existing[@]})); then
  tar -czf "backups/pre_field_stack_20260816_$(date +%H%M%S).tar.gz" "${existing[@]}"
fi

unzip -q -o "$archive" -d "$workspace"
chmod 0755 \
  scripts/start_field_stack.sh \
  scripts/ida_alt_ws \
  scripts/install_field_service.sh \
  scripts/field_device_discovery.sh
echo "SYNC_OK"
ls -l \
  src/ida_bringup/launch/field_stack.launch.py \
  scripts/start_field_stack.sh \
  systemd/ida-canonical-field.service.in
