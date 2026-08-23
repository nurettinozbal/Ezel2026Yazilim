#!/usr/bin/env bash
# Legacy cleanup: raw lidar/costmap is no longer sent to YKI over Wi-Fi/WebSocket.
set -euo pipefail

sudo systemctl disable --now ida-yki-perception.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/ida-yki-perception.service
sudo systemctl daemon-reload
echo "ida-yki-perception kaldirildi. Yerel ROS lidar/fuzyon ve map.mp4 kaydi etkilenmez."


