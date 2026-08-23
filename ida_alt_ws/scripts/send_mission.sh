#!/usr/bin/env bash
# Görevi yükle + başlat (costmap P1/P2'de akmaya başlar).
# Kullanım: ./send_mission.sh <lat> <lon>   (varsayılan: 40.86, 29.25)
source "$(dirname "$0")/jetson_env.sh"

LAT="${1:-40.86}"
LON="${2:-29.25}"

echo "== Görev: [$LAT, $LON] =="
ros2 topic pub /mission/waypoints std_msgs/String "{\"data\":\"{\\\"waypoints\\\":[{\\\"lat\\\":$LAT,\\\"lon\\\":$LON}]}\"}" --once
sleep 0.5
ros2 topic pub /mission/start std_msgs/String '{"data":"{\"start\":true}"}' --once

echo "== Görev başlatıldı. İzle: =="
echo "  ros2 topic echo /autonomy/state"
echo "  ros2 topic echo /planning/costmap"
