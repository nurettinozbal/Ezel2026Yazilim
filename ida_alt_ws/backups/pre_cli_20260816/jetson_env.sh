#!/usr/bin/env bash
# Jetson ROS2 ortamını yükler — HER terminalde source et.
# ~/.bashrc'e eklersen otomatik yüklenir:
#   echo "source ~/ida_ws/scripts/jetson_env.sh" >> ~/.bashrc
#
# Kullanım: source ~/ida_ws/scripts/jetson_env.sh
# NOT: venv YOK — sistem Python + pip --user paketleri (rplidar-roboticia vb.)

# ROS2 Humble
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash

# Workspace, script konumundan bulunur; takım workspace'ine sabitlenmez.
_IDA_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export IDA_WS="${IDA_WS:-$(cd -- "$_IDA_SCRIPT_DIR/.." && pwd)}"
[ -f "$IDA_WS/install/setup.bash" ] && source "$IDA_WS/install/setup.bash"

# Kolaylık: workspace kısayolu + paket yolu (Python import için)
export PYTHONPATH=$IDA_WS/src:$IDA_WS/install:$PYTHONPATH

echo "[jetson_env] ROS2 + ida_ws yüklendi (venv yok, sistem Python)"
