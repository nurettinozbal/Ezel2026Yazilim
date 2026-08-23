#!/usr/bin/env bash
# Değişiklik yapmadan sim sunucusunun hazır olduğunu doğrular.
set -euo pipefail

SIM_WS="${IDA_SIM_WS:-$HOME/ida_sim_ws}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"

require_file() {
  [[ -e "$1" ]] || { echo "EKSİK: $1" >&2; exit 2; }
}
require_command() {
  command -v "$1" >/dev/null || { echo "EKSİK KOMUT: $1" >&2; exit 3; }
}

require_file /opt/ros/humble/setup.bash
require_file "$SIM_WS/install/setup.bash"
require_file "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
require_file "$ARDUPILOT_DIR/build/sitl/bin/ardurover"

set +u
source /opt/ros/humble/setup.bash
source "$SIM_WS/install/setup.bash"
set -u

for cmd in ros2 colcon gazebo gzserver python3; do require_command "$cmd"; done
python3 - <<'PY'
import mavsdk, pymavlink, serial
print("Python runtime: OK")
PY

if [[ "${IDA_VERIFY_GPU:-false}" == "true" ]]; then
  require_command nvidia-smi
  nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
  python3 - <<'PY'
import torch
assert torch.cuda.is_available(), "PyTorch CUDA GPU'yu görmüyor"
print("PyTorch CUDA:", torch.__version__, torch.version.cuda, torch.cuda.get_device_name(0))
import ultralytics
print("Ultralytics:", ultralytics.__version__)
import tensorrt
print("TensorRT:", tensorrt.__version__)
PY
fi

ros2 pkg prefix ida_bringup >/dev/null
ros2 pkg prefix ida_autonomy >/dev/null
ros2 pkg prefix ida_planning >/dev/null
ros2 pkg prefix gazebo_ros >/dev/null

python3 "$SIM_WS/src/ida_planning/test_saf.py"
python3 "$SIM_WS/src/ida_autonomy/test/test_saf.py"

echo "SIM SUNUCU DOĞRULAMA: OK"
