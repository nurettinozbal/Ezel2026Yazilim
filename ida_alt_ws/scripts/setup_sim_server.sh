#!/usr/bin/env bash
# Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic / ArduPilot Rover SITL kurulumu.
# Tekrar çalıştırılabilir; GPU/NVIDIA sürücüsüne dokunmaz.
set -euo pipefail

SIM_USER="${SUDO_USER:-${USER}}"
if [[ "$SIM_USER" == "root" ]]; then
  echo "HATA: Scripti normal kullanıcıyla çalıştırın; sudo'yu script kullanır." >&2
  exit 2
fi
SIM_HOME="$(getent passwd "$SIM_USER" | cut -d: -f6)"
SIM_WS="${IDA_SIM_WS:-$SIM_HOME/ida_sim_ws}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$SIM_HOME/ardupilot}"
ARDUPILOT_REF="${ARDUPILOT_REF:-Rover-4.7.0}"
# Temel sim kurulumu varsayilan olarak GPU/ML paketlerinden bagimsizdir.
# YOLO/CUDA/TensorRT daha sonra acikca IDA_INSTALL_GPU=true ile eklenir.
INSTALL_GPU="${IDA_INSTALL_GPU:-false}"

if [[ "$(. /etc/os-release; printf '%s' "$VERSION_ID")" != "22.04" ]]; then
  echo "HATA: Bu kurulum Ubuntu 22.04 için kilitlidir." >&2
  exit 3
fi
if [[ ! -d "$SIM_WS/src" ]]; then
  echo "HATA: ROS kaynakları bulunamadı: $SIM_WS/src" >&2
  exit 4
fi

export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
sudo apt-get install -y \
  ca-certificates curl gnupg lsb-release locales software-properties-common \
  git build-essential cmake ninja-build pkg-config \
  python3-dev python3-pip python3-setuptools python3-venv python3-yaml

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe

sudo install -d -m 0755 /usr/share/keyrings
if [[ ! -s /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
ARCH="$(dpkg --print-architecture)"
CODENAME="$(. /etc/os-release; printf '%s' "$UBUNTU_CODENAME")"
printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n' \
  "$ARCH" "$CODENAME" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt-get update
sudo apt-get install -y \
  ros-humble-desktop ros-dev-tools \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-msgs \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-compressed-image-transport \
  python3-rosdep python3-vcstool python3-colcon-common-extensions

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

if [[ ! -d "$ARDUPILOT_DIR/.git" ]]; then
  git clone --branch "$ARDUPILOT_REF" --depth 1 --recurse-submodules \
    https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
else
  EXISTING_REF="$(git -C "$ARDUPILOT_DIR" describe --tags --always --dirty)"
  if [[ "$EXISTING_REF" != "$ARDUPILOT_REF" ]]; then
    echo "HATA: $ARDUPILOT_DIR zaten var ve beklenen ref değil: $EXISTING_REF" >&2
    echo "Mevcut clone'a reset/checkout uygulanmadı." >&2
    exit 5
  fi
fi

"$ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh" -y

# ArduPilot prereq script'i MAVProxy/pymavlink'i kurar. IDA bridge'in ek
# Python runtime'ları sistem ROS Python'uyla uyumlu user-site'a kurulur.
python3 -m pip install --user --upgrade \
  "mavsdk>=2.12,<4" "pyserial>=3.5,<4" "websockets>=10,<16"

if [[ "$INSTALL_GPU" == "true" ]]; then
  if ! command -v nvidia-smi >/dev/null || ! nvidia-smi >/dev/null; then
    echo "HATA: IDA_INSTALL_GPU=true fakat çalışan NVIDIA sürücüsü yok." >&2
    exit 6
  fi
  # RTX 4050 sunucuda doğrulanan r580 sürücü CUDA 13.0 ile uyumludur.
  # cuda-toolkit metapackage'i yerine minor sürüm kilitlenir; driver paketi
  # kurulmaz/değiştirilmez.
  CUDA_KEYRING=/tmp/cuda-keyring_1.1-1_all.deb
  if [[ ! -f /usr/share/keyrings/cuda-archive-keyring.gpg ]]; then
    curl -fsSL \
      https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
      -o "$CUDA_KEYRING"
    sudo dpkg -i "$CUDA_KEYRING"
  fi
  sudo apt-get update
  sudo apt-get install -y cuda-toolkit-13-0

  # PyTorch wheel kendi CUDA runtime'ını taşır; cu130 seçimi r580 ile
  # aynı major hatta kalır. Ultralytics checkpoint metadata'sı 8.3.183/184.
  python3 -m pip install --user --upgrade \
    torch==2.12.1 torchvision==0.27.1 \
    --index-url https://download.pytorch.org/whl/cu130
  python3 -m pip install --user --upgrade \
    ultralytics==8.3.184 opencv-python-headless "numpy<3"
  python3 -m pip install --user --upgrade tensorrt-cu13
fi

set +u
source /opt/ros/humble/setup.bash
source "$SIM_HOME/.profile" 2>/dev/null || true
set -u

pushd "$ARDUPILOT_DIR" >/dev/null
./waf configure --board sitl
./waf rover
popd >/dev/null

pushd "$SIM_WS" >/dev/null
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys="mavsdk pymavlink python3-pymavlink python3-mavsdk"
colcon build --symlink-install --event-handlers console_direct+
popd >/dev/null

cat >"$SIM_WS/sim_env.sh" <<EOF
#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$SIM_WS/install/setup.bash"
export PATH="$ARDUPILOT_DIR/Tools/autotest:\$PATH"
export IDA_SIM_WS="$SIM_WS"
export ARDUPILOT_DIR="$ARDUPILOT_DIR"
export PATH="/usr/local/cuda-13.0/bin:\$PATH"
EOF
chmod 0755 "$SIM_WS/sim_env.sh"

{
  echo "created_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "ubuntu=$(lsb_release -ds)"
  echo "ros=humble"
  echo "gazebo=$(gazebo --version 2>/dev/null | head -1)"
  echo "ardupilot_ref=$ARDUPILOT_REF"
  echo "ardupilot_commit=$(git -C "$ARDUPILOT_DIR" rev-parse HEAD)"
  echo "python=$(python3 --version 2>&1)"
  if command -v nvidia-smi >/dev/null; then
    echo "nvidia_driver=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)"
    echo "gpu=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)"
  fi
  if [[ -x /usr/local/cuda-13.0/bin/nvcc ]]; then
    echo "cuda_toolkit=$(/usr/local/cuda-13.0/bin/nvcc --version | tail -1)"
  fi
  python3 -m pip freeze --user | sed 's/^/pip=/'
} >"$SIM_WS/sim_environment.lock.txt"

echo "KURULUM TAMAM: $SIM_WS"
echo "Sonraki adım: $SIM_WS/scripts/verify_sim_server.sh"
