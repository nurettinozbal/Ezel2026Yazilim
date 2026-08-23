#!/usr/bin/env bash
# =============================================================================
# sim_kurulum.sh — TEKNOFEST İDA simülasyon ortamı kurulumu (Ubuntu 22.04, tamamen unattended)
#
# Kurulan bileşenler:
#   1. Sistem güncellemesi (çekirdek upgrade'e toleranslı)
#   2. Base paketler (build-essential, cmake, pymavlink build bağımlılıkları, GStreamer)
#   3. ROS2 Humble (ros-humble-desktop + Gazebo ROS paketleri + colcon + rosdep)
#   4. Gazebo Classic 11 (Ubuntu 22.04'te 'gazebo' paketi = Classic 11.x)
#   5. Python bağımlılıkları (mavsdk, pymavlink, pyserial, future — PEP 668 için --break-system-packages)
#   6. ArduPilot (git clone + prereqs + Rover SITL build — uzun sürer)
#   7. İDA workspace (~/ida_ws/src)
#   8. ~/.bashrc düzenlemeleri (idempotent)
#
# Kullanım: önce 'sudo -v' çalıştır (script -n ile sudo kullanır), sonra:
#   bash ~/sim_kurulum.sh 2>&1 | tee -a ~/sim_kurulum.log
#
# DİKKAT: Bu dosya Windows'ta yazıldığı için LF satır sonuyla kaydedilmiştir.
# CRLF'e dönüşürse bash "command not found" hatası verir — scp ile göndermeden
# önce 'file sim_kurulum.sh' ile kontrol edin (ASCII text olmalı).
# =============================================================================

set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

# Tüm çıktıyı hem ekrana hem log dosyasına yaz (en başta, tek noktadan).
exec > >(tee -a "$HOME/sim_kurulum.log") 2>&1

echo "=== IDA simülasyon kurulumu başladı: $(date -u '+%Y-%m-%d %H:%M:%S UTC') ==="
echo "Log: $HOME/sim_kurulum.log"

# --- sudo -n kontrolü: şifre önbelleği yoksa devam etme ----------------------
if ! sudo -n true 2>/dev/null; then
    echo "HATA: önce 'sudo -v' çalıştır"
    exit 1
fi

# =============================================================================
# [1/8] Sistem güncellemesi
# =============================================================================
echo "== [1/8] apt update + upgrade =="
sudo -n apt update -y
# Çekirdek upgrade SSH'ı kesebilir (Proxmox VM) — başarısızlık durumunda durma.
sudo -n apt -y upgrade || true

# =============================================================================
# [2/8] Base paketler
# =============================================================================
echo "== [2/8] Base paketler =="
sudo -n apt -y install git curl wget ca-certificates gnupg lsb-release python3-pip python3-venv build-essential cmake pkg-config python3-numpy python3-matplotlib libxml2-dev libxslt1-dev liblzma-dev python3-lxml gstreamer1.0-plugins-base gstreamer1.0-plugins-good

# =============================================================================
# [3/8] ROS2 Humble
# =============================================================================
echo "== [3/8] ROS2 Humble kurulumu =="
sudo -n apt -y install software-properties-common
sudo -n add-apt-repository universe -y
sudo -n locale-gen en_US en_US.UTF-8
sudo -n update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

sudo -n curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo -n tee /etc/apt/sources.list.d/ros2.list
sudo -n apt update -y
sudo -n apt -y install ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros python3-colcon-common-extensions python3-rosdep

# =============================================================================
# [4/8] Gazebo Classic 11
# =============================================================================
echo "== [4/8] Gazebo Classic 11 =="
sudo -n apt -y install gazebo libgazebo11-dev gazebo-common

# =============================================================================
# [5/8] Python bağımlılıkları (MAVSDK + pymavlink)
# =============================================================================
echo "== [5/8] Python bağımlılıkları =="
# Ubuntu 22.04 Python 3.10'da PEP 668 (externally-managed) nedeniyle
# --break-system-packages gerekir. pymavlink 'future' modülüne bağımlıdır.
python3 -m pip install --break-system-packages mavsdk pymavlink pyserial
python3 -m pip install --break-system-packages future

# =============================================================================
# [6/8] ArduPilot + Rover SITL
# =============================================================================
echo "== [6/8] ArduPilot kurulumu + Rover SITL build =="
if [ ! -d "$HOME/ardupilot" ]; then
    git clone --recurse-submodules --depth 1 https://github.com/ArduPilot/ardupilot.git "$HOME/ardupilot"
fi
cd "$HOME/ardupilot"
git submodule update --init --recursive
if [ -f Tools/environment_install/install-prereqs-ubuntu.sh ]; then
    # NOT: ArduPilot'ın install-prereqs script'i '-y' flag'ini DESTEKLEMEZ
    # (argüman parsing'i yok); script apt çağrılarında zaten '-y' +
    # DEBIAN_FRONTEND=noninteractive kullanır. Bu yüzden flagsiz çağrılır.
    # 'yes |' pipe'ı gerekmeden de çalışır (script'te interaktif onay yok).
    Tools/environment_install/install-prereqs-ubuntu.sh
else
    echo "UYARI: install-prereqs-ubuntu.sh bulunamadı — prereq kurulumu atlanıyor"
fi
# Rover SITL binary'si (uzun sürer — waf derlemesi)
./waf configure --board sitl
./waf rover
cd "$HOME"

# =============================================================================
# [7/8] İDA workspace
# =============================================================================
echo "== [7/8] İDA workspace =="
mkdir -p "$HOME/ida_ws/src" "$HOME/ida_ws/scripts"
[ -f "$HOME/ida_ws/src/.gitkeep" ] || touch "$HOME/ida_ws/src/.gitkeep"

# =============================================================================
# [8/8] ~/.bashrc düzenlemeleri (idempotent)
# =============================================================================
echo "== [8/8] .bashrc düzenlemeleri =="
BASHRC="$HOME/.bashrc"

if ! grep -q "source /opt/ros/humble/setup.bash" "$BASHRC" 2>/dev/null; then
    echo "source /opt/ros/humble/setup.bash" >> "$BASHRC"
fi

if ! grep -q "source ~/ida_ws/install/setup.bash" "$BASHRC" 2>/dev/null; then
    echo "[ -f ~/ida_ws/install/setup.bash ] && source ~/ida_ws/install/setup.bash" >> "$BASHRC"
fi

if ! grep -q "export IDA_WS=" "$BASHRC" 2>/dev/null; then
    echo 'export IDA_WS="$HOME/ida_ws"' >> "$BASHRC"
fi

if ! grep -q "PYTHONPATH.*ida_ws" "$BASHRC" 2>/dev/null; then
    echo 'export PYTHONPATH="$IDA_WS/src:$IDA_WS/install:$PYTHONPATH"' >> "$BASHRC"
fi

# =============================================================================
# Doğrulama blokları (başarısızlık kurulumu durdurmaz — hepsi || true)
# =============================================================================
echo "== Doğrulama =="
source /opt/ros/humble/setup.bash || true
ros2 --version || true
gazebo --version | head -1 || true
python3 -c "import mavsdk, pymavlink, serial; print('python deps OK')" || true
"$HOME/ardupilot/build/sitl/bin/ardurover" --help >/dev/null 2>&1 && echo 'ardurover build OK' || true

echo ""
echo "KURULUM TAMAM"
echo "Log dosyası: $HOME/sim_kurulum.log"
echo "Sonraki adım: scp -r src/* ezel@<sunucu>:~/ida_ws/src/ + colcon build (README_AUTONOMY_ROS2.md)"
