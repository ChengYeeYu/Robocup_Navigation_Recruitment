#!/usr/bin/env bash
# One-shot provisioning script for a bare Ubuntu 22.04 (Jammy) machine.
# Installs ROS 2 Humble, Gazebo Classic 11, Nav2, and TurtleBot3 packages.
#
# Usage:
#   sudo bash src/tools/install_dependencies.sh                 # install only, print bashrc lines
#   sudo bash src/tools/install_dependencies.sh --yes-append-bashrc   # also append them for you
#
# Safe to re-run (idempotent): re-running just re-confirms everything is present.
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "This script installs system packages and must be run with sudo:" >&2
  echo "  sudo bash src/tools/install_dependencies.sh" >&2
  exit 1
fi

APPEND_BASHRC=0
if [[ "${1:-}" == "--yes-append-bashrc" ]]; then
  APPEND_BASHRC=1
fi

if ! grep -q "Ubuntu 22.04" /etc/os-release 2>/dev/null; then
  echo "WARNING: this script targets Ubuntu 22.04 (Jammy) / ROS 2 Humble." >&2
  echo "Your /etc/os-release does not report Ubuntu 22.04 — continuing anyway, but things may not work." >&2
fi

REAL_USER="${SUDO_USER:-$(whoami)}"
REAL_HOME=$(getent passwd "${REAL_USER}" | cut -d: -f6)

echo "==> [1/6] Locale setup"
apt update -y
apt install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "==> [2/6] Enabling Universe repo + adding the ROS 2 apt repository"
apt install -y software-properties-common curl gnupg lsb-release
add-apt-repository -y universe

install -m 0755 -d /usr/share/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  > /etc/apt/sources.list.d/ros2.list

echo "==> [3/6] Installing ROS 2 Humble, Gazebo Classic, Nav2, TurtleBot3"
apt update -y
apt install -y \
  ros-humble-desktop \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-navigation2 \
  ros-humble-turtlebot3-bringup \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros \
  ros-humble-slam-toolbox \
  gazebo \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-numpy \
  python3-yaml \
  build-essential \
  cmake

echo "==> [4/6] rosdep init/update"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
sudo -u "${REAL_USER}" rosdep update

echo "==> [5/6] Shell setup"
BASHRC_LINES=$(cat <<'EOF'
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
EOF
)

if [[ "${APPEND_BASHRC}" -eq 1 ]]; then
  BASHRC_FILE="${REAL_HOME}/.bashrc"
  if ! grep -q "source /opt/ros/humble/setup.bash" "${BASHRC_FILE}" 2>/dev/null; then
    {
      echo ""
      echo "# --- added by robocup_nav_rec/src/tools/install_dependencies.sh ---"
      echo "${BASHRC_LINES}"
    } >> "${BASHRC_FILE}"
    chown "${REAL_USER}" "${BASHRC_FILE}"
    echo "Appended ROS 2 setup lines to ${BASHRC_FILE}"
  else
    echo "${BASHRC_FILE} already sources ROS 2 Humble, skipping append."
  fi
else
  echo "Add these two lines to your ~/.bashrc (or re-run with --yes-append-bashrc to do it automatically):"
  echo "-------------------------------------------------------------"
  echo "${BASHRC_LINES}"
  echo "-------------------------------------------------------------"
fi

echo "==> [6/6] Done."
echo "Open a new terminal (or 'source ~/.bashrc') then verify with:"
echo "  ros2 --version"
echo "  gazebo --version"
