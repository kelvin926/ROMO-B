#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "$(. /etc/os-release && printf '%s' "$VERSION_ID")" != "22.04" ]]; then
  printf 'ERROR: Ubuntu 22.04 is required.\n' >&2
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'ERROR: ROS 2 Humble is not installed at /opt/ros/humble.\n' >&2
  exit 1
fi

apt_packages=(
  build-essential cmake ccache curl git gh libapr1-dev libaprutil1-dev libasio-dev
  libboost-system-dev libpcl-dev libyaml-cpp-dev
  nlohmann-json3-dev
  python3-colcon-common-extensions python3-pip python3-rosdep python3-setuptools
  python3-serial python3-vcstool python3-venv python3-yaml
)

ros_packages=(
  ros-humble-diagnostic-updater
  ros-humble-joint-state-publisher
  ros-humble-nav2-bringup
  ros-humble-nav2-collision-monitor
  ros-humble-nav2-mppi-controller
  ros-humble-nav2-velocity-smoother
  ros-humble-navigation2
  ros-humble-pcl-ros
  ros-humble-pointcloud-to-laserscan
  ros-humble-robot-localization
  ros-humble-rmw-cyclonedds-cpp
  ros-humble-ros-testing
  ros-humble-sophus
  ros-humble-teleop-twist-keyboard
  ros-humble-twist-mux
  ros-humble-xacro
)

printf 'Installing Ubuntu and ROS dependencies (sudo required)...\n'
sudo apt-get update
sudo apt-get install -y --no-install-recommends "${apt_packages[@]}" "${ros_packages[@]}"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

if ! id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
  sudo usermod -aG dialout "$USER"
  printf 'Added %s to dialout. A full logout/login or reboot is required.\n' "$USER"
fi

mkdir -p "$repo_root/data/local" "$repo_root/config/local"

printf 'Host packages are installed.\n'
if [[ -f /var/run/reboot-required ]]; then
  printf 'REBOOT REQUIRED: reboot before NVIDIA or hardware validation.\n'
fi
printf 'Next: %s/scripts/fetch_dependencies.sh\n' "$repo_root"
