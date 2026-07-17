#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
project_only=false
if [[ "${1:-}" == "--project-only" ]]; then
  project_only=true
elif [[ -n "${1:-}" ]]; then
  printf 'Usage: %s [--project-only]\n' "$0" >&2
  exit 2
fi

# ROS/ament setup files read a few optional variables without `${name:-}`.
# Temporarily disable nounset while sourcing them, then restore our strict mode.
source_ros_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

source_ros_setup /opt/ros/humble/setup.bash
if [[ -f "$repo_root/autoware/install/setup.bash" && \
      -f "$repo_root/autoware/install/.romo_b_build_complete" ]]; then
  source_ros_setup "$repo_root/autoware/install/setup.bash"
fi
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2

if [[ "$project_only" == false ]]; then
  "$repo_root/scripts/fetch_dependencies.sh"

  sdk_source="$repo_root/vendor_ws/sdk/Livox-SDK2"
  sdk_build="$repo_root/vendor_ws/sdk/build"
  sdk_install="$repo_root/vendor_ws/sdk/install"
  cmake -S "$sdk_source" -B "$sdk_build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$sdk_install"
  cmake --build "$sdk_build" --parallel 2
  cmake --install "$sdk_build"

  rosdep install --from-paths "$repo_root/vendor_ws/src" --ignore-src -r -y
  colcon --log-base "$repo_root/vendor_ws/log" build \
    --base-paths "$repo_root/vendor_ws/src" \
    --build-base "$repo_root/vendor_ws/build" \
    --install-base "$repo_root/vendor_ws/install" \
    --symlink-install --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
      -DLIVOX_LIDAR_SDK_LIBRARY="$sdk_install/lib/liblivox_lidar_sdk_shared.so" \
      -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$sdk_install/include"
  source_ros_setup "$repo_root/vendor_ws/install/setup.bash"
fi

rosdep install --from-paths "$repo_root/robot_ws/src" --ignore-src -r -y \
  --skip-keys "ament_python livox_ros_driver2 lidar_localization_ros2 lidarslam \
autoware_adapi_v1_msgs autoware_control_msgs autoware_perception_msgs \
autoware_planning_msgs autoware_vehicle_msgs \
autoware_euclidean_cluster_object_detector autoware_localization_msgs \
unique_identifier_msgs"
colcon --log-base "$repo_root/robot_ws/log" build \
  --base-paths "$repo_root/robot_ws/src" \
  --build-base "$repo_root/robot_ws/build" \
  --install-base "$repo_root/robot_ws/install" \
  --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

printf 'Build complete. Source %s/robot_ws/install/setup.bash\n' "$repo_root"
