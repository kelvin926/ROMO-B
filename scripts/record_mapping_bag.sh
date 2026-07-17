#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
stamp="$(date +%Y%m%d-%H%M%S)"
output="${1:-$repo_root/data/local/bags/mapping-$stamp}"
mkdir -p "$(dirname "$output")"

# shellcheck disable=SC1091
source "$repo_root/scripts/source_env.sh"

printf 'Recording to %s. Drive only in RC Manual mode at walking speed.\n' "$output"
ros2 bag record --storage sqlite3 --output "$output" \
  /sensing/lidar/top/pointcloud_raw \
  /sensing/imu/livox_raw \
  /sensing/imu/imu_raw \
  /wheel/odometry_raw \
  /joint_states \
  /tf /tf_static \
  /romo_b/platform_status \
  /diagnostics
