#!/usr/bin/env bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$repo_root/vendor_ws/install/setup.bash" ]]; then
  source "$repo_root/vendor_ws/install/setup.bash"
fi
if [[ -f "$repo_root/robot_ws/install/setup.bash" ]]; then
  source "$repo_root/robot_ws/install/setup.bash"
fi
export ROMO_B_ROOT="$repo_root"
