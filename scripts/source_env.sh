#!/usr/bin/env bash

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# This file may be sourced by a strict-mode shell. ROS/ament setup scripts
# access optional variables that are legitimately unset, so preserve and
# temporarily relax the caller's nounset setting.
romo_b_nounset_was_enabled=false
if [[ $- == *u* ]]; then
  romo_b_nounset_was_enabled=true
  set +u
fi

source /opt/ros/humble/setup.bash
acados_root="${ACADOS_SOURCE_DIR:-$repo_root/autoware/deps/acados}"
if [[ -f "$acados_root/.romo_b_ready" ]]; then
  export ACADOS_SOURCE_DIR="$acados_root"
  export CMAKE_PREFIX_PATH="$acados_root:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$acados_root/lib:${LD_LIBRARY_PATH:-}"
  export PATH="$acados_root/bin:${PATH:-}"
fi
if [[ -f "$repo_root/autoware/install/setup.bash" && \
      -f "$repo_root/autoware/install/.romo_b_build_complete" ]]; then
  source "$repo_root/autoware/install/setup.bash"
fi
if [[ -f "$repo_root/vendor_ws/install/setup.bash" ]]; then
  source "$repo_root/vendor_ws/install/setup.bash"
fi
if [[ -f "$repo_root/robot_ws/install/setup.bash" ]]; then
  source "$repo_root/robot_ws/install/setup.bash"
fi
export ROMO_B_ROOT="$repo_root"

if [[ "$romo_b_nounset_was_enabled" == true ]]; then
  set -u
fi
unset romo_b_nounset_was_enabled
unset acados_root
