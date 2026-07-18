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

# Autoware 1.8.0 creates package index entries for a small set of optional
# CUDA/TensorRT packages even when their CMake files intentionally return early
# on a CPU-only host.  Colcon then prints a misleading missing-local-setup line
# for each entry.  Hide only those exact known messages; every other setup
# diagnostic remains visible.  Set ROMO_B_SHOW_OPTIONAL_AUTOWARE_WARNINGS=1 to
# display them while diagnosing a GPU installation.
_romo_b_filter_optional_autoware_stderr() {
  while IFS= read -r line; do
    if [[ "${ROMO_B_SHOW_OPTIONAL_AUTOWARE_WARNINGS:-0}" != 1 ]]; then
      case "$line" in
        "not found: \"$repo_root/autoware/install/autoware_cuda_utils/share/autoware_cuda_utils/local_setup.bash\"" | \
        "not found: \"$repo_root/autoware/install/autoware_tensorrt_common/share/autoware_tensorrt_common/local_setup.bash\"" | \
        "not found: \"$repo_root/autoware/install/autoware_tensorrt_classifier/share/autoware_tensorrt_classifier/local_setup.bash\"" | \
        "not found: \"$repo_root/autoware/install/autoware_tensorrt_plugins/share/autoware_tensorrt_plugins/local_setup.bash\"" | \
        "not found: \"$repo_root/autoware/install/bevdet_vendor/share/bevdet_vendor/local_setup.bash\"" | \
        "not found: \"$repo_root/autoware/install/cuda_blackboard/share/cuda_blackboard/local_setup.bash\"" | \
        "not found: \"$repo_root/autoware/install/autoware_cuda_pointcloud_preprocessor/share/autoware_cuda_pointcloud_preprocessor/local_setup.bash\"" | \
        "not found: \"$repo_root/autoware/install/autoware_ground_segmentation_cuda/share/autoware_ground_segmentation_cuda/local_setup.bash\"")
          continue
          ;;
      esac
    fi
    printf '%s\n' "$line" >&2
  done
}
acados_root="${ACADOS_SOURCE_DIR:-$repo_root/autoware/deps/acados}"
if [[ -f "$acados_root/.romo_b_ready" ]]; then
  export ACADOS_SOURCE_DIR="$acados_root"
  export CMAKE_PREFIX_PATH="$acados_root:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$acados_root/lib:${LD_LIBRARY_PATH:-}"
  export PATH="$acados_root/bin:${PATH:-}"
fi
if [[ -f "$repo_root/autoware/install/local_setup.bash" && \
      -f "$repo_root/autoware/install/.romo_b_build_complete" ]]; then
  source "$repo_root/autoware/install/local_setup.bash" \
    2> >(_romo_b_filter_optional_autoware_stderr)
fi
if [[ -f "$repo_root/vendor_ws/install/local_setup.bash" ]]; then
  source "$repo_root/vendor_ws/install/local_setup.bash"
fi
if [[ -f "$repo_root/robot_ws/install/local_setup.bash" ]]; then
  source "$repo_root/robot_ws/install/local_setup.bash"
fi
export ROMO_B_ROOT="$repo_root"

# Full Autoware field logs showed Fast DDS shared-memory lock failures and
# participant crashes. Prefer Autoware's supported Cyclone DDS RMW when it is
# installed. Existing hosts fall back to Fast DDS with shared memory disabled
# until setup_host.sh installs Cyclone DDS.
if [[ -n "${ROMO_B_RMW_IMPLEMENTATION:-}" ]]; then
  export RMW_IMPLEMENTATION="$ROMO_B_RMW_IMPLEMENTATION"
elif [[ -f /opt/ros/humble/share/rmw_cyclonedds_cpp/package.xml ]]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
else
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
fi
if [[ "$RMW_IMPLEMENTATION" == rmw_fastrtps_cpp ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$repo_root/config/dds/fastdds_udp.xml"
else
  unset FASTRTPS_DEFAULT_PROFILES_FILE
fi

if [[ "$romo_b_nounset_was_enabled" == true ]]; then
  set -u
fi
unset romo_b_nounset_was_enabled
unset acados_root
unset -f _romo_b_filter_optional_autoware_stderr
