#!/usr/bin/env bash
set -uo pipefail

mode="${1:---preflight}"
if [[ "$mode" != "--preflight" && "$mode" != "--hardware" && "$mode" != "--autoware" ]]; then
  printf 'Usage: %s [--preflight|--hardware|--autoware]\n' "$0" >&2
  exit 2
fi

failures=0
waiting=0

pass() { printf 'PASS              %s\n' "$*"; }
warn() { printf 'WARN              %s\n' "$*"; }
wait_for() { printf 'WAITING_HARDWARE  %s\n' "$*"; waiting=$((waiting + 1)); }
fail() { printf 'FAIL              %s\n' "$*"; failures=$((failures + 1)); }
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

version_id="$(. /etc/os-release && printf '%s' "$VERSION_ID")"
[[ "$version_id" == "22.04" ]] && pass 'Ubuntu 22.04' || fail "Ubuntu $version_id (22.04 required)"
[[ -f /opt/ros/humble/setup.bash ]] && pass 'ROS 2 Humble installation' || fail 'ROS 2 Humble missing'

if [[ -f /var/run/reboot-required ]]; then
  warn 'system reboot required'
else
  pass 'no pending system reboot'
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  if nvidia-smi >/dev/null 2>&1; then pass 'NVIDIA driver/userspace match'; else warn 'nvidia-smi failed'; fi
fi

required_commands=(colcon git gh python3 rosdep)
for command_name in "${required_commands[@]}"; do
  command -v "$command_name" >/dev/null 2>&1 && pass "$command_name available" || fail "$command_name missing"
done
command -v vcs >/dev/null 2>&1 && pass 'vcstool available' || warn 'vcstool missing; run setup_host.sh'
python3 -c 'import serial' >/dev/null 2>&1 && pass 'pyserial available' || warn 'pyserial missing; run setup_host.sh'

if [[ "$mode" != "--autoware" ]]; then
  if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    pass "$USER belongs to dialout"
  else
    [[ "$mode" == "--hardware" ]] && fail "$USER is not in dialout" || warn "$USER is not in dialout"
  fi

  serial_links=(/dev/serial/by-id/*)
  if [[ -e "${serial_links[0]}" ]]; then
    pass "serial adapter detected: ${serial_links[0]}"
    serial_target="$(readlink -f "${serial_links[0]}")"
    if [[ -r "$serial_target" && -w "$serial_target" ]]; then
      pass "serial port accessible: $serial_target"
    else
      [[ "$mode" == "--hardware" ]] && fail "serial port not accessible: $serial_target" || wait_for 'serial permission'
    fi
  else
    [[ "$mode" == "--hardware" ]] && fail 'USB-RS232 adapter not detected' || wait_for 'USB-RS232 adapter'
  fi

  wired_if="$(find /sys/class/net -mindepth 1 -maxdepth 1 -printf '%f\n' 2>/dev/null | grep -Ev '^(lo|wl)' | head -n 1)"
  if [[ -n "$wired_if" ]]; then
    pass "wired interface detected: $wired_if"
  else
    [[ "$mode" == "--hardware" ]] && fail 'no wired/USB Ethernet interface for Mid-360' || wait_for 'USB Ethernet adapter'
  fi

  hardware_yaml="$repo_root/config/local/hardware.yaml"
  if [[ -f "$hardware_yaml" ]]; then
    pass 'local hardware configuration exists'
    if python3 - "$hardware_yaml" <<'PY'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1])) or {}
lidar = d.get("lidar", {})
raise SystemExit(0 if lidar.get("calibrated") is True and lidar.get("lidar_ip") not in (None, "pending") else 1)
PY
    then
      pass 'LiDAR IP and measured transform approved'
    else
      [[ "$mode" == "--hardware" ]] && fail 'LiDAR IP/transform not approved' || wait_for 'LiDAR IP and measured transform'
    fi
  else
    [[ "$mode" == "--hardware" ]] && fail 'config/local/hardware.yaml missing' || wait_for 'local hardware configuration'
  fi
fi

if [[ "$mode" == "--autoware" ]]; then
  autoware_root="$repo_root/autoware"
  expected_commit=5b27e88e84683deb4afaf0aa917f80082a608871
  if [[ -d "$autoware_root/.git" ]] && \
     [[ "$(git -C "$autoware_root" rev-parse HEAD 2>/dev/null)" == "$expected_commit" ]]; then
    pass 'Autoware meta repository is pinned to 1.8.0'
  else
    fail 'Autoware 1.8.0 checkout missing or at the wrong revision'
  fi
  if [[ -f "$autoware_root/install/setup.bash" && \
        -f "$autoware_root/install/.romo_b_build_complete" ]]; then
    pass 'Autoware install overlay is complete'
  else
    fail 'Autoware install overlay incomplete; run setup_autoware.sh'
  fi
  acados_root="${ACADOS_SOURCE_DIR:-$autoware_root/deps/acados}"
  expected_acados_commit=7e1d1152c1babd6ea04af1c9d73444fe8381057b
  if [[ -f "$acados_root/.romo_b_ready" && -f "$acados_root/cmake/acadosConfig.cmake" && \
        -x "$acados_root/.venv/bin/python3" && \
        "$(git -C "$acados_root" rev-parse HEAD 2>/dev/null)" == "$expected_acados_commit" ]]; then
    pass "acados v0.5.3 local dependency exists: $acados_root"
  else
    fail 'acados v0.5.3 is incomplete; run scripts/setup_acados.sh'
  fi

  planning_target="$autoware_root/src/launcher/autoware_launch/autoware_launch/config/planning/preset/romo_b_preset.yaml"
  control_target="$autoware_root/src/launcher/autoware_launch/autoware_launch/config/control/preset/romo_b_preset.yaml"
  if cmp -s "$repo_root/config/autoware/presets/romo_b_planning_preset.yaml" "$planning_target" && \
     cmp -s "$repo_root/config/autoware/presets/romo_b_control_preset.yaml" "$control_target"; then
    pass 'ROMO-B Autoware presets applied'
  else
    fail 'Autoware overrides are stale; run apply_autoware_overrides.sh and rebuild'
  fi
  if python3 "$repo_root/scripts/customize_autoware_rviz.py" \
       --autoware-root "$autoware_root" --check; then
    pass 'ROMO-B LiDAR-first Autoware RViz layout applied'
  else
    fail 'Autoware RViz layout is stale; run apply_autoware_overrides.sh'
  fi
  static_target="$autoware_root/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml"
  if python3 - "$static_target" <<'PY'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))["/**"]["ros__parameters"]["avoidance"]
valid = (
    d["target_filtering"]["target_type"]["unknown"] is True
    and d["target_filtering"]["unstable_classification_time"] == 0.0
    and d["safety_check"]["target_type"]["unknown"] is True
    and d["avoidance"]["longitudinal"]["nominal_avoidance_speed"] <= 0.20
)
raise SystemExit(0 if valid else 1)
PY
  then
    pass 'low-speed static UNKNOWN avoidance override applied'
  else
    fail 'static avoidance override missing or stale'
  fi

  velocity_target="$autoware_root/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/common/autoware_velocity_smoother/velocity_smoother.param.yaml"
  speed_guard="$repo_root/robot_ws/src/romo_b_autoware/config/speed_limit_guard.yaml"
  if python3 - "$velocity_target" "$speed_guard" <<'PY'
import sys, yaml
velocity = yaml.safe_load(open(sys.argv[1]))["/**"]["ros__parameters"]
guard = yaml.safe_load(open(sys.argv[2]))["romo_b_autoware_speed_limit_guard"]["ros__parameters"]
valid = (
    velocity["max_vel"] <= 0.20
    and velocity["min_curve_velocity"] <= 0.05
    and velocity["replan_vel_deviation"] <= 0.10
    and velocity["engage_velocity"] <= 0.10
    and velocity["stopping_velocity"] <= 0.10
    and guard["max_velocity_mps"] == 0.14
)
raise SystemExit(0 if valid else 1)
PY
  then
    pass 'ROMO-B velocity smoother and independent 0.14 m/s planning guard applied'
  else
    fail 'velocity smoother override or planning speed guard is missing/stale'
  fi

  if (
    set +u
    source "$repo_root/scripts/source_env.sh"
    set -u
    for package in autoware_launch autoware_map_projection_loader \
      autoware_map_loader autoware_map_msgs \
      autoware_euclidean_cluster_object_detector \
      autoware_internal_planning_msgs romo_b_autoware romo_b_description \
      romo_b_launch; do
      ros2 pkg prefix "$package" >/dev/null || exit 1
    done
  ); then
    pass 'required Autoware and ROMO-B packages are discoverable'
  else
    fail 'one or more required Autoware packages are not discoverable'
  fi

  map_path="${AUTOWARE_MAP:-$repo_root/data/local/maps/mapping-20260717-195653/autoware}"
  map_missing=0
  for filename in lanelet2_map.osm pointcloud_map.pcd map_projector_info.yaml; do
    [[ -e "$map_path/$filename" ]] || map_missing=$((map_missing + 1))
  done
  if [[ "$map_missing" -eq 0 ]]; then
    pass "Autoware map bundle exists: $map_path"
  else
    fail "Autoware map bundle incomplete: $map_path"
  fi
fi

if [[ "$failures" -gt 0 ]]; then
  printf 'RESULT: FAIL (%d checks failed)\n' "$failures"
  exit 1
fi
if [[ "$waiting" -gt 0 ]]; then
  printf 'RESULT: WAITING_HARDWARE (%d items)\n' "$waiting"
else
  printf 'RESULT: READY\n'
fi
