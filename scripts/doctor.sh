#!/usr/bin/env bash
set -uo pipefail

mode="${1:---preflight}"
if [[ "$mode" != "--preflight" && "$mode" != "--hardware" ]]; then
  printf 'Usage: %s [--preflight|--hardware]\n' "$0" >&2
  exit 2
fi

failures=0
waiting=0

pass() { printf 'PASS              %s\n' "$*"; }
warn() { printf 'WARN              %s\n' "$*"; }
wait_for() { printf 'WAITING_HARDWARE  %s\n' "$*"; waiting=$((waiting + 1)); }
fail() { printf 'FAIL              %s\n' "$*"; failures=$((failures + 1)); }

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

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
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

if [[ "$failures" -gt 0 ]]; then
  printf 'RESULT: FAIL (%d checks failed)\n' "$failures"
  exit 1
fi
if [[ "$waiting" -gt 0 ]]; then
  printf 'RESULT: WAITING_HARDWARE (%d items)\n' "$waiting"
else
  printf 'RESULT: READY\n'
fi
