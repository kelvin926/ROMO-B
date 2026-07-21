#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
stamp="$(date +%Y%m%d-%H%M%S)"
run_name="mapping-$stamp"
map_dir="${1:-$repo_root/data/local/maps/$run_name}"
bag_dir="${2:-$repo_root/data/local/bags/$run_name}"

# shellcheck disable=SC1091
source "$repo_root/scripts/source_env.sh"

# Livox SDK permits multiple processes to bind the same UDP ports, but only one
# receives a coherent stream. Refuse a second mapping/hardware stack even when
# it belongs to another ROS_DOMAIN_ID, which `ros2 node list` cannot see.
existing_processes="$(
  pgrep -af \
    'livox_ros_driver2_node|rko_lio/(online_node|offline_node)|graph_based_slam_node|romo_b_serial_bridge|mapping_live\.launch\.py' \
    || true
)"
if [[ -n "$existing_processes" ]]; then
  printf 'Another ROMO-B/Livox process is already running:\n%s\n' "$existing_processes" >&2
  printf 'Stop its launch terminal with Ctrl+C before starting live mapping.\n' >&2
  exit 2
fi

if [[ ! -r /dev/romo_b_pcu || ! -w /dev/romo_b_pcu ]]; then
  printf '/dev/romo_b_pcu is missing or inaccessible. Reconnect USB-RS232 and check dialout membership.\n' >&2
  exit 2
fi
if command -v nmcli >/dev/null 2>&1 && \
   nmcli -t -f NAME connection show | grep -qx 'romo-b-livox'; then
  if ! nmcli connection up romo-b-livox >/dev/null; then
    printf 'Could not activate the romo-b-livox Ethernet profile. Reconnect USB Ethernet.\n' >&2
    exit 2
  fi
fi
if ! ping -c 1 -W 1 192.168.1.113 >/dev/null; then
  printf 'Mid-360 at 192.168.1.113 is unreachable. Check power and USB Ethernet.\n' >&2
  exit 2
fi

mkdir -p "$map_dir" "$(dirname "$bag_dir")"

printf 'LIVE MAP DIRECTORY: %s\n' "$map_dir"
printf 'RAW BAG DIRECTORY:  %s\n' "$bag_dir"
printf 'Keep the robot completely still until RKO-LIO odometry appears, then drive in RC Manual.\n'
printf 'Save before Ctrl+C with: %s/scripts/save_live_mapping.sh %s\n' "$repo_root" "$map_dir"

exec ros2 launch romo_b_bringup mapping_live.launch.py \
  hardware_config:="$repo_root/config/local/hardware.yaml" \
  livox_config:="$repo_root/config/local/MID360_config.json" \
  rko_param_file:="$repo_root/config/local/rko_lio_mid360.yaml" \
  save_dir:="$map_dir" \
  bag_path:="$bag_dir" \
  record_bag:=true \
  use_rviz:=true
