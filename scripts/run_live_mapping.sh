#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
stamp="$(date +%Y%m%d-%H%M%S)"
run_name="mapping-$stamp"
map_dir="${1:-$repo_root/data/local/maps/$run_name}"
bag_dir="${2:-$repo_root/data/local/bags/$run_name}"

mkdir -p "$map_dir" "$(dirname "$bag_dir")"

# shellcheck disable=SC1091
source "$repo_root/scripts/source_env.sh"

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
