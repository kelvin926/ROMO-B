#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
map_run="${MAP_RUN:-$repo_root/data/local/maps/mapping-20260717-195653}"
map_path="${AUTOWARE_MAP:-$map_run/autoware}"
pcd_map="${PCD_MAP:-$map_run/map.pcd}"
hardware_config="${HARDWARE_CONFIG:-$repo_root/config/local/hardware.yaml}"
livox_config="${LIVOX_CONFIG:-$repo_root/config/local/MID360_config.json}"
receive_only="${RECEIVE_ONLY:-true}"

if [[ "$receive_only" != true && "$receive_only" != false ]]; then
  printf 'RECEIVE_ONLY must be true or false.\n' >&2
  exit 2
fi
if [[ ! -f "$repo_root/autoware/install/setup.bash" || \
      ! -f "$repo_root/autoware/install/.romo_b_build_complete" ]]; then
  printf 'Autoware is not built. Run ./scripts/setup_autoware.sh first.\n' >&2
  exit 2
fi
for required in \
  "$hardware_config" "$livox_config" "$pcd_map" \
  "$map_path/lanelet2_map.osm" "$map_path/pointcloud_map.pcd" \
  "$map_path/map_projector_info.yaml"; do
  if [[ ! -e "$required" ]]; then
    printf 'Missing field input: %s\n' "$required" >&2
    exit 2
  fi
done

source "$repo_root/scripts/source_env.sh"
if ros2 node list 2>/dev/null | grep -Fxq /romo_b_serial_bridge; then
  printf 'A ROMO-B bridge is already running. Stop previous launches first.\n' >&2
  exit 3
fi

if [[ "$receive_only" == true ]]; then
  printf '%s\n' 'SAFE MODE: PCU is receive-only; arm requests cannot move the robot.'
else
  printf '%s\n' 'TX MODE: zero frames may be sent, but the bridge remains DISARMED.'
fi

exec ros2 launch romo_b_autoware field.launch.py \
  hardware_config:="$hardware_config" \
  livox_config:="$livox_config" \
  pcd_map:="$pcd_map" \
  map_path:="$map_path" \
  receive_only:="$receive_only" \
  use_rviz:="${USE_RVIZ:-true}"
