#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$repo_root/scripts/source_env.sh"

if ros2 node list 2>/dev/null | grep -Fxq /romo_b_serial_bridge; then
  printf '%s\n' \
    'A ROMO-B bridge is already running. Stop every previous hardware/field launch first.' >&2
  exit 3
fi

map_run="${MAP_RUN:-$repo_root/data/local/maps/mapping-20260721-134953}"
pcd_map="${PCD_MAP:-$map_run/map.pcd}"
map_yaml="${MAP_YAML:-$map_run/nav2-raycast/map.yaml}"
waypoint_file="${WAYPOINT_FILE:-$repo_root/config/local/waypoints.yaml}"
hardware_config="${HARDWARE_CONFIG:-$repo_root/config/local/hardware.yaml}"
livox_config="${LIVOX_CONFIG:-$repo_root/config/local/MID360_config.json}"

for required in "$pcd_map" "$map_yaml" "$waypoint_file" "$hardware_config" "$livox_config"; do
  if [[ ! -f "$required" ]]; then
    printf 'Missing required file: %s\n' "$required" >&2
    exit 2
  fi
done

use_operator_ui="${USE_OPERATOR_UI:-true}"
if [[ "$use_operator_ui" == "true" ]] && ros2 node list 2>/dev/null | grep -Fxq /romo_b_operator_ui; then
  printf '%s\n' \
    'An operator console is already running at http://127.0.0.1:8765/; reusing it.'
  use_operator_ui=false
fi

exec ros2 launch romo_b_bringup field_navigation.launch.py \
  hardware_config:="$hardware_config" \
  livox_config:="$livox_config" \
  pcd_map:="$pcd_map" \
  map:="$map_yaml" \
  waypoint_file:="$waypoint_file" \
  max_speed_mps:="${MAX_SPEED_MPS:-0.5}" \
  use_operator_ui:="$use_operator_ui" \
  open_operator_ui_browser:="${OPEN_OPERATOR_UI_BROWSER:-true}" \
  use_rviz:="${USE_RVIZ:-true}"
