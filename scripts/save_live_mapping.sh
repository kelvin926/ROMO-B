#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
map_dir="${1:-}"

if [[ -z "$map_dir" ]]; then
  map_dir="$(find "$repo_root/data/local/maps" -mindepth 1 -maxdepth 1 \
    -type d -name 'mapping-*' -printf '%T@ %p\n' 2>/dev/null \
    | sort -n | tail -1 | cut -d' ' -f2-)"
fi
if [[ -z "$map_dir" || ! -d "$map_dir" ]]; then
  printf 'No live mapping directory found. Pass the path printed by run_live_mapping.sh.\n' >&2
  exit 2
fi

map_dir="$(realpath "$map_dir")"
case "$map_dir" in
  "$repo_root"/data/local/maps/*) ;;
  *)
    printf 'Refusing unexpected map directory: %s\n' "$map_dir" >&2
    exit 2
    ;;
esac

# shellcheck disable=SC1091
source "$repo_root/scripts/source_env.sh"

printf 'Requesting graph optimization and map save to %s ...\n' "$map_dir"
for _ in $(seq 1 20); do
  ros2 service list 2>/dev/null | grep -qx '/map_save' && break
  sleep 0.5
done
if ! ros2 service list 2>/dev/null | grep -qx '/map_save'; then
  printf '/map_save is unavailable; is run_live_mapping.sh still running?\n' >&2
  exit 1
fi
ros2 service call /map_save std_srvs/srv/Empty '{}'

deadline=$((SECONDS + 90))
while (( SECONDS < deadline )); do
  if [[ -s "$map_dir/map.pcd" && -s "$map_dir/pose_graph.g2o" ]]; then
    break
  fi
  sleep 1
done
if [[ ! -s "$map_dir/map.pcd" || ! -s "$map_dir/pose_graph.g2o" ]]; then
  printf 'Map save did not produce map.pcd and pose_graph.g2o within 90 seconds.\n' >&2
  exit 1
fi

mkdir -p "$map_dir/nav2-raycast"
ros2 run romo_b_perception pcd_to_occupancy \
  "$map_dir/map.pcd" "$map_dir/nav2-raycast/map" \
  0.05 0.10 1.80 "$map_dir/pose_graph.g2o" 15.0 0.171

printf 'SAVED: %s/map.pcd\n' "$map_dir"
printf 'NAV2:  %s/nav2-raycast/map.yaml\n' "$map_dir"
printf 'You may now stop the live-mapping launch with Ctrl+C.\n'
