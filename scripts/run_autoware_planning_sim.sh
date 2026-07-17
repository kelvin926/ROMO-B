#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
map_run="${MAP_RUN:-$repo_root/data/local/maps/mapping-20260717-195653}"
map_path="${AUTOWARE_MAP:-$map_run/autoware}"

if [[ ! -f "$repo_root/autoware/install/setup.bash" || \
      ! -f "$repo_root/autoware/install/.romo_b_build_complete" ]]; then
  printf 'Autoware is not built. Run ./scripts/setup_autoware.sh first.\n' >&2
  exit 2
fi
for required in lanelet2_map.osm pointcloud_map.pcd map_projector_info.yaml; do
  if [[ ! -e "$map_path/$required" ]]; then
    printf 'Missing Autoware map file: %s\n' "$map_path/$required" >&2
    exit 2
  fi
done

source "$repo_root/scripts/source_env.sh"
exec ros2 launch romo_b_autoware planning_sim.launch.py \
  map_path:="$map_path" use_rviz:="${USE_RVIZ:-true}"
