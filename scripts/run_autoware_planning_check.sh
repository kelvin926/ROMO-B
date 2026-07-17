#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
map_run="${MAP_RUN:-$repo_root/data/local/maps/mapping-20260717-195653}"
map_path="${AUTOWARE_MAP:-$map_run/autoware}"
domain_id="${ROMO_B_AUTOWARE_PLANNING_DOMAIN_ID:-100}"
if [[ ! "$domain_id" =~ ^[0-9]+$ ]] || ((domain_id < 1 || domain_id > 232)); then
  printf 'ROMO_B_AUTOWARE_PLANNING_DOMAIN_ID must be 1..232.\n' >&2
  exit 2
fi
if [[ ! -f "$repo_root/autoware/install/.romo_b_build_complete" ]]; then
  printf 'Autoware build is not marked complete.\n' >&2
  exit 2
fi
for required in lanelet2_map.osm pointcloud_map.pcd map_projector_info.yaml; do
  [[ -e "$map_path/$required" ]] || {
    printf 'Missing Autoware map file: %s\n' "$map_path/$required" >&2
    exit 2
  }
done

export ROS_DOMAIN_ID="$domain_id"
source "$repo_root/scripts/source_env.sh"
run_id="autoware-planning-$(date +%Y%m%d-%H%M%S)"
output_dir="$repo_root/data/local/validation/$run_id"
mkdir -p "$output_dir"
launch_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$launch_pid" ]]; then
    # The launch process and every child run in a private session.  Signaling
    # the whole group avoids leaving late-shutting Autoware components or DDS
    # shared-memory locks behind for the next validation scenario.
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 12); do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -KILL -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

printf 'Isolated ROS_DOMAIN_ID=%s; planning simulator only, no serial bridge.\n' "$ROS_DOMAIN_ID"
setsid ros2 launch romo_b_autoware planning_sim.launch.py \
  map_path:="$map_path" use_rviz:=false initial_engage_state:=false \
  >"$output_dir/planning_sim.log" 2>&1 &
launch_pid="$!"

python3 "$repo_root/scripts/check_autoware_planning.py" \
  --map-path "$map_path" --output "$output_dir/result.json" "$@" || {
    tail -n 200 "$output_dir/planning_sim.log" >&2
    exit 1
  }
printf 'PASS: Autoware route/UNKNOWN-obstacle result is in %s\n' "$output_dir"
