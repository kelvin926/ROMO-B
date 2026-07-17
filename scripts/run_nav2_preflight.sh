#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -ne 2 ]]; then
  printf 'Usage: %s MAP_YAML POSE_GRAPH\n' "$0" >&2
  exit 2
fi
map_yaml="$(realpath "$1")"
pose_graph="$(realpath "$2")"
for required in "$map_yaml" "$pose_graph"; do
  [[ -f "$required" ]] || { printf 'Missing input: %s\n' "$required" >&2; exit 1; }
done

domain_id="${ROMO_B_NAV2_VALIDATION_DOMAIN_ID:-98}"
if [[ ! "$domain_id" =~ ^[0-9]+$ ]] || ((domain_id < 1 || domain_id > 232)); then
  printf 'ROMO_B_NAV2_VALIDATION_DOMAIN_ID must be 1..232.\n' >&2
  exit 2
fi
export ROS_DOMAIN_ID="$domain_id"
# shellcheck disable=SC1091
source "$repo_root/scripts/source_env.sh"

run_id="nav2-$(date +%Y%m%d-%H%M%S)"
output_dir="$repo_root/data/local/validation/$run_id"
mkdir -p "$output_dir"
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  for _ in $(seq 1 12); do
    running=false
    for pid in "${pids[@]}"; do
      kill -0 "$pid" 2>/dev/null && running=true
    done
    [[ "$running" == false ]] && break
    sleep 0.25
  done
  for pid in "${pids[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

printf 'Isolated ROS_DOMAIN_ID=%s; no serial bridge is launched.\n' "$ROS_DOMAIN_ID"
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id map --child-frame-id odom >"$output_dir/map_odom.log" 2>&1 &
pids+=("$!")
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id odom --child-frame-id base_link >"$output_dir/odom_base.log" 2>&1 &
pids+=("$!")
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z -0.171 --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id base_link --child-frame-id base_footprint >"$output_dir/base_footprint.log" 2>&1 &
pids+=("$!")
ros2 launch romo_b_navigation navigation.launch.py \
  map:="$map_yaml" \
  waypoint_file:="$repo_root/robot_ws/src/romo_b_navigation/config/waypoints/example.yaml" \
  >"$output_dir/nav2.log" 2>&1 &
pids+=("$!")

for _ in $(seq 1 40); do
  state="$(ros2 lifecycle get /collision_monitor 2>/dev/null || true)"
  [[ "$state" == active* ]] && break
  sleep 0.5
done
if [[ "$state" != active* ]]; then
  tail -n 80 "$output_dir/nav2.log" >&2
  printf 'FAIL: Nav2 lifecycle did not become active.\n' >&2
  exit 1
fi

python3 "$repo_root/scripts/check_nav2_plan.py" "$pose_graph" 1 60 \
  | tee "$output_dir/plan_1_60.json"
python3 "$repo_root/scripts/check_nav2_plan.py" "$pose_graph" 70 130 \
  | tee "$output_dir/plan_70_130.json"
python3 "$repo_root/scripts/check_cmd_pipeline.py" \
  | tee "$output_dir/command_pipeline.json"
printf 'PASS: Nav2 preflight outputs are in %s\n' "$output_dir"
