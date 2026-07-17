#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -lt 2 || $# -gt 3 ]]; then
  printf 'Usage: %s BAG_PATH MAP_RUN_DIR [RATE]\n' "$0" >&2
  exit 2
fi

bag_path="$(realpath "$1")"
map_run="$(realpath "$2")"
rate="${3:-1.0}"
pcd_map="$map_run/map.pcd"
pose_graph="$map_run/pose_graph.g2o"
for required in "$bag_path" "$pcd_map" "$pose_graph"; do
  if [[ ! -e "$required" ]]; then
    printf 'Missing replay input: %s\n' "$required" >&2
    exit 1
  fi
done

# Do not inherit ROS_DOMAIN_ID: an interactive shell may be using the physical
# robot's domain. A dedicated variable is required for an intentional override.
replay_domain_id="${ROMO_B_REPLAY_DOMAIN_ID:-97}"
if [[ ! "$replay_domain_id" =~ ^[0-9]+$ ]] ||
  ((replay_domain_id < 1 || replay_domain_id > 232)); then
  printf 'ROMO_B_REPLAY_DOMAIN_ID must be an integer from 1 to 232.\n' >&2
  exit 2
fi
export ROS_DOMAIN_ID="$replay_domain_id"
# This launch contains no serial bridge and publishes no velocity topic.
# shellcheck disable=SC1091
source "$repo_root/scripts/source_env.sh"

run_id="localization-$(date +%Y%m%d-%H%M%S)"
validation_dir="$repo_root/data/local/validation/$run_id"
output_bag="$validation_dir/output"
mkdir -p "$validation_dir"

printf 'Isolated ROS_DOMAIN_ID=%s; no robot command node will be launched.\n' "$ROS_DOMAIN_ID"
printf 'Localization replay output: %s\n' "$validation_dir"
ros2 launch romo_b_bringup localization_replay.launch.py \
  bag_path:="$bag_path" \
  pcd_map:="$pcd_map" \
  pose_graph:="$pose_graph" \
  hardware_config:="$repo_root/config/local/hardware.yaml" \
  output_bag:="$output_bag" \
  rate:="$rate"

python3 "$repo_root/scripts/analyze_localization_bag.py" \
  "$output_bag" \
  --output-json "$validation_dir/summary.json"
