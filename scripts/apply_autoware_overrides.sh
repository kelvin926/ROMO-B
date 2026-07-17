#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
autoware_root="${AUTOWARE_ROOT:-$repo_root/autoware}"
expected_commit=5b27e88e84683deb4afaf0aa917f80082a608871

if [[ ! -d "$autoware_root/.git" ]]; then
  printf 'Autoware checkout not found: %s\n' "$autoware_root" >&2
  exit 1
fi
actual_commit="$(git -C "$autoware_root" rev-parse HEAD)"
if [[ "$actual_commit" != "$expected_commit" ]]; then
  printf 'Refusing overrides: expected Autoware %s, found %s\n' \
    "$expected_commit" "$actual_commit" >&2
  exit 1
fi

planning_preset="$autoware_root/src/launcher/autoware_launch/autoware_launch/config/planning/preset"
control_preset="$autoware_root/src/launcher/autoware_launch/autoware_launch/config/control/preset"
dynamic_config="$autoware_root/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module"
for directory in "$planning_preset" "$control_preset" "$dynamic_config"; do
  [[ -d "$directory" ]] || {
    printf 'Autoware 1.8.0 layout mismatch: %s\n' "$directory" >&2
    exit 1
  }
done

install -m 0644 "$repo_root/config/autoware/presets/romo_b_planning_preset.yaml" \
  "$planning_preset/romo_b_preset.yaml"
install -m 0644 "$repo_root/config/autoware/presets/romo_b_control_preset.yaml" \
  "$control_preset/romo_b_preset.yaml"
install -m 0644 "$repo_root/config/autoware/planning/dynamic_obstacle_avoidance.param.yaml" \
  "$dynamic_config/dynamic_obstacle_avoidance.param.yaml"

printf 'Applied ROMO-B Autoware 1.8.0 presets and dynamic UNKNOWN avoidance config.\n'
