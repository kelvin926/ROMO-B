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
static_config="$autoware_root/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module"
velocity_config="$autoware_root/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/common/autoware_velocity_smoother"
for directory in "$planning_preset" "$control_preset" "$dynamic_config" "$static_config" "$velocity_config"; do
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

# Keep every upstream parameter that ROMO-B does not override. Reading each
# pristine file from the pinned autoware_launch commit makes this idempotent
# even when the working tree already contains a previous generated merge.
merge_yaml_override() {
  python3 - \
    "$autoware_root/src/launcher/autoware_launch" \
    "$1" \
    "$2" <<'PY'
import pathlib
import subprocess
import sys
import yaml

checkout = pathlib.Path(sys.argv[1])
override_path = pathlib.Path(sys.argv[2])
target = pathlib.Path(sys.argv[3])
relative = target.relative_to(checkout)
original = subprocess.check_output(
    ["git", "-C", str(checkout), "show", f"HEAD:{relative}"], text=True
)
base = yaml.safe_load(original)
override = yaml.safe_load(override_path.read_text(encoding="utf-8"))

def merge(destination, source):
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(destination.get(key), dict):
            merge(destination[key], value)
        else:
            destination[key] = value

merge(base, override)
target.write_text(yaml.safe_dump(base, sort_keys=False), encoding="utf-8")
PY
}

merge_yaml_override \
  "$repo_root/config/autoware/planning/static_obstacle_avoidance.override.yaml" \
  "$static_config/static_obstacle_avoidance.param.yaml"
merge_yaml_override \
  "$repo_root/config/autoware/planning/velocity_smoother.override.yaml" \
  "$velocity_config/velocity_smoother.param.yaml"

printf 'Applied ROMO-B Autoware 1.8.0 presets, speed cap, and UNKNOWN avoidance configs.\n'
