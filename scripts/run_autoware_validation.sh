#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

printf '%s\n' 'ROMO-B Autoware validation: isolated ROS domains, no serial bridge.'
"$repo_root/scripts/doctor.sh" --autoware
"$repo_root/scripts/run_autoware_localization_interface_check.sh"
"$repo_root/scripts/run_autoware_safety_gate_check.sh"
"$repo_root/scripts/run_autoware_perception_check.sh"
"$repo_root/scripts/run_autoware_waypoint_sender_check.sh"
"$repo_root/scripts/run_autoware_planning_check.sh" \
  --obstacle-offset 0.30 --object-diameter 0.40 --expect avoid
ROMO_B_AUTOWARE_PLANNING_DOMAIN_ID=103 \
  "$repo_root/scripts/run_autoware_planning_check.sh" \
  --obstacle-offset 0.00 --object-diameter 0.60 --expect stop
printf '%s\n' 'PASS: all software-only Autoware acceptance checks completed.'
