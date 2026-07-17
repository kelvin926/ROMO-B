#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
domain_id="${ROMO_B_AUTOWARE_LOCALIZATION_DOMAIN_ID:-102}"
if [[ ! "$domain_id" =~ ^[0-9]+$ ]] || ((domain_id < 1 || domain_id > 232)); then
  printf 'ROMO_B_AUTOWARE_LOCALIZATION_DOMAIN_ID must be 1..232.\n' >&2
  exit 2
fi
if [[ ! -f "$repo_root/autoware/install/.romo_b_build_complete" ]]; then
  printf 'Autoware build is not marked complete.\n' >&2
  exit 2
fi

export ROS_DOMAIN_ID="$domain_id"
source "$repo_root/scripts/source_env.sh"
run_id="autoware-localization-interface-$(date +%Y%m%d-%H%M%S)"
output_dir="$repo_root/data/local/validation/$run_id"
mkdir -p "$output_dir"
node_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$node_pid" ]]; then
    kill -INT "$node_pid" 2>/dev/null || true
    for _ in $(seq 1 12); do
      kill -0 "$node_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM "$node_pid" 2>/dev/null || true
    wait "$node_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

printf 'Isolated ROS_DOMAIN_ID=%s; no localization sensor or hardware node is launched.\n' "$ROS_DOMAIN_ID"
node_executable="$(ros2 pkg prefix romo_b_autoware)/lib/romo_b_autoware/localization_interface"
"$node_executable" >"$output_dir/interface.log" 2>&1 &
node_pid="$!"

python3 "$repo_root/scripts/check_autoware_localization_interface.py" \
  --output "$output_dir/result.json" || {
    tail -n 100 "$output_dir/interface.log" >&2
    exit 1
  }
printf 'PASS: Autoware localization interface validation is in %s\n' "$output_dir"
