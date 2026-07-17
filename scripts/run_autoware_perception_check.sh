#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
domain_id="${ROMO_B_AUTOWARE_VALIDATION_DOMAIN_ID:-99}"
if [[ ! "$domain_id" =~ ^[0-9]+$ ]] || ((domain_id < 1 || domain_id > 232)); then
  printf 'ROMO_B_AUTOWARE_VALIDATION_DOMAIN_ID must be 1..232.\n' >&2
  exit 2
fi
if [[ ! -f "$repo_root/autoware/install/.romo_b_build_complete" ]]; then
  printf 'Autoware build is not marked complete.\n' >&2
  exit 2
fi

export ROS_DOMAIN_ID="$domain_id"
source "$repo_root/scripts/source_env.sh"
run_id="autoware-perception-$(date +%Y%m%d-%H%M%S)"
output_dir="$repo_root/data/local/validation/$run_id"
mkdir -p "$output_dir"
launch_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$launch_pid" ]]; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 12); do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 8); do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -KILL -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

printf 'Isolated ROS_DOMAIN_ID=%s; no hardware or velocity nodes are launched.\n' "$ROS_DOMAIN_ID"
setsid ros2 launch romo_b_autoware perception.launch.py \
  >"$output_dir/perception.log" 2>&1 &
launch_pid="$!"

python3 "$repo_root/scripts/check_autoware_perception.py" \
  --output "$output_dir/result.json" || {
    tail -n 100 "$output_dir/perception.log" >&2
    exit 1
  }
printf 'PASS: Autoware perception validation is in %s\n' "$output_dir"
