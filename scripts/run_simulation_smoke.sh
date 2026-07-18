#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
domain_id="${ROMO_B_SIM_SMOKE_DOMAIN_ID:-113}"
if [[ ! "$domain_id" =~ ^[0-9]+$ ]] || ((domain_id < 1 || domain_id > 232)); then
  printf 'ROMO_B_SIM_SMOKE_DOMAIN_ID must be 1..232.\n' >&2
  exit 2
fi

export ROS_DOMAIN_ID="$domain_id"
source "$repo_root/scripts/source_env.sh"

smoke_dir="$(mktemp -d /tmp/romo_b_sim_smoke.XXXXXX)"
device="$smoke_dir/romo_b_pcu"
launch_log="$smoke_dir/simulation.log"
status_log="$smoke_dir/platform_status.yaml"
launch_pid=""

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$launch_pid" ]]; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.10
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  rm -rf -- "$smoke_dir"
}
trap cleanup EXIT INT TERM

printf 'Isolated ROS_DOMAIN_ID=%s; temporary simulated serial only.\n' "$ROS_DOMAIN_ID"
setsid ros2 launch romo_b_sim simulation.launch.py device:="$device" \
  >"$launch_log" 2>&1 &
launch_pid="$!"

active=false
for _ in $(seq 1 100); do
  if ros2 lifecycle get /romo_b_serial_bridge 2>/dev/null | grep -Fq active; then
    active=true
    break
  fi
  sleep 0.10
done
if [[ "$active" != true ]]; then
  tail -n 120 "$launch_log" >&2
  printf 'FAIL: simulated serial bridge did not become active.\n' >&2
  exit 1
fi

if ! timeout 8 ros2 topic echo /romo_b/platform_status --once >"$status_log"; then
  tail -n 120 "$launch_log" >&2
  printf 'FAIL: simulated platform status was not published.\n' >&2
  exit 1
fi

for expected in 'state: 1' 'connected: true' 'auto_mode: false' 'estop: false'; do
  if ! grep -Fq "$expected" "$status_log"; then
    cat "$status_log" >&2
    printf 'FAIL: simulated platform status is missing %s.\n' "$expected" >&2
    exit 1
  fi
done

printf 'SIMULATION_SMOKE_OK: lifecycle active, connected, disarmed, no E-stop\n'
