#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$repo_root/scripts/source_env.sh"

if ros2 node list 2>/dev/null | grep -Fxq /romo_b_serial_bridge; then
  printf 'ROMO-B 브리지가 이미 실행 중입니다. 기존 하드웨어/주행 작업을 먼저 종료하세요.\n' >&2
  exit 3
fi

exec ros2 launch romo_b_bringup robot_control.launch.py \
  hardware_config:="$repo_root/config/local/hardware.yaml" \
  livox_config:="$repo_root/config/local/MID360_config.json" \
  max_speed_mps:="${MAX_SPEED_MPS:-0.5}"
