#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROMO_B_ROOT="$repo_root"
source "$repo_root/scripts/source_env.sh"

exec ros2 run romo_b_operator_ui operator_ui --ros-args \
  -p open_browser:="${OPEN_BROWSER:-true}" \
  -p host:="${OPERATOR_UI_HOST:-127.0.0.1}" \
  -p port:="${OPERATOR_UI_PORT:-8765}"
