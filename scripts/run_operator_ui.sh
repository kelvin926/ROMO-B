#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROMO_B_ROOT="$repo_root"
source "$repo_root/scripts/source_env.sh"

operator_ui="$repo_root/robot_ws/install/romo_b_operator_ui/lib/romo_b_operator_ui/operator_ui"
if [[ ! -x "$operator_ui" ]]; then
  printf 'Operator UI is not built: %s\n' "$operator_ui" >&2
  exit 2
fi

# Run the installed node directly so systemd tracks the actual web/ROS process.
# Child field operations use their own process groups and remain observable
# across a web-console restart.
exec "$operator_ui" --ros-args \
  -p open_browser:="${OPEN_BROWSER:-true}" \
  -p host:="${OPERATOR_UI_HOST:-127.0.0.1}" \
  -p port:="${OPERATOR_UI_PORT:-8765}"
