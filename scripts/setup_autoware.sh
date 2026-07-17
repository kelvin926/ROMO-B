#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
autoware_root="$repo_root/autoware"
fetch_only=false
if [[ "${1:-}" == "--fetch-only" ]]; then
  fetch_only=true
elif [[ -n "${1:-}" ]]; then
  printf 'Usage: %s [--fetch-only]\n' "$0" >&2
  exit 2
fi

if [[ ! -d "$autoware_root/.git" ]]; then
  git clone https://github.com/autowarefoundation/autoware.git "$autoware_root"
fi
git -C "$autoware_root" fetch --tags --prune
git -C "$autoware_root" checkout --detach 5b27e88e84683deb4afaf0aa917f80082a608871
mkdir -p "$autoware_root/src"
(
  cd "$autoware_root"
  # Re-importing updates existing checkouts to the revisions pinned by the
  # Autoware 1.8.0 manifest instead of silently preserving local drift.
  vcs import src < repositories/autoware.repos
)

if [[ "$fetch_only" == true ]]; then
  printf 'Autoware 1.8.0 sources fetched at %s.\n' "$autoware_root"
  exit 0
fi

"$repo_root/scripts/apply_autoware_overrides.sh"

printf 'Autoware environment installation needs sudo and can take about an hour.\n'
(
  cd "$autoware_root"
  ./setup-dev-env.sh -y --module universe --no-nvidia --ros-distro humble
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  rosdep install --from-paths src --ignore-src -r -y
  export CMAKE_BUILD_PARALLEL_LEVEL=2
  export MAKEFLAGS=-j2
  colcon build --symlink-install --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
)
