#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vendor_root="$repo_root/vendor_ws"
src_root="$vendor_root/src"
sdk_root="$vendor_root/sdk/Livox-SDK2"

if ! command -v vcs >/dev/null 2>&1; then
  printf 'ERROR: vcs is missing. Run scripts/setup_host.sh first.\n' >&2
  exit 1
fi

mkdir -p "$src_root" "$vendor_root/sdk"
vcs import "$src_root" --input "$repo_root/dependencies/vendor.repos" --skip-existing

if [[ ! -d "$sdk_root/.git" ]]; then
  git clone https://github.com/Livox-SDK/Livox-SDK2.git "$sdk_root"
fi
git -C "$sdk_root" fetch --tags --prune
git -C "$sdk_root" checkout --detach f5d9375f84efe2b15bc0a052d3e18482ed13adf4

slam_root="$src_root/lidar_slam_ros2"
if [[ -d "$slam_root/.git" ]]; then
  git -C "$slam_root" submodule update --init --recursive
  # A newer, shared ndt_omp_ros2 checkout is pinned at vendor_ws/src. Keep the
  # upstream submodule for source integrity but prevent duplicate ROS packages.
  touch "$slam_root/Thirdparty/ndt_omp_ros2/COLCON_IGNORE"

  patch_root="$repo_root/dependencies/patches/lidar_slam_ros2"
  if [[ -d "$patch_root" ]]; then
    while IFS= read -r -d '' patch_file; do
      if git -C "$slam_root" apply --unidiff-zero --reverse --check "$patch_file" >/dev/null 2>&1; then
        printf 'ALREADY  lidar_slam_ros2 patch: %s\n' "$(basename "$patch_file")"
      elif git -C "$slam_root" apply --unidiff-zero --check "$patch_file"; then
        git -C "$slam_root" apply --unidiff-zero "$patch_file"
        printf 'APPLIED  lidar_slam_ros2 patch: %s\n' "$(basename "$patch_file")"
      else
        printf 'ERROR: cannot apply vendor patch %s\n' "$patch_file" >&2
        exit 1
      fi
    done < <(find "$patch_root" -maxdepth 1 -type f -name '*.patch' -print0 | sort -z)
  fi
fi

localization_root="$src_root/lidar_localization_ros2"
if [[ -d "$localization_root/.git" ]]; then
  patch_root="$repo_root/dependencies/patches/lidar_localization_ros2"
  if [[ -d "$patch_root" ]]; then
    while IFS= read -r -d '' patch_file; do
      if git -C "$localization_root" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
        printf 'ALREADY  lidar_localization_ros2 patch: %s\n' "$(basename "$patch_file")"
      elif git -C "$localization_root" apply --check "$patch_file"; then
        git -C "$localization_root" apply "$patch_file"
        printf 'APPLIED  lidar_localization_ros2 patch: %s\n' "$(basename "$patch_file")"
      else
        printf 'ERROR: cannot apply vendor patch %s\n' "$patch_file" >&2
        exit 1
      fi
    done < <(find "$patch_root" -maxdepth 1 -type f -name '*.patch' -print0 | sort -z)
  fi
fi

driver_root="$src_root/livox_ros_driver2"
if [[ -d "$driver_root" ]]; then
  cp "$driver_root/package_ROS2.xml" "$driver_root/package.xml"
  rm -rf "$driver_root/launch"
  cp -a "$driver_root/launch_ROS2" "$driver_root/launch"
fi

printf 'Verifying dependency commits...\n'
python3 - "$repo_root/dependencies/lock.yaml" "$src_root" "$sdk_root" <<'PY'
import pathlib
import subprocess
import sys
import yaml

lock_path, src_root, sdk_root = sys.argv[1:]
lock = yaml.safe_load(pathlib.Path(lock_path).read_text())
paths = {
    "Livox-SDK2": pathlib.Path(sdk_root),
    "livox_ros_driver2": pathlib.Path(src_root) / "livox_ros_driver2",
    "lidar_slam_ros2": pathlib.Path(src_root) / "lidar_slam_ros2",
    "lidar_localization_ros2": pathlib.Path(src_root) / "lidar_localization_ros2",
    "ndt_omp_ros2": pathlib.Path(src_root) / "ndt_omp_ros2",
}
failed = False
for name, path in paths.items():
    expected = lock["repositories"][name]["commit"]
    actual = subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
    state = "OK" if actual == expected else "MISMATCH"
    print(f"{state:8} {name}: {actual}")
    failed |= actual != expected
raise SystemExit(1 if failed else 0)
PY

printf 'Dependencies are pinned and ready below %s.\n' "$vendor_root"
