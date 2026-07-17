#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
map_run="${1:-${MAP_RUN:-$repo_root/data/local/maps/mapping-20260717-195653}}"
pose_graph="${POSE_GRAPH:-$map_run/pose_graph.g2o}"
pointcloud_map="${PCD_MAP:-$map_run/map.pcd}"
output="${AUTOWARE_MAP:-$map_run/autoware}"

for required in "$pose_graph" "$pointcloud_map"; do
  if [[ ! -f "$required" ]]; then
    printf 'Missing map input: %s\n' "$required" >&2
    exit 2
  fi
done

python3 "$repo_root/scripts/generate_autoware_corridor_map.py" \
  --pose-graph "$pose_graph" \
  --pointcloud-map "$pointcloud_map" \
  --output "$output"

python3 - "$output" <<'PY'
import json
import pathlib
import sys
import xml.etree.ElementTree as ET

root = pathlib.Path(sys.argv[1])
ET.parse(root / "lanelet2_map.osm")
report = json.loads((root / "generation_report.json").read_text(encoding="utf-8"))
assert report["lanelet_count"] > 0
assert report["centerline_point_count"] > report["lanelet_count"]
assert (root / "pointcloud_map.pcd").is_file()
assert (root / "map_projector_info.yaml").read_text(encoding="utf-8").strip() == "projector_type: Local"
print(f"PASS: {report['lanelet_count']} lanelets, {report['centerline_point_count']} centerline points")
PY

printf 'Autoware map ready: %s\n' "$output"
