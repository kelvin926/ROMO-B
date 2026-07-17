#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 4 ]]; then
  printf 'Usage: %s TYPE ID FILE [URI]\n' "$0" >&2
  exit 2
fi

artifact_type="$1"
artifact_id="$2"
artifact_path="$(readlink -f "$3")"
artifact_uri="${4:-pending}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
manifest="$repo_root/data/manifests/$artifact_id.yaml"

if [[ ! -f "$artifact_path" ]]; then
  printf 'ERROR: artifact is not a regular file: %s\n' "$artifact_path" >&2
  exit 1
fi
if [[ ! "$artifact_id" =~ ^[a-zA-Z0-9._-]+$ ]]; then
  printf 'ERROR: ID may contain only letters, digits, dot, underscore, and dash.\n' >&2
  exit 1
fi

python3 - "$manifest" "$artifact_type" "$artifact_id" "$artifact_path" "$artifact_uri" <<'PY'
import datetime
import hashlib
import pathlib
import sys
import yaml

manifest, kind, artifact_id, source, uri = sys.argv[1:]
path = pathlib.Path(source)
h = hashlib.sha256()
with path.open("rb") as stream:
    for block in iter(lambda: stream.read(1024 * 1024), b""):
        h.update(block)
data = {
    "schema_version": 1,
    "id": artifact_id,
    "type": kind,
    "created_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    "source_filename": path.name,
    "sensor": {"model": "Livox Mid-360", "serial": "pending"},
    "frames": {"fixed": "map", "sensor": "livox_frame"},
    "size_bytes": path.stat().st_size,
    "sha256": h.hexdigest(),
    "uri": uri,
}
pathlib.Path(manifest).write_text(yaml.safe_dump(data, sort_keys=False))
print(manifest)
PY
