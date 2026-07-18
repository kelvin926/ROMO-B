#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
max_bytes="${MAX_TRACKED_FILE_BYTES:-10485760}"
failures=0

while IFS= read -r -d '' path; do
  [[ -e "$repo_root/$path" ]] || continue
  size="$(stat -Lc '%s' "$repo_root/$path")"
  if (( size > max_bytes )); then
    printf 'Tracked file exceeds %d bytes: %s (%d bytes)\n' \
      "$max_bytes" "$path" "$size" >&2
    failures=$((failures + 1))
  fi
done < <(git -C "$repo_root" ls-files -z)

if (( failures > 0 )); then
  printf 'Move large artifacts outside Git and update data/manifests.\n' >&2
  exit 1
fi

printf 'PASS: all tracked files are at most %d bytes.\n' "$max_bytes"
