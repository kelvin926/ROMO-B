#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
helper_source="$repo_root/scripts/openarm_can_link.sh"
helper_target="/usr/local/sbin/romo-b-openarm-can-link"
sudoers_target="/etc/sudoers.d/romo-b-openarm-can-link"
temporary_sudoers="$(mktemp)"
trap 'rm -f "$temporary_sudoers"' EXIT

printf '%%sudo ALL=(root) NOPASSWD: %s *\n' "$helper_target" >"$temporary_sudoers"
chmod 0440 "$temporary_sudoers"

sudo install -o root -g root -m 0755 "$helper_source" "$helper_target"
sudo visudo -cf "$temporary_sudoers"
sudo install -o root -g root -m 0440 "$temporary_sudoers" "$sudoers_target"

printf 'OpenArm CAN browser helper installed: %s\n' "$helper_target"
