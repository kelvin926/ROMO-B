#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unit_source="$repo_root/config/systemd/romo-b-operator-ui.service"
unit_dir="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
unit_target="$unit_dir/romo-b-operator-ui.service"

mkdir -p "$unit_dir"
ln -sfn "$unit_source" "$unit_target"
systemctl --user daemon-reload
systemctl --user enable --now romo-b-operator-ui.service
systemctl --user --no-pager --full status romo-b-operator-ui.service
