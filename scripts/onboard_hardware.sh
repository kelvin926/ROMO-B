#!/usr/bin/env bash
set -euo pipefail

mode="${1:---discover}"
force="false"
if [[ "${2:-}" == "--force" ]]; then
  force="true"
elif [[ -n "${2:-}" ]]; then
  printf 'ERROR: unknown argument: %s\n' "$2" >&2
  exit 2
fi
if [[ "$mode" != "--discover" && "$mode" != "--generate" && \
  "$mode" != "--apply" && "$mode" != "--receive-only" ]]; then
  printf 'Usage: %s [--discover|--generate [--force]|--apply|--receive-only]\n' "$0" >&2
  exit 2
fi
if [[ "$force" == "true" && "$mode" != "--generate" ]]; then
  printf 'ERROR: --force is valid only with --generate.\n' >&2
  exit 2
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
hardware_yaml="$repo_root/config/local/hardware.yaml"
if [[ "$mode" == "--generate" && -e "$hardware_yaml" && "$force" != "true" ]]; then
  printf 'ERROR: %s already exists; refusing to replace measured hardware settings.\n' \
    "$hardware_yaml" >&2
  printf 'Use --generate --force only when intentionally onboarding different hardware.\n' >&2
  exit 1
fi
serial_link="$(find /dev/serial/by-id -mindepth 1 -maxdepth 1 -type l -print 2>/dev/null | head -n 1 || true)"
wired_if="$(find /sys/class/net -mindepth 1 -maxdepth 1 -printf '%f\n' 2>/dev/null | grep -Ev '^(lo|wl)' | head -n 1 || true)"

printf 'USB-RS232 by-id: %s\n' "${serial_link:-not detected}"
if [[ -n "$serial_link" ]]; then
  serial_target="$(readlink -f "$serial_link")"
  printf 'USB-RS232 device: %s\n' "$serial_target"
  udevadm info --query=property --name="$serial_target" | \
    grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL)=' || true
fi
printf 'Candidate wired interface: %s\n' "${wired_if:-not detected}"
printf 'No serial data was transmitted.\n'

if [[ "$mode" == "--discover" ]]; then
  exit 0
fi

if [[ "$mode" == "--receive-only" ]]; then
  exec python3 "$repo_root/scripts/serial_receive_only.py"
fi

mkdir -p "$(dirname "$hardware_yaml")"
if [[ "$mode" == "--generate" ]]; then
  adapter_serial="pending"
  if [[ -n "$serial_link" ]]; then
    adapter_serial="$(udevadm info --query=property --name="$(readlink -f "$serial_link")" | sed -n 's/^ID_SERIAL=//p')"
  fi
  generate_args=(
    --output "$hardware_yaml"
    --adapter-serial "$adapter_serial"
    --interface "${wired_if:-pending}"
  )
  if [[ "$force" == "true" ]]; then
    generate_args+=(--force)
  fi
  python3 "$repo_root/scripts/generate_hardware_config.py" "${generate_args[@]}"
  exit 0
fi

if [[ ! -f "$hardware_yaml" ]]; then
  printf 'ERROR: %s is missing; run --generate first.\n' "$hardware_yaml" >&2
  exit 1
fi

if [[ -z "$serial_link" ]]; then
  printf 'ERROR: USB-RS232 adapter is required for --apply.\n' >&2
  exit 1
fi
vendor_id="$(udevadm info --query=property --name="$(readlink -f "$serial_link")" | sed -n 's/^ID_VENDOR_ID=//p')"
model_id="$(udevadm info --query=property --name="$(readlink -f "$serial_link")" | sed -n 's/^ID_MODEL_ID=//p')"
serial_short="$(udevadm info --query=property --name="$(readlink -f "$serial_link")" | sed -n 's/^ID_SERIAL_SHORT=//p')"
rule="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vendor_id\", ATTRS{idProduct}==\"$model_id\", ATTRS{serial}==\"$serial_short\", SYMLINK+=\"romo_b_pcu\", GROUP=\"dialout\", MODE=\"0660\""
printf '%s\n' "$rule" | sudo tee /etc/udev/rules.d/99-romo-b-pcu.rules >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty

readarray -t lidar_network < <(python3 - "$hardware_yaml" \
  "$repo_root/config/local/MID360_config.json" \
  "$repo_root/config/local/rko_lio_mid360.yaml" <<'PY'
import json
import math
import pathlib
import sys
import yaml

hardware_path, json_path, rko_path = sys.argv[1:]
data = yaml.safe_load(pathlib.Path(hardware_path).read_text()) or {}
lidar = data.get("lidar", {})
host_ip = str(lidar.get("host_ip", "pending"))
lidar_ip = str(lidar.get("lidar_ip", "pending"))
interface = str(lidar.get("interface", "pending"))
print(host_ip)
print(lidar_ip)
print(interface)
if "pending" in (host_ip, lidar_ip, interface):
    print("LiDAR network is pending; Livox JSON was not generated.", file=sys.stderr)
    raise SystemExit(0)
config = {
    "lidar_summary_info": {"lidar_type": 8},
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100, "push_msg_port": 56200,
            "point_data_port": 56300, "imu_data_port": 56400,
            "log_data_port": 56500,
        },
        "host_net_info": {
            "cmd_data_ip": host_ip, "cmd_data_port": 56101,
            "push_msg_ip": host_ip, "push_msg_port": 56201,
            "point_data_ip": host_ip, "point_data_port": 56301,
            "imu_data_ip": host_ip, "imu_data_port": 56401,
            "log_data_ip": "", "log_data_port": 56501,
        },
    },
    "lidar_configs": [{
        "ip": lidar_ip, "pcl_data_type": 1, "pattern_mode": 0,
        "extrinsic_parameter": {
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "x": 0, "y": 0, "z": 0,
        },
    }],
}
pathlib.Path(json_path).write_text(json.dumps(config, indent=2) + "\n")
print(f"Generated {json_path}", file=sys.stderr)

transform = lidar.get("transform", {})
x = float(transform.get("x", 0.0))
y = float(transform.get("y", 0.0))
z = float(transform.get("z", 0.0))
roll = float(transform.get("roll", 0.0))
pitch = float(transform.get("pitch", 0.0))
yaw = float(transform.get("yaw", 0.0))
cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
qx = sr * cp * cy - cr * sp * sy
qy = cr * sp * cy + sr * cp * sy
qz = cr * cp * sy - sr * sp * cy
qw = cr * cp * cy + sr * sp * sy
sensor_to_base = [qx, qy, qz, qw, x, y, z]
rko_config = {
    "/**": {
        "ros__parameters": {
            "extrinsic_imu2base_quat_xyzw_xyz": sensor_to_base,
            "extrinsic_lidar2base_quat_xyzw_xyz": sensor_to_base,
            "initialization_phase": True,
        },
    },
}
pathlib.Path(rko_path).write_text(yaml.safe_dump(rko_config, sort_keys=False))
print(f"Generated {rko_path}", file=sys.stderr)
PY
)
host_ip="${lidar_network[0]:-pending}"
lidar_ip="${lidar_network[1]:-pending}"
configured_if="${lidar_network[2]:-pending}"

if [[ "$configured_if" != "pending" && "$host_ip" != "pending" ]]; then
  if nmcli -t -f NAME connection show | grep -qx romo-b-livox; then
    sudo nmcli connection modify romo-b-livox ipv4.method manual \
      ipv4.addresses "$host_ip/24" ipv4.gateway '' ipv4.never-default yes \
      connection.interface-name "$configured_if"
  else
    sudo nmcli connection add type ethernet ifname "$configured_if" con-name romo-b-livox \
      ipv4.method manual ipv4.addresses "$host_ip/24" ipv4.never-default yes
  fi
fi
printf 'Applied udev and available LiDAR network settings. No motion command was sent.\n'
