#!/usr/bin/env bash
set -euo pipefail

# Root-owned runtime helper for the browser console. It only accepts one or two
# existing SocketCAN interfaces named canN and always applies the fixed
# OpenArm-v1 Classic CAN bitrate. It never opens a CAN socket or transmits a frame.
if (( $# < 1 || $# > 2 )); then
  printf 'usage: %s canN [canN]\n' "$0" >&2
  exit 2
fi

for interface in "$@"; do
  if [[ ! "$interface" =~ ^can[0-9]+$ ]]; then
    printf 'invalid SocketCAN interface: %s\n' "$interface" >&2
    exit 2
  fi
  if [[ ! -d "/sys/class/net/$interface" ]]; then
    printf 'SocketCAN interface does not exist: %s\n' "$interface" >&2
    exit 3
  fi
  if [[ "$(<"/sys/class/net/$interface/type")" != "280" ]]; then
    printf 'network interface is not SocketCAN: %s\n' "$interface" >&2
    exit 3
  fi
done
for interface in "$@"; do
  details="$(/usr/sbin/ip -details link show dev "$interface")"
  if [[ "$details" == *"<"*"UP"*">"* && "$details" == *"bitrate 1000000"* ]]; then
    continue
  fi
  /usr/sbin/ip link set dev "$interface" down || true
  /usr/sbin/ip link set dev "$interface" type can bitrate 1000000 restart-ms 100
  /usr/sbin/ip link set dev "$interface" txqueuelen 1000
  /usr/sbin/ip link set dev "$interface" up
done
