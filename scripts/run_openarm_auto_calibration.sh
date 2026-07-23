#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "사용법: $0 <left|right> <SocketCAN 인터페이스>" >&2
  exit 2
fi

ARM_SIDE=$1
CAN_INTERFACE=$2
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROMO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
PYTHON_BIN="${ROMO_ROOT}/openarm/.venv/bin/python3"
CALIBRATION_TOOL="${ROMO_ROOT}/openarm/src/openarm_can/setup/openarm-can-zero-position-calibration"

if [[ "${ARM_SIDE}" != "left" && "${ARM_SIDE}" != "right" ]]; then
  echo "팔 선택은 left 또는 right여야 합니다." >&2
  exit 2
fi
if [[ ! "${CAN_INTERFACE}" =~ ^[A-Za-z0-9_.:-]{1,15}$ ]]; then
  echo "SocketCAN 인터페이스 이름 형식이 올바르지 않습니다." >&2
  exit 2
fi
if [[ ! -d "/sys/class/net/${CAN_INTERFACE}" ]] || \
   [[ "$(<"/sys/class/net/${CAN_INTERFACE}/type")" != "280" ]]; then
  echo "${CAN_INTERFACE}은 SocketCAN 인터페이스가 아닙니다." >&2
  exit 3
fi
if [[ $(( $(<"/sys/class/net/${CAN_INTERFACE}/flags") & 1 )) -eq 0 ]]; then
  echo "${CAN_INTERFACE} 인터페이스가 UP 상태가 아닙니다." >&2
  exit 3
fi
if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "OpenArm Python 환경이 없습니다. setup_openarm_can_python.sh를 먼저 실행하세요." >&2
  exit 4
fi

"${PYTHON_BIN}" -c 'import openarm_can, numpy' >/dev/null

export PYTHONUNBUFFERED=1
exec "${PYTHON_BIN}" -u "${CALIBRATION_TOOL}" \
  --canport "${CAN_INTERFACE}" \
  --arm-side "${ARM_SIDE}_arm" \
  --robot-version v1
