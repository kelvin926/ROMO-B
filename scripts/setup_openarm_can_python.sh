#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROMO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
VENV_DIR="${ROMO_ROOT}/openarm/.venv"
PYTHON_DIR="${ROMO_ROOT}/openarm/src/openarm_can/python"

# Ubuntu's python3-venv may be present without the versioned ensurepip wheel.
# The distro pip remains importable through system-site-packages, so this also
# works on a fresh Jammy host without requiring a second privileged package
# installation merely to bootstrap pip inside this project-local environment.
python3 -m venv --without-pip --system-site-packages "${VENV_DIR}"

if ! "${VENV_DIR}/bin/python3" -m pip --version >/dev/null 2>&1; then
  printf 'ERROR: python3-pip is required to prepare OpenArm bindings.\n' >&2
  exit 1
fi

"${VENV_DIR}/bin/python3" -m pip install \
  --disable-pip-version-check \
  "scikit-build-core==0.10.7" \
  "nanobind==2.10.2"

"${VENV_DIR}/bin/python3" -m pip install \
  --disable-pip-version-check \
  --no-build-isolation \
  --no-deps \
  "${PYTHON_DIR}"

"${VENV_DIR}/bin/python3" - <<'PY'
import openarm_can

print(f"OpenArm CAN Python 준비 완료: {openarm_can.__version__}")
PY
