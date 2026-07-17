#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
acados_root="${ACADOS_SOURCE_DIR:-$repo_root/autoware/deps/acados}"
expected_tag=v0.5.3
expected_commit=7e1d1152c1babd6ea04af1c9d73444fe8381057b
renderer_version=v0.2.0

if [[ -f "$acados_root/.romo_b_ready" && \
      -f "$acados_root/cmake/acadosConfig.cmake" && \
      -x "$acados_root/.venv/bin/python3" && \
      -x "$acados_root/bin/t_renderer" && \
      "$(git -C "$acados_root" rev-parse HEAD 2>/dev/null)" == "$expected_commit" ]]; then
  printf 'acados %s already ready at %s\n' "$expected_tag" "$acados_root"
  exit 0
fi

if [[ ! -d "$acados_root/.git" ]]; then
  mkdir -p "$(dirname "$acados_root")"
  git clone --branch "$expected_tag" --depth 1 --recurse-submodules \
    --shallow-submodules https://github.com/acados/acados.git "$acados_root"
fi
git -C "$acados_root" fetch --depth 1 origin "refs/tags/$expected_tag:refs/tags/$expected_tag"
if [[ "$(git -C "$acados_root" rev-list -n 1 "$expected_tag")" != "$expected_commit" ]]; then
  printf 'acados tag %s does not match dependencies/lock.yaml.\n' "$expected_tag" >&2
  exit 3
fi
git -C "$acados_root" checkout --detach "$expected_commit"
git -C "$acados_root" submodule update --init --recursive --depth 1

cmake -S "$acados_root" -B "$acados_root/build" \
  -DACADOS_WITH_QPOASES=ON \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$acados_root"
cmake --build "$acados_root/build" --parallel 2
cmake --install "$acados_root/build"

case "$(uname -m)" in
  x86_64) renderer_arch=amd64 ;;
  aarch64) renderer_arch=arm64 ;;
  *) printf 'Unsupported acados renderer architecture: %s\n' "$(uname -m)" >&2; exit 2 ;;
esac
mkdir -p "$acados_root/bin"
renderer="$acados_root/bin/t_renderer"
if [[ ! -x "$renderer" ]]; then
  curl --fail --location --retry 3 \
    "https://github.com/acados/tera_renderer/releases/download/$renderer_version/t_renderer-$renderer_version-linux-$renderer_arch" \
    --output "$renderer"
  chmod 0755 "$renderer"
fi

if [[ ! -x "$acados_root/.venv/bin/python3" || \
      ! -x "$acados_root/.venv/bin/pip" ]]; then
  if python3 -m venv "$acados_root/.venv" 2>/dev/null && \
     [[ -x "$acados_root/.venv/bin/pip" ]]; then
    :
  else
    # Ubuntu's minimal Python install may omit python3-venv. Keep this setup
    # reproducible and sudo-free by bootstrapping virtualenv inside the ignored
    # dependency directory instead of modifying the host Python installation.
    bootstrap_dir="$acados_root/.virtualenv-bootstrap"
    python3 -m pip install --disable-pip-version-check --upgrade \
      --target "$bootstrap_dir" virtualenv
    PYTHONPATH="$bootstrap_dir" python3 -m virtualenv --clear "$acados_root/.venv"
  fi
fi
"$acados_root/.venv/bin/pip" install --disable-pip-version-check \
  --upgrade pip casadi sympy
"$acados_root/.venv/bin/pip" install --disable-pip-version-check \
  --editable "$acados_root/interfaces/acados_template"

touch "$acados_root/.romo_b_ready"
printf 'acados %s ready at %s\n' "$expected_tag" "$acados_root"
