#!/bin/bash
# Build the acados shared library used by the runners.
#
# Run from any directory. The repository keeps acados as a git submodule; the
# script builds it in place and installs the Python interface into .venv.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

ACADOS_PIN="a58e76ad328d97aea180b55344e80eae346f8d15"

if [[ ! -d vendor/acados ]]; then
    git submodule update --init --recursive vendor/acados
fi
if [[ "$(git -C vendor/acados rev-parse HEAD)" != "$ACADOS_PIN" ]]; then
    echo "Unexpected acados version; preserve it and resolve the version explicitly." >&2
    exit 1
fi

uv pip install --python .venv/bin/python cmake==4.4.4 ninja==1.13.2
.venv/bin/cmake -S vendor/acados -B vendor/acados/build -G Ninja \
    -DCMAKE_MAKE_PROGRAM="$ROOT/.venv/bin/ninja" \
    -DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++ \
    -DCMAKE_BUILD_TYPE=Release -DBLASFEO_TARGET=GENERIC -DHPIPM_TARGET=GENERIC \
    -DACADOS_WITH_OPENMP=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
.venv/bin/cmake --build vendor/acados/build --target install -j 6
uv pip install --python .venv/bin/python -e vendor/acados/interfaces/acados_template
ACADOS_SOURCE_DIR="$ROOT/vendor/acados" .venv/bin/python -c \
    'from acados_template.utils import get_tera; print(get_tera())'
