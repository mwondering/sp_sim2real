#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM2REAL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${SIM2REAL_ROOT}"
exec uv run python teleop/serve_xrobot_teleop.py \
  --robot g1 \
  --config config/g1/retarget/teleop-server.yaml \
  "$@"
