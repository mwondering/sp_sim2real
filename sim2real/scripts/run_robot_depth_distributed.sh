#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM2REAL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

: "${ROBOT_CONTROL_IP:?Set ROBOT_CONTROL_IP to the robot computer control-network IPv4 address}"
DEPTH_PORT="${DEPTH_PORT:-28811}"
DEPTH_ENDPOINT="tcp://${ROBOT_CONTROL_IP}:${DEPTH_PORT}"

EXTRA_ARGS=()
if [[ -n "${D435I_SERIAL:-}" ]]; then
  EXTRA_ARGS+=(--serial-number "${D435I_SERIAL}")
fi
if [[ -n "${D435I_WORKER_PYTHON:-}" ]]; then
  EXTRA_ARGS+=(--worker-python "${D435I_WORKER_PYTHON}")
fi

PYTHON_CMD=()
if [[ -n "${ROBOT_DEPTH_PYTHON:-}" ]]; then
  if [[ ! -x "${ROBOT_DEPTH_PYTHON}" ]]; then
    echo "[DistributedDepth] ROBOT_DEPTH_PYTHON is not executable: ${ROBOT_DEPTH_PYTHON}" >&2
    exit 1
  fi
  PYTHON_CMD=("${ROBOT_DEPTH_PYTHON}")
elif [[ -x "${SIM2REAL_ROOT}/.venv_depth/bin/python" ]]; then
  PYTHON_CMD=("${SIM2REAL_ROOT}/.venv_depth/bin/python")
elif command -v uv >/dev/null 2>&1; then
  PYTHON_CMD=(uv run python)
else
  echo "[DistributedDepth] no Python runtime found; create .venv_depth or set ROBOT_DEPTH_PYTHON" >&2
  exit 1
fi

echo "[DistributedDepth] robot=${ROBOT_CONTROL_IP} bind=${DEPTH_ENDPOINT}"
echo "[DistributedDepth] python=${PYTHON_CMD[*]}"
cd "${SIM2REAL_ROOT}"
exec "${PYTHON_CMD[@]}" src/depth_camera_real.py \
  --config config/g1/teleop-upper-lower-locomani-real-distributed.yaml \
  --depth-bind "${DEPTH_ENDPOINT}" \
  "${EXTRA_ARGS[@]}" \
  "$@"
