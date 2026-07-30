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

echo "[DistributedDepth] robot=${ROBOT_CONTROL_IP} bind=${DEPTH_ENDPOINT}"
cd "${SIM2REAL_ROOT}"
exec uv run python src/depth_camera_real.py \
  --config config/g1/teleop-upper-lower-locomani-real-distributed.yaml \
  --depth-bind "${DEPTH_ENDPOINT}" \
  "${EXTRA_ARGS[@]}" \
  "$@"
