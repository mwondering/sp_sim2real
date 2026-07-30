#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM2REAL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

: "${SERVER_CONTROL_IP:?Set SERVER_CONTROL_IP to the policy server control-network IPv4 address}"
: "${ROBOT_CONTROL_IP:?Set ROBOT_CONTROL_IP to the robot computer control-network IPv4 address}"
DEPTH_PORT="${DEPTH_PORT:-28811}"
DEPTH_ENDPOINT="tcp://${ROBOT_CONTROL_IP}:${DEPTH_PORT}"

if command -v ping >/dev/null 2>&1; then
  if ! ping -c 1 -W 1 "${ROBOT_CONTROL_IP}" >/dev/null 2>&1; then
    echo "[DistributedPolicy] robot ${ROBOT_CONTROL_IP} is not reachable" >&2
    exit 1
  fi
fi

echo "[DistributedPolicy] server=${SERVER_CONTROL_IP} robot=${ROBOT_CONTROL_IP}"
echo "[DistributedPolicy] state=udp://${SERVER_CONTROL_IP}:55001"
echo "[DistributedPolicy] command=udp://${ROBOT_CONTROL_IP}:55002"
echo "[DistributedPolicy] depth=${DEPTH_ENDPOINT}"

cd "${SIM2REAL_ROOT}"
exec uv run python src/teleop_upper_lower_locomani.py \
  --target real \
  --whole-body-policy heft \
  --terrain-class 2 \
  --task-config config/g1/teleop-upper-lower-locomani-real-distributed.yaml \
  --controller-config config/g1/controller-distributed.yaml \
  --state-bind-host "${SERVER_CONTROL_IP}" \
  --cmd-host "${ROBOT_CONTROL_IP}" \
  --depth-connect "${DEPTH_ENDPOINT}" \
  "$@"
