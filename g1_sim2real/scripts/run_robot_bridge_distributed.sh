#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G1_SIM2REAL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${G1_SIM2REAL_ROOT}/.." && pwd)"
ROBOT_ENV_FILE="${ROBOT_DISTRIBUTED_ENV_FILE:-${REPO_ROOT}/robot_distributed.env}"

if [[ -f "${ROBOT_ENV_FILE}" ]]; then
  # shellcheck source=/dev/null
  source "${ROBOT_ENV_FILE}"
fi

: "${SERVER_CONTROL_IP:?Set SERVER_CONTROL_IP to the policy server control-network IPv4 address}"
: "${ROBOT_CONTROL_IP:?Set ROBOT_CONTROL_IP to the robot computer control-network IPv4 address}"
: "${G1_NET:?Set G1_NET to the robot-side G1 DDS interface name}"

export G1_BRIDGE_CONFIG="${G1_BRIDGE_CONFIG:-${G1_SIM2REAL_ROOT}/config/g1_bridge_teleop_upper_lower_locomani_distributed.yaml}"

echo "[DistributedBridge] G1 DDS interface=${G1_NET}"
echo "[DistributedBridge] state->${SERVER_CONTROL_IP}:55001"
echo "[DistributedBridge] command<-${SERVER_CONTROL_IP} on ${ROBOT_CONTROL_IP}:55002"

exec bash "${G1_SIM2REAL_ROOT}/scripts/run_bridge.sh" \
  --state-host "${SERVER_CONTROL_IP}" \
  --cmd-bind-host "${ROBOT_CONTROL_IP}" \
  --cmd-allowed-host "${SERVER_CONTROL_IP}" \
  "$@"
