#!/usr/bin/env bash
set -euo pipefail

# deploy branch entry point. It starts the two processes owned by the deploy side:
# the robot/simulator bridge and the SPV5-2 policy runtime.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

SESSION="${SESSION:-g1_spv5_deploy}"
TARGET="${TARGET:-real}"                       # real | sim
REFERENCE_HOST="${REFERENCE_HOST:-${G1_REF_HOST:-${SERVER_WIFI_IP:-}}}"
VR_REQ_PORT="${VR_REQ_PORT:-28701}"
VR_POSE_PORT="${VR_POSE_PORT:-28702}"
VR_CTRL_PORT="${VR_CTRL_PORT:-28703}"
STATE_PORT="${STATE_PORT:-55001}"
CMD_PORT="${CMD_PORT:-55002}"
G1_DDS_IFACE="${G1_DDS_IFACE:-eth0}"
G1_BRIDGE_BUILD_DIR="${G1_BRIDGE_BUILD_DIR:-build_onboard}"
BRIDGE_CPU_SET="${BRIDGE_CPU_SET:-2-3}"
POLICY_CPU_SET="${POLICY_CPU_SET:-4-7}"

COMPONENT=""
STOP=false
REPLACE=false
DETACH=false
ASSUME_YES=false

usage() {
  cat <<'EOF'
Usage:
  REFERENCE_HOST=<pico-host-ip> bash scripts/launch_deploy.sh --real
  REFERENCE_HOST=127.0.0.1       bash scripts/launch_deploy.sh --sim

Options:
  --real | --sim                  Select G1 or MuJoCo deployment.
  --component bridge|policy       Run one component without creating tmux.
  --detach                        Create the two-window session without attaching.
  --replace                       Replace an existing session with the same name.
  --stop                          Stop the deploy tmux session.
  --yes                           Skip the real-robot confirmation prompt.

Configurable environment variables:
  SESSION, REFERENCE_HOST, VR_REQ_PORT, VR_POSE_PORT, VR_CTRL_PORT,
  STATE_PORT, CMD_PORT, G1_DDS_IFACE, G1_BRIDGE_BUILD_DIR,
  BRIDGE_CPU_SET, POLICY_CPU_SET
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --real) TARGET=real; shift ;;
    --sim) TARGET=sim; shift ;;
    --component) COMPONENT="${2:-}"; shift 2 ;;
    --component=*) COMPONENT="${1#*=}"; shift ;;
    --detach) DETACH=true; shift ;;
    --replace) REPLACE=true; shift ;;
    --stop) STOP=true; shift ;;
    --yes) ASSUME_YES=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

case "${TARGET}" in
  real|sim) ;;
  *) echo "TARGET must be real or sim" >&2; exit 2 ;;
esac
case "${COMPONENT}" in
  ""|bridge|policy) ;;
  *) echo "--component must be bridge or policy" >&2; exit 2 ;;
esac

if "${STOP}"; then
  if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux is required" >&2
    exit 1
  fi
  if tmux has-session -t "${SESSION}" 2>/dev/null; then
    tmux send-keys -t "${SESSION}:policy" C-c 2>/dev/null || true
    tmux send-keys -t "${SESSION}:bridge" C-c 2>/dev/null || true
    tmux kill-session -t "${SESSION}"
  fi
  exit 0
fi

if [[ "${TARGET}" == "sim" && -z "${REFERENCE_HOST}" ]]; then
  REFERENCE_HOST="127.0.0.1"
fi
if [[ "${COMPONENT}" != "bridge" && -z "${REFERENCE_HOST}" ]]; then
  echo "REFERENCE_HOST is required; set it to the external pico host IP" >&2
  exit 2
fi

run_bridge() {
  if [[ "${TARGET}" == "real" ]]; then
    cd "${REPO_ROOT}/g1_sim2real"
    exec env \
      G1_NET="${G1_DDS_IFACE}" \
      G1_BRIDGE_BUILD_DIR="${G1_BRIDGE_BUILD_DIR}" \
      taskset -c "${BRIDGE_CPU_SET}" \
      bash scripts/run_bridge.sh \
        --state-host 127.0.0.1 --state-port "${STATE_PORT}" \
        --cmd-bind-host 127.0.0.1 --cmd-port "${CMD_PORT}" \
        --cmd-allowed-host 127.0.0.1
  fi

  cd "${REPO_ROOT}/sim2real"
  exec env \
    G1_STATE_PORT="${STATE_PORT}" \
    G1_CMD_PORT="${CMD_PORT}" \
    uv run src/sim2sim.py --robot g1
}

run_policy() {
  cd "${REPO_ROOT}/sim2real"
  local -a command=(
    env
    "G1_REF_HOST=${REFERENCE_HOST}"
    "G1_REF_REQ_PORT=${VR_REQ_PORT}"
    "G1_REF_POSE_PORT=${VR_POSE_PORT}"
    "G1_REF_CTRL_PORT=${VR_CTRL_PORT}"
    "G1_STATE_PORT=${STATE_PORT}"
    "G1_CMD_PORT=${CMD_PORT}"
  )
  if [[ "${TARGET}" == "real" ]]; then
    command+=(taskset -c "${POLICY_CPU_SET}")
  fi
  command+=(uv run src/deploy.py --robot g1 --tracking-config tracking_spv5_2.yaml)
  exec "${command[@]}"
}

if [[ "${COMPONENT}" == "bridge" ]]; then
  run_bridge
elif [[ "${COMPONENT}" == "policy" ]]; then
  run_policy
fi

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required" >&2
  exit 1
fi

if [[ "${TARGET}" == "real" ]]; then
  if [[ "${G1_BRIDGE_BUILD_DIR}" = /* ]]; then
    bridge_bin="${G1_BRIDGE_BUILD_DIR}/g1_udp_bridge"
  else
    bridge_bin="${REPO_ROOT}/g1_sim2real/${G1_BRIDGE_BUILD_DIR}/g1_udp_bridge"
  fi
  if [[ ! -x "${bridge_bin}" ]]; then
    echo "Bridge binary not found: ${bridge_bin}" >&2
    echo "Build it first: cd ${REPO_ROOT}/g1_sim2real && G1_BRIDGE_BUILD_DIR=${G1_BRIDGE_BUILD_DIR} bash scripts/build.sh" >&2
    exit 1
  fi
  if ! "${ASSUME_YES}"; then
    echo "This starts SPV5-2 motor control on the real G1."
    read -r -p "Type RUN to continue: " answer
    [[ "${answer}" == "RUN" ]] || exit 1
  fi
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  if ! "${REPLACE}"; then
    echo "tmux session already exists: ${SESSION}; use --replace or --stop" >&2
    exit 1
  fi
  tmux kill-session -t "${SESSION}"
fi

printf -v common_env \
  'REFERENCE_HOST=%q VR_REQ_PORT=%q VR_POSE_PORT=%q VR_CTRL_PORT=%q STATE_PORT=%q CMD_PORT=%q G1_DDS_IFACE=%q G1_BRIDGE_BUILD_DIR=%q BRIDGE_CPU_SET=%q POLICY_CPU_SET=%q' \
  "${REFERENCE_HOST}" "${VR_REQ_PORT}" "${VR_POSE_PORT}" "${VR_CTRL_PORT}" \
  "${STATE_PORT}" "${CMD_PORT}" "${G1_DDS_IFACE}" "${G1_BRIDGE_BUILD_DIR}" \
  "${BRIDGE_CPU_SET}" "${POLICY_CPU_SET}"
printf -v script_path '%q' "${SCRIPT_DIR}/launch_deploy.sh"

tmux new-session -d -s "${SESSION}" -n bridge
tmux new-window -t "${SESSION}" -n policy
tmux set-option -t "${SESSION}" mouse on
tmux send-keys -t "${SESSION}:bridge" "${common_env} bash ${script_path} --${TARGET} --component bridge" C-m
tmux send-keys -t "${SESSION}:policy" "${common_env} bash ${script_path} --${TARGET} --component policy" C-m

echo "Deploy session started: ${SESSION} (bridge, policy)"
echo "Stop it: SESSION=${SESSION} bash scripts/launch_deploy.sh --stop"
if ! "${DETACH}"; then
  exec tmux attach -t "${SESSION}"
fi
