#!/usr/bin/env bash
set -euo pipefail

# pico branch entry point. It starts only the two external-host processes:
# RoboticsServiceProcess (for live PICO) and the reference/Viser server.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM2REAL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

SESSION="${SESSION:-g1_spv5_pico}"
SOURCE_MODE="${SOURCE_MODE:-pico}"       # pico | motion | raw-replay
REFERENCE_BIND_IP="${REFERENCE_BIND_IP:-0.0.0.0}"
SERVER_IP="${SERVER_IP:-127.0.0.1}"
VR_REQ_PORT="${VR_REQ_PORT:-28701}"
VR_POSE_PORT="${VR_POSE_PORT:-28702}"
VR_CTRL_PORT="${VR_CTRL_PORT:-28703}"
VIEWER_BIND_IP="${VIEWER_BIND_IP:-0.0.0.0}"
VIEWER_PORT="${VIEWER_PORT:-8080}"
MOTION_FILE="${MOTION_FILE:-}"
MOTION_ROOT="${MOTION_ROOT:-${SIM2REAL_ROOT}/../motion}"
MOTION_SELECT_BIND_IP="${MOTION_SELECT_BIND_IP:-127.0.0.1}"
MOTION_SELECT_CONNECT_IP="${MOTION_SELECT_CONNECT_IP:-127.0.0.1}"
MOTION_SELECT_PORT="${MOTION_SELECT_PORT:-28704}"
XR_SERVICE_DIR="${XR_SERVICE_DIR:-/opt/apps/roboticsservice}"

STOP=false
REPLACE=false
DETACH=false

usage() {
  cat <<'EOF'
Usage:
  bash scripts/launch_pico.sh --source pico
  bash scripts/launch_pico.sh --source motion
  bash scripts/launch_pico.sh --source raw-replay --motion-file /path/to/raw.npz

Options:
  --source pico|motion|raw-replay
  --motion-file PATH   Optional initial selection for motion; required for raw-replay
  --detach
  --replace
  --stop

Configurable environment variables:
  SESSION, REFERENCE_BIND_IP, SERVER_IP, VR_REQ_PORT, VR_POSE_PORT,
  VR_CTRL_PORT, VIEWER_BIND_IP, VIEWER_PORT, XR_SERVICE_DIR,
  MOTION_ROOT, MOTION_SELECT_BIND_IP, MOTION_SELECT_CONNECT_IP,
  MOTION_SELECT_PORT
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --source) SOURCE_MODE="${2:-}"; shift 2 ;;
    --source=*) SOURCE_MODE="${1#*=}"; shift ;;
    --motion-file) MOTION_FILE="${2:-}"; shift 2 ;;
    --motion-file=*) MOTION_FILE="${1#*=}"; shift ;;
    --detach) DETACH=true; shift ;;
    --replace) REPLACE=true; shift ;;
    --stop) STOP=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required" >&2
  exit 1
fi

if "${STOP}"; then
  if tmux has-session -t "${SESSION}" 2>/dev/null; then
    tmux send-keys -t "${SESSION}:reference" C-c 2>/dev/null || true
    tmux kill-session -t "${SESSION}"
  fi
  exit 0
fi

case "${SOURCE_MODE}" in
  pico|motion|raw-replay) ;;
  *) echo "Invalid SOURCE_MODE=${SOURCE_MODE}" >&2; exit 2 ;;
esac
if [[ "${SOURCE_MODE}" == "raw-replay" ]]; then
  if [[ -z "${MOTION_FILE}" ]]; then
    echo "${SOURCE_MODE} requires --motion-file PATH" >&2
    exit 2
  fi
  if [[ ! -f "${MOTION_FILE}" ]]; then
    echo "Motion file not found: ${MOTION_FILE}" >&2
    exit 1
  fi
  MOTION_FILE="$(realpath "${MOTION_FILE}")"
elif [[ "${SOURCE_MODE}" == "motion" ]]; then
  if [[ ! -d "${MOTION_ROOT}" ]]; then
    echo "Motion root not found: ${MOTION_ROOT}" >&2
    exit 1
  fi
  MOTION_ROOT="$(realpath "${MOTION_ROOT}")"
  if [[ -n "${MOTION_FILE}" ]]; then
    if [[ ! -f "${MOTION_FILE}" ]]; then
      echo "Motion file not found: ${MOTION_FILE}" >&2
      exit 1
    fi
    MOTION_FILE="$(realpath "${MOTION_FILE}")"
  fi
elif [[ ! -f "${XR_SERVICE_DIR}/runService.sh" ]]; then
  echo "XR service launcher not found: ${XR_SERVICE_DIR}/runService.sh" >&2
  exit 1
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  if ! "${REPLACE}"; then
    echo "tmux session already exists: ${SESSION}; use --replace or --stop" >&2
    exit 1
  fi
  tmux kill-session -t "${SESSION}"
fi

if [[ "${SOURCE_MODE}" == "pico" ]]; then
  printf -v xr_dir '%q' "${XR_SERVICE_DIR}"
  SOURCE_WINDOW="xr-service"
  SOURCE_CMD="cd ${xr_dir} && exec bash runService.sh"
  ATTACH_WINDOW="reference"
elif [[ "${SOURCE_MODE}" == "motion" ]]; then
  printf -v motion_root_q '%q' "${MOTION_ROOT}"
  printf -v select_addr_q '%q' "tcp://${MOTION_SELECT_CONNECT_IP}:${MOTION_SELECT_PORT}"
  SOURCE_WINDOW="motion-select"
  SOURCE_CMD="cd ${SIM2REAL_ROOT@Q} && exec uv run python teleop/motion_select.py --motion-root ${motion_root_q} --connect-addr ${select_addr_q}"
  ATTACH_WINDOW="motion-select"
else
  printf -v source_q '%q' "${SOURCE_MODE}"
  SOURCE_WINDOW="source-info"
  SOURCE_CMD="echo XR\ service\ is\ not\ required\ for\ SOURCE_MODE=${source_q}; exec \"\${SHELL:-/bin/bash}\""
  ATTACH_WINDOW="reference"
fi

printf -v sim_root_q '%q' "${SIM2REAL_ROOT}"
printf -v ref_cmd \
  'cd %s && SOURCE_MODE=%q MOTION_FILE=%q MOTION_ROOT=%q MOTION_SELECT_BIND_IP=%q MOTION_SELECT_PORT=%q REFERENCE_BIND_IP=%q VR_REQ_PORT=%q VR_POSE_PORT=%q VR_CTRL_PORT=%q VIEWER_BIND_IP=%q VIEWER_PORT=%q exec bash scripts/run_reference_server.sh' \
  "${sim_root_q}" "${SOURCE_MODE}" "${MOTION_FILE}" "${MOTION_ROOT}" \
  "${MOTION_SELECT_BIND_IP}" "${MOTION_SELECT_PORT}" "${REFERENCE_BIND_IP}" \
  "${VR_REQ_PORT}" "${VR_POSE_PORT}" "${VR_CTRL_PORT}" "${VIEWER_BIND_IP}" \
  "${VIEWER_PORT}"

tmux new-session -d -s "${SESSION}" -n "${SOURCE_WINDOW}"
tmux new-window -t "${SESSION}" -n reference
tmux set-option -t "${SESSION}" mouse on
tmux send-keys -t "${SESSION}:reference" "${ref_cmd}" C-m
tmux send-keys -t "${SESSION}:${SOURCE_WINDOW}" "${SOURCE_CMD}" C-m
tmux select-window -t "${SESSION}:${ATTACH_WINDOW}"

echo "PICO/reference session started: ${SESSION} (${SOURCE_WINDOW}, reference)"
echo "Reference viewer: http://${SERVER_IP}:${VIEWER_PORT}"
echo "Stop it: SESSION=${SESSION} bash scripts/launch_pico.sh --stop"
if ! "${DETACH}"; then
  exec tmux attach -t "${SESSION}"
fi
