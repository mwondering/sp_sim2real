#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM2REAL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

SOURCE_MODE="${SOURCE_MODE:-pico}"
REFERENCE_BIND_IP="${REFERENCE_BIND_IP:-0.0.0.0}"
VR_REQ_PORT="${VR_REQ_PORT:-28701}"
VR_POSE_PORT="${VR_POSE_PORT:-28702}"
VR_CTRL_PORT="${VR_CTRL_PORT:-28703}"
VIEWER_BIND_IP="${VIEWER_BIND_IP:-0.0.0.0}"
VIEWER_PORT="${VIEWER_PORT:-8080}"

COMMON_ARGS=(
  --req-bind-addr "tcp://${REFERENCE_BIND_IP}:${VR_REQ_PORT}"
  --rep-bind-addr "tcp://${REFERENCE_BIND_IP}:${VR_POSE_PORT}"
  --ctrl-bind-addr "tcp://${REFERENCE_BIND_IP}:${VR_CTRL_PORT}"
  --viewer-host "${VIEWER_BIND_IP}"
  --viewer-port "${VIEWER_PORT}"
)

cd "${SIM2REAL_ROOT}"
case "${SOURCE_MODE}" in
  pico)
    exec uv run python teleop/serve_xrobot_teleop.py \
      --robot g1 \
      --config config/g1/retarget/teleop.yaml \
      "${COMMON_ARGS[@]}" \
      "$@"
    ;;
  motion)
    if [[ -z "${MOTION_FILE:-}" ]]; then
      echo "SOURCE_MODE=motion requires MOTION_FILE=/path/to/motion.npz" >&2
      exit 2
    fi
    exec uv run python teleop/serve_motion_reference.py \
      "${MOTION_FILE}" \
      --config config/g1/retarget/teleop.yaml \
      --loop \
      "${COMMON_ARGS[@]}" \
      "$@"
    ;;
  raw-replay)
    if [[ -z "${MOTION_FILE:-}" ]]; then
      echo "SOURCE_MODE=raw-replay requires MOTION_FILE=/path/to/xrobot_raw.npz" >&2
      exit 2
    fi
    exec uv run python teleop/replay_xrobot_motion.py \
      "${MOTION_FILE}" \
      --robot g1 \
      --zmq \
      --loop \
      "${COMMON_ARGS[@]}" \
      "$@"
    ;;
  *)
    echo "SOURCE_MODE must be pico, motion, or raw-replay; got: ${SOURCE_MODE}" >&2
    exit 2
    ;;
esac
