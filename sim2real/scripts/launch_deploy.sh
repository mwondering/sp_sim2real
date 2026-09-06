#!/usr/bin/env bash
set -euo pipefail

# Unified SPV5-2 deployment launcher. The default remains an external PICO
# reference host; --onboard-pico adds XR Service and retargeting on the G1.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

SESSION="${SESSION:-g1_spv5_deploy}"
TARGET="${TARGET:-real}"                               # real | sim
SOURCE_MODE="${SOURCE_MODE:-pico}"                     # pico | motion | motion-vis
PICO_RUNTIME="${PICO_RUNTIME:-external}"               # external | onboard
REFERENCE_HOST="${REFERENCE_HOST:-${G1_REF_HOST:-${SERVER_WIFI_IP:-}}}"
REFERENCE_BIND_IP="${REFERENCE_BIND_IP:-127.0.0.1}"
VR_REQ_PORT="${VR_REQ_PORT:-28701}"
VR_POSE_PORT="${VR_POSE_PORT:-28702}"
VR_CTRL_PORT="${VR_CTRL_PORT:-28703}"
REF_BUFFER_DELAY_S="${REF_BUFFER_DELAY_S:-}"
RETARGET_LOOKBACK_MS="${RETARGET_LOOKBACK_MS:-}"
PICO_PROJECT_DIR="${PICO_PROJECT_DIR:-${REPO_ROOT}/sim2real/venv/pico}"
XR_SERVICE_SCRIPT="${XR_SERVICE_SCRIPT:-/opt/apps/roboticsservice/runService.sh}"
XR_SERVICE_BIN="${XR_SERVICE_BIN:-}"
VIEWER_ENABLED="${VIEWER_ENABLED:-true}"
VIEWER_BIND_IP="${VIEWER_BIND_IP:-0.0.0.0}"
VIEWER_PORT="${VIEWER_PORT:-8080}"
VIEWER_URL_HOST="${VIEWER_URL_HOST:-<G1-IP>}"
MOTION_ROOT="${MOTION_ROOT:-${REPO_ROOT}/motion}"
MOTION_SELECT_HOST="${MOTION_SELECT_HOST:-127.0.0.1}"
MOTION_SELECT_PORT="${MOTION_SELECT_PORT:-28562}"
MOTION_REFERENCE_SELECT_PORT="${MOTION_REFERENCE_SELECT_PORT:-28704}"
STATE_PORT="${STATE_PORT:-55001}"
CMD_PORT="${CMD_PORT:-55002}"
G1_DDS_IFACE="${G1_DDS_IFACE:-eth0}"
G1_BRIDGE_BUILD_DIR="${G1_BRIDGE_BUILD_DIR:-build_onboard}"
RETARGET_CPU_SET="${RETARGET_CPU_SET:-0-1}"
XR_SERVICE_CPU_SET="${XR_SERVICE_CPU_SET:-}"
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
  bash scripts/launch_deploy.sh --real --onboard-pico
  REFERENCE_HOST=<pico-host-ip> bash scripts/launch_deploy.sh --real
  bash scripts/launch_deploy.sh --real --source motion
  bash scripts/launch_deploy.sh --real --source motion-vis

Options:
  --real | --sim
      Select G1 or MuJoCo deployment.
  --source pico|motion|motion-vis
      pico: live PICO reference (default).
      motion: lowest-overhead onboard NPZ playback, without a reference viewer.
      motion-vis: onboard NPZ reference server plus MJViser visualization.
  --pico-runtime external|onboard
      Place XR Service and PICO retargeting externally or on this machine.
  --onboard-pico
      Shortcut for --source pico --pico-runtime onboard.
  --external-pico
      Shortcut for --source pico --pico-runtime external.
  --viewer | --no-viewer
      Enable or disable reference MJViser (enabled by default).
  --component bridge|policy|xr-service|reference|motion-select
      Run one component without creating tmux.
  --detach
      Create the tmux session without attaching.
  --replace
      Replace an existing session with the same name.
  --stop
      Stop the deploy tmux session.
  --yes
      Skip the real-robot confirmation prompt.

Important environment variables:
  SESSION, REFERENCE_HOST, VR_REQ_PORT, VR_POSE_PORT, VR_CTRL_PORT,
  PICO_RUNTIME, PICO_PROJECT_DIR, XR_SERVICE_SCRIPT, XR_SERVICE_BIN,
  REF_BUFFER_DELAY_S, RETARGET_LOOKBACK_MS,
  VIEWER_BIND_IP, VIEWER_PORT, VIEWER_URL_HOST,
  MOTION_ROOT, MOTION_SELECT_HOST, MOTION_SELECT_PORT,
  MOTION_REFERENCE_SELECT_PORT, STATE_PORT, CMD_PORT,
  G1_DDS_IFACE, G1_BRIDGE_BUILD_DIR,
  RETARGET_CPU_SET, XR_SERVICE_CPU_SET, BRIDGE_CPU_SET, POLICY_CPU_SET
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --real) TARGET=real; shift ;;
    --sim) TARGET=sim; shift ;;
    --source) SOURCE_MODE="${2:-}"; shift 2 ;;
    --source=*) SOURCE_MODE="${1#*=}"; shift ;;
    --pico-runtime) PICO_RUNTIME="${2:-}"; shift 2 ;;
    --pico-runtime=*) PICO_RUNTIME="${1#*=}"; shift ;;
    --onboard-pico) SOURCE_MODE=pico; PICO_RUNTIME=onboard; shift ;;
    --external-pico) SOURCE_MODE=pico; PICO_RUNTIME=external; shift ;;
    --viewer) VIEWER_ENABLED=true; shift ;;
    --no-viewer) VIEWER_ENABLED=false; shift ;;
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
case "${SOURCE_MODE}" in
  pico|motion|motion-vis) ;;
  *) echo "--source must be pico, motion, or motion-vis" >&2; exit 2 ;;
esac
case "${PICO_RUNTIME}" in
  external|onboard) ;;
  *) echo "--pico-runtime must be external or onboard" >&2; exit 2 ;;
esac
case "${VIEWER_ENABLED}" in
  true|false) ;;
  *) echo "VIEWER_ENABLED must be true or false" >&2; exit 2 ;;
esac
case "${COMPONENT}" in
  ""|bridge|policy|xr-service|reference|motion-select) ;;
  *) echo "--component must be bridge, policy, xr-service, reference, or motion-select" >&2; exit 2 ;;
esac

if "${STOP}"; then
  if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux is required" >&2
    exit 1
  fi
  if tmux has-session -t "${SESSION}" 2>/dev/null; then
    for window in policy bridge reference xr-service motion-select; do
      tmux send-keys -t "${SESSION}:${window}" C-c 2>/dev/null || true
    done
    tmux kill-session -t "${SESSION}"
  fi
  exit 0
fi

POLICY_SOURCE_MODE="${SOURCE_MODE}"
REFERENCE_SOURCE_MODE=""
USE_ONBOARD_REFERENCE=false
USE_XR_SERVICE=false

case "${SOURCE_MODE}" in
  pico)
    if [[ "${PICO_RUNTIME}" == "onboard" ]]; then
      USE_ONBOARD_REFERENCE=true
      USE_XR_SERVICE=true
      REFERENCE_SOURCE_MODE=pico
      REFERENCE_HOST=127.0.0.1
      # MimicLite uses latest-only local transport. Remove wireless jitter
      # buffering and the retarget lookback by default for the onboard path.
      REF_BUFFER_DELAY_S="${REF_BUFFER_DELAY_S:-0.0}"
      RETARGET_LOOKBACK_MS="${RETARGET_LOOKBACK_MS:-0.0}"
    elif [[ "${TARGET}" == "sim" && -z "${REFERENCE_HOST}" ]]; then
      REFERENCE_HOST=127.0.0.1
    fi
    ;;
  motion)
    POLICY_SOURCE_MODE=motion
    ;;
  motion-vis)
    # Reuse the policy's reference protocol so playback and MJViser observe
    # exactly the same qpos stream.
    POLICY_SOURCE_MODE=pico
    USE_ONBOARD_REFERENCE=true
    REFERENCE_SOURCE_MODE=motion
    REFERENCE_HOST=127.0.0.1
    REF_BUFFER_DELAY_S="${REF_BUFFER_DELAY_S:-0.0}"
    ;;
esac

if [[ "${SOURCE_MODE}" == "pico" && "${PICO_RUNTIME}" == "external" &&
      "${COMPONENT}" != "bridge" && -z "${REFERENCE_HOST}" ]]; then
  echo "REFERENCE_HOST is required for external PICO mode" >&2
  echo "Use --onboard-pico to run XR Service and retargeting on the G1" >&2
  exit 2
fi

if [[ "${SOURCE_MODE}" == "motion" || "${SOURCE_MODE}" == "motion-vis" ]]; then
  if [[ ! -d "${MOTION_ROOT}" ]]; then
    echo "Onboard motion root not found: ${MOTION_ROOT}" >&2
    exit 1
  fi
  MOTION_ROOT="$(realpath "${MOTION_ROOT}")"
fi

require_pico_environment() {
  local pico_python="${PICO_PROJECT_DIR}/.venv/bin/python"
  if [[ ! -x "${pico_python}" ]]; then
    echo "PICO environment not found: ${pico_python}" >&2
    echo "Run: uv sync --project ${PICO_PROJECT_DIR}" >&2
    exit 1
  fi
  if [[ ! -f "${REPO_ROOT}/sim2real/config/g1/retarget/teleop.yaml" ]]; then
    echo "Missing onboard retarget config" >&2
    exit 1
  fi
  if ! "${pico_python}" -c 'import mujoco, mink, mjviser, zmq' >/dev/null 2>&1; then
    echo "PICO environment is incomplete: ${PICO_PROJECT_DIR}" >&2
    echo "Run: uv sync --project ${PICO_PROJECT_DIR}" >&2
    exit 1
  fi
  if "${USE_XR_SERVICE}" && ! "${pico_python}" -c 'import xrobotoolkit_sdk' >/dev/null 2>&1; then
    echo "xrobotoolkit_sdk is missing from the PICO environment" >&2
    echo "Run: cd ${REPO_ROOT}/sim2real && bash install_xrobottoolkit_sdk.sh" >&2
    exit 1
  fi
}

if "${USE_ONBOARD_REFERENCE}" && [[ "${COMPONENT}" != "bridge" && "${COMPONENT}" != "policy" ]]; then
  require_pico_environment
fi

xr_service_library_path() {
  local service_dir="$1"
  local library_path="${service_dir}:${service_dir}/lib:${service_dir}/SDK/arm64"
  if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
    library_path="${LD_LIBRARY_PATH}:${library_path}"
  fi
  printf '%s' "${library_path}"
}

check_xr_service_runtime() {
  local service_dir="$1"
  local service_bin="$2"
  local library_path
  library_path="$(xr_service_library_path "${service_dir}")"
  local ldd_output
  local ldd_status=0

  set +e
  ldd_output="$(env LD_LIBRARY_PATH="${library_path}" ldd "${service_bin}" 2>&1)"
  ldd_status=$?
  set -e

  if (( ldd_status != 0 )) || grep -Eq \
      'not found|cannot open shared object file' <<<"${ldd_output}"; then
    echo "XRoboToolkit PC Service is incompatible with this system:" >&2
    printf '%s\n' "${ldd_output}" >&2
    echo >&2
    echo "G1 Ubuntu 20.04 requires:" >&2
    echo "  XRoboToolkit-PC-Service_1.0.0.0_arm64_ubuntu20.04.deb" >&2
    echo "Do not symlink libicuuc.so.66 to libicuuc.so.70." >&2
    echo "Install the compatible package with:" >&2
    echo "  bash install_xrobottoolkit_pc_service.sh /path/to/package.deb" >&2
    exit 1
  fi
}

if "${USE_XR_SERVICE}" && [[ "${COMPONENT}" == "" || "${COMPONENT}" == "xr-service" ]]; then
  if [[ ! -f "${XR_SERVICE_SCRIPT}" ]]; then
    echo "XRoboToolkit service launcher not found: ${XR_SERVICE_SCRIPT}" >&2
    echo "Install the ARM64 XRoboToolkit PC Service package first" >&2
    exit 1
  fi
  xr_service_dir="$(dirname "${XR_SERVICE_SCRIPT}")"
  xr_service_bin="${XR_SERVICE_BIN:-${xr_service_dir}/RoboticsServiceProcess}"
  if [[ ! -x "${xr_service_bin}" ]]; then
    echo "XRoboToolkit service binary not found or not executable: ${xr_service_bin}" >&2
    echo "Install the ARM64 XRoboToolkit PC Service package first" >&2
    exit 1
  fi
  check_xr_service_runtime "${xr_service_dir}" "${xr_service_bin}"
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
    "G1_MOTION_SOURCE=${POLICY_SOURCE_MODE}"
    "G1_MOTION_ROOT=${MOTION_ROOT}"
    "G1_MOTION_SELECT_HOST=${MOTION_SELECT_HOST}"
    "G1_MOTION_SELECT_PORT=${MOTION_SELECT_PORT}"
    "G1_STATE_PORT=${STATE_PORT}"
    "G1_CMD_PORT=${CMD_PORT}"
  )
  if [[ -n "${REF_BUFFER_DELAY_S}" ]]; then
    command+=("G1_REF_BUFFER_DELAY_S=${REF_BUFFER_DELAY_S}")
  fi
  if [[ "${TARGET}" == "real" ]]; then
    command+=(taskset -c "${POLICY_CPU_SET}")
  fi
  command+=(uv run src/deploy.py --robot g1 --tracking-config tracking_spv5_2.yaml)
  exec "${command[@]}"
}

run_xr_service() {
  if ! "${USE_XR_SERVICE}"; then
    echo "xr-service is available only in onboard PICO mode" >&2
    exit 2
  fi
  local service_dir
  service_dir="$(cd "$(dirname "${XR_SERVICE_SCRIPT}")" && pwd)"
  local service_bin="${XR_SERVICE_BIN:-${service_dir}/RoboticsServiceProcess}"

  # The vendor ARM64 runService.sh backgrounds RoboticsServiceProcess and then
  # exits successfully. Running the binary in the foreground keeps tmux, Ctrl-C,
  # and service lifetime coupled while preserving the wrapper's runtime paths.
  local service_library_path
  service_library_path="$(xr_service_library_path "${service_dir}")"
  export LD_LIBRARY_PATH="${service_library_path}"
  export QT_PLUGIN_PATH="${service_dir}/plugins/${QT_PLUGIN_PATH:+:${QT_PLUGIN_PATH}}"
  export QT_QML_PATH="${service_dir}/qml/${QT_QML_PATH:+:${QT_QML_PATH}}"

  cd "${service_dir}"
  if [[ -n "${XR_SERVICE_CPU_SET}" ]]; then
    exec taskset -c "${XR_SERVICE_CPU_SET}" "${service_bin}"
  fi
  exec "${service_bin}"
}

run_reference() {
  if ! "${USE_ONBOARD_REFERENCE}"; then
    echo "reference is available with --onboard-pico or --source motion-vis" >&2
    exit 2
  fi
  require_pico_environment

  cd "${REPO_ROOT}/sim2real"
  local -a command=(
    env
    "SOURCE_MODE=${REFERENCE_SOURCE_MODE}"
    "PICO_PROJECT_DIR=${PICO_PROJECT_DIR}"
    "REFERENCE_BIND_IP=${REFERENCE_BIND_IP}"
    "VR_REQ_PORT=${VR_REQ_PORT}"
    "VR_POSE_PORT=${VR_POSE_PORT}"
    "VR_CTRL_PORT=${VR_CTRL_PORT}"
    "RETARGET_LOOKBACK_MS=${RETARGET_LOOKBACK_MS}"
    "VIEWER_BIND_IP=${VIEWER_BIND_IP}"
    "VIEWER_PORT=${VIEWER_PORT}"
    "MOTION_ROOT=${MOTION_ROOT}"
    "MOTION_SELECT_BIND_IP=127.0.0.1"
    "MOTION_SELECT_PORT=${MOTION_REFERENCE_SELECT_PORT}"
  )
  if [[ "${TARGET}" == "real" && -n "${RETARGET_CPU_SET}" ]]; then
    command+=(taskset -c "${RETARGET_CPU_SET}")
  fi
  command+=(bash scripts/run_reference_server.sh)
  if ! "${VIEWER_ENABLED}"; then
    command+=(--no-viewer)
  fi
  exec "${command[@]}"
}

run_motion_select() {
  case "${SOURCE_MODE}" in
    motion)
      cd "${REPO_ROOT}/sim2real"
      exec env \
        G1_MOTION_ROOT="${MOTION_ROOT}" \
        G1_MOTION_SELECT_HOST="${MOTION_SELECT_HOST}" \
        G1_MOTION_SELECT_PORT="${MOTION_SELECT_PORT}" \
        uv run src/motion_select.py
      ;;
    motion-vis)
      require_pico_environment
      cd "${REPO_ROOT}/sim2real"
      exec uv run --project "${PICO_PROJECT_DIR}" --no-sync python \
        teleop/motion_select.py \
        --motion-root "${MOTION_ROOT}" \
        --connect-addr "tcp://127.0.0.1:${MOTION_REFERENCE_SELECT_PORT}"
      ;;
    *)
      echo "motion-select is available only with --source motion or motion-vis" >&2
      exit 2
      ;;
  esac
}

case "${COMPONENT}" in
  bridge) run_bridge ;;
  policy) run_policy ;;
  xr-service) run_xr_service ;;
  reference) run_reference ;;
  motion-select) run_motion_select ;;
esac

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

common_env=""
append_env() {
  local quoted
  printf -v quoted '%q' "$2"
  common_env+="$1=${quoted} "
}
append_env SOURCE_MODE "${SOURCE_MODE}"
append_env PICO_RUNTIME "${PICO_RUNTIME}"
append_env REFERENCE_HOST "${REFERENCE_HOST}"
append_env REFERENCE_BIND_IP "${REFERENCE_BIND_IP}"
append_env VR_REQ_PORT "${VR_REQ_PORT}"
append_env VR_POSE_PORT "${VR_POSE_PORT}"
append_env VR_CTRL_PORT "${VR_CTRL_PORT}"
append_env REF_BUFFER_DELAY_S "${REF_BUFFER_DELAY_S}"
append_env RETARGET_LOOKBACK_MS "${RETARGET_LOOKBACK_MS}"
append_env PICO_PROJECT_DIR "${PICO_PROJECT_DIR}"
append_env XR_SERVICE_SCRIPT "${XR_SERVICE_SCRIPT}"
append_env XR_SERVICE_BIN "${XR_SERVICE_BIN}"
append_env VIEWER_ENABLED "${VIEWER_ENABLED}"
append_env VIEWER_BIND_IP "${VIEWER_BIND_IP}"
append_env VIEWER_PORT "${VIEWER_PORT}"
append_env VIEWER_URL_HOST "${VIEWER_URL_HOST}"
append_env MOTION_ROOT "${MOTION_ROOT}"
append_env MOTION_SELECT_HOST "${MOTION_SELECT_HOST}"
append_env MOTION_SELECT_PORT "${MOTION_SELECT_PORT}"
append_env MOTION_REFERENCE_SELECT_PORT "${MOTION_REFERENCE_SELECT_PORT}"
append_env STATE_PORT "${STATE_PORT}"
append_env CMD_PORT "${CMD_PORT}"
append_env G1_DDS_IFACE "${G1_DDS_IFACE}"
append_env G1_BRIDGE_BUILD_DIR "${G1_BRIDGE_BUILD_DIR}"
append_env RETARGET_CPU_SET "${RETARGET_CPU_SET}"
append_env XR_SERVICE_CPU_SET "${XR_SERVICE_CPU_SET}"
append_env BRIDGE_CPU_SET "${BRIDGE_CPU_SET}"
append_env POLICY_CPU_SET "${POLICY_CPU_SET}"
printf -v script_path '%q' "${SCRIPT_DIR}/launch_deploy.sh"

component_command() {
  local component_quoted
  printf -v component_quoted '%q' "$1"
  # Supplying a tmux shell-command makes its parent shell non-interactive.
  # The explicit bash flags also prevent machine-specific ROS prompts in
  # ~/.bashrc or profile files from consuming the deployment command.
  printf '%sexec bash --noprofile --norc %s --%s --component %s' \
    "${common_env}" "${script_path}" "${TARGET}" "${component_quoted}"
}

first_component=bridge
if "${USE_XR_SERVICE}"; then
  first_component=xr-service
elif "${USE_ONBOARD_REFERENCE}"; then
  first_component=reference
fi
tmux new-session -d -s "${SESSION}" -n "${first_component}" \
  "$(component_command "${first_component}")"
tmux set-window-option -t "${SESSION}:${first_component}" remain-on-exit on
tmux set-option -t "${SESSION}" mouse on

create_component_window() {
  local component="$1"
  if [[ "${component}" == "${first_component}" ]]; then
    return
  fi
  tmux new-window -d -t "${SESSION}" -n "${component}" \
    "$(component_command "${component}")"
  tmux set-window-option -t "${SESSION}:${component}" remain-on-exit on 2>/dev/null || true
}

if "${USE_XR_SERVICE}"; then
  create_component_window xr-service
fi
if "${USE_ONBOARD_REFERENCE}"; then
  create_component_window reference
fi
create_component_window bridge
create_component_window policy
if [[ "${SOURCE_MODE}" == "motion" || "${SOURCE_MODE}" == "motion-vis" ]]; then
  create_component_window motion-select
  tmux select-window -t "${SESSION}:motion-select"
elif "${USE_ONBOARD_REFERENCE}"; then
  tmux select-window -t "${SESSION}:reference"
else
  tmux select-window -t "${SESSION}:policy"
fi

if "${USE_XR_SERVICE}"; then
  echo "Deploy session started: ${SESSION} (xr-service, reference, bridge, policy; onboard PICO)"
elif [[ "${SOURCE_MODE}" == "motion-vis" ]]; then
  echo "Deploy session started: ${SESSION} (reference, bridge, policy, motion-select; visualized motion)"
elif [[ "${SOURCE_MODE}" == "motion" ]]; then
  echo "Deploy session started: ${SESSION} (bridge, policy, motion-select; direct onboard motion)"
else
  echo "Deploy session started: ${SESSION} (bridge, policy; external PICO)"
fi
if "${USE_ONBOARD_REFERENCE}" && "${VIEWER_ENABLED}"; then
  echo "Reference viewer: http://${VIEWER_URL_HOST}:${VIEWER_PORT}"
fi
echo "Stop it: SESSION=${SESSION} bash scripts/launch_deploy.sh --stop"
if ! "${DETACH}"; then
  exec tmux attach -t "${SESSION}"
fi
