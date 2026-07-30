#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${SCRIPT_DIR}"
SIM2REAL_ROOT="${REPO_ROOT}/sim2real"
G1_SIM2REAL_ROOT="${REPO_ROOT}/g1_sim2real"
DEPTH_VENV="${SIM2REAL_ROOT}/.venv_depth"
ENV_FILE="${REPO_ROOT}/robot_distributed.env"

SERVER_IP="${SERVER_CONTROL_IP:-10.42.0.1}"
ROBOT_IP="${ROBOT_CONTROL_IP:-10.42.0.2}"
G1_INTERFACE="${G1_NET:-}"
PYTHON_BIN="${ROBOT_SETUP_PYTHON:-}"
WORKER_PYTHON="${D435I_WORKER_PYTHON:-}"
INSTALL_SYSTEM_PACKAGES=1

log() {
  echo "[RobotSetup] $*"
}

die() {
  echo "[RobotSetup] ERROR: $*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: bash setup_robot_distributed.sh [options]

Configure the robot-side D435i environment and compile the G1 UDP bridge.

Options:
  --server-ip IP         Policy server control IP (default: 10.42.0.1)
  --robot-ip IP          Robot computer control IP (default: 10.42.0.2)
  --g1-net INTERFACE     G1 DDS interface; auto-detects 192.168.123.x
  --python PATH          Python >= 3.10 used to create .venv_depth
  --worker-python PATH   Existing Python with numpy and pyrealsense2
  --skip-system-packages Do not install missing apt packages
  -h, --help             Show this help

Environment equivalents:
  SERVER_CONTROL_IP, ROBOT_CONTROL_IP, G1_NET,
  ROBOT_SETUP_PYTHON, D435I_WORKER_PYTHON
EOF
}

while (($# > 0)); do
  case "$1" in
    --server-ip)
      (($# >= 2)) || die "--server-ip requires a value"
      SERVER_IP="$2"
      shift 2
      ;;
    --robot-ip)
      (($# >= 2)) || die "--robot-ip requires a value"
      ROBOT_IP="$2"
      shift 2
      ;;
    --g1-net)
      (($# >= 2)) || die "--g1-net requires a value"
      G1_INTERFACE="$2"
      shift 2
      ;;
    --python)
      (($# >= 2)) || die "--python requires a value"
      PYTHON_BIN="$2"
      shift 2
      ;;
    --worker-python)
      (($# >= 2)) || die "--worker-python requires a value"
      WORKER_PYTHON="$2"
      shift 2
      ;;
    --skip-system-packages)
      INSTALL_SYSTEM_PACKAGES=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
done

[[ -f "${SIM2REAL_ROOT}/requirements-robot-depth.txt" ]] ||
  die "run this script from a complete repository checkout"
[[ -f "${G1_SIM2REAL_ROOT}/scripts/build.sh" ]] ||
  die "missing g1_sim2real/scripts/build.sh"

validate_ipv4() {
  local address="$1"
  local octets=()
  local octet
  IFS=. read -r -a octets <<<"${address}"
  ((${#octets[@]} == 4)) || return 1
  for octet in "${octets[@]}"; do
    [[ "${octet}" =~ ^[0-9]+$ ]] || return 1
    ((10#${octet} <= 255)) || return 1
  done
}

validate_ipv4 "${SERVER_IP}" || die "invalid server IPv4 address: ${SERVER_IP}"
validate_ipv4 "${ROBOT_IP}" || die "invalid robot IPv4 address: ${ROBOT_IP}"

APT_PREFIX=()
if ((EUID != 0)); then
  if command -v sudo >/dev/null 2>&1; then
    APT_PREFIX=(sudo)
  else
    APT_PREFIX=()
  fi
fi

apt_install() {
  (($# > 0)) || return 0
  ((INSTALL_SYSTEM_PACKAGES == 1)) ||
    die "missing system packages; rerun without --skip-system-packages"
  command -v apt-get >/dev/null 2>&1 ||
    die "apt-get is unavailable; install these packages manually: $*"
  if ((EUID != 0)) && ((${#APT_PREFIX[@]} == 0)); then
    die "sudo is unavailable; install these packages as root: $*"
  fi
  log "installing system packages: $*"
  "${APT_PREFIX[@]}" apt-get update
  "${APT_PREFIX[@]}" apt-get install -y "$@"
}

MISSING_PACKAGES=()
command -v cmake >/dev/null 2>&1 || MISSING_PACKAGES+=(cmake)
command -v c++ >/dev/null 2>&1 || MISSING_PACKAGES+=(build-essential)
command -v ip >/dev/null 2>&1 || MISSING_PACKAGES+=(iproute2)
command -v lsusb >/dev/null 2>&1 || MISSING_PACKAGES+=(usbutils)

if [[ -z "${PYTHON_BIN}" ]]; then
  if command -v python3.10 >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python3.10)"
  elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python3)"
  else
    MISSING_PACKAGES+=(python3 python3-venv)
  fi
fi

if ((${#MISSING_PACKAGES[@]} > 0)); then
  apt_install "${MISSING_PACKAGES[@]}"
fi

if [[ -z "${PYTHON_BIN}" ]]; then
  PYTHON_BIN="$(command -v python3)"
elif [[ "${PYTHON_BIN}" != */* ]]; then
  PYTHON_BIN="$(command -v "${PYTHON_BIN}")" ||
    die "Python command not found: ${PYTHON_BIN}"
fi
[[ -x "${PYTHON_BIN}" ]] || die "Python is not executable: ${PYTHON_BIN}"

"${PYTHON_BIN}" -c \
  'import sys; assert sys.version_info >= (3, 10), "Python >= 3.10 is required"' ||
  die "Python >= 3.10 is required: ${PYTHON_BIN}"
PYTHON_VERSION="$("${PYTHON_BIN}" -c \
  'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
log "using Python ${PYTHON_VERSION}: ${PYTHON_BIN}"

if [[ -z "${G1_INTERFACE}" ]]; then
  mapfile -t G1_CANDIDATES < <(
    ip -o -4 address show |
      awk '$4 ~ /^192[.]168[.]123[.]/ {print $2}' |
      sort -u
  )
  if ((${#G1_CANDIDATES[@]} == 1)); then
    G1_INTERFACE="${G1_CANDIDATES[0]}"
    log "auto-detected G1 DDS interface: ${G1_INTERFACE}"
  elif ((${#G1_CANDIDATES[@]} == 0)); then
    die "cannot find a 192.168.123.x interface; rerun with --g1-net INTERFACE"
  else
    die "multiple G1 interfaces found (${G1_CANDIDATES[*]}); use --g1-net"
  fi
fi
ip link show dev "${G1_INTERFACE}" >/dev/null 2>&1 ||
  die "network interface does not exist: ${G1_INTERFACE}"

if ! ip -o -4 address show |
  awk -v expected="${ROBOT_IP}" \
    '{split($4, address, "/"); if (address[1] == expected) found=1} END {exit !found}'
then
  log "WARNING: ${ROBOT_IP} is not currently assigned to this computer"
fi

if [[ ! -x "${DEPTH_VENV}/bin/python" ]]; then
  log "creating camera environment: ${DEPTH_VENV}"
  if ! "${PYTHON_BIN}" -m venv "${DEPTH_VENV}"; then
    VENV_PACKAGE="python${PYTHON_VERSION}-venv"
    log "venv creation failed; trying package ${VENV_PACKAGE}"
    apt_install "${VENV_PACKAGE}"
    "${PYTHON_BIN}" -m venv "${DEPTH_VENV}"
  fi
fi

DEPTH_PYTHON="${DEPTH_VENV}/bin/python"
"${DEPTH_PYTHON}" -c \
  'import sys; assert sys.version_info >= (3, 10)' ||
  die "existing .venv_depth uses an unsupported Python; recreate it"

log "installing minimal camera dependencies"
"${DEPTH_PYTHON}" -m pip install --upgrade pip
"${DEPTH_PYTHON}" -m pip install \
  -r "${SIM2REAL_ROOT}/requirements-robot-depth.txt"

realsense_worker_ok() {
  local candidate="$1"
  [[ -x "${candidate}" ]] || return 1
  "${candidate}" -c 'import numpy, pyrealsense2' >/dev/null 2>&1
}

CAMERA_MODE=""
if [[ -n "${WORKER_PYTHON}" ]]; then
  if [[ "${WORKER_PYTHON}" != */* ]]; then
    WORKER_PYTHON="$(command -v "${WORKER_PYTHON}")" ||
      die "worker Python command not found"
  fi
  realsense_worker_ok "${WORKER_PYTHON}" ||
    die "worker Python cannot import both numpy and pyrealsense2: ${WORKER_PYTHON}"
  CAMERA_MODE="worker"
elif "${DEPTH_PYTHON}" -c 'import pyrealsense2' >/dev/null 2>&1; then
  CAMERA_MODE="direct"
else
  for candidate in \
    /home/unitree/miniconda3/envs/realsense/bin/python \
    /home/unitree/miniconda3/envs/locodist/bin/python
  do
    if realsense_worker_ok "${candidate}"; then
      WORKER_PYTHON="${candidate}"
      CAMERA_MODE="worker"
      break
    fi
  done
fi

if [[ -z "${CAMERA_MODE}" ]]; then
  log "pyrealsense2 not found; trying direct installation"
  if "${DEPTH_PYTHON}" -m pip install pyrealsense2 &&
    "${DEPTH_PYTHON}" -c 'import pyrealsense2' >/dev/null 2>&1
  then
    CAMERA_MODE="direct"
  else
    die "pyrealsense2 is unavailable; rerun with --worker-python /path/to/python"
  fi
fi
log "D435i mode: ${CAMERA_MODE}"

ARCH="$(uname -m)"
SDK_LIBRARY="${G1_SIM2REAL_ROOT}/third_party/unitree_sdk2/lib/${ARCH}/libunitree_sdk2.a"
[[ -f "${SDK_LIBRARY}" ]] ||
  die "Unitree SDK2 library is missing for architecture ${ARCH}: ${SDK_LIBRARY}"

BUILD_DIR="${G1_SIM2REAL_ROOT}/build_locomani"
log "compiling G1 bridge for ${ARCH}"
G1_BRIDGE_BUILD_DIR="${BUILD_DIR}" \
  bash "${G1_SIM2REAL_ROOT}/scripts/build.sh"
[[ -x "${BUILD_DIR}/g1_udp_bridge" ]] ||
  die "bridge build did not produce ${BUILD_DIR}/g1_udp_bridge"

log "running robot-side smoke checks"
"${DEPTH_PYTHON}" -m py_compile \
  "${SIM2REAL_ROOT}/src/depth_camera_real.py" \
  "${SIM2REAL_ROOT}/src/runtime/d435i_source.py" \
  "${SIM2REAL_ROOT}/src/runtime/d435i_worker.py" \
  "${SIM2REAL_ROOT}/src/runtime/depth_pipeline.py" \
  "${SIM2REAL_ROOT}/src/runtime/zmq_stream.py"
bash -n \
  "${SIM2REAL_ROOT}/scripts/run_robot_depth_distributed.sh" \
  "${G1_SIM2REAL_ROOT}/scripts/run_robot_bridge_distributed.sh"
"${BUILD_DIR}/g1_udp_bridge" --help >/dev/null

write_default_export() {
  local key="$1"
  local value="$2"
  printf 'if [[ -z "${%s:-}" ]]; then export %s=%q; fi\n' \
    "${key}" "${key}" "${value}"
}

ENV_TEMP="$(mktemp "${ENV_FILE}.tmp.XXXXXX")"
trap 'rm -f "${ENV_TEMP:-}"' EXIT
{
  echo '# Generated by setup_robot_distributed.sh; shellcheck shell=bash'
  write_default_export ROBOT_REPO_ROOT "${REPO_ROOT}"
  write_default_export SERVER_CONTROL_IP "${SERVER_IP}"
  write_default_export ROBOT_CONTROL_IP "${ROBOT_IP}"
  write_default_export G1_NET "${G1_INTERFACE}"
  write_default_export G1_BRIDGE_BUILD_DIR "${BUILD_DIR}"
  write_default_export ROBOT_DEPTH_PYTHON "${DEPTH_PYTHON}"
  if [[ "${CAMERA_MODE}" == "worker" ]]; then
    write_default_export D435I_WORKER_PYTHON "${WORKER_PYTHON}"
  fi
} >"${ENV_TEMP}"
chmod 600 "${ENV_TEMP}"
mv "${ENV_TEMP}" "${ENV_FILE}"
trap - EXIT

log "configuration written: ${ENV_FILE}"
log "setup complete"
echo
echo "Robot camera:"
echo "  bash ${SIM2REAL_ROOT}/scripts/run_robot_depth_distributed.sh"
echo
echo "Robot bridge:"
echo "  bash ${G1_SIM2REAL_ROOT}/scripts/run_robot_bridge_distributed.sh"
