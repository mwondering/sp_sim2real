#!/usr/bin/env bash
set -euo pipefail

EXPECTED_PACKAGE="XRoboToolkit-PC-Service_1.0.0.0_arm64_ubuntu20.04.deb"
ARTIFACT_URL="https://drive.google.com/drive/folders/1lrPyiiy7anyG3P4wHNIQQQlydboLPd9e"
ARCHIVE_URL="https://drive.usercontent.google.com/download?id=1lfbd2ZEVtS9gEQVQhTA1Y_I2Dg3BJ0LV&export=download&confirm=t"
ARCHIVE_SHA256="694cd6e6e826c6b427a1bc1c84a5f2f50673cba86042205c563b48af604eeb0a"
PACKAGE_SHA256="7ade5b6b48ce8fb1ea0874f5d202608cfbaeecdd076d5e216d66eca203cb516b"
ARCHIVE_MEMBER="jetpack5-aarch64/xrobotservice/${EXPECTED_PACKAGE}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_PACKAGE="${SCRIPT_DIR}/third_party/prebuilt/jetpack5-aarch64/xrobotservice/${EXPECTED_PACKAGE}"

usage() {
  cat <<EOF
Usage:
  bash install_xrobottoolkit_pc_service.sh --download
  bash install_xrobottoolkit_pc_service.sh /path/to/${EXPECTED_PACKAGE}

Installs an XRoboToolkit PC Service package only after validating it with the
Ubuntu 20.04/aarch64 runtime linker. With no argument, the script uses the
local package below when present, otherwise it downloads the pinned MimicLite
JetPack 5 archive. Pass --download to force a verified download.

  ${DEFAULT_PACKAGE}

Obtain the compatible package from the MimicLite JetPack 5 shared artifacts.
  ${ARTIFACT_URL}
The upstream generic/headless ARM64 release requires newer ICU, GLIBC, and
GLIBCXX versions and is intentionally rejected on G1 Ubuntu 20.04.
EOF
}

case "${1:-}" in
  -h|--help)
    usage
    exit 0
    ;;
esac

if (( $# > 1 )); then
  usage >&2
  exit 2
fi

download_package=false
case "${1:-}" in
  "")
    if [[ -f "${DEFAULT_PACKAGE}" ]]; then
      package_path="${DEFAULT_PACKAGE}"
    else
      download_package=true
    fi
    ;;
  --download)
    download_package=true
    ;;
  -*)
    echo "Unknown option: $1" >&2
    usage >&2
    exit 2
    ;;
  *)
    package_path="$1"
    ;;
esac

if [[ ! -r /etc/os-release ]]; then
  echo "Cannot identify the operating system: /etc/os-release is missing" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "20.04" ]]; then
  echo "This installer is only for G1 Ubuntu 20.04; detected: ${PRETTY_NAME:-unknown}" >&2
  exit 1
fi

case "$(uname -m)" in
  aarch64|arm64) ;;
  *)
    echo "This installer requires aarch64/arm64; detected: $(uname -m)" >&2
    exit 1
    ;;
esac

for tool in dpkg dpkg-deb ldd mktemp realpath; do
  if ! command -v "${tool}" >/dev/null 2>&1; then
    echo "Required tool not found: ${tool}" >&2
    exit 1
  fi
done

stage_dir="$(mktemp -d -t xrobot-service-ubuntu20.XXXXXX)"
cleanup() {
  rm -rf -- "${stage_dir}"
}
trap cleanup EXIT

if "${download_package}"; then
  for tool in curl sha256sum unzip; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
      echo "Required download tool not found: ${tool}" >&2
      exit 1
    fi
  done

  archive_path="${stage_dir}/jetpack5-aarch64.zip"
  echo "[install_xrobottoolkit_pc_service] downloading pinned MimicLite JetPack 5 artifact"
  curl -L --fail --retry 3 --retry-delay 2 --connect-timeout 15 \
    -o "${archive_path}" "${ARCHIVE_URL}"
  echo "${ARCHIVE_SHA256}  ${archive_path}" | sha256sum -c -

  download_dir="${stage_dir}/download"
  mkdir -p "${download_dir}"
  unzip -j "${archive_path}" "${ARCHIVE_MEMBER}" -d "${download_dir}"
  package_path="${download_dir}/${EXPECTED_PACKAGE}"
  echo "${PACKAGE_SHA256}  ${package_path}" | sha256sum -c -
fi

if [[ ! -f "${package_path}" ]]; then
  echo "Ubuntu 20.04-compatible XR Service package not found: ${package_path}" >&2
  echo "Expected package: ${EXPECTED_PACKAGE}" >&2
  echo "Copy it from the MimicLite JetPack 5 shared artifacts, then rerun this script." >&2
  echo "Artifacts: ${ARTIFACT_URL}" >&2
  exit 1
fi
package_path="$(realpath "${package_path}")"

package_arch="$(dpkg-deb -f "${package_path}" Architecture)"
if [[ "${package_arch}" != "arm64" ]]; then
  echo "XR Service package architecture must be arm64, got: ${package_arch}" >&2
  exit 1
fi

package_root="${stage_dir}/package-root"
mkdir -p "${package_root}"
dpkg-deb -x "${package_path}" "${package_root}"
staged_service_dir="${package_root}/opt/apps/roboticsservice"
staged_service_bin="${staged_service_dir}/RoboticsServiceProcess"
if [[ ! -x "${staged_service_bin}" ]]; then
  echo "Package does not contain an executable RoboticsServiceProcess" >&2
  exit 1
fi

staged_library_path="${staged_service_dir}:${staged_service_dir}/lib:${staged_service_dir}/SDK/arm64"
set +e
ldd_output="$(env LD_LIBRARY_PATH="${staged_library_path}" ldd "${staged_service_bin}" 2>&1)"
ldd_status=$?
set -e
if (( ldd_status != 0 )) || grep -Eq \
    'not found|cannot open shared object file' <<<"${ldd_output}"; then
  echo "Refusing to install an XR Service package incompatible with Ubuntu 20.04:" >&2
  printf '%s\n' "${ldd_output}" >&2
  echo >&2
  echo "Required package: ${EXPECTED_PACKAGE}" >&2
  echo "Do not repair this with ICU or GLIBC compatibility symlinks." >&2
  exit 1
fi

echo "[install_xrobottoolkit_pc_service] pre-install runtime check: OK"
if (( EUID == 0 )); then
  dpkg -i "${package_path}"
else
  if ! command -v sudo >/dev/null 2>&1; then
    echo "sudo is required to install ${package_path}" >&2
    exit 1
  fi
  sudo dpkg -i "${package_path}"
fi

installed_service_dir="/opt/apps/roboticsservice"
installed_service_bin="${installed_service_dir}/RoboticsServiceProcess"
if [[ ! -x "${installed_service_bin}" ]]; then
  echo "Installation completed without ${installed_service_bin}" >&2
  exit 1
fi

installed_library_path="${installed_service_dir}:${installed_service_dir}/lib:${installed_service_dir}/SDK/arm64"
set +e
ldd_output="$(env LD_LIBRARY_PATH="${installed_library_path}" ldd "${installed_service_bin}" 2>&1)"
ldd_status=$?
set -e
if (( ldd_status != 0 )) || grep -Eq \
    'not found|cannot open shared object file' <<<"${ldd_output}"; then
  echo "Installed XR Service still has unresolved runtime dependencies:" >&2
  printf '%s\n' "${ldd_output}" >&2
  exit 1
fi

echo "[install_xrobottoolkit_pc_service] installed runtime check: OK"
echo "[install_xrobottoolkit_pc_service] installed: ${installed_service_bin}"
