#!/usr/bin/env bash
# Real-robot entry for additive Skill 6: running + IMU heading hold + leg
# odometry.  No RealSense, ROS, vision UDP, or camera process is started.
#
# The binary must be rebuilt on the robot's ARM64 computer.  This script
# intentionally refuses x86_64 binaries and refuses to start unless the
# operator explicitly confirms the E-stop and test authorization.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${G1_RUNNING_STRAIGHT_ROOT:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
RL_SAR_ROOT="${PROJECT_ROOT}/rl_sar"
ROBOT_INTERFACE="${G1_ROBOT_INTERFACE:-enP8p1s0}"
BINARY="${G1_RUNNING_STRAIGHT_BINARY:-${RL_SAR_ROOT}/cmake_build_running_vision_adapter/bin/rl_real_g1}"
TARGET_M="${G1_RUNNING_STRAIGHT_TARGET_M:-110.0}"
SPEED_MPS="${G1_RUNNING_STRAIGHT_SPEED_MPS:-3.5}"
RAMP_S="${G1_RUNNING_STRAIGHT_RAMP_S:-5.0}"
MAX_DURATION_S="${G1_RUNNING_STRAIGHT_MAX_DURATION_S:-120.0}"
LOG_DIR="${G1_RUNNING_STRAIGHT_LOG_DIR:-${PROJECT_ROOT}/logs}"

if [[ "${G1_RUNNING_STRAIGHT_REAL_APPROVED:-0}" != "1" ||
      "${G1_RUNNING_STRAIGHT_ESTOP_READY:-0}" != "1" ]]; then
  echo "REFUSING TO START: set G1_RUNNING_STRAIGHT_REAL_APPROVED=1 and" >&2
  echo "G1_RUNNING_STRAIGHT_ESTOP_READY=1 only after a spotter, clear lane," >&2
  echo "and a reachable physical E-stop are confirmed." >&2
  exit 3
fi
if [[ "${ROBOT_INTERFACE}" == "lo" ]]; then
  echo "REFUSING TO START: lo is simulation-only." >&2
  exit 4
fi
if ! ip link show dev "${ROBOT_INTERFACE}" >/dev/null 2>&1; then
  echo "REFUSING TO START: interface not found: ${ROBOT_INTERFACE}" >&2
  exit 4
fi
if [[ ! -x "${BINARY}" ]]; then
  echo "REFUSING TO START: ARM64 running controller not found: ${BINARY}" >&2
  echo "Build the additive controller on the robot computer first." >&2
  exit 5
fi
if [[ ! -f "${RL_SAR_ROOT}/policy/g1/running/config.yaml" ||
      ! -f "${RL_SAR_ROOT}/policy/g1/running/policy.pt" ]]; then
  echo "REFUSING TO START: running policy files are incomplete." >&2
  exit 5
fi
if ! grep -Eq '^[[:space:]]*num_observations:[[:space:]]*96([[:space:]]|$)' \
    "${RL_SAR_ROOT}/policy/g1/running/config.yaml"; then
  echo "REFUSING TO START: running policy is not the tested 96D interface." >&2
  exit 5
fi

ARCH="$(uname -m)"
BINARY_INFO="$(file -b "${BINARY}")"
if [[ "${ARCH}" != "aarch64" && "${ARCH}" != "arm64" ]]; then
  echo "REFUSING TO START: this entry is for the robot ARM64 computer; host=${ARCH}" >&2
  exit 5
fi
if ! grep -Eiq 'ARM aarch64|ARM64|aarch64' <<<"${BINARY_INFO}"; then
  echo "REFUSING TO START: controller is not an ARM64 ELF: ${BINARY_INFO}" >&2
  exit 5
fi
if pgrep -f '(^|/)(rl_real_g1|rl_real_g1_running|g1_ctrl)([[:space:]]|$)' >/dev/null; then
  echo "REFUSING TO START: another motion controller is already running." >&2
  exit 2
fi
if pgrep -f 'run_realsense_ros2.py|realsense2_camera_node' >/dev/null; then
  echo "REFUSING TO START: a vision process is already running; stop it before" >&2
  echo "using this no-vision entry so sensor ownership is unambiguous." >&2
  exit 2
fi

mkdir -p "${LOG_DIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="${LOG_DIR}/running_straight_robot_${STAMP}.log"
echo "LOG=${LOG}"
echo "No-vision Skill 6: IMU heading hold + leg odometry, target=${TARGET_M} m"
echo "Speed=${SPEED_MPS} m/s, smooth ramp=${RAMP_S} s, max duration=${MAX_DURATION_S} s"
echo "Keyboard sequence: 0 -> 1 -> 6; P is passive; 9 is get-down."
echo "Do not press 5 or 7 during this test."

cd "${RL_SAR_ROOT}"
G1_RUNNING_STRAIGHT_TARGET_M="${TARGET_M}" \
G1_RUNNING_STRAIGHT_SPEED_MPS="${SPEED_MPS}" \
G1_RUNNING_STRAIGHT_RAMP_S="${RAMP_S}" \
G1_RUNNING_STRAIGHT_MAX_DURATION_S="${MAX_DURATION_S}" \
G1_RUNNING_STRAIGHT_REQUIRE_LEG_ODOM="${G1_RUNNING_STRAIGHT_REQUIRE_LEG_ODOM:-1}" \
G1_RUNNING_STRAIGHT_HEADING_KP="${G1_RUNNING_STRAIGHT_HEADING_KP:-1.50}" \
G1_RUNNING_STRAIGHT_HEADING_KD="${G1_RUNNING_STRAIGHT_HEADING_KD:-0.08}" \
G1_RUNNING_STRAIGHT_HEADING_MAX_CMD="${G1_RUNNING_STRAIGHT_HEADING_MAX_CMD:-0.30}" \
G1_RUNNING_STRAIGHT_ODOM_START_TIMEOUT_S="${G1_RUNNING_STRAIGHT_ODOM_START_TIMEOUT_S:-8.0}" \
G1_RUNNING_STRAIGHT_ODOM_STALE_TIMEOUT_S="${G1_RUNNING_STRAIGHT_ODOM_STALE_TIMEOUT_S:-1.0}" \
  "${BINARY}" "${ROBOT_INTERFACE}" 2>&1 | tee "${LOG}"
