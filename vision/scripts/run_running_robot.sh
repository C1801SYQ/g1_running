#!/usr/bin/env bash
# Real-robot entry for Skill 5 (running policy) and Skill 6 (straight mission).
# The controller is the additive ARM64 build; the operator selects the state
# with the keyboard after startup: 0 -> 1 -> 5 or 6.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${G1_RUNNING_ROOT:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
RL_SAR_ROOT="${PROJECT_ROOT}/rl_sar"
ROBOT_INTERFACE="${G1_ROBOT_INTERFACE:-enP8p1s0}"
BINARY="${G1_RUNNING_BINARY:-${RL_SAR_ROOT}/cmake_build_running_vision_adapter/bin/rl_real_g1}"

if [[ "${G1_RUNNING_REAL_APPROVED:-0}" != "1" ||
      "${G1_RUNNING_ESTOP_READY:-0}" != "1" ]]; then
  echo "REFUSING TO START: set G1_RUNNING_REAL_APPROVED=1 and" >&2
  echo "G1_RUNNING_ESTOP_READY=1 only after a spotter, clear lane, and" >&2
  echo "a reachable physical E-stop are confirmed." >&2
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
  exit 5
fi
if [[ "$(uname -m)" != "aarch64" && "$(uname -m)" != "arm64" ]]; then
  echo "REFUSING TO START: this entry is for the robot ARM64 computer." >&2
  exit 5
fi
if ! file -b "${BINARY}" | grep -Eiq 'ARM aarch64|ARM64|aarch64'; then
  echo "REFUSING TO START: controller is not an ARM64 ELF." >&2
  exit 5
fi
if [[ ! -f "${RL_SAR_ROOT}/policy/g1/running/config.yaml" ||
      ! -f "${RL_SAR_ROOT}/policy/g1/running/policy.pt" ]]; then
  echo "REFUSING TO START: 96D running policy/config is incomplete." >&2
  exit 5
fi
if ! grep -Eq '^[[:space:]]*num_observations:[[:space:]]*96([[:space:]]|$)' \
    "${RL_SAR_ROOT}/policy/g1/running/config.yaml"; then
  echo "REFUSING TO START: running policy is not the tested 96D interface." >&2
  exit 5
fi
if pgrep -f '(^|/)(rl_real_g1|rl_real_g1_running|g1_ctrl)([[:space:]]|$)' >/dev/null; then
  echo "REFUSING TO START: another motion controller is already running." >&2
  exit 2
fi
if pgrep -f 'run_realsense_ros2.py|realsense2_camera_node' >/dev/null; then
  echo "REFUSING TO START: a vision process is running; stop it first." >&2
  exit 2
fi

echo "ARM64 Skill 5/6 controller ready: ${BINARY}"
echo "Keyboard sequence: 0 -> 1 -> 5 (Skill 5 running) or 6 (Skill 6 straight)"
echo "Transition is the same InitRL(running) -> live-state capture path as dance."
echo "P is passive; 9 is get-down. Keep the physical E-stop within reach."

cd "${RL_SAR_ROOT}"
exec "${BINARY}" "${ROBOT_INTERFACE}"
