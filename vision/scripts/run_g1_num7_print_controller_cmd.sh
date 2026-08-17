#!/usr/bin/env bash
# Num7 (Skill 7) controller-command printer.
#
# This helper ONLY prints the rl_real_g1 startup command with the exact
# environment that must match the vision terminal. It never starts the
# controller and never sends any motion command.
#
# Usage (controller terminal):
#   bash vision/scripts/run_g1_num7_print_controller_cmd.sh
#
# The printed command includes every G1_NUM7_* variable. Set the same values
# as the vision terminal (G1_NUM7_TARGET_M etc.) so both ends agree.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
RL_SAR_ROOT="${G1_RUNNING_ROOT:-${PROJECT_ROOT}}/rl_sar"
ROBOT_INTERFACE="${G1_ROBOT_INTERFACE:-}"

if [[ -z "${ROBOT_INTERFACE}" ]]; then
    echo "Refusing to print a runnable controller command: set G1_ROBOT_INTERFACE" >&2
    echo "to the verified physical DDS interface on the Jetson." >&2
    exit 2
fi
if [[ "${ROBOT_INTERFACE}" == "lo" ]]; then
    echo "Refusing: G1_ROBOT_INTERFACE=lo is simulation-only." >&2
    exit 2
fi
if ! ip link show dev "${ROBOT_INTERFACE}" >/dev/null 2>&1; then
    echo "Refusing: network interface does not exist: ${ROBOT_INTERFACE}" >&2
    exit 2
fi

TARGET_M="${G1_NUM7_TARGET_M:-1.00}"
if [[ -n "${G1_NUM7_MAX_DURATION_S:-}" ]]; then
  MAX_DURATION_S="${G1_NUM7_MAX_DURATION_S}"
else
  MAX_DURATION_S="$(python3 -c 'import sys; target=float(sys.argv[1]); print(f"{max(7.0, target / 0.5 + 5.0):.1f}")' "${TARGET_M}")"
fi
STOP_MARGIN_M="${G1_NUM7_STOP_MARGIN_M:-0.0}"
DISTANCE_SCALE="${G1_NUM7_DISTANCE_SCALE:-0.60}"
START_TIMEOUT_S="${G1_NUM7_START_TIMEOUT_S:-2.0}"
SETTLE_S="${G1_NUM7_SETTLE_S:-0.5}"

# Clamp the target to the shared 0.05..1.00 range (same as C++ and Python).
TARGET_M="$(python3 -c 'import sys; v=float(sys.argv[1]); print(f"{min(max(v,0.05),200.00):.3f}")' "${TARGET_M}")"

echo "Num7 controller command (run this in the controller terminal):"
echo "--------------------------------------------------------------------"
cat <<CMD
cd "${RL_SAR_ROOT}"
G1_NUM7_TARGET_M="${TARGET_M}" \\
G1_NUM7_MAX_DURATION_S="${MAX_DURATION_S}" \\
G1_NUM7_STOP_MARGIN_M="${STOP_MARGIN_M}" \\
G1_NUM7_DISTANCE_SCALE="${DISTANCE_SCALE}" \\
G1_NUM7_START_TIMEOUT_S="${START_TIMEOUT_S}" \\
G1_NUM7_SETTLE_S="${SETTLE_S}" \\
G1_VISION_MAX_VX=0.50 \\
G1_VISION_MAX_WZ=0.25 \\
G1_VISION_STATUS_PORT=15002 \\
    ./cmake_build/bin/rl_real_g1 "${ROBOT_INTERFACE}"
CMD
echo "--------------------------------------------------------------------"
echo "Keep G1_NUM7_TARGET_M identical to the vision terminal."
echo "Verified physical DDS interface: ${ROBOT_INTERFACE}"
echo "G1_NUM7_STOP_MARGIN_M is C++-only; Python stops at the same target."
echo "This helper did NOT start anything."
