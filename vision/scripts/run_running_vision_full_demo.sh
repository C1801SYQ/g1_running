#!/usr/bin/env bash
# Headless/GUI-capable MuJoCo + RGB-D running-backed Num7 mission.
# This uses no real-robot network and never writes robot low-level commands.
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_ROOT}/.." && pwd)"
PROJECT_ROOT="${G1_RUNNING_VISION_ROOT:-${SCRIPT_ROOT}}"
RUNNING_REPOSITORY="${G1_RUNNING_VISION_REPO:-${REPOSITORY_ROOT}}"
UNITREE_ROOT="${UNITREE_ROOT:-/home/ubuntu}"
MUJOCO_ROOT="${UNITREE_ROOT}/unitree_mujoco"
RUNNING_ROOT="${RUNNING_REPOSITORY}/rl_sar"
RUNNING_BINARY="${G1_RUNNING_VISION_BINARY:-${RUNNING_ROOT}/cmake_build_running_vision_adapter/bin/rl_real_g1}"
TARGET_M="${G1_RUNNING_VISION_TARGET_M:-110.0}"
CRUISE_SPEED="${G1_RUNNING_VISION_CRUISE_SPEED_MPS:-4.5}"
MAX_DURATION_S="${G1_RUNNING_VISION_MAX_DURATION_S:-260.0}"

if [[ ! -x "${RUNNING_BINARY}" ]]; then
  echo "Missing running-vision controller: ${RUNNING_BINARY}" >&2
  echo "Run build_running_vision_controller.sh in the target conda env first." >&2
  exit 1
fi
if [[ ! -d "${MUJOCO_ROOT}/simulate_python" ]]; then
  echo "Missing unitree_mujoco: ${MUJOCO_ROOT}" >&2
  exit 1
fi

source "${PROJECT_ROOT}/scripts/activate_g1race_dds.sh"
export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

cd "${PROJECT_ROOT}"
python scripts/prepare_unitree_scene.py \
  --unitree-mujoco "${MUJOCO_ROOT}" \
  --dofs 29 \
  --lane-width 2.1 \
  --lane-count 4 \
  --target-lane-index 1 \
  --line-width 0.10 \
  --track-length 115.0 \
  --finish-distance 100.0

LOG_DIR="${PROJECT_ROOT}/logs"
mkdir -p "${LOG_DIR}"
LOG="${LOG_DIR}/running_vision_$(date +%Y%m%d_%H%M%S).log"
echo "Logging running-vision simulation to ${LOG}"

SIM_PID=""
cleanup() {
  if [[ -n "${SIM_PID}" ]]; then
    kill "${SIM_PID}" 2>/dev/null || true
    wait "${SIM_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

python scripts/run_unitree_camera_sim.py \
  --unitree-mujoco "${MUJOCO_ROOT}" \
  --scene "${MUJOCO_ROOT}/unitree_robots/g1/scene_race_29dof.xml" \
  --domain-id "${G1_RACE_DOMAIN_ID}" \
  --interface "${G1_RACE_INTERFACE}" \
  --mission runningvision110m \
  --speed "${CRUISE_SPEED}" \
  --running-vision-target-m "${TARGET_M}" \
  --running-vision-max-duration-s "${MAX_DURATION_S}" \
  --startup-support-seconds 16.0 \
  --startup-support-fade-seconds 0.60 \
  --startup-support-until-mission \
  --show-debug 2>&1 | tee -a "${LOG}" &
SIM_PID=$!
sleep 2
if ! kill -0 "${SIM_PID}" 2>/dev/null; then
  echo "MuJoCo camera simulation exited during startup." >&2
  wait "${SIM_PID}" 2>/dev/null || true
  exit 1
fi

echo
echo "MuJoCo/DDS is ready. Keep keyboard focus in this terminal."
echo "Manual test sequence:"
echo "  0  Passive -> GetUp"
echo "  1  after standing steadily, enter locomotion state 1"
echo "  7  enter the running-backed visual 110 m mission"
echo "Stop the controller and simulator with Ctrl+C."
echo

G1_VISION_MAX_VX="${G1_VISION_MAX_VX:-${CRUISE_SPEED}}" \
G1_VISION_MAX_WZ="${G1_VISION_MAX_WZ:-0.35}" \
G1_VISION_STATUS_PORT="${G1_VISION_STATUS_PORT:-15002}" \
  "${RUNNING_BINARY}" "${G1_RACE_INTERFACE:-lo}" 2>&1 | tee -a "${LOG}"
