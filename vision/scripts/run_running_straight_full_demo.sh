#!/usr/bin/env bash
# MuJoCo smoke/demo for the additive Skill 6 running-only 110 m mission.
#
# The camera bridge is kept only because it is the existing Unitree DDS
# transport used by this local simulator.  It never enables a vision mode and
# it sends zero UDP commands; the C++ running-straight state owns vx, vy and wz.
# No real robot network or lowcmd is used.
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ROOT="${G1_RUNNING_STRAIGHT_ROOT:-$(cd "${SCRIPT_ROOT}/.." && pwd)}"
UNITREE_ROOT="${UNITREE_ROOT:-/home/ubuntu}"
MUJOCO_ROOT="${UNITREE_ROOT}/unitree_mujoco"
RUNNING_ROOT="${PROJECT_ROOT}/rl_sar"
RUNNING_BINARY="${G1_RUNNING_STRAIGHT_BINARY:-${RUNNING_ROOT}/cmake_build_running_vision_adapter/bin/rl_real_g1}"
TARGET_M="${G1_RUNNING_STRAIGHT_TARGET_M:-110.0}"
SPEED="${G1_RUNNING_STRAIGHT_SPEED_MPS:-3.5}"
RAMP_S="${G1_RUNNING_STRAIGHT_RAMP_S:-5.0}"
MAX_DURATION_S="${G1_RUNNING_STRAIGHT_MAX_DURATION_S:-120.0}"
RUN_SECONDS="${G1_RUNNING_STRAIGHT_RUN_SECONDS:-60}"
HEADLESS="${G1_RUNNING_STRAIGHT_HEADLESS:-1}"
STARTUP_SUPPORT_S="${G1_RUNNING_STRAIGHT_STARTUP_SUPPORT_S:-16.0}"
FALL_HEIGHT="${G1_RUNNING_STRAIGHT_FALL_HEIGHT:-0.45}"
FALL_CONFIRM_S="${G1_RUNNING_STRAIGHT_FALL_CONFIRM_S:-0.60}"

if [[ ! -x "${RUNNING_BINARY}" ]]; then
    echo "Missing additive running controller: ${RUNNING_BINARY}" >&2
    echo "Run vision/scripts/build_running_vision_controller.sh first." >&2
    exit 1
fi
if [[ ! -d "${MUJOCO_ROOT}/simulate_python" ]]; then
    echo "Missing unitree_mujoco: ${MUJOCO_ROOT}" >&2
    exit 1
fi
if pgrep -x unitree_mujoco >/dev/null &&
   [[ "${ALLOW_EXISTING_UNITREE_SIM:-0}" != "1" ]]; then
    echo "An existing unitree_mujoco process was detected; close it first." >&2
    exit 1
fi

source "${SCRIPT_ROOT}/scripts/activate_g1race_dds.sh"
export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
if [[ -z "${XAUTHORITY:-}" && -d "${XDG_RUNTIME_DIR}" ]]; then
    XAUTHORITY="$(${G1_RACE_PYTHON:-python} - <<'PY'
from pathlib import Path
import os

runtime = Path(os.environ["XDG_RUNTIME_DIR"])
paths = sorted(
    runtime.glob(".mutter-Xwaylandauth.*"),
    key=lambda path: path.stat().st_mtime,
    reverse=True,
)
print(paths[0] if paths else "")
PY
    )"
    export XAUTHORITY
fi
export MUJOCO_GL="${MUJOCO_GL:-egl}"

free_udp_port() {
    "${G1_RACE_PYTHON:-python}" -c \
        'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1]); s.close()'
}

COMMAND_PORT="${G1_RUNNING_STRAIGHT_COMMAND_PORT:-$(free_udp_port)}"
STATUS_PORT="${G1_RUNNING_STRAIGHT_STATUS_PORT:-$(free_udp_port)}"
if [[ "${COMMAND_PORT}" == "${STATUS_PORT}" ]]; then
    STATUS_PORT="$(free_udp_port)"
fi

cd "${PROJECT_ROOT}"
python "${SCRIPT_ROOT}/scripts/prepare_unitree_scene.py" \
    --unitree-mujoco "${MUJOCO_ROOT}" \
    --dofs 29 \
    --lane-width 2.1 \
    --lane-count 4 \
    --target-lane-index 1 \
    --line-width 0.10 \
    --track-length 115.0 \
    --finish-distance 100.0

SIM_DISPLAY_ARGS=()
if [[ "${HEADLESS}" != "0" ]]; then
    SIM_DISPLAY_ARGS+=(--headless)
fi

LOG_DIR="${PROJECT_ROOT}/logs"
mkdir -p "${LOG_DIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SIM_LOG="${LOG_DIR}/running_straight_${STAMP}_sim.log"
CTRL_LOG="${LOG_DIR}/running_straight_${STAMP}_controller.log"
echo "SIM_LOG=${SIM_LOG}"
echo "CTRL_LOG=${CTRL_LOG}"
echo "COMMAND_PORT=${COMMAND_PORT} STATUS_PORT=${STATUS_PORT}"

SIM_PID=""
CTRL_PID=""
cleanup() {
    set +e
    [[ -z "${CTRL_PID}" ]] || kill "${CTRL_PID}" 2>/dev/null || true
    [[ -z "${SIM_PID}" ]] || kill "${SIM_PID}" 2>/dev/null || true
    [[ -z "${CTRL_PID}" ]] || wait "${CTRL_PID}" 2>/dev/null || true
    [[ -z "${SIM_PID}" ]] || wait "${SIM_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Match Skill 7's startup lifecycle: keep the virtual tether until the
# controller confirms state 1 / mission entry, then fade it out.  This is only
# a MuJoCo startup aid; it does not send a vision command or affect the policy.
python "${SCRIPT_ROOT}/scripts/run_unitree_camera_sim.py" \
    --unitree-mujoco "${MUJOCO_ROOT}" \
    --scene "${MUJOCO_ROOT}/unitree_robots/g1/scene_race_29dof.xml" \
    --domain-id "${G1_RACE_DOMAIN_ID}" \
    --interface "${G1_RACE_INTERFACE}" \
    --udp-port "${COMMAND_PORT}" \
    --status-port "${STATUS_PORT}" \
    --mission sprint100m \
    --speed "${SPEED}" \
    --camera-fps 30 \
    --depth-fps 10 \
    --viewer-fps 24 \
    --debug-fps 5 \
    "${SIM_DISPLAY_ARGS[@]}" \
    --auto-start-policy \
    --auto-start-mission sprint100m \
    --policy-start-delay 1.0 \
    --startup-support-seconds "${STARTUP_SUPPORT_S}" \
    --startup-support-fade-seconds 0.60 \
    --startup-support-until-mission \
    --fall-height "${FALL_HEIGHT}" \
    --fall-confirm-seconds "${FALL_CONFIRM_S}" \
    --status-interval 0.5 \
    --run-seconds "${RUN_SECONDS}" \
    >"${SIM_LOG}" 2>&1 &
SIM_PID=$!

sleep 1
if ! kill -0 "${SIM_PID}" 2>/dev/null; then
    echo "MuJoCo/DDS exited during startup; controller was not started." >&2
    wait "${SIM_PID}" 2>/dev/null || true
    exit 1
fi

cd "${RUNNING_ROOT}"
G1_RUNNING_STRAIGHT_TARGET_M="${TARGET_M}" \
G1_RUNNING_STRAIGHT_SPEED_MPS="${SPEED}" \
G1_RUNNING_STRAIGHT_RAMP_S="${RAMP_S}" \
G1_RUNNING_STRAIGHT_MAX_DURATION_S="${MAX_DURATION_S}" \
G1_VISION_UDP_PORT="${COMMAND_PORT}" \
G1_VISION_STATUS_PORT="${STATUS_PORT}" \
    "${RUNNING_BINARY}" "${G1_RACE_INTERFACE}" \
    2>&1 | tee "${CTRL_LOG}"
