#!/usr/bin/env bash
set -uo pipefail

PROJECT_ROOT="${G1_RACE_ROOT:-${HOME}/g1_race_vision}"
UNITREE_ROOT="${UNITREE_ROOT:-${HOME}/unitree_ws}"
MUJOCO_ROOT="${UNITREE_ROOT}/unitree_mujoco"
RUNNING_ROOT="${G1_RUNNING_ROOT:-${UNITREE_ROOT}/g1_running}/rl_sar"
PYTHON="${G1_RACE_PYTHON:-${HOME}/miniconda3/envs/g1race/bin/python}"
RUN_SECONDS="${1:-12}"
SPEED="${2:-0.20}"
OUTPUT_DIR="${PROJECT_ROOT}/output/g1_running_smoke"
SIM_LOG="${OUTPUT_DIR}/simulation.log"
CTRL_LOG="${OUTPUT_DIR}/controller.log"

export MUJOCO_GL="${MUJOCO_GL:-egl}"

mkdir -p "${OUTPUT_DIR}"
rm -f "${SIM_LOG}" "${CTRL_LOG}"

cd "${PROJECT_ROOT}"
"${PYTHON}" scripts/prepare_unitree_scene.py \
    --unitree-mujoco "${MUJOCO_ROOT}" \
    --dofs 29 \
    --lane-width 2.0 \
    --line-width 0.10 \
    --track-length 100.0

"${PYTHON}" scripts/run_unitree_camera_sim.py \
    --unitree-mujoco "${MUJOCO_ROOT}" \
    --scene "${MUJOCO_ROOT}/unitree_robots/g1/scene_race_29dof.xml" \
    --domain-id 0 \
    --interface lo \
    --camera-fps 5 \
    --speed "${SPEED}" \
    --headless \
    --run-seconds "${RUN_SECONDS}" \
    --status-interval 1 \
    --startup-support-seconds 8.0 \
    >"${SIM_LOG}" 2>&1 &
sim_pid=$!

ctrl_pid=""
cleanup() {
    [[ -z "${ctrl_pid}" ]] || kill "${ctrl_pid}" 2>/dev/null || true
    kill "${sim_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 0.5
cd "${RUNNING_ROOT}"
ctrl_seconds=$((RUN_SECONDS + 1))
{
    sleep 2
    printf '0'
} |
G1_AUTO_RUNNING=1 \
timeout "${ctrl_seconds}s" \
    ./cmake_build/bin/rl_real_g1 lo >"${CTRL_LOG}" 2>&1 &
ctrl_pid=$!

wait "${sim_pid}"
sim_status=$?
wait "${ctrl_pid}"
ctrl_status=$?
trap - EXIT INT TERM

echo "SIM_STATUS=${sim_status} CTRL_STATUS=${ctrl_status}"
echo "--- simulation"
tail -n 40 "${SIM_LOG}"
echo "--- controller key events"
tr '\r' '\n' <"${CTRL_LOG}" |
    grep -E 'vision|FSM|Entered|GetUp|Running|Policy|ERROR|Error|error' |
    head -n 30 || true
echo "--- controller command tail"
tr '\r' '\n' <"${CTRL_LOG}" |
    grep -E 'RL Run|vision|ERROR|Error|error' |
    tail -n 20 || true

if [[ "${sim_status}" -ne 0 ]]; then
    exit "${sim_status}"
fi
if [[ "${ctrl_status}" -ne 0 && "${ctrl_status}" -ne 124 ]]; then
    exit "${ctrl_status}"
fi
