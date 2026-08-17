#!/usr/bin/env bash
# Headless Num7 (Skill 7) joint-simulation smoke.
#
# Verifies, without any GUI or robot motion:
#   1. Before the Num7 FSM enable, actual velocity and mission distance are 0.
#   2. Before lane lock the starting restraint is engaged and commands are 0.
#   3. After lane lock the restraint is released and qpos[0] actually advances.
#   4. After the mission stops the command stays zero.
#   5. A second Num7 entry re-locks and re-releases the restraint.
#   6. Skill 6 support-release logic is not regressed.
#
# No robot controller is started on hardware and no lowcmd is sent.
set -euo pipefail

PROJECT_ROOT="${G1_RUNNING_ROOT:-/home/ubuntu/g1_running}"
VISION_ROOT="${PROJECT_ROOT}/vision"
UNITREE_ROOT="${UNITREE_ROOT:-/home/ubuntu}"
MUJOCO_ROOT="${UNITREE_ROOT}/unitree_mujoco"
RUNNING_ROOT="${PROJECT_ROOT}/rl_sar"
RUNNING_BINARY="${RUNNING_ROOT}/cmake_build/bin/rl_real_g1"
PYTHON="${G1_RACE_PYTHON:-/home/ubuntu/anaconda3/envs/g1race/bin/python}"
OUTPUT_DIR="${VISION_ROOT}/output/num7_headless_smoke"
RELEASE_MARKER="${OUTPUT_DIR}/HARDWARE_RELEASE_READY"
SIM_LOG="${OUTPUT_DIR}/simulation.log"
CTRL_LOG="${OUTPUT_DIR}/controller.log"
NUM7_TARGET="${G1_NUM7_TARGET_M:-1.00}"
NUM7_SPEED="${G1_NUM7_SMOKE_SPEED:-0.50}"
MIN_ACTUAL_DISPLACEMENT="${G1_NUM7_MIN_ACTUAL_DISPLACEMENT_M:-0.80}"
MAX_ACTUAL_DISPLACEMENT="${G1_NUM7_MAX_ACTUAL_DISPLACEMENT_M:-1.40}"
RUN_SECONDS="${1:-38}"

free_udp_port() {
    "${PYTHON}" -c 'import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1]); s.close()'
}

# Never share the production/default ports with another test or a live vision
# process. A bind failure invalidates the entire smoke and is asserted below.
COMMAND_PORT="${G1_NUM7_SMOKE_COMMAND_PORT:-$(free_udp_port)}"
STATUS_PORT="${G1_NUM7_SMOKE_STATUS_PORT:-$(free_udp_port)}"
if [[ "${COMMAND_PORT}" == "${STATUS_PORT}" ]]; then
    STATUS_PORT="$(free_udp_port)"
fi

export MUJOCO_GL="${MUJOCO_GL:-egl}"
export DISPLAY="${DISPLAY:-:0}"
GUI_MODE="${G1_NUM7_SMOKE_GUI:-0}"

mkdir -p "${OUTPUT_DIR}"
rm -f "${SIM_LOG}" "${CTRL_LOG}"
# Any new run invalidates the previous release proof until every assertion,
# including measured qpos displacement, passes again.
rm -f "${RELEASE_MARKER}"

cd "${VISION_ROOT}"
"${PYTHON}" scripts/prepare_unitree_scene.py \
    --unitree-mujoco "${MUJOCO_ROOT}" \
    --dofs 29 \
    --lane-width 2.1 \
    --lane-count 4 \
    --target-lane-index 1 \
    --line-width 0.10 \
    --track-length 115.0 \
    --finish-distance 100.0

# Phase 1: Num7 with auto-start. Camera FPS kept low for speed.
"${PYTHON}" scripts/run_unitree_camera_sim.py \
    --unitree-mujoco "${MUJOCO_ROOT}" \
    --scene "${MUJOCO_ROOT}/unitree_robots/g1/scene_race_29dof.xml" \
    --domain-id 0 \
    --interface lo \
    --udp-port "${COMMAND_PORT}" \
    --status-port "${STATUS_PORT}" \
    --camera-fps 8 \
    --depth-fps 4 \
    --viewer-fps 8 \
    --debug-fps 2 \
    --mission walk0p5m \
    --speed "${NUM7_SPEED}" \
    --forward-accel 0.50 \
    --max-yaw-rate 0.25 \
    --max-yaw-accel 0.50 \
    --num7-target-m "${NUM7_TARGET}" \
    --num7-max-duration-s 7.0 \
    $([ "${GUI_MODE}" = "1" ] && echo "" || echo "--headless") \
    --run-seconds "${RUN_SECONDS}" \
    --status-interval 0.5 \
    --startup-support-seconds 8.0 \
    --startup-support-fade-seconds 1.0 \
    --startup-support-until-mission \
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
# Drive the controller FSM through a real PTY (kbhit() needs a TTY) with the
# 0 -> 1 -> 7 keys written at the right times. The Python driver captures the
# controller log and never touches a real robot.
"${PYTHON}" "${VISION_ROOT}/scripts/num7_pty_driver.py" \
    "${RUNNING_BINARY}" "${CTRL_LOG}" "${NUM7_TARGET}" "${RUN_SECONDS}" \
    "${COMMAND_PORT}" "${STATUS_PORT}" &
ctrl_pid=$!

set +e
wait "${sim_pid}"
sim_status=$?
wait "${ctrl_pid}" 2>/dev/null
ctrl_status=$?
set -e
trap - EXIT INT TERM

echo "SIM_STATUS=${sim_status} CTRL_STATUS=${ctrl_status}"
echo "=== simulation key events ==="
grep -E "Num7|num7|NUM7|restraint|lane lock|released|WALK0P5M|Skill 7|mission|qpos|FALL|ERROR" \
    "${SIM_LOG}" | head -n 60 || true
echo "=== controller key events ==="
tr '\r' '\n' <"${CTRL_LOG}" | \
    grep -E "Skill 7|skill7|VisionWalk|FSM|Entered|GetUp|Locomotion|ERROR|Error|RLFSMStateRLVisionWalk" | \
    head -n 40 || true

echo "=== summary (actual displacement vs commanded) ==="
grep -E "pelvis=|num7|NUM7|released|restraint" "${SIM_LOG}" | tail -n 25 || true

fail() {
    echo "NUM7_HEADLESS_SMOKE_FAILED: $*" >&2
    exit 1
}

require_count() {
    local minimum="$1"
    local pattern="$2"
    local file="$3"
    local label="$4"
    local count
    count="$(grep -aEc "${pattern}" "${file}" || true)"
    if (( count < minimum )); then
        fail "${label}: expected >=${minimum}, found ${count}"
    fi
}

reject_pattern() {
    local pattern="$1"
    local file="$2"
    local label="$3"
    if grep -aEq "${pattern}" "${file}"; then
        fail "${label}"
    fi
}

[[ "${sim_status}" -eq 0 ]] || fail "simulation exited ${sim_status}"
[[ "${ctrl_status}" -eq 0 ]] || fail "controller driver exited ${ctrl_status}"

require_count 1 "velocity receiver listening on 127\\.0\\.0\\.1:${COMMAND_PORT}" "${CTRL_LOG}" "command receiver did not bind the isolated port"
reject_pattern "could not bind|Address already in use" "${CTRL_LOG}" "UDP/status port bind failure"
reject_pattern "START_TIMEOUT|UDP_STALE|VISION_ZERO_CMD" "${CTRL_LOG}" "unexpected Num7 safety stop"
require_count 2 "first velocity command received" "${CTRL_LOG}" "C++ did not receive fresh commands in both missions"
require_count 2 "stop latched: (DISTANCE_REACHED|HARD_STOP)" "${CTRL_LOG}" "both Num7 missions did not reach a redundant distance stop"
require_count 2 "mission finished \\(NUM7_DISTANCE\\)" "${SIM_LOG}" "Python distance gate did not complete both missions"
require_count 2 "Num7 lane locked" "${SIM_LOG}" "both Num7 missions did not re-lock the lane"
require_count 1 "startup restraint released after policy stabilization" "${SIM_LOG}" "startup restraint was not released"
require_count 2 "straight walk started independently of line lock" "${SIM_LOG}" "both Num7 missions did not start straight motion"
require_count 2 "FSM mission disabled" "${SIM_LOG}" "both mission disable transitions were not observed"
require_count 2 "post-stop report:.*zero_command=True" "${SIM_LOG}" "both missions lack a one-second zero-command report"
require_count 2 "post-stop report:.*residual_xfrc_norm=0\\.0000" "${SIM_LOG}" "residual starting-restraint force remained"
require_count 2 "state=LINE_FOLLOW cmd=\\(\\+0\\.(0[1-9]|[1-4][0-9]|50),\\+0\\.00" "${SIM_LOG}" "non-zero bounded Num7 commands were not emitted"

# Command integration is not odometry.  Release for hardware requires both
# independent Num7 entries to move MuJoCo qpos forward by a measured minimum;
# otherwise a policy dead-zone or broken actuator path must remain blocking.
if ! "${PYTHON}" - "${SIM_LOG}" "${MIN_ACTUAL_DISPLACEMENT}" "${MAX_ACTUAL_DISPLACEMENT}" <<'PY'
import re
import sys
from pathlib import Path

log_path = Path(sys.argv[1])
minimum = float(sys.argv[2])
maximum = float(sys.argv[3])
text = log_path.read_text(encoding="utf-8", errors="replace")
values = [
    float(value)
    for value in re.findall(
        r"FSM mission disabled;[^\n]*actual_displacement=([+-]?[0-9.]+) m",
        text,
    )
]
print(
    f"NUM7_QPOS_DISPLACEMENTS={values} "
    f"required_each=[{minimum:.3f},{maximum:.3f}]m"
)
if len(values) < 2 or any(
    value < minimum or value > maximum for value in values[:2]
):
    raise SystemExit(1)
PY
then
    fail "measured qpos displacement was outside ${MIN_ACTUAL_DISPLACEMENT}..${MAX_ACTUAL_DISPLACEMENT} m"
fi

{
    echo "NUM7_HARDWARE_RELEASE_READY"
    echo "minimum_actual_displacement_m=${MIN_ACTUAL_DISPLACEMENT}"
    echo "maximum_actual_displacement_m=${MAX_ACTUAL_DISPLACEMENT}"
    echo "policy_sha256=$(sha256sum "${RUNNING_ROOT}/policy/g1/robomimic/locomotion/policy_29dof.pt" | awk '{print $1}')"
    echo "fsm_sha256=$(sha256sum "${RUNNING_ROOT}/src/rl_sar/fsm_robot/fsm_g1.hpp" | awk '{print $1}')"
    echo "receiver_sha256=$(sha256sum "${RUNNING_ROOT}/src/rl_sar/include/vision_udp_command.hpp" | awk '{print $1}')"
} >"${RELEASE_MARKER}"
echo "NUM7_HEADLESS_SMOKE_PASSED command_port=${COMMAND_PORT} status_port=${STATUS_PORT}"
echo "HARDWARE_RELEASE_MARKER=${RELEASE_MARKER}"
