#!/usr/bin/env bash
# Headless Skill 6 (100 m sprint) joint-simulation smoke.
#
# Verifies, without any GUI:
#   1. The C++ controller enters the visual sprint (Skill 6) mode.
#   2. The vision node locks the lane and emits bounded correction commands.
#   3. The robot crosses the 100 m finish line without falling.
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
OUTPUT_DIR="${VISION_ROOT}/output/skill6_headless_smoke"
SIM_LOG="${OUTPUT_DIR}/simulation.log"
CTRL_LOG="${OUTPUT_DIR}/controller.log"
SPEED="${G1_SKILL6_SMOKE_SPEED:-5.10}"
MIN_FINISH_X="${G1_SKILL6_MIN_FINISH_X:-99.0}"
RUN_SECONDS="${1:-55}"

free_udp_port() {
    "${PYTHON}" -c 'import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1]); s.close()'
}
COMMAND_PORT="${G1_SKILL6_SMOKE_COMMAND_PORT:-$(free_udp_port)}"
STATUS_PORT="${G1_SKILL6_SMOKE_STATUS_PORT:-$(free_udp_port)}"
if [[ "${COMMAND_PORT}" == "${STATUS_PORT}" ]]; then
    STATUS_PORT="$(free_udp_port)"
fi

export MUJOCO_GL="${MUJOCO_GL:-egl}"
export DISPLAY="${DISPLAY:-:0}"
GUI_MODE="${G1_SKILL6_SMOKE_GUI:-0}"

mkdir -p "${OUTPUT_DIR}"
rm -f "${SIM_LOG}" "${CTRL_LOG}"

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

"${PYTHON}" scripts/run_unitree_camera_sim.py \
    --unitree-mujoco "${MUJOCO_ROOT}" \
    --scene "${MUJOCO_ROOT}/unitree_robots/g1/scene_race_29dof.xml" \
    --domain-id 0 \
    --interface lo \
    --udp-port "${COMMAND_PORT}" \
    --status-port "${STATUS_PORT}" \
    --camera-fps 30 \
    --depth-fps 10 \
    --viewer-fps 30 \
    --debug-fps 2 \
    --mission sprint100m \
    --speed "${SPEED}" \
    --forward-accel 3.00 \
    --max-yaw-rate 0.35 \
    --max-yaw-accel 1.50 \
    --lateral-kp 1.20 \
    --heading-kp 0.20 \
    --imu-heading-kp 1.20 \
    --finish-line-x 100.0 \
    --stop-at-x 115.0 \
    --finish-approach-distance 5.0 \
    --fall-height 0.45 \
    --fall-confirm-seconds 0.60 \
    $([ "${GUI_MODE}" = "1" ] && echo "" || echo "--headless") \
    --run-seconds "${RUN_SECONDS}" \
    --status-interval 0.5 \
    --startup-support-seconds 12.0 \
    --startup-support-fade-seconds 2.0 \
    --startup-support-until-skill6 \
    >"${SIM_LOG}" 2>&1 &
sim_pid=$!

ctrl_pid=""
cleanup() {
    [[ -z "${ctrl_pid}" ]] || kill "${ctrl_pid}" 2>/dev/null || true
    kill "${sim_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 1.0
cd "${RUNNING_ROOT}"
"${PYTHON}" "${VISION_ROOT}/scripts/skill6_pty_driver.py" \
    "${RUNNING_BINARY}" "${CTRL_LOG}" "${RUN_SECONDS}" \
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
grep -aE "Skill 6|sprint|SPRINT|finish|FALL|fall|lane|vision|correction|cmd=" \
    "${SIM_LOG}" | head -n 80 || true
echo "=== controller key events ==="
tr '\r' '\n' <"${CTRL_LOG}" | grep -aE "Skill 6|sprint|Sprint|GetUp|FSM|Entered|Locomotion|ERROR|Error" | head -n 40 || true

fail() {
    echo "SKILL6_HEADLESS_SMOKE_FAILED: $*" >&2
    exit 1
}
require_count() {
    local minimum="$1"; local pattern="$2"; local file="$3"; local label="$4"
    local count; count="$(grep -aEc "${pattern}" "${file}" || true)"
    if (( count < minimum )); then
        fail "${label}: expected >=${minimum}, found ${count}"
    fi
}
reject_pattern() {
    local pattern="$1"; local file="$2"; local label="$3"
    if grep -aEq "${pattern}" "${file}"; then
        fail "${label}"
    fi
}

[[ "${sim_status}" -eq 0 ]] || fail "simulation exited ${sim_status}"

require_count 1 "velocity receiver listening on 127\\.0\\.0\\.1:${COMMAND_PORT}" "${CTRL_LOG}" "command receiver did not bind the isolated port"
reject_pattern "could not bind|Address already in use" "${CTRL_LOG}" "UDP/status port bind failure"
require_count 1 "SPRINT100M entered" "${SIM_LOG}" "vision did not confirm Skill 6 sprint mode"
require_count 1 "\\[finish\\] crossed x=" "${SIM_LOG}" "robot did not cross the 100 m finish line"
reject_pattern "FALL|falling|fell|fall confirmed" "${SIM_LOG}" "robot fell during the sprint"

# The robot must actually advance ~100 m (finish line), not just emit commands.
if ! "${PYTHON}" - "${SIM_LOG}" "${MIN_FINISH_X}" <<'PY'
import re
import sys
from pathlib import Path
text = Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace")
values = [float(v) for v in re.findall(r"\[finish\] crossed x=([+-]?[0-9.]+) m", text)]
print(f"SKILL6_FINISH_X={values}")
if not values or max(values) < float(sys.argv[2]):
    raise SystemExit(1)
PY
then
    fail "finish x below ${MIN_FINISH_X} m"
fi

echo "SKILL6_HEADLESS_SMOKE_PASSED"
