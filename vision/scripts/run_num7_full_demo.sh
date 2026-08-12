#!/usr/bin/env bash
# Skill 7 (Num7) joint simulation: 1.0 m vision walk with the locomotion
# policy at 0.50 m/s and automatic stop + return to the
# stable locomotion state.
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ROOT="${G1_RACE_ROOT:-${SCRIPT_ROOT}}"
REPOSITORY_ROOT="$(cd "${PROJECT_ROOT}/.." && pwd)"

# Num7 requires the repository root to be explicitly set for reproducible runs.
UNITREE_ROOT="${UNITREE_ROOT:-/home/ubuntu}"
MUJOCO_ROOT="${UNITREE_ROOT}/unitree_mujoco"
RUNNING_REPOSITORY="${G1_RUNNING_ROOT:-/home/ubuntu/g1_running}"
RUNNING_ROOT="${RUNNING_REPOSITORY}/rl_sar"
RUNNING_BINARY="${RUNNING_ROOT}/cmake_build/bin/rl_real_g1"
STATUS_PORT="${G1_VISION_STATUS_PORT:-15002}"

if [[ ! -x "${RUNNING_BINARY}" ]]; then
    echo "找不到已经编译的 g1_running 控制器：${RUNNING_BINARY}"
    echo "请先执行：bash ${PROJECT_ROOT}/scripts/build_skill6.sh"
    exit 1
fi
if [[ ! -f "${RUNNING_ROOT}/policy/g1/robomimic/locomotion/config.yaml" ]]; then
    echo "找不到 locomotion 策略配置："
    echo "  ${RUNNING_ROOT}/policy/g1/robomimic/locomotion/config.yaml"
    exit 1
fi
if [[ ! -d "${MUJOCO_ROOT}/simulate_python" ]]; then
    echo "找不到 unitree_mujoco：${MUJOCO_ROOT}"
    echo "请设置 UNITREE_ROOT=/home/ubuntu"
    exit 1
fi
if pgrep -x unitree_mujoco >/dev/null &&
   [[ "${ALLOW_EXISTING_UNITREE_SIM:-0}" != "1" ]]; then
    echo "检测到旧的 unitree_mujoco 正在运行。请先关闭旧窗口再执行。"
    exit 1
fi

# shellcheck source=activate_g1race_dds.sh
source "${PROJECT_ROOT}/scripts/activate_g1race_dds.sh"

export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
if [[ -z "${XAUTHORITY:-}" && -d "${XDG_RUNTIME_DIR}" ]]; then
    XAUTHORITY="$(
        find "${XDG_RUNTIME_DIR}" -maxdepth 1 \
            -name '.mutter-Xwaylandauth.*' -type f \
            -printf '%T@ %p\n' 2>/dev/null |
            sort -nr | head -n 1 | cut -d' ' -f2-
    )"
    export XAUTHORITY
fi

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

sim_pid=""
cleanup() {
    trap - EXIT INT TERM
    [[ -z "${sim_pid}" ]] || kill "${sim_pid}" 2>/dev/null || true
    [[ -z "${sim_pid}" ]] || wait "${sim_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "启动 Num7 联合仿真（1.0 m 视觉行走，0.50 m/s）……"
python scripts/run_unitree_camera_sim.py \
    --unitree-mujoco "${MUJOCO_ROOT}" \
    --scene "${MUJOCO_ROOT}/unitree_robots/g1/scene_race_29dof.xml" \
    --domain-id "${G1_RACE_DOMAIN_ID}" \
    --interface "${G1_RACE_INTERFACE}" \
    --status-port "${STATUS_PORT}" \
    --camera-fps 30 \
    --depth-fps 10 \
    --viewer-fps 24 \
    --debug-fps 5 \
    --show-debug \
    --debug-snapshot "${PROJECT_ROOT}/debug_output/num7_dashboard_latest.png" \
    --mission walk0p5m \
    --speed 0.50 \
    --forward-accel 0.50 \
    --max-yaw-rate 0.25 \
    --max-yaw-accel 0.50 \
    --lateral-kp 0.65 \
    --heading-kp 0.20 \
    --imu-heading-kp 1.20 \
    --error-filter-alpha 0.32 \
    --vision-enable-delay 0.0 \
    --num7-target-m "${G1_NUM7_TARGET_M:-1.00}" \
    --num7-max-duration-s "${G1_NUM7_MAX_DURATION_S:-7.0}" \
    --num7-distance-scale "${G1_NUM7_DISTANCE_SCALE:-0.80}" \
    --startup-support-seconds 16.0 \
    --startup-support-fade-seconds 0.60 \
    --startup-support-until-mission \
    --fall-height 0.45 \
    --fall-confirm-seconds 0.60 &
sim_pid=$!

sleep 1
if ! kill -0 "${sim_pid}" 2>/dev/null; then
    echo "MuJoCo/DDS process exited during startup; controller was not started."
    wait "${sim_pid}" 2>/dev/null || true
    exit 1
fi
echo
echo "MuJoCo/DDS 已启动。现在这个终端就是 rl_sar 控制终端。"
echo "请把键盘输入留在本终端，按下面顺序操作："
echo "  0  从初始 Passive 进入 GetUp（起立）"
echo "  1  等机器人完全站稳后，进入状态 1"
echo "  7  在状态 1 中进入视觉行走 Skill 7（约 1.0 m）"
echo "注意：在 Passive 或起立过程中按 7 不会启动，也不会自动起立。"
echo "停止全部程序：在本终端按 Ctrl+C。"
echo
echo "启动 C1801SYQ/g1_running 的 rl_sar 29 自由度控制器……"
cd "${RUNNING_ROOT}"
G1_VISION_MAX_VX="${G1_VISION_MAX_VX:-0.50}" \
G1_VISION_MAX_WZ="${G1_VISION_MAX_WZ:-0.25}" \
G1_NUM7_TARGET_M="${G1_NUM7_TARGET_M:-1.00}" \
G1_NUM7_STOP_MARGIN_M="${G1_NUM7_STOP_MARGIN_M:-0.0}" \
G1_NUM7_DISTANCE_SCALE="${G1_NUM7_DISTANCE_SCALE:-0.80}" \
G1_NUM7_MAX_DURATION_S="${G1_NUM7_MAX_DURATION_S:-7.0}" \
G1_NUM7_START_TIMEOUT_S="${G1_NUM7_START_TIMEOUT_S:-2.0}" \
G1_NUM7_SETTLE_S="${G1_NUM7_SETTLE_S:-0.5}" \
G1_VISION_STATUS_PORT="${STATUS_PORT}" \
    "${RUNNING_BINARY}" "${G1_RACE_INTERFACE}"
