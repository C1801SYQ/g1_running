#!/usr/bin/env bash
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ROOT="${G1_RACE_ROOT:-${SCRIPT_ROOT}}"
REPOSITORY_ROOT="$(cd "${PROJECT_ROOT}/.." && pwd)"
UNITREE_ROOT="${UNITREE_ROOT:-${HOME}/unitree_ws}"
MUJOCO_ROOT="${UNITREE_ROOT}/unitree_mujoco"
DEFAULT_RUNNING_REPOSITORY="${UNITREE_ROOT}/g1_running"
if [[ -f "${REPOSITORY_ROOT}/rl_sar/src/rl_sar/include/vision_udp_command.hpp" ]]; then
    # Also support a repository where the integration patch has deliberately
    # been applied in place.
    DEFAULT_RUNNING_REPOSITORY="${REPOSITORY_ROOT}"
fi
RUNNING_REPOSITORY="${G1_RUNNING_ROOT:-${DEFAULT_RUNNING_REPOSITORY}}"
RUNNING_ROOT="${RUNNING_REPOSITORY}/rl_sar"
RUNNING_BINARY="${RUNNING_ROOT}/cmake_build/bin/rl_real_g1"

if [[ ! -x "${RUNNING_BINARY}" ]]; then
    echo "找不到已经编译的 g1_running 控制器：${RUNNING_BINARY}"
    echo "请先执行：bash ${PROJECT_ROOT}/scripts/build_skill6.sh"
    exit 1
fi
if [[ ! -f "${RUNNING_ROOT}/policy/g1/running/policy.pt" ]]; then
    echo "找不到跑步策略：${RUNNING_ROOT}/policy/g1/running/policy.pt"
    exit 1
fi
if [[ ! -d "${MUJOCO_ROOT}/simulate_python" ]]; then
    echo "找不到 unitree_mujoco：${MUJOCO_ROOT}"
    exit 1
fi
if pgrep -x unitree_mujoco >/dev/null &&
   [[ "${ALLOW_EXISTING_UNITREE_SIM:-0}" != "1" ]]; then
    echo "检测到旧的 unitree_mujoco 正在运行。请先关闭旧窗口再执行。"
    exit 1
fi

# Keep Python, the Unitree bridge and rl_sar on the same Conda/DDS setup.
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

echo "启动 G1、D435i 类 RGB 相机、白线识别和视觉纠偏……"
python scripts/run_unitree_camera_sim.py \
    --unitree-mujoco "${MUJOCO_ROOT}" \
    --scene "${MUJOCO_ROOT}/unitree_robots/g1/scene_race_29dof.xml" \
    --domain-id "${G1_RACE_DOMAIN_ID}" \
    --interface "${G1_RACE_INTERFACE}" \
    --camera-fps 30 \
    --depth-fps 10 \
    --viewer-fps 24 \
    --debug-fps 5 \
    --show-debug \
    --debug-snapshot "${HOME}/g1_camera_dashboard_latest.png" \
    --speed "${G1_RACE_SPEED:-5.10}" \
    --forward-accel "${G1_RACE_ACCEL:-1.20}" \
    --vision-enable-delay 17.0 \
    --finish-line-x 100.0 \
    --stop-at-x 115.0 \
    --finish-approach-distance 5.0 \
    --keep-open-after-finish \
    --startup-support-seconds 16.0 \
    --startup-support-fade-seconds 3.0 \
    --fall-height 0.45 \
    --fall-confirm-seconds 0.60 &
sim_pid=$!

# Give the Python DDS participant one second to initialize.  Do not wait for
# an X11 window here: under GNOME Wayland a terminal can have an XAUTHORITY
# that cannot enumerate XWayland windows even though MuJoCo is visible.  That
# old five-second delay allowed the startup support to expire during GetUp.
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
echo "  6  在状态 1 中进入视觉百米冲刺 Skill 6"
echo "注意：在 Passive 或起立过程中按 6 不会启动，也不会自动起立。"
echo "停止全部程序：在本终端按 Ctrl+C。"
echo
echo "启动 C1801SYQ/g1_running 的 rl_sar 29 自由度控制器……"
cd "${RUNNING_ROOT}"
G1_VISION_MAX_VX="${G1_VISION_MAX_VX:-5.10}" \
    "${RUNNING_BINARY}" "${G1_RACE_INTERFACE}"
