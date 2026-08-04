#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_ROOT="${G1_RUNNING_ROOT:-${HOME}/unitree_ws/g1_running}"
REPOSITORY="https://github.com/C1801SYQ/g1_running.git"
COMMIT="4d06065aa9445b8af4db5d465fa79f67736e5e36"
POLICY_SHA256="e3705c5ce94c32c00a4e1900a5e2024deea3a4e22f7b20fbc268f3bcb7e51e57"
INTEGRATION="${PROJECT_ROOT}/integrations/g1_running"

if [[ ! -d "${TARGET_ROOT}/.git" ]]; then
    if [[ -e "${TARGET_ROOT}" ]]; then
        echo "目标路径已经存在，但不是 Git 仓库：${TARGET_ROOT}"
        exit 1
    fi
    mkdir -p "$(dirname "${TARGET_ROOT}")"
    echo "克隆 g1_running 到 ${TARGET_ROOT} ..."
    git clone "${REPOSITORY}" "${TARGET_ROOT}"
fi

current_url="$(git -C "${TARGET_ROOT}" remote get-url origin)"
if [[ "${current_url}" != "${REPOSITORY}" &&
      "${current_url}" != "git@github.com:C1801SYQ/g1_running.git" ]]; then
    echo "现有仓库的 origin 不是 ${REPOSITORY}：${current_url}"
    exit 1
fi

if ! git -C "${TARGET_ROOT}" cat-file -e "${COMMIT}^{commit}" 2>/dev/null; then
    echo "获取固定版本 ${COMMIT} ..."
    git -C "${TARGET_ROOT}" fetch origin "${COMMIT}"
fi

TARGET_HEADER="${TARGET_ROOT}/rl_sar/src/rl_sar/include/vision_udp_command.hpp"
if [[ -n "$(git -C "${TARGET_ROOT}" status --porcelain)" ]]; then
    current_commit="$(git -C "${TARGET_ROOT}" rev-parse HEAD)"
    if [[ "${current_commit}" != "${COMMIT}" ]]; then
        echo "已有补丁基于其他版本 ${current_commit}，期望 ${COMMIT}。"
        exit 1
    fi
    unexpected="$(
        git -C "${TARGET_ROOT}" status --porcelain |
            grep -Ev \
                'rl_sar/src/rl_sar/(fsm_robot/fsm_g1.hpp|include/rl_real_g1.hpp|include/vision_udp_command.hpp|src/rl_real_g1.cpp)$' \
            || true
    )"
    if [[ -n "${unexpected}" ]]; then
        echo "g1_running 工作区存在其他未提交修改。为避免覆盖，安装已停止："
        printf '%s\n' "${unexpected}"
        exit 1
    fi
    if [[ -f "${TARGET_HEADER}" ]] &&
       ! cmp -s "${INTEGRATION}/vision_udp_command.hpp" "${TARGET_HEADER}"; then
        echo "现有 vision_udp_command.hpp 与本项目版本不同，安装已停止。"
        exit 1
    fi
else
    git -C "${TARGET_ROOT}" checkout --detach "${COMMIT}"
fi

install -m 0644 \
    "${INTEGRATION}/vision_udp_command.hpp" \
    "${TARGET_HEADER}"

apply_patch_once() {
    local patch_file="$1"
    local marker_file="$2"
    local marker_text="$3"
    if grep -Fq "${marker_text}" "${TARGET_ROOT}/${marker_file}"; then
        echo "已经应用，跳过：$(basename "${patch_file}")"
    elif git -C "${TARGET_ROOT}" apply --check "${patch_file}" 2>/dev/null; then
        git -C "${TARGET_ROOT}" apply "${patch_file}"
        echo "已应用：$(basename "${patch_file}")"
    else
        echo "补丁与固定版本不匹配：${patch_file}"
        exit 1
    fi
}

apply_patch_once \
    "${INTEGRATION}/g1_running_vision.patch" \
    "rl_sar/src/rl_sar/include/rl_real_g1.hpp" \
    "VisionUdpCommandReceiver vision_udp_command"
apply_patch_once \
    "${INTEGRATION}/g1_running_quiet_getup.patch" \
    "rl_sar/src/rl_sar/fsm_robot/fsm_g1.hpp" \
    '2.0f, "", true'
apply_patch_once \
    "${INTEGRATION}/g1_running_sim_autostart.patch" \
    "rl_sar/src/rl_sar/fsm_robot/fsm_g1.hpp" \
    'std::getenv("G1_AUTO_RUNNING")'
apply_patch_once \
    "${INTEGRATION}/g1_running_quiet_run.patch" \
    "rl_sar/src/rl_sar/fsm_robot/fsm_g1.hpp" \
    'std::getenv("G1_VERBOSE_RUNNING")'
apply_patch_once \
    "${INTEGRATION}/g1_running_skill6.patch" \
    "rl_sar/src/rl_sar/fsm_robot/fsm_g1.hpp" \
    'RLFSMStateRLVisionSprint100m'

policy_file="${TARGET_ROOT}/rl_sar/policy/g1/running/policy.pt"
actual_sha256="$(sha256sum "${policy_file}" | awk '{print $1}')"
if [[ "${actual_sha256}" != "${POLICY_SHA256}" ]]; then
    echo "策略文件校验失败：${policy_file}"
    echo "期望：${POLICY_SHA256}"
    echo "实际：${actual_sha256}"
    exit 1
fi

echo "策略校验通过，开始编译 rl_real_g1 ..."
cd "${TARGET_ROOT}/rl_sar"

# The upstream CMake target builds every supported robot by default. On a G1
# workstation that unnecessarily pulls in A1/LCM headers and can fail before
# the G1 target is reached. Prepare the inference runtimes, then build only
# rl_real_g1. The standalone official SDK supplies one common header omitted
# from the vendored SDK snapshot in the pinned upstream revision.
bash scripts/download_inference_runtime.sh
UNITREE_SDK2_ROOT="${UNITREE_SDK2_ROOT:-${HOME}/unitree_ws/unitree_sdk2}"
if [[ ! -f "${UNITREE_SDK2_ROOT}/include/unitree/common/log/log.hpp" ]]; then
    echo "找不到官方 unitree_sdk2：${UNITREE_SDK2_ROOT}"
    exit 1
fi
cmake src/rl_sar -B cmake_build \
    -DUSE_CMAKE=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-I${UNITREE_SDK2_ROOT}/include"
cmake --build cmake_build \
    --target rl_real_g1 \
    -j"${G1_BUILD_JOBS:-2}"

binary="${TARGET_ROOT}/rl_sar/cmake_build/bin/rl_real_g1"
if [[ ! -x "${binary}" ]]; then
    echo "编译完成，但没有找到 ${binary}"
    exit 1
fi

echo
echo "g1_running 已安装完成：${binary}"
echo "下一步：bash ${PROJECT_ROOT}/scripts/run_vm_full_demo.sh"
