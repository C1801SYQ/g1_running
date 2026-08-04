#!/usr/bin/env bash
set -euo pipefail

VISION_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPOSITORY_ROOT="$(cd "${VISION_ROOT}/.." && pwd)"
DEFAULT_RUNNING_REPOSITORY="${HOME}/unitree_ws/g1_running"
if [[ -f "${REPOSITORY_ROOT}/rl_sar/src/rl_sar/include/vision_udp_command.hpp" ]]; then
    DEFAULT_RUNNING_REPOSITORY="${REPOSITORY_ROOT}"
fi
RUNNING_REPOSITORY="${G1_RUNNING_ROOT:-${DEFAULT_RUNNING_REPOSITORY}}"
RL_SAR_ROOT="${RUNNING_REPOSITORY}/rl_sar"
POLICY_FILE="${RL_SAR_ROOT}/policy/g1/running/policy.pt"
POLICY_SHA256="e3705c5ce94c32c00a4e1900a5e2024deea3a4e22f7b20fbc268f3bcb7e51e57"

if [[ ! -f "${RL_SAR_ROOT}/src/rl_sar/include/vision_udp_command.hpp" ]]; then
    echo "找不到 Skill 6 的视觉 UDP 接口；请确认当前仓库包含视觉扩展。"
    exit 1
fi
if [[ ! -f "${POLICY_FILE}" ]]; then
    echo "找不到原仓库的 running policy：${POLICY_FILE}"
    exit 1
fi

actual_sha256="$(sha256sum "${POLICY_FILE}" | awk '{print $1}')"
if [[ "${actual_sha256}" != "${POLICY_SHA256}" ]]; then
    echo "running policy 校验失败。脚本不会替换或修改原模型。"
    echo "期望：${POLICY_SHA256}"
    echo "实际：${actual_sha256}"
    exit 1
fi

cd "${RL_SAR_ROOT}"
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

binary="${RL_SAR_ROOT}/cmake_build/bin/rl_real_g1"
if [[ ! -x "${binary}" ]]; then
    echo "编译结束，但没有找到：${binary}"
    exit 1
fi

echo "Skill 5/6 共用控制器编译完成：${binary}"
