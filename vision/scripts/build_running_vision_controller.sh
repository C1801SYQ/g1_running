#!/usr/bin/env bash
# Build an additive running-vision controller without editing upstream files.
# The forced include registers fsm_g1_running_adapter.hpp for rl_real_g1; the
# output must be compiled on the target architecture before robot deployment.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PROJECT_ROOT="${G1_RUNNING_VISION_REPO:-${DEFAULT_PROJECT_ROOT}}"
RL_SAR_ROOT="${PROJECT_ROOT}/rl_sar"
ADAPTER="${RL_SAR_ROOT}/src/rl_sar/fsm_robot/fsm_g1_running_adapter.hpp"
BUILD_DIR="${G1_RUNNING_VISION_BUILD_DIR:-${RL_SAR_ROOT}/cmake_build_running_vision_adapter}"
WRAPPER_DIR="${PROJECT_ROOT}/vision/cmake/running_vision_rl_sar"

if [[ ! -f "${ADAPTER}" ]]; then
  echo "Adapter not found: ${ADAPTER}" >&2
  exit 1
fi
if [[ ! -f "${WRAPPER_DIR}/CMakeLists.txt" ]]; then
  echo "CMake wrapper not found: ${WRAPPER_DIR}/CMakeLists.txt" >&2
  exit 1
fi

cmake -S "${WRAPPER_DIR}" -B "${BUILD_DIR}" \
  -DRL_SAR_SOURCE_DIR="${RL_SAR_ROOT}/src/rl_sar" \
  -DG1_RUNNING_ADAPTER="${ADAPTER}" \
  -DUSE_CMAKE=ON
cmake --build "${BUILD_DIR}" -j"${G1_BUILD_JOBS:-2}" --target rl_real_g1

BIN="${BUILD_DIR}/bin/rl_real_g1"
file "${BIN}"
echo "Built additive running-vision controller: ${BIN}"
echo "Do not copy this binary across architectures; build again on ARM64."
