#!/usr/bin/env bash

# Activate the reproducible Python environment and the CycloneDDS runtime used
# by both unitree_mujoco and rl_sar. Source this file from a Bash terminal:
#   source vision/scripts/activate_g1race_dds.sh

# conda's activate/deactivate hooks (qt-main_*.sh) reference variables that
# are unbound under `set -u` in the calling script.  Remember the caller's
# `nounset` state, relax it for the whole conda dance and restore it on exit.
_nounset_on=0
if [[ $- == *u* ]]; then _nounset_on=1; fi
set +u

restore_nounset() {
    if [[ ${_nounset_on} -eq 1 ]]; then set -u; fi
}
trap restore_nounset EXIT RETURN

if command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)"
elif [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
    # shellcheck source=/dev/null
    source "${HOME}/miniconda3/etc/profile.d/conda.sh"
elif [[ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]]; then
    # shellcheck source=/dev/null
    source "${HOME}/anaconda3/etc/profile.d/conda.sh"
else
    echo "错误：找不到 Conda。请先安装 Anaconda/Miniconda，或把 conda 加入 PATH。" >&2
    return 1 2>/dev/null || exit 1
fi

if ! conda env list | awk '{print $1}' | grep -qx 'g1race'; then
    echo "错误：Conda 环境 g1race 不存在。" >&2
    echo "请在仓库根目录执行：conda env create -f vision/environment.yml" >&2
    return 1 2>/dev/null || exit 1
fi
# conda's activate/deactivate hooks (qt-main_*.sh) reference variables that
# are unbound under `set -u` in the calling script.  Temporarily relax
# `nounset` around `conda activate` and restore the caller's option afterwards.
conda activate g1race

# unitree_sdk2py 1.0.1 must use the matching CycloneDDS 0.10.2 library.
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-${HOME}/cyclonedds-0.10.2/install}"
if [[ ! -f "${CYCLONEDDS_HOME}/lib/libddsc.so" ]]; then
    echo "错误：找不到 CycloneDDS 0.10.2：${CYCLONEDDS_HOME}/lib/libddsc.so" >&2
    echo "请执行：bash vision/scripts/install_cyclonedds_runtime.sh" >&2
    return 1 2>/dev/null || exit 1
fi

export LD_LIBRARY_PATH="${CYCLONEDDS_HOME}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

# Disable shared memory. The Python bridge and rl_sar communicate over the
# loopback interface; this avoids the mixed Python/C++ dds_write crash.
DDS_NO_SHM='<Domain id="any"><SharedMemory><Enable>false</Enable></SharedMemory></Domain>'
export CYCLONEDDS_URI="${G1_RACE_CYCLONEDDS_URI:-${DDS_NO_SHM}}"

# rl_real_g1 initializes Unitree DDS with domain 0 in its C++ entry point.
export G1_RACE_DOMAIN_ID=0
export G1_RACE_INTERFACE="${G1_RACE_INTERFACE:-lo}"

echo "已激活 Conda 环境：${CONDA_DEFAULT_ENV}"
echo "CycloneDDS：${CYCLONEDDS_HOME}（domain=${G1_RACE_DOMAIN_ID}, interface=${G1_RACE_INTERFACE}, shared-memory=off）"
