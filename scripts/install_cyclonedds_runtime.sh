#!/usr/bin/env bash
set -euo pipefail

VERSION="0.10.2"
SOURCE_ROOT="${HOME}/cyclonedds-${VERSION}"
PREFIX="${SOURCE_ROOT}/install"
ARCHIVE="$(mktemp --suffix=.tar.gz)"
EXPECTED_SHA256="bc84e137e0c8a055b8cd97fbeafec94e36de1b0c2e88800896a82384fd867ae5"

cleanup() {
    rm -f "${ARCHIVE}"
}
trap cleanup EXIT

echo "下载 CycloneDDS ${VERSION} 源码……"
curl -fL --retry 3 --connect-timeout 15 \
    "https://github.com/eclipse-cyclonedds/cyclonedds/archive/refs/tags/${VERSION}.tar.gz" \
    -o "${ARCHIVE}"
echo "${EXPECTED_SHA256}  ${ARCHIVE}" | sha256sum --check --status

mkdir -p "${SOURCE_ROOT}"
tar -xzf "${ARCHIVE}" --strip-components=1 -C "${SOURCE_ROOT}"

echo "编译不含 Iceoryx 共享内存的 CycloneDDS ${VERSION}……"
cmake -S "${SOURCE_ROOT}" -B "${SOURCE_ROOT}/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF \
    -DENABLE_SHM=OFF
cmake --build "${SOURCE_ROOT}/build" --parallel 2
cmake --install "${SOURCE_ROOT}/build"

if command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)"
elif [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
    # shellcheck source=/dev/null
    source "${HOME}/miniconda3/etc/profile.d/conda.sh"
elif [[ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]]; then
    # shellcheck source=/dev/null
    source "${HOME}/anaconda3/etc/profile.d/conda.sh"
else
    echo "找不到 Conda。"
    exit 1
fi
conda activate g1race

echo "针对同一份 CycloneDDS ${VERSION} 重新编译 Python 绑定……"
export CYCLONEDDS_HOME="${PREFIX}"
python -m pip install \
    --force-reinstall \
    --no-cache-dir \
    --no-binary=:all: \
    --no-deps \
    "cyclonedds==${VERSION}"

python - <<'PY'
from importlib.metadata import version
from pathlib import Path

from cyclonedds.__library__ import library_path

expected = Path.home() / "cyclonedds-0.10.2" / "install" / "lib" / "libddsc.so"
actual = Path(library_path).resolve()
if version("cyclonedds") != "0.10.2" or actual != expected.resolve():
    raise SystemExit(
        f"CycloneDDS 校验失败：Python={version('cyclonedds')}, library={actual}"
    )
print(f"CycloneDDS 运行时校验通过：Python 0.10.2 -> {actual}")
PY
