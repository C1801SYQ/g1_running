#!/usr/bin/env bash
set -euo pipefail

PID_FILE="${HOME}/.g1_race_gui.pid"

if [[ ! -f "${PID_FILE}" ]]; then
    echo "没有找到正在运行的 G1 仿真。"
    exit 0
fi

launcher_pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
if [[ ! "${launcher_pid}" =~ ^[0-9]+$ ]]; then
    rm -f "${PID_FILE}"
    echo "PID 文件无效，已清理。"
    exit 0
fi

if kill -0 "${launcher_pid}" 2>/dev/null; then
    kill -TERM -- "-${launcher_pid}" 2>/dev/null || kill -TERM "${launcher_pid}" 2>/dev/null || true
    for _ in {1..50}; do
        kill -0 "${launcher_pid}" 2>/dev/null || break
        sleep 0.1
    done
    if kill -0 "${launcher_pid}" 2>/dev/null; then
        kill -KILL -- "-${launcher_pid}" 2>/dev/null || kill -KILL "${launcher_pid}" 2>/dev/null || true
    fi
fi

rm -f "${PID_FILE}"
# Give DDS discovery sockets time to disappear before a possible restart.
sleep 4
echo "G1 MuJoCo 仿真已停止，现在可以安全地重新启动。"
