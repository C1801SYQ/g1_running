#!/usr/bin/env bash
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_SCRIPT="${SCRIPT_ROOT}/scripts/run_vm_full_demo.sh"
if [[ ! -f "${RUN_SCRIPT}" &&
      -f "${HOME}/g1_race_vision/scripts/run_vm_full_demo.sh" ]]; then
    # Support copying this small launcher to ~/start_g1_race.sh or
    # ~/run_g1_race.sh while keeping the maintained scripts in one place.
    RUN_SCRIPT="${HOME}/g1_race_vision/scripts/run_vm_full_demo.sh"
fi

if [[ ! -f "${RUN_SCRIPT}" ]]; then
    echo "找不到启动脚本：${RUN_SCRIPT}"
    exit 1
fi

echo "本启动器现在使用前台交互模式，确保 rl_sar 能收到键盘 0、1、6。"
echo "请不要关闭这个终端；MuJoCo 和机器人相机窗口会另外打开。"
exec bash "${RUN_SCRIPT}"
