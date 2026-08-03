#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${G1_RACE_ROOT:-${HOME}/g1_race_vision}"
RUN_SCRIPT="${PROJECT_ROOT}/scripts/run_vm_full_demo.sh"
PID_FILE="${HOME}/.g1_race_gui.pid"
LOG_FILE="${HOME}/g1_race_run.log"

if [[ ! -x "${RUN_SCRIPT}" ]]; then
    echo "找不到启动脚本：${RUN_SCRIPT}"
    exit 1
fi

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

find_mujoco_window() {
    xwininfo -root -tree 2>/dev/null |
        awk '/"MuJoCo :/ {print $1; exit}'
}

find_camera_window() {
    xwininfo -root -tree 2>/dev/null |
        awk '/"G1 robot camera/ {print $1; exit}'
}

focus_window() {
    local window_id="$1"
    local camera_id
    camera_id="$(find_camera_window)"
    # MuJoCo/OpenCV are XWayland windows.  When this launcher is invoked from
    # SSH or from a terminal on another GNOME workspace, Mutter can focus the
    # window without switching the visible workspace.  Mark both windows as
    # visible on every workspace so "start" can always bring them into view.
    xprop -id "${window_id}" -f _NET_WM_DESKTOP 32c \
        -set _NET_WM_DESKTOP 4294967295 >/dev/null 2>&1 || true
    if [[ -n "${camera_id}" ]]; then
        xprop -id "${camera_id}" -f _NET_WM_DESKTOP 32c \
            -set _NET_WM_DESKTOP 4294967295 >/dev/null 2>&1 || true
    fi
    # Lay out the external viewer and RGB-D dashboard side-by-side, then put
    # the MuJoCo viewer in front of any terminal window.
    python3 - "${window_id}" "${camera_id}" <<'PY'
import ctypes
import sys

x11 = ctypes.CDLL("libX11.so.6")
x11.XOpenDisplay.restype = ctypes.c_void_p
display = x11.XOpenDisplay(None)
if display:
    display_ptr = ctypes.c_void_p(display)
    main_window = int(sys.argv[1], 0)
    x11.XMoveResizeWindow(
        display_ptr, ctypes.c_ulong(main_window), 20, 60, 1080, 820
    )
    if len(sys.argv) > 2 and sys.argv[2]:
        camera_window = int(sys.argv[2], 0)
        x11.XMoveResizeWindow(
            display_ptr, ctypes.c_ulong(camera_window), 1130, 60, 1050, 790
        )
        x11.XMapRaised(display_ptr, ctypes.c_ulong(camera_window))
    x11.XMapRaised(display_ptr, ctypes.c_ulong(main_window))
    x11.XSetInputFocus(display_ptr, ctypes.c_ulong(main_window), 1, 0)
    x11.XFlush(display_ptr)
    x11.XCloseDisplay(display_ptr)
PY
}

stop_process_group() {
    local pid="$1"
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    for _ in {1..50}; do
        kill -0 "${pid}" 2>/dev/null || return 0
        sleep 0.1
    done
    kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
}

if [[ -f "${PID_FILE}" ]]; then
    old_pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
    if [[ "${old_pid}" =~ ^[0-9]+$ ]] && kill -0 "${old_pid}" 2>/dev/null; then
        window_id="$(find_mujoco_window)"
        if [[ -n "${window_id}" ]]; then
            if grep -Eq \
                'state=FINISHED_WINDOW_HELD|\[stop\] post-finish deceleration complete' \
                "${LOG_FILE}" 2>/dev/null; then
                echo "检测到上一轮已经完成，正在自动重置并开始新一轮……"
                stop_process_group "${old_pid}"
                sleep 4
            else
                focus_window "${window_id}"
                echo "G1 仿真已经在运行，已把 MuJoCo 窗口切到最前面。"
                echo "若机器人还在起点，请等待起立和双白线锁定；启动约 17 秒后会自动起跑。"
                exit 0
            fi
        else
            echo "检测到未完成的旧启动，正在自动清理……"
            stop_process_group "${old_pid}"
            sleep 4
        fi
    fi
    rm -f "${PID_FILE}"
fi

: >"${LOG_FILE}"
for attempt in 1 2 3; do
    echo "[launcher] startup attempt ${attempt}/3" >>"${LOG_FILE}"

    # setsid and nohup keep the simulation alive after this terminal closes.
    setsid nohup env \
        DISPLAY="${DISPLAY}" \
        XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}" \
        XAUTHORITY="${XAUTHORITY:-}" \
        bash "${RUN_SCRIPT}" >>"${LOG_FILE}" 2>&1 </dev/null &
    launcher_pid=$!
    echo "${launcher_pid}" >"${PID_FILE}"

    window_id=""
    race_ready=0
    # Window enumeration is unreliable when this script is launched from a
    # GNOME Wayland terminal with a different XAUTHORITY.  It must never be a
    # reason to kill a healthy simulation. Treat Skill 6 plus the first
    # LINE_FOLLOW status as readiness; window discovery is only for focusing.
    for _ in {1..150}; do
        if [[ -z "${window_id}" ]]; then
            window_id="$(find_mujoco_window)"
        fi
        if grep -q 'Skill 6 entered: visual 100 m sprint' "${LOG_FILE}" 2>/dev/null &&
           grep -q 'state=LINE_FOLLOW' "${LOG_FILE}" 2>/dev/null; then
            race_ready=1
            break
        fi
        kill -0 "${launcher_pid}" 2>/dev/null || break
        sleep 0.2
    done

    if kill -0 "${launcher_pid}" 2>/dev/null; then
        if [[ -z "${window_id}" ]]; then
            window_id="$(find_mujoco_window)"
        fi
        if [[ -n "${window_id}" ]]; then
            focus_window "${window_id}"
        fi
        if [[ "${race_ready}" == "1" ]]; then
            echo "G1 已进入 LINE_FOLLOW，机器人正在跑（PID ${launcher_pid}）。"
        else
            echo "G1 仿真仍在运行，但 30 秒内尚未进入 LINE_FOLLOW。"
            echo "启动器会保留现场，不会再因为查不到窗口而误杀进程。"
        fi
        echo "关闭终端不会停止仿真。停止命令：bash ~/stop_g1_race.sh"
        exit 0
    fi

    echo "[launcher] attempt ${attempt} failed; cleaning up" >>"${LOG_FILE}"
    stop_process_group "${launcher_pid}"
    rm -f "${PID_FILE}"
    # CycloneDDS may need a moment to retire the previous participant before
    # a new Unitree bridge joins the same domain/interface.
    sleep 4
done

echo "连续三次启动失败，最后的日志如下："
tail -n 40 "${LOG_FILE}" 2>/dev/null || true
exit 1
