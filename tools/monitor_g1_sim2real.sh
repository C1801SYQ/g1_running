#!/usr/bin/env bash
set -euo pipefail

# Read-only monitor for one G1 sim2real run.  All paths are supplied by the
# launcher so this script cannot silently attach to another experiment.
ROOT_DIR="${G1_RUNNING_ROOT:-/home/ubuntu/g1_running}"
TARGET_SESSION="${TARGET_TMUX_SESSION:?TARGET_TMUX_SESSION is required}"
TRAIN_LOG="${TRAIN_LOG:?TRAIN_LOG is required}"
RUN_DIR="${RUN_DIR:?RUN_DIR is required}"
MONITOR_LOG="${MONITOR_LOG:?MONITOR_LOG is required}"
POLL_SECONDS="${POLL_SECONDS:-3600}"

mkdir -p "$(dirname "$MONITOR_LOG")"

latest_line() {
    local pattern="$1"
    rg -n "$pattern" "$TRAIN_LOG" 2>/dev/null | tail -1 | sed 's/^[0-9]*://' || true
}

latest_checkpoint() {
    find "$RUN_DIR" -maxdepth 2 -type f -name 'model_*.pt' -printf '%T@ %p\n' 2>/dev/null \
        | sort -n | tail -1 | cut -d' ' -f2- || true
}

report_once() {
    local now log_mtime log_age checkpoint
    now="$(date '+%F %T %z')"
    if [[ -f "$TRAIN_LOG" ]]; then
        log_mtime="$(stat -c '%y' "$TRAIN_LOG")"
        log_age="$(( $(date +%s) - $(stat -c '%Y' "$TRAIN_LOG") ))"
    else
        log_mtime="missing"
        log_age="-1"
    fi
    checkpoint="$(latest_checkpoint)"
    local checkpoint_dir=""
    local agent_cfg=""
    if [[ -n "$checkpoint" ]]; then
        checkpoint_dir="$(dirname "$checkpoint")"
        agent_cfg="$checkpoint_dir/params/agent.yaml"
    fi

    {
        printf '\n[%s] G1 sim2real monitor\n' "$now"
        printf 'tmux=%s log_age_seconds=%s log_mtime=%s\n' \
            "$(tmux has-session -t "$TARGET_SESSION" 2>/dev/null && echo alive || echo stopped)" \
            "$log_age" "$log_mtime"
        printf 'latest_checkpoint=%s\n' "${checkpoint:-none}"
        printf '%s\n' "$(latest_line 'Learning iteration' | sed 's/^[[:space:]]*//')"
        printf '%s\n' "$(latest_line 'Mean reward:' | sed 's/^[[:space:]]*//')"
        printf '%s\n' "$(latest_line 'Mean episode length:' | sed 's/^[[:space:]]*//')"
        printf '%s\n' "$(latest_line 'Mean action noise std' | sed 's/^[[:space:]]*//')"
        if [[ -f "$agent_cfg" ]]; then
            printf 'configured_learning_rate=%s\n' "$(rg -n 'learning_rate:' "$agent_cfg" | tail -1 | sed 's/.*learning_rate:[[:space:]]*//')"
        fi
        for band in speed_0_1p2 speed_1p2_3 speed_3_4p5 speed_4p5_5p1 speed_5p1_7; do
            printf '[%s]\n' "$band"
            for metric in sample_fraction command_x actual_x abs_error tracking_ratio; do
                printf '%s\n' "$(latest_line "${band}_${metric}" | sed 's/^[[:space:]]*//')"
            done
        done
        for metric in error_vel_xy error_vel_yaw; do
            printf '%s\n' "$(latest_line "Metrics/base_velocity/${metric}" | sed 's/^[[:space:]]*//')"
        done
        for term in time_out base_height bad_orientation; do
            printf '%s\n' "$(latest_line "Episode_Termination/${term}" | sed 's/^[[:space:]]*//')"
        done
        for term in straight_line_position root_height_rate action_jerk waist_velocity low_speed_height high_speed_posture high_speed_undertracking; do
            printf '%s\n' "$(latest_line "Episode_Reward/${term}" | sed 's/^[[:space:]]*//')"
        done

        if [[ -f "$TRAIN_LOG" ]]; then
            local errors
            errors="$(tail -n 2500 "$TRAIN_LOG" | rg -n 'Traceback|RuntimeError|out of memory|CUDA error|NaN|Inf' || true)"
            if [[ -n "$errors" ]]; then
                printf 'runtime_errors=\n%s\n' "$errors"
            else
                printf 'runtime_errors=none_in_recent_log\n'
            fi
        fi
    } >> "$MONITOR_LOG"
}

if [[ "${1:-}" == "--once" ]]; then
    report_once
    exit 0
fi

while true; do
    report_once
    if ! tmux has-session -t "$TARGET_SESSION" 2>/dev/null && [[ -f "$TRAIN_LOG" ]]; then
        # The launcher emits one final --once report before exiting.  This
        # guard prevents a detached monitor from living forever after training.
        exit 0
    fi
    sleep "$POLL_SECONDS"
done
