#!/usr/bin/env bash
set -euo pipefail

# Short-cycle watchdog for exactly one 4 m/s / nominal 100 m training run.
# It may restart this target once after a confirmed failure, but it never
# touches another tmux, checkpoint, log, or training.
ROOT_DIR="${G1_RUNNING_ROOT:-/home/ubuntu/g1_running}"
PROJECT_DIR="$ROOT_DIR/clean_training/unitree_rl_lab"
RUN_ROOT="$PROJECT_DIR/logs/rsl_rl/unitree_g1_29dof_sprint_robust_highspeed"
TARGET_SESSION="${TARGET_SESSION:-g1_robust_4m100m_from85800_20260825_011300}"
RUN_NAME="${RUN_NAME:-robust_4m100m_from85800_20260825_011300}"
TRAIN_LOG="${TRAIN_LOG:-$PROJECT_DIR/output/${RUN_NAME}.log}"
WATCH_LOG="${WATCH_LOG:-$PROJECT_DIR/output/${RUN_NAME}_watchdog.log}"
POLL_SECONDS="${POLL_SECONDS:-15}"
WARMUP_ITERS="${WARMUP_ITERS:-500}"
MAX_RESTARTS=1
PID_FILE="$PROJECT_DIR/output/g1_4m100m_watchdog.pid"
EVIDENCE_ROOT="$PROJECT_DIR/output/g1_4m100m_watchdog_evidence"
SEED_RUN="2026-08-22_11-38-46_robust_highspeed_retention_from80250_20260822_1139"
SEED_CHECKPOINT="model_85800.pt"
SEED_RUN="${SEED_RUN_OVERRIDE:-$SEED_RUN}"
SEED_CHECKPOINT="${SEED_CHECKPOINT_OVERRIDE:-$SEED_CHECKPOINT}"
TASK_ID="${TASK_ID:-Unitree-G1-29dof-Sprint-Robust-4m-100m}"
MAX_ITERATIONS="${MAX_ITERATIONS:-10000}"

mkdir -p "$(dirname "$WATCH_LOG")" "$EVIDENCE_ROOT"
if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE" 2>/dev/null || echo 0)" 2>/dev/null; then
    echo "watchdog already running: $(cat "$PID_FILE")" >&2
    exit 0
fi
echo "$$" > "$PID_FILE"
trap 'rm -f "$PID_FILE"' EXIT

log() {
    printf '[%s] %s\n' "$(date '+%F %T %z')" "$*" | tee -a "$WATCH_LOG"
}

latest_line() {
    local pattern="$1"
    rg "$pattern" "$TRAIN_LOG" 2>/dev/null | tail -1 || true
}

latest_number() {
    local pattern="$1"
    latest_line "$pattern" | sed -E 's/.*:[[:space:]]*([-+]?[0-9]+(\.[0-9]+)?([eE][-+]?[0-9]+)?).*/\1/'
}

latest_iteration() {
    latest_line 'Learning iteration' | sed -E 's/.*Learning iteration[[:space:]]+([0-9]+)\/.*$/\1/'
}

number_or_blank() {
    local value="$1"
    if [[ "$value" =~ ^[-+]?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$ ]]; then
        printf '%s' "$value"
    else
        printf ''
    fi
}

is_gt() { awk -v a="$1" -v b="$2" 'BEGIN { exit !(a > b) }'; }
is_lt() { awk -v a="$1" -v b="$2" 'BEGIN { exit !(a < b) }'; }

current_run_dir() {
    find "$RUN_ROOT" -mindepth 1 -maxdepth 1 -type d -name "*_${RUN_NAME}" -printf '%T@ %f\n' 2>/dev/null \
        | sort -n | tail -1 | cut -d' ' -f2-
}

latest_checkpoint_name() {
    local run_dir="$1"
    [[ -n "$run_dir" && -d "$RUN_ROOT/$run_dir" ]] || return 0
    find "$RUN_ROOT/$run_dir" -maxdepth 1 -type f -name 'model_*.pt' -printf '%f\n' 2>/dev/null \
        | sort -V | tail -1
}

checkpoint_iteration() {
    basename "$1" 2>/dev/null | sed -E 's/model_([0-9]+)\.pt/\1/'
}

save_evidence() {
    local reason="$1"
    local stamp
    stamp="$(date '+%Y%m%d_%H%M%S')"
    local dir="$EVIDENCE_ROOT/${stamp}_${reason}"
    mkdir -p "$dir"
    {
        echo "reason=$reason"
        echo "target_session=$TARGET_SESSION"
        echo "run_name=$RUN_NAME"
        echo "train_log=$TRAIN_LOG"
        echo "stable_run=$STABLE_RUN"
        echo "stable_checkpoint=$STABLE_CHECKPOINT"
        echo "last_iteration=$LAST_ITERATION"
        echo "last_health=$LAST_HEALTH"
        echo "restart_count=$RESTART_COUNT"
    } > "$dir/metadata.txt"
    tail -n 3000 "$TRAIN_LOG" > "$dir/train_log_tail.txt" 2>/dev/null || true
    tmux capture-pane -p -t "$TARGET_SESSION" -S -300 > "$dir/tmux_tail.txt" 2>/dev/null || true
    git -C "$ROOT_DIR" diff -- \
        clean_training/unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/sprint_robust_4m_100m_env_cfg.py \
        clean_training/unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/agents/sprint_robust_4m_100m_ppo_cfg.py \
        tools/start_g1_4m100m_from85800.sh \
        tools/watchdog_g1_4m100m.sh > "$dir/config_git_diff.txt" 2>/dev/null || true
    log "evidence_saved=$dir"
}

stop_target() {
    if tmux has-session -t "$TARGET_SESSION" 2>/dev/null; then
        log "stopping_target=$TARGET_SESSION reason=$1"
        tmux send-keys -t "$TARGET_SESSION" C-c
        for _ in $(seq 1 20); do
            tmux has-session -t "$TARGET_SESSION" 2>/dev/null || return 0
            sleep 3
        done
        tmux send-keys -t "$TARGET_SESSION" C-c
        sleep 5
        if tmux has-session -t "$TARGET_SESSION" 2>/dev/null; then
            log "target_did_not_exit_gracefully; killing_only_target_session=$TARGET_SESSION"
            tmux kill-session -t "$TARGET_SESSION"
        fi
    fi
}

restart_from_stable() {
    local tag="$1"
    local new_run="robust_4m100m_from85800_${tag}"
    TARGET_SESSION="g1_robust_4m100m_from85800_${tag}"
    RUN_NAME="$new_run"
    TRAIN_LOG="$PROJECT_DIR/output/${RUN_NAME}.log"
    WATCH_LOG="$PROJECT_DIR/output/${RUN_NAME}_watchdog.log"
    log "restart_from_stable session=$TARGET_SESSION run=$RUN_NAME seed_run=$STABLE_RUN seed_checkpoint=$STABLE_CHECKPOINT"
    SEED_RUN_OVERRIDE="$STABLE_RUN" SEED_CHECKPOINT_OVERRIDE="$STABLE_CHECKPOINT" TASK_ID="$TASK_ID" MAX_ITERATIONS="$MAX_ITERATIONS" \
        "$ROOT_DIR/tools/start_g1_4m100m_from85800.sh" "$tag" "$STABLE_RUN" "$STABLE_CHECKPOINT" >> "$WATCH_LOG" 2>&1
    BASE_ITERATION="$(checkpoint_iteration "$STABLE_CHECKPOINT")"
    BASE_ITERATION="${BASE_ITERATION:-0}"
    WARMUP_UNTIL=$((BASE_ITERATION + WARMUP_ITERS))
    LAST_ITERATION=""
    LAST_PROGRESS_TIME="$(date +%s)"
    BAD_STREAK=0
    LAST_HEALTH="restarted"
}

report_once() {
    local iteration reward episode error_xy error_yaw bad_height bad_orientation straight action_jerk
    iteration="$(latest_iteration)"
    reward="$(number_or_blank "$(latest_number 'Mean reward')")"
    episode="$(number_or_blank "$(latest_number 'Mean episode length')")"
    error_xy="$(number_or_blank "$(latest_number 'Metrics/base_velocity/error_vel_xy')")"
    error_yaw="$(number_or_blank "$(latest_number 'Metrics/base_velocity/error_vel_yaw')")"
    bad_height="$(number_or_blank "$(latest_number 'Episode_Termination/base_height')")"
    bad_orientation="$(number_or_blank "$(latest_number 'Episode_Termination/bad_orientation')")"
    straight="$(number_or_blank "$(latest_number 'Episode_Reward/straight_line_position')")"
    action_jerk="$(number_or_blank "$(latest_number 'Episode_Reward/action_jerk')")"
    local now log_age
    now="$(date +%s)"
    log_age="-1"
    [[ -f "$TRAIN_LOG" ]] && log_age=$((now - $(stat -c '%Y' "$TRAIN_LOG")))

    if [[ -n "$iteration" && "$iteration" =~ ^[0-9]+$ ]]; then
        if [[ "$iteration" != "$LAST_ITERATION" ]]; then
            LAST_ITERATION="$iteration"
            LAST_PROGRESS_TIME="$now"
        fi
    fi

    local current_dir latest_ckpt
    current_dir="$(current_run_dir)"
    latest_ckpt="$(latest_checkpoint_name "$current_dir")"
    if [[ "$LAST_HEALTH" == "healthy" && -n "$current_dir" && -n "$latest_ckpt" ]]; then
        STABLE_RUN="$current_dir"
        STABLE_CHECKPOINT="$latest_ckpt"
    fi

    printf '[%s] target=%s iter=%s reward=%s episode=%s e_xy=%s e_yaw=%s base_height=%s bad_orientation=%s straight=%s jerk=%s log_age=%ss stable=%s/%s\n' \
        "$(date '+%F %T %z')" "$TARGET_SESSION" "${iteration:-NA}" "${reward:-NA}" "${episode:-NA}" \
        "${error_xy:-NA}" "${error_yaw:-NA}" "${bad_height:-NA}" "${bad_orientation:-NA}" \
        "${straight:-NA}" "${action_jerk:-NA}" "$log_age" "$STABLE_RUN" "$STABLE_CHECKPOINT" >> "$WATCH_LOG"

    REPORT_ITERATION="$iteration"
    REPORT_REWARD="$reward"
    REPORT_EPISODE="$episode"
    REPORT_ERROR_XY="$error_xy"
    REPORT_ERROR_YAW="$error_yaw"
    REPORT_BASE_HEIGHT="$bad_height"
    REPORT_BAD_ORIENTATION="$bad_orientation"
    REPORT_STRAIGHT="$straight"
    REPORT_LOG_AGE="$log_age"
}

evaluate_health() {
    local now="$(date +%s)"
    local runtime_errors=""
    if [[ -f "$TRAIN_LOG" ]]; then
        # Match actual error tokens only.  In particular, `Inf` must not match
        # the ubiquitous normal log prefix `[Info]` during Isaac startup.
        runtime_errors="$(tail -n 3000 "$TRAIN_LOG" | rg -n '\\bTraceback\\b|\\bRuntimeError\\b|\\bout of memory\\b|\\bCUDA out of memory\\b|\\bCUDA error\\b|\\bNaN\\b|\\bInf\\b' | rg -v 'Warp CUDA error|Multiple Installable Client Drivers|cuDeviceGetUuid' || true)"
    fi
    if [[ -n "$runtime_errors" ]]; then
        LAST_HEALTH="runtime_error"
        log "ALERT runtime_error=$runtime_errors"
        return 2
    fi
    if [[ "$REPORT_LOG_AGE" != "-1" && "$REPORT_LOG_AGE" -gt 180 && "$LAST_PROGRESS_TIME" -lt "$now" ]]; then
        if (( now - LAST_PROGRESS_TIME > 180 )); then
            LAST_HEALTH="log_stalled"
            log "ALERT log_stalled age=${REPORT_LOG_AGE}s last_iteration=${LAST_ITERATION}"
            return 2
        fi
    fi
    [[ "$REPORT_ITERATION" =~ ^[0-9]+$ ]] || { LAST_HEALTH="initializing"; return 0; }
    if (( REPORT_ITERATION < WARMUP_UNTIL )); then
        LAST_HEALTH="warmup"
        BAD_STREAK=0
        return 0
    fi

    local issue=0
    [[ -n "$REPORT_ERROR_XY" ]] && is_gt "$REPORT_ERROR_XY" 1.5 && issue=1
    [[ -n "$REPORT_ERROR_YAW" ]] && is_gt "$REPORT_ERROR_YAW" 1.5 && issue=1
    [[ -n "$REPORT_BASE_HEIGHT" ]] && is_gt "$REPORT_BASE_HEIGHT" 0.02 && issue=1
    [[ -n "$REPORT_BAD_ORIENTATION" ]] && is_gt "$REPORT_BAD_ORIENTATION" 0.02 && issue=1
    [[ -n "$REPORT_EPISODE" ]] && is_lt "$REPORT_EPISODE" 900 && issue=1
    [[ -n "$REPORT_REWARD" ]] && is_lt "$REPORT_REWARD" 30 && issue=1
    [[ -n "$REPORT_STRAIGHT" ]] && is_lt "$REPORT_STRAIGHT" -0.70 && issue=1

    if (( issue == 1 )); then
        BAD_STREAK=$((BAD_STREAK + 1))
        LAST_HEALTH="degraded_${BAD_STREAK}/3"
        log "warning degraded=${BAD_STREAK}/3 iter=$REPORT_ITERATION e_xy=${REPORT_ERROR_XY:-NA} e_yaw=${REPORT_ERROR_YAW:-NA} base_height=${REPORT_BASE_HEIGHT:-NA} bad_orientation=${REPORT_BAD_ORIENTATION:-NA} reward=${REPORT_REWARD:-NA} episode=${REPORT_EPISODE:-NA} straight=${REPORT_STRAIGHT:-NA}"
        (( BAD_STREAK >= 3 )) && return 1
    else
        BAD_STREAK=0
        LAST_HEALTH="healthy"
    fi
    return 0
}

if [[ "${1:-}" == "--dry-run" ]]; then
    echo "watchdog_dry_run target=$TARGET_SESSION log=$TRAIN_LOG poll=${POLL_SECONDS}s warmup=${WARMUP_ITERS} max_restarts=$MAX_RESTARTS"
    exit 0
fi

RESTART_COUNT=0
BASE_ITERATION="$(checkpoint_iteration "$SEED_CHECKPOINT")"
BASE_ITERATION="${BASE_ITERATION:-0}"
WARMUP_UNTIL=$((BASE_ITERATION + WARMUP_ITERS))
LAST_ITERATION=""
LAST_PROGRESS_TIME="$(date +%s)"
BAD_STREAK=0
LAST_HEALTH="initializing"
STABLE_RUN="$SEED_RUN"
STABLE_CHECKPOINT="$SEED_CHECKPOINT"

log "watchdog_started target=$TARGET_SESSION poll=${POLL_SECONDS}s warmup_until=$WARMUP_UNTIL max_restarts=$MAX_RESTARTS"
while true; do
    report_once
    if ! tmux has-session -t "$TARGET_SESSION" 2>/dev/null; then
        log "ALERT target_tmux_stopped=$TARGET_SESSION"
        save_evidence "tmux_stopped"
        exit 1
    fi
    evaluate_health || health_status=$?
    health_status="${health_status:-0}"
    if (( health_status == 2 )); then
        if (( RESTART_COUNT >= MAX_RESTARTS )); then
            save_evidence "${LAST_HEALTH//\//_}"
            stop_target "${LAST_HEALTH}"
            log "STOPPED after restart limit; manual review required"
            exit 2
        fi
        save_evidence "${LAST_HEALTH//\//_}"
        stop_target "${LAST_HEALTH}"
        RESTART_COUNT=$((RESTART_COUNT + 1))
        restart_from_stable "recovery_$(date +%Y%m%d_%H%M%S)_from_${STABLE_CHECKPOINT%.pt}"
        health_status=0
    elif (( health_status == 1 )); then
        if (( RESTART_COUNT >= MAX_RESTARTS )); then
            save_evidence "persistent_degradation"
            stop_target "persistent_degradation"
            log "STOPPED after restart limit; manual review required"
            exit 2
        fi
        save_evidence "persistent_degradation"
        stop_target "persistent_degradation"
        RESTART_COUNT=$((RESTART_COUNT + 1))
        restart_from_stable "recovery_$(date +%Y%m%d_%H%M%S)_from_${STABLE_CHECKPOINT%.pt}"
        health_status=0
    fi
    unset health_status
    sleep "$POLL_SECONDS"
done
