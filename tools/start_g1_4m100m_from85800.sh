#!/usr/bin/env bash
set -euo pipefail

# Start the independent fixed 4 m/s, nominal 100 m fine-tuning task.
# This launcher never stops or edits another training.
ROOT_DIR="${G1_RUNNING_ROOT:-/home/ubuntu/g1_running}"
PROJECT_DIR="$ROOT_DIR/clean_training/unitree_rl_lab"
MONITOR_SCRIPT="$ROOT_DIR/tools/monitor_g1_sim2real.sh"
DRY_RUN=0
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=1
    TAG="$(date +%Y%m%d_%H%M%S)"
else
    TAG="${1:-$(date +%Y%m%d_%H%M%S)}"
fi
SEED_RUN="${2:-2026-08-22_11-38-46_robust_highspeed_retention_from80250_20260822_1139}"
SEED_CHECKPOINT="${3:-model_85800.pt}"
TASK_ID="${TASK_ID:-Unitree-G1-29dof-Sprint-Robust-4m-100m}"
MAX_ITERATIONS="${MAX_ITERATIONS:-10000}"
if [[ "${2:-}" == "--dry-run" ]]; then
    SEED_RUN="2026-08-22_11-38-46_robust_highspeed_retention_from80250_20260822_1139"
    SEED_CHECKPOINT="model_85800.pt"
fi
RUN_PREFIX="${RUN_PREFIX:-robust_4m100m_from85800}"
SESSION_PREFIX="${SESSION_PREFIX:-g1_robust_4m100m_from85800}"
SESSION="${SESSION_OVERRIDE:-${SESSION_PREFIX}_${TAG}}"
RUN_NAME="${RUN_NAME_OVERRIDE:-${RUN_PREFIX}_${TAG}}"
TRAIN_LOG="$PROJECT_DIR/output/${RUN_NAME}.log"
RUN_DIR="$PROJECT_DIR/logs/rsl_rl/unitree_g1_29dof_sprint_robust_highspeed"
MONITOR_LOG="$PROJECT_DIR/output/${RUN_NAME}_monitor.log"

if [[ "$DRY_RUN" == "1" || "${2:-}" == "--dry-run" ]]; then
    printf 'session=%s\nrun_name=%s\ntrain_log=%s\nmonitor_log=%s\n' \
        "$SESSION" "$RUN_NAME" "$TRAIN_LOG" "$MONITOR_LOG"
    printf 'seed_run=%s\nseed_checkpoint=%s\ntask_id=%s\nmax_iterations=%s\n' "$SEED_RUN" "$SEED_CHECKPOINT" "$TASK_ID" "$MAX_ITERATIONS"
    exit 0
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "[ERROR] target tmux already exists: $SESSION" >&2
    exit 1
fi
if [[ ! -x "$PROJECT_DIR/unitree_rl_lab.sh" ]]; then
    echo "[ERROR] launcher missing: $PROJECT_DIR/unitree_rl_lab.sh" >&2
    exit 1
fi
if [[ ! -f "$RUN_DIR/$SEED_RUN/$SEED_CHECKPOINT" ]]; then
    echo "[ERROR] seed checkpoint missing: $RUN_DIR/$SEED_RUN/$SEED_CHECKPOINT" >&2
    exit 1
fi

inner_cmd="source /home/ubuntu/anaconda3/etc/profile.d/conda.sh && conda activate env_isaaclab && cd '$PROJECT_DIR' && export TARGET_TMUX_SESSION='$SESSION' TRAIN_LOG='$TRAIN_LOG' RUN_DIR='$RUN_DIR' MONITOR_LOG='$MONITOR_LOG' POLL_SECONDS=3600 || exit 1; bash '$MONITOR_SCRIPT' >/dev/null 2>&1 & monitor_pid=\$!; status=0; '$PROJECT_DIR/unitree_rl_lab.sh' -t --task '$TASK_ID' --num_envs 8196 --max_iterations '$MAX_ITERATIONS' --resume --load_run '$SEED_RUN' --checkpoint '$SEED_CHECKPOINT' --run_name '$RUN_NAME' --seed 42 2>&1 | tee '$TRAIN_LOG' || status=\${PIPESTATUS[0]}; bash '$MONITOR_SCRIPT' --once; kill \$monitor_pid 2>/dev/null || true; wait \$monitor_pid 2>/dev/null || true; exit \$status"

tmux new-session -d -s "$SESSION" "bash -lc $(printf '%q' "$inner_cmd")"
echo "started tmux=$SESSION"
echo "train_log=$TRAIN_LOG"
echo "monitor_log=$MONITOR_LOG"
