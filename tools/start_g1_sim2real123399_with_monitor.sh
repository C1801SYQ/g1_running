#!/usr/bin/env bash
set -euo pipefail

# Start one independent 8196-env run and bind its read-only monitor before
# Isaac Sim is launched.  This script does not stop or inspect other runs.
ROOT_DIR="${G1_RUNNING_ROOT:-/home/ubuntu/g1_running}"
PROJECT_DIR="$ROOT_DIR/clean_training/unitree_rl_lab"
SEED_RUN="2026-08-23_19-43-55_robust_highspeed_retention_repair_from113400_20260823"
SEED_CHECKPOINT="model_123399.pt"
DRY_RUN=0
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=1
    shift
fi
TAG="${1:-$(date +%Y%m%d_%H%M%S)}"
SESSION="g1_sim2real123399_${TAG}"
RUN_NAME="sim2real_mild_from123399_${TAG}"
TRAIN_LOG="$PROJECT_DIR/output/${RUN_NAME}.log"
RUN_DIR="$PROJECT_DIR/logs/rsl_rl/unitree_g1_29dof_sprint_robust_highspeed"
MONITOR_LOG="$PROJECT_DIR/output/${RUN_NAME}_monitor.log"
MONITOR_SCRIPT="$ROOT_DIR/tools/monitor_g1_sim2real.sh"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "[ERROR] target tmux already exists: $SESSION" >&2
    exit 1
fi
if [[ ! -f "$PROJECT_DIR/logs/rsl_rl/unitree_g1_29dof_sprint_robust_highspeed/$SEED_RUN/$SEED_CHECKPOINT" ]]; then
    echo "[ERROR] seed checkpoint not found" >&2
    exit 1
fi

inner_cmd="source /home/ubuntu/anaconda3/etc/profile.d/conda.sh && conda activate env_isaaclab && cd '$PROJECT_DIR' && export TARGET_TMUX_SESSION='$SESSION' TRAIN_LOG='$TRAIN_LOG' RUN_DIR='$RUN_DIR' MONITOR_LOG='$MONITOR_LOG' POLL_SECONDS=3600 && bash '$MONITOR_SCRIPT' >/dev/null 2>&1 & monitor_pid=\$!; status=0; ./unitree_rl_lab.sh -t --task Unitree-G1-29dof-Sprint-Sim2Real-123399 --num_envs 8196 --resume --load_run '$SEED_RUN' --checkpoint '$SEED_CHECKPOINT' --reset_optimizer --run_name '$RUN_NAME' 2>&1 | tee '$TRAIN_LOG' || status=\${PIPESTATUS[0]}; bash '$MONITOR_SCRIPT' --once; kill \$monitor_pid 2>/dev/null || true; wait \$monitor_pid 2>/dev/null || true; exit \$status"

if [[ "$DRY_RUN" == "1" ]]; then
    printf 'tmux=%s\nrun_name=%s\ntrain_log=%s\nmonitor_log=%s\n' \
        "$SESSION" "$RUN_NAME" "$TRAIN_LOG" "$MONITOR_LOG"
    printf 'inner_command=%s\n' "$inner_cmd"
    exit 0
fi

tmux new-session -d -s "$SESSION" "bash -lc $(printf '%q' "$inner_cmd")"
echo "started tmux=$SESSION"
echo "train_log=$TRAIN_LOG"
echo "monitor_log=$MONITOR_LOG"
