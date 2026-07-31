#!/bin/bash
# Training monitor - detects stalls and logs progress
set -e

source /home/ubuntu/anaconda3/etc/profile.d/conda.sh
conda activate env_isaaclab

ROBOT_RL_DIR=/home/ubuntu/robot_rl
cd $ROBOT_RL_DIR
export PYTHONPATH=$ROBOT_RL_DIR/transfer:$PYTHONPATH

ENV_TYPE="running_clf_29dof"
LOG_ROOT="$ROBOT_RL_DIR/logs/g1_policies/running-clf-29dof/running_clf_29dof"
MONITOR_LOG="$ROBOT_RL_DIR/monitor_results.txt"
STATE_FILE="/tmp/monitor_29dof_state"
STALL_MINUTES=25

NOW=$(date '+%Y-%m-%d %H:%M:%S')
NOW_EPOCH=$(date +%s)
log() { echo "$@" | tee -a "$MONITOR_LOG"; }

log ""
log "==== $NOW ===="

# ── 1. Training alive? ──
TRAIN_PID=$(ps aux | grep "train_policy.py.*$ENV_TYPE" | grep -v grep | awk '{print $2}' | head -1)
if [ -z "$TRAIN_PID" ]; then
    log "[FATAL] Training NOT running!"
    exit 1
fi
log "[OK] PID=$TRAIN_PID"

# ── 2. Active run & checkpoint ──
ACTIVE_RUN=$(ls -t "$LOG_ROOT" 2>/dev/null | head -1)
RUN_DIR="$LOG_ROOT/$ACTIVE_RUN"
LATEST_CKPT=$(ls -t "$RUN_DIR"/model_*.pt 2>/dev/null | head -1)
if [ -z "$LATEST_CKPT" ]; then
    log "[INFO] No checkpoints yet."; exit 0
fi
CKPT_NUM=$(basename "$LATEST_CKPT" .pt | sed 's/model_//')
CKPT_AGE=$(( (NOW_EPOCH - $(stat -c %Y "$LATEST_CKPT")) / 60 ))
log "[INFO] Run: $ACTIVE_RUN | ckpt: model_${CKPT_NUM}.pt (${CKPT_AGE}min ago)"

# ── 3. Stall detection ──
STALLED=0
TB_EVENT=$(ls -t "$RUN_DIR"/events.out.tfevents* 2>/dev/null | head -1)
if [ -n "$TB_EVENT" ]; then
    TB_AGE=$(( (NOW_EPOCH - $(stat -c %Y "$TB_EVENT")) / 60 ))
    if [ "$TB_AGE" -gt "$STALL_MINUTES" ]; then
        STALLED=1
    fi
fi

if [ "$STALLED" -eq 1 ]; then
    log "[STALLED] ckpt=${CKPT_AGE}min tb=${TB_AGE}min — KILL AND RESTART!"
    exit 1
fi
log "[OK] Progressing (ckpt ${CKPT_AGE}min old)"

# ── 4. Progress ──
PREV="0"; [ -f "$STATE_FILE" ] && PREV=$(cat "$STATE_FILE")
echo "$CKPT_NUM" > "$STATE_FILE"
log "[PROGRESS] +$((CKPT_NUM - PREV)) iters"

# ── 5. Rewards ──
if [ -n "$TB_EVENT" ]; then
    REWARD_INFO=$(python3 -c "
import sys; sys.path.insert(0, '$ROBOT_RL_DIR')
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
ea = EventAccumulator('$TB_EVENT'); ea.Reload()
try:
    mr = ea.Scalars('Train/mean_reward')
    bh = ea.Scalars('Episode_Termination/base_height')
    clf = ea.Scalars('Episode_Reward/clf_reward')
    el = ea.Scalars('Train/mean_episode_length')
    parts = []
    if mr and len(mr)>=3:
        vals = [f'{s.value:.1f}' for s in mr[-3:]]
        parts.append(f'reward=[{vals[0]}..{vals[-1]}]')
    if clf and len(clf)>=3:
        vals = [f'{s.value:.2f}' for s in clf[-3:]]
        parts.append(f'clf=[{vals[0]}..{vals[-1]}]')
    if el and len(el)>=1:
        parts.append(f'ep={el[-1].value:.0f}s')
    if bh and len(bh)>=1:
        parts.append(f'falls={bh[-1].value:.0f}')
    print(' | '.join(parts))
except Exception as e:
    print(f'tb error: {e}')
" 2>/dev/null)
    log "[REWARD] $REWARD_INFO"
fi
