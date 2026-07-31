#!/bin/bash
# Auto train-test-adjust loop for 29dof sim2sim
# Stops when robot survives >30s without falling in MuJoCo

set -e
source /home/ubuntu/anaconda3/etc/profile.d/conda.sh
conda activate env_isaaclab

ROBOT_RL_DIR=/home/ubuntu/robot_rl
cd $ROBOT_RL_DIR
export PYTHONPATH=$ROBOT_RL_DIR/transfer:$PYTHONPATH

ROUND=1
MAX_ROUNDS=10

while [ $ROUND -le $MAX_ROUNDS ]; do
    echo "========================================"
    echo "  AUTO TRAINING LOOP - ROUND $ROUND"
    echo "========================================"

    # 1. TRAIN
    echo "[$(date)] Starting training round $ROUND..."
    pip install -e source/robot_rl/ -q
    $ISAACLAB_PATH/isaaclab.sh -p scripts/rsl_rl/train_policy.py \
        --env_type=running_clf_29dof --headless --num_envs=4096 --max_iterations=10000

    echo "[$(date)] Training complete."

    # 2. EXPORT policy
    echo "[$(date)] Exporting policy..."
    $ISAACLAB_PATH/isaaclab.sh -p scripts/rsl_rl/play_policy.py \
        --env_type=running_clf_29dof --num_envs=1 --export_policy --headless

    # 3. FIND latest run
    LATEST_RUN=$(ls -t $ROBOT_RL_DIR/logs/g1_policies/running-clf-29dof/running_clf_29dof/ | head -1)
    echo "[$(date)] Testing run: $LATEST_RUN"

    # 4. TEST in MuJoCo headless
    timeout 30 python transfer/sim/g1_runner.py \
        --env_type=running_clf_29dof --scene=29dof_basic_scene \
        --load_run=$LATEST_RUN --headless --log

    # 5. ANALYZE
    MUJOCO_LOG_DIR=$(ls -td $ROBOT_RL_DIR/logs/g1_policies/running-clf-29dof/running_clf_29dof/$LATEST_RUN/mujoco_logs/*/ 2>/dev/null | head -1)
    if [ -z "$MUJOCO_LOG_DIR" ] || [ ! -f "$MUJOCO_LOG_DIR/sim_log.csv" ]; then
        echo "[$(date)] MuJoCo test failed (no log). Will retry with more randomization."
        FALL_TIME=0
    else
        FALL_TIME=$(python3 -c "
import csv
f = '$MUJOCO_LOG_DIR/sim_log.csv'
with open(f) as fh: rows = list(csv.reader(fh))
for i, row in enumerate(rows):
    if float(row[3]) < 0.5:
        print(f'{i*0.02:.1f}')
        break
else:
    print('999')
" 2>/dev/null)
    fi

    echo "[$(date)] Fall time: ${FALL_TIME}s"

    # 6. CHECK if good enough
    if (( $(echo "$FALL_TIME > 30" | bc -l) )); then
        echo "[$(date)] SUCCESS! Robot runs >30s in MuJoCo."
        exit 0
    fi

    # 7. ADJUST config - expand randomization further
    echo "[$(date)] Expanding randomization for round $((ROUND+1))..."
    ROUND=$((ROUND+1))

    # Widen randomization ranges
    python3 -c "
import re
cfg = '$ROBOT_RL_DIR/source/robot_rl/robot_rl/tasks/manager_based/robot_rl/g1/g1_running_clf_29dof_env_cfg.py'
with open(cfg) as f:
    content = f.read()

# Expand stiffness range by 0.1 each round
for old, new in [
    ('(0.5, 1.5)', f'(0.4, 1.6)'),
    ('(0.5, 3.0)', f'(0.3, 3.5)'),
    ('(0.7, 1.3)', f'(0.6, 1.4)'),
]:
    content = content.replace(old, new)

with open(cfg, 'w') as f:
    f.write(content)
print('Config updated')
"
done

echo "[$(date)] Max rounds reached."
