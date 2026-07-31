#!/bin/bash
# Auto train-test-adjust loop for 29dof sim2sim
source /home/ubuntu/anaconda3/etc/profile.d/conda.sh
conda activate env_isaaclab

ROBOT_RL_DIR=/home/ubuntu/robot_rl
cd $ROBOT_RL_DIR
ISAACLAB_PATH=/home/ubuntu/IsaacLab
export PYTHONPATH=$ROBOT_RL_DIR/transfer:$PYTHONPATH

ROUND=1
while [ $ROUND -le 5 ]; do
    echo "========================================"
    echo "  AUTO LOOP ROUND $ROUND — $(date)"
    echo "========================================"

    # Wait if training already running
    if ps aux | grep -q "[t]rain_policy"; then
        echo "Waiting for current training to finish..."
        while ps aux | grep -q "[t]rain_policy"; do sleep 60; done
    fi

    # Find latest run
    LATEST_RUN=$(ls -t logs/g1_policies/running-clf-29dof/running_clf_29dof/ | head -1)
    echo "Latest run: $LATEST_RUN"

    # Export if needed
    EXPORT_DIR="logs/g1_policies/running-clf-29dof/running_clf_29dof/$LATEST_RUN/exported"
    if [ ! -f "$EXPORT_DIR/policy.onnx" ]; then
        echo "Exporting policy..."
        $ISAACLAB_PATH/isaaclab.sh -p scripts/rsl_rl/play_policy.py \
            --env_type=running_clf_29dof --num_envs=1 --export_policy --headless
    fi

    # Test in MuJoCo
    echo "Testing in MuJoCo headless..."
    timeout 30 python transfer/sim/g1_runner.py \
        --env_type=running_clf_29dof --scene=29dof_basic_scene \
        --load_run=$LATEST_RUN --headless --log 2>/dev/null

    # Analyze
    MUJOCO_DIR=$(ls -td logs/g1_policies/running-clf-29dof/running_clf_29dof/$LATEST_RUN/mujoco_logs/*/ 2>/dev/null | head -1)
    FALL_TIME=0
    if [ -n "$MUJOCO_DIR" ] && [ -f "$MUJOCO_DIR/sim_log.csv" ]; then
        FALL_TIME=$(python3 -c "
import csv
with open('$MUJOCO_DIR/sim_log.csv') as f:
    rows = list(csv.reader(f))
for i, row in enumerate(rows):
    if float(row[3]) < 0.5:
        print(f'{i*0.02:.1f}')
        break
else:
    print('999')
")
    fi
    echo "Fall time: ${FALL_TIME}s"

    # Check success
    if [ "$(echo "$FALL_TIME > 30" | bc -l)" = "1" ]; then
        echo "SUCCESS! Robot survives >30s in MuJoCo!"
        break
    fi

    # Not good enough - expand random and retrain
    echo "Expanding randomization..."
    CFG="source/robot_rl/robot_rl/tasks/manager_based/robot_rl/g1/g1_running_clf_29dof_env_cfg.py"
    python3 "$ROBOT_RL_DIR/expand_random.py" "$CFG"

    pip install -e source/robot_rl/ -q
    echo "Starting retraining round $ROUND..."
    $ISAACLAB_PATH/isaaclab.sh -p scripts/rsl_rl/train_policy.py \
        --env_type=running_clf_29dof --headless --num_envs=4096 --max_iterations=10000

    ROUND=$((ROUND + 1))
done
echo "Auto loop finished at $(date)"
