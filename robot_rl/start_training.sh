#!/bin/bash
# Start high-speed 29dof training
# Run directly in terminal, NOT via nohup/background

cd /home/ubuntu/robot_rl
source /home/ubuntu/anaconda3/etc/profile.d/conda.sh
conda activate env_isaaclab
pip install -e source/robot_rl/ -q

# Clean any leftover locks
rm -f /home/ubuntu/isaacsim/kit/data/Kit/Isaac-Sim/5.0/user.config.json.lock
rm -rf /home/ubuntu/isaacsim/kit/cache/Kit/Isaac-Sim/5.0/omni.kvdb*

echo "Starting high-speed training (1.0-5.1 m/s, 29dof, 4096 envs, 10000 iters)..."
/home/ubuntu/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train_policy.py \
    --env_type=running_clf_29dof --headless --num_envs=4096 \
    --max_iterations=10000 --logger tensorboard --run_name=highspeed
