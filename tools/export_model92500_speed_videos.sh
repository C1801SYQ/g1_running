#!/usr/bin/env bash
set -eo pipefail

source /home/ubuntu/anaconda3/etc/profile.d/conda.sh
set +e
conda activate env_isaaclab
set -e

cd /home/ubuntu/g1_running/clean_training/unitree_rl_lab
CKPT=/home/ubuntu/g1_running/clean_training/unitree_rl_lab/logs/rsl_rl/unitree_g1_29dof_sprint_balanced/2026-08-23_12-52-08_balanced_recovery_from90000_20260823_125203/model_92750.pt
OUT=/home/ubuntu/g1_running/clean_training/unitree_rl_lab/output

for spec in 0:0ms 1:1ms 2:2ms 3:3ms 4.5:4p5ms 5.1:5p1ms 6:6ms 7:7ms; do
    speed=${spec%%:*}
    label=${spec#*:}
    ./unitree_rl_lab.sh -p --headless \
        --task Unitree-G1-29dof-Sprint-Balanced \
        --num_envs 1 --video --video_length 500 \
        --fixed_lin_vel_x "$speed" \
        --video_name_prefix "model_92750_${label}_500steps" \
        --experiment_name unitree_g1_29dof_sprint_balanced \
        --checkpoint "$CKPT" \
        2>&1 | tee "$OUT/preview_model92750_${label}.log"
done
