#!/usr/bin/env bash
set -eo pipefail

cd /home/ubuntu/g1_running/clean_training/unitree_rl_lab
if [[ "${CONDA_DEFAULT_ENV:-}" != "env_isaaclab" ]]; then
    unalias isaaclab 2>/dev/null || true
    source /home/ubuntu/anaconda3/etc/profile.d/conda.sh
    set +e
    conda activate env_isaaclab
    set -e
fi
export PATH=/home/ubuntu/anaconda3/envs/env_isaaclab/bin:$PATH

# Parallel launch is intentional: keep PID 375759 untouched and use the
# independently verified free GPU/memory headroom for this second run.

STAMP=$(date +%Y%m%d_%H%M%S)
SMOKE_LOG=/home/ubuntu/g1_running/clean_training/unitree_rl_lab/output/smoke_balanced_recovery_from90000_${STAMP}.log
if ! ./unitree_rl_lab.sh -t --headless \
    --task Unitree-G1-29dof-Sprint-Balanced \
    --num_envs 8196 --max_iterations 1 \
    --resume \
    --reset_optimizer \
    --load_run 2026-08-22_22-59-02_robust_speed4p4_to7_stable_from80250_20260822_230000 \
    --checkpoint model_90000.pt \
    --run_name smoke_balanced_recovery_from90000_${STAMP} \
    2>&1 | tee "$SMOKE_LOG"; then
    echo "SMOKE_FAILED_LOG=$SMOKE_LOG"
    exit 21
fi

RUN_STAMP=$(date +%Y%m%d_%H%M%S)
LOG=/home/ubuntu/g1_running/clean_training/unitree_rl_lab/output/balanced_recovery_from90000_${RUN_STAMP}.log
exec ./unitree_rl_lab.sh -t --headless \
    --task Unitree-G1-29dof-Sprint-Balanced \
    --num_envs 8196 --max_iterations 30000 \
    --resume \
    --reset_optimizer \
    --load_run 2026-08-22_22-59-02_robust_speed4p4_to7_stable_from80250_20260822_230000 \
    --checkpoint model_90000.pt \
    --run_name balanced_recovery_from90000_${RUN_STAMP} \
    2>&1 | tee "$LOG"
