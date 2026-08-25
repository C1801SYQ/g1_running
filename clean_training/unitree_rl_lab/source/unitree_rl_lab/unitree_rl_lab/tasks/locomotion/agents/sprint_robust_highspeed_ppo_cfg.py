from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlPpoAlgorithmCfg

from .rsl_rl_ppo_cfg import SprintBalancedPPORunnerCfg


@configclass
class SprintRobustHighSpeedPPORunnerCfg(SprintBalancedPPORunnerCfg):
    """Retention-first PPO fine-tuning for the verified high-speed gait."""

    experiment_name = "unitree_g1_29dof_sprint_robust_highspeed"
    max_iterations = 10000
    save_interval = 100
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.10,
        entropy_coef=0.002,
        num_learning_epochs=3,
        num_mini_batches=4,
        # A fresh optimizer is used on restart; the smaller step limits
        # catastrophic drift while the verified 4.5--5.1 m/s replay mixture
        # re-establishes robustness.
        learning_rate=5.0e-6,
        schedule="fixed",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.005,
        max_grad_norm=1.0,
    )
