from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlPpoAlgorithmCfg

from .sprint_robust_4m_100m_ppo_cfg import SprintRobust4m100mPPORunnerCfg


@configclass
class SprintRobust4m100mStablePPORunnerCfg(SprintRobust4m100mPPORunnerCfg):
    """Conservative PPO continuation from the last stable 4 m/s checkpoint."""

    experiment_name = "unitree_g1_29dof_sprint_robust_highspeed"
    max_iterations = 5000
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.10,
        entropy_coef=0.001,
        num_learning_epochs=2,
        num_mini_batches=4,
        learning_rate=1.5e-5,
        schedule="fixed",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.005,
        max_grad_norm=1.0,
    )
