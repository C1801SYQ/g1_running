from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlPpoAlgorithmCfg

from .sprint_robust_highspeed_ppo_cfg import SprintRobustHighSpeedPPORunnerCfg


@configclass
class SprintRobust4m100mPPORunnerCfg(SprintRobustHighSpeedPPORunnerCfg):
    """model_85800-compatible PPO for the fixed 4 m/s straight-run task."""

    # Keep logs in the existing Robust HighSpeed experiment root so the
    # absolute model_85800 checkpoint can be resumed without copying it.
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
        learning_rate=5.0e-5,
        schedule="fixed",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.005,
        max_grad_norm=1.0,
    )
