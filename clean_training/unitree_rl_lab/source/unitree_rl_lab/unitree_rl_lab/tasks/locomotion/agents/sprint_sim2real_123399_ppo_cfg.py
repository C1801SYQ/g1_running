from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlPpoAlgorithmCfg

from .sprint_robust_highspeed_ppo_cfg import SprintRobustHighSpeedPPORunnerCfg


@configclass
class SprintSim2Real123399PPORunnerCfg(SprintRobustHighSpeedPPORunnerCfg):
    """Conservative actuator-aware fine-tuning that protects model_123399."""

    # Keep the established experiment root so the training launcher can
    # resolve --load_run/--checkpoint without copying or overwriting the seed.
    experiment_name = "unitree_g1_29dof_sprint_robust_highspeed"
    max_iterations = 6000
    save_interval = 100
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.05,
        entropy_coef=0.001,
        num_learning_epochs=2,
        num_mini_batches=4,
        learning_rate=5.0e-6,
        schedule="fixed",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.003,
        max_grad_norm=0.5,
    )
