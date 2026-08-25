import gymnasium as gym

gym.register(
    id="Unitree-G1-29dof-Velocity",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.velocity_env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.velocity_env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": f"unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)

gym.register(
    id="Unitree-G1-29dof-Sprint",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.sprint_env_cfg:SprintEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.sprint_env_cfg:SprintPlayEnvCfg",
        "rsl_rl_cfg_entry_point": f"unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:SprintPPORunnerCfg",
    },
)

gym.register(
    id="Unitree-G1-29dof-Sprint-Refine",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.sprint_env_cfg:SprintRefineEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.sprint_env_cfg:SprintRefinePlayEnvCfg",
        "rsl_rl_cfg_entry_point": f"unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:SprintRefinePPORunnerCfg",
    },
)

gym.register(
    id="Unitree-G1-29dof-Sprint-Balanced",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.sprint_env_cfg:SprintBalancedEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.sprint_env_cfg:SprintPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:SprintBalancedPPORunnerCfg",
    },
)

# Independent task: additive registration only; existing tasks and runs are untouched.
gym.register(
    id="Unitree-G1-29dof-Sprint-Robust-HighSpeed",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.sprint_robust_highspeed_env_cfg:SprintRobustHighSpeedEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.sprint_robust_highspeed_env_cfg:SprintRobustHighSpeedPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.sprint_robust_highspeed_ppo_cfg:SprintRobustHighSpeedPPORunnerCfg",
    },
)

# Independent fixed-speed straight-run task initialized from model_85800.
# Existing tasks and runs remain untouched.
gym.register(
    id="Unitree-G1-29dof-Sprint-Robust-4m-100m",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.sprint_robust_4m_100m_env_cfg:SprintRobust4m100mEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.sprint_robust_4m_100m_env_cfg:SprintRobust4m100mPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.sprint_robust_4m_100m_ppo_cfg:SprintRobust4m100mPPORunnerCfg",
    },
)

# Conservative stability continuation from the last stable 4 m/s checkpoint.
gym.register(
    id="Unitree-G1-29dof-Sprint-Robust-4m-100m-Stable",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.sprint_robust_4m_100m_stable_env_cfg:SprintRobust4m100mStableEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.sprint_robust_4m_100m_stable_env_cfg:SprintRobust4m100mStablePlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.sprint_robust_4m_100m_stable_ppo_cfg:SprintRobust4m100mStablePPORunnerCfg",
    },
)

# Independent actuator/sensor-aware fine-tuning task initialized from the
# video-confirmed model_123399 checkpoint.  Existing tasks remain untouched.
gym.register(
    id="Unitree-G1-29dof-Sprint-Sim2Real-123399",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.sprint_sim2real_123399_env_cfg:SprintSim2Real123399EnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.sprint_sim2real_123399_env_cfg:SprintSim2Real123399PlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.sprint_sim2real_123399_ppo_cfg:SprintSim2Real123399PPORunnerCfg",
    },
)
