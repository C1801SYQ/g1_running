from __future__ import annotations

import torch
from isaaclab.assets import Articulation
from isaaclab.envs.mdp import observations as isaac_observations
from isaaclab.managers import SceneEntityCfg


def _ensure_bias_state(env, asset: Articulation):
    if not hasattr(env, "_g1_sim2real_ang_vel_bias"):
        env._g1_sim2real_ang_vel_bias = torch.zeros(env.num_envs, 3, device=asset.device)
        env._g1_sim2real_gravity_bias = torch.zeros(env.num_envs, 3, device=asset.device)
        env._g1_sim2real_joint_pos_bias = torch.zeros_like(asset.data.joint_pos)
        env._g1_sim2real_joint_vel_bias = torch.zeros_like(asset.data.joint_vel)
    return env


def reset_sim2real_sensor_bias(
    env,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    ang_vel_bias_range: tuple[float, float] = (-0.03, 0.03),
    gravity_bias_range: tuple[float, float] = (-0.01, 0.01),
    joint_pos_bias_range: tuple[float, float] = (-0.002, 0.002),
    joint_vel_bias_range: tuple[float, float] = (-0.05, 0.05),
):
    """Reset slowly varying IMU and encoder biases for sim-to-real training."""
    asset: Articulation = env.scene[asset_cfg.name]
    env = _ensure_bias_state(env, asset)
    if env_ids is None:
        env_ids = slice(None)
        count = env.num_envs
    else:
        env_ids = env_ids.to(device=asset.device)
        count = len(env_ids)
    env._g1_sim2real_ang_vel_bias[env_ids] = torch.empty(count, 3, device=asset.device).uniform_(*ang_vel_bias_range)
    env._g1_sim2real_gravity_bias[env_ids] = torch.empty(count, 3, device=asset.device).uniform_(*gravity_bias_range)
    env._g1_sim2real_joint_pos_bias[env_ids] = torch.empty(
        count, asset.num_joints, device=asset.device
    ).uniform_(*joint_pos_bias_range)
    env._g1_sim2real_joint_vel_bias[env_ids] = torch.empty(
        count, asset.num_joints, device=asset.device
    ).uniform_(*joint_vel_bias_range)


def base_ang_vel(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    asset: Articulation = env.scene[asset_cfg.name]
    env = _ensure_bias_state(env, asset)
    return isaac_observations.base_ang_vel(env, asset_cfg) + env._g1_sim2real_ang_vel_bias


def projected_gravity(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    asset: Articulation = env.scene[asset_cfg.name]
    env = _ensure_bias_state(env, asset)
    return isaac_observations.projected_gravity(env, asset_cfg) + env._g1_sim2real_gravity_bias


def joint_pos_rel(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    asset: Articulation = env.scene[asset_cfg.name]
    env = _ensure_bias_state(env, asset)
    return isaac_observations.joint_pos_rel(env, asset_cfg) + env._g1_sim2real_joint_pos_bias[:, asset_cfg.joint_ids]


def joint_vel_rel(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")):
    asset: Articulation = env.scene[asset_cfg.name]
    env = _ensure_bias_state(env, asset)
    return isaac_observations.joint_vel_rel(env, asset_cfg) + env._g1_sim2real_joint_vel_bias[:, asset_cfg.joint_ids]
