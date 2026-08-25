from __future__ import annotations

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg


def randomize_joint_default_pos(
    env,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    pos_distribution_params: tuple[float, float],
    operation: str = "add",
):
    """Randomize per-environment default joint offsets and action offsets."""
    if operation != "add":
        raise ValueError(f"Only additive default-joint randomization is supported, got: {operation}")

    asset: Articulation = env.scene[asset_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)
    env_ids = env_ids.to(device=asset.device)

    default_joint_pos = asset.data.default_joint_pos[env_ids].clone()
    joint_pos = default_joint_pos[:, asset_cfg.joint_ids].clone()
    joint_pos += torch.empty_like(joint_pos).uniform_(*pos_distribution_params)
    joint_pos_limits = asset.data.soft_joint_pos_limits[env_ids][:, asset_cfg.joint_ids]
    joint_pos = joint_pos.clamp_(joint_pos_limits[..., 0], joint_pos_limits[..., 1])

    default_joint_pos[:, asset_cfg.joint_ids] = joint_pos
    asset.data.default_joint_pos[env_ids] = default_joint_pos
    action_term = env.action_manager.get_term("JointPositionAction")
    action_offset = action_term._offset[env_ids].clone()
    action_offset[:, asset_cfg.joint_ids] = joint_pos
    action_term._offset[env_ids] = action_offset


def randomize_actuator_effort_limit(
    env,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    effort_scale_distribution_params: tuple[float, float] = (0.90, 1.10),
):
    """Randomize per-motor output capability while keeping the action contract unchanged.

    The G1 sprint asset uses implicit actuators, so the standard actuator-gain
    randomizer does not change the motor torque ceiling. This event samples a
    per-environment/per-joint scale for the configured effort limit and writes
    it both to the actuator model and PhysX at startup.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)
    elif isinstance(env_ids, slice):
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)
    else:
        env_ids = env_ids.to(device=asset.device)

    for actuator in asset.actuators.values():
        effort_limit = actuator.effort_limit_sim[env_ids].clone()
        scale = torch.empty_like(effort_limit).uniform_(*effort_scale_distribution_params)
        effort_limit = effort_limit * scale
        actuator.effort_limit_sim[env_ids] = effort_limit
        actuator.effort_limit[env_ids] = effort_limit
        asset.write_joint_effort_limit_to_sim(
            effort_limit,
            joint_ids=actuator.joint_indices,
            env_ids=env_ids,
        )


def reset_joints_by_offset(
    env,
    env_ids: torch.Tensor,
    position_range: tuple[float, float],
    velocity_range: tuple[float, float],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Reset selected joints with additive position and velocity offsets."""
    asset: Articulation = env.scene[asset_cfg.name]
    env_ids = env_ids.to(device=asset.device)

    default_joint_pos = asset.data.default_joint_pos[env_ids].clone()
    default_joint_vel = asset.data.default_joint_vel[env_ids].clone()
    joint_pos = default_joint_pos[:, asset_cfg.joint_ids].clone()
    joint_vel = default_joint_vel[:, asset_cfg.joint_ids].clone()

    joint_pos += torch.empty_like(joint_pos).uniform_(*position_range)
    joint_vel += torch.empty_like(joint_vel).uniform_(*velocity_range)
    joint_pos_limits = asset.data.soft_joint_pos_limits[env_ids][:, asset_cfg.joint_ids]
    joint_vel_limits = asset.data.soft_joint_vel_limits[env_ids][:, asset_cfg.joint_ids]
    joint_pos = joint_pos.clamp_(joint_pos_limits[..., 0], joint_pos_limits[..., 1])
    joint_vel = joint_vel.clamp_(-joint_vel_limits, joint_vel_limits)

    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids, joint_ids=asset_cfg.joint_ids)
