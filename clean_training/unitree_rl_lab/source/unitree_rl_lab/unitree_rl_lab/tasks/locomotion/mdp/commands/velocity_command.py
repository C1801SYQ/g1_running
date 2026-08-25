from __future__ import annotations

from collections.abc import Sequence
from dataclasses import MISSING

import torch

from isaaclab.envs.mdp import UniformVelocityCommand, UniformVelocityCommandCfg
from isaaclab.utils import configclass


@configclass
class UniformLevelVelocityCommandCfg(UniformVelocityCommandCfg):
    limit_ranges: UniformVelocityCommandCfg.Ranges = MISSING


class RampUniformVelocityCommand(UniformVelocityCommand):
    """Uniform velocity commands with a short, per-environment command ramp.

    The policy still receives the same three velocity-command values.  Only the
    command generator changes: every newly sampled target is reached from the
    previous command over a randomized short interval, which makes start,
    stop, and speed-change transitions continuous without changing the policy
    or deployment observation/action contract.
    """

    cfg: "RampUniformLevelVelocityCommandCfg"

    def __init__(self, cfg: "RampUniformLevelVelocityCommandCfg", env):
        super().__init__(cfg, env)
        self.target_vel_command_b = torch.zeros_like(self.vel_command_b)
        self.ramp_start_vel_command_b = torch.zeros_like(self.vel_command_b)
        self.ramp_elapsed = torch.zeros(self.num_envs, device=self.device)
        self.ramp_duration = torch.ones(self.num_envs, device=self.device)

    def _resample_command(self, env_ids: Sequence[int]):
        # Preserve the current command as the start point, then sample a new
        # target exactly as UniformVelocityCommand does.
        self.ramp_start_vel_command_b[env_ids] = self.vel_command_b[env_ids]
        r = torch.empty(len(env_ids), device=self.device)
        self.target_vel_command_b[env_ids, 0] = r.uniform_(*self.cfg.ranges.lin_vel_x)
        self.target_vel_command_b[env_ids, 1] = r.uniform_(*self.cfg.ranges.lin_vel_y)
        self.target_vel_command_b[env_ids, 2] = r.uniform_(*self.cfg.ranges.ang_vel_z)

        if self.cfg.heading_command:
            self.heading_target[env_ids] = r.uniform_(*self.cfg.ranges.heading)
            self.is_heading_env[env_ids] = r.uniform_(0.0, 1.0) <= self.cfg.rel_heading_envs

        self.is_standing_env[env_ids] = r.uniform_(0.0, 1.0) <= self.cfg.rel_standing_envs
        self.target_vel_command_b[env_ids] *= (~self.is_standing_env[env_ids]).unsqueeze(-1)
        self.ramp_elapsed[env_ids] = 0.0
        self.ramp_duration[env_ids] = r.uniform_(*self.cfg.ramp_time_range)

    def _update_command(self):
        self.ramp_elapsed = torch.minimum(
            self.ramp_elapsed + self._env.step_dt,
            self.ramp_duration,
        )
        alpha = (self.ramp_elapsed / self.ramp_duration.clamp_min(1.0e-6)).clamp(0.0, 1.0)
        self.vel_command_b[:] = self.ramp_start_vel_command_b + alpha.unsqueeze(-1) * (
            self.target_vel_command_b - self.ramp_start_vel_command_b
        )
        super()._update_command()


@configclass
class RampUniformLevelVelocityCommandCfg(UniformLevelVelocityCommandCfg):
    class_type: type = RampUniformVelocityCommand
    ramp_time_range: tuple[float, float] = (0.2, 1.0)
