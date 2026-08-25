from __future__ import annotations

from collections.abc import Sequence

import torch
from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction
from isaaclab.utils import configclass


class DelayedSmoothedJointPositionAction(JointPositionAction):
    """Joint-position action with small randomized actuator delay and EMA smoothing.

    The action dimension and affine mapping remain identical to the original
    JointPositionAction, so a previously trained 29-DoF policy can be loaded.
    Delay is sampled per environment on reset from [0, max_delay_steps].
    """

    def __init__(self, cfg: DelayedSmoothedJointPositionActionCfg, env):
        super().__init__(cfg, env)
        self._max_delay_steps = int(cfg.max_delay_steps)
        self._smoothing_alpha = float(cfg.smoothing_alpha)
        if self._max_delay_steps < 0:
            raise ValueError("max_delay_steps must be non-negative")
        if not 0.0 < self._smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        self._delay_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._action_history = torch.zeros(
            self.num_envs, self._max_delay_steps + 1, self.action_dim, device=self.device
        )
        self._last_applied_action = torch.zeros_like(self._raw_actions)

    def _sample_delay(self, env_ids):
        count = self.num_envs if isinstance(env_ids, slice) else len(env_ids)
        self._delay_steps[env_ids] = torch.randint(
            0, self._max_delay_steps + 1, (count,), device=self.device
        )

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        if env_ids is None:
            env_ids = slice(None)
        super().reset(env_ids)
        self._sample_delay(env_ids)
        current_position = self._asset.data.joint_pos[:, self._joint_ids]
        self._action_history[env_ids] = current_position[env_ids, None, :]
        self._last_applied_action[env_ids] = current_position[env_ids]

    def process_actions(self, actions: torch.Tensor):
        super().process_actions(actions)
        if self._max_delay_steps > 0:
            self._action_history[:, 1:] = self._action_history[:, :-1].clone()
        self._action_history[:, 0] = self._processed_actions
        delay_indices = self._delay_steps[:, None, None].expand(-1, 1, self.action_dim)
        delayed_action = torch.gather(self._action_history, 1, delay_indices).squeeze(1)
        smoothed_action = self._smoothing_alpha * delayed_action
        smoothed_action += (1.0 - self._smoothing_alpha) * self._last_applied_action
        self._processed_actions[:] = smoothed_action
        self._last_applied_action[:] = smoothed_action


@configclass
class DelayedSmoothedJointPositionActionCfg(JointPositionActionCfg):
    """Configuration preserving the original 29-DoF action contract."""

    class_type: type = DelayedSmoothedJointPositionAction
    max_delay_steps: int = 1
    smoothing_alpha: float = 0.95
