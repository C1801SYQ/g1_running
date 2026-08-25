from __future__ import annotations

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_apply, quat_apply_inverse, yaw_quat

from .commands.velocity_command import RampUniformVelocityCommand


SPEED_MONITOR_BANDS = (
    ("speed_0_1p2", 0.0, 1.2),
    ("speed_1p2_3", 1.2, 3.0),
    ("speed_3_4p5", 3.0, 4.5),
    ("speed_4p5_5p1", 4.5, 5.1001),
    ("speed_5p1_7", 5.1, 7.0001),
)

# Fractions are conditional on a moving command. Together with the 15%
# standing probability in the environment config they yield a retention-first
# mixture: 15% low, 15% medium, 10% medium-high, 50% verified high
# (4.5--5.1 m/s), 10% exact-frontier (5.1 m/s), and 15% standing.  The
# previous run gradually expanded a 5.1--7.0 m/s frontier and then lost the
# video-verified 4.5/5.1 gait.  Keep the verified frontier fixed until a
# separate, explicitly validated extension is available.
MOVING_SPEED_TARGET_BANDS = (
    (0.0, 1.2, 0.15 / 0.85),
    (1.2, 3.0, 0.15 / 0.85),
    (3.0, 4.5, 0.10 / 0.85),
    (4.5, 5.1, 0.50 / 0.85),
    (5.1, 5.1001, 0.10 / 0.85),
)

# Retention recovery deliberately does not introduce a moving frontier.  The
# constants remain explicit so a later, separately validated extension can
# re-enable a curriculum without silently changing this recovery baseline.
FRONTIER_SPEED_START = 5.1
FRONTIER_SPEED_END = 5.1
FRONTIER_RAMP_STEPS = 1


class MonitoredRampUniformVelocityCommand(RampUniformVelocityCommand):
    """Ramp command with per-speed-band telemetry for proactive monitoring."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        for band_name, _, _ in SPEED_MONITOR_BANDS:
            for metric_name in ("sample_fraction", "command_x", "actual_x", "abs_error", "tracking_ratio"):
                self.metrics[f"{band_name}_{metric_name}"] = torch.zeros(self.num_envs, device=self.device)

    def _update_metrics(self):
        super()._update_metrics()
        max_command_step = self.cfg.resampling_time_range[1] / self._env.step_dt
        command_x = self.vel_command_b[:, 0]
        actual_x = self.robot.data.root_lin_vel_b[:, 0]

        for band_name, lower, upper in SPEED_MONITOR_BANDS:
            mask = (command_x >= lower) & (command_x < upper)
            mask_float = mask.to(dtype=command_x.dtype)
            safe_command = command_x.clamp_min(0.1)
            self.metrics[f"{band_name}_sample_fraction"] += mask_float / max_command_step
            self.metrics[f"{band_name}_command_x"] += torch.where(mask, command_x, 0.0) / max_command_step
            self.metrics[f"{band_name}_actual_x"] += torch.where(mask, actual_x, 0.0) / max_command_step
            self.metrics[f"{band_name}_abs_error"] += (
                torch.where(mask, torch.abs(actual_x - command_x), 0.0) / max_command_step
            )
            self.metrics[f"{band_name}_tracking_ratio"] += (
                torch.where(mask, actual_x / safe_command, 0.0) / max_command_step
            )


class StratifiedMonitoredRampVelocityCommand(MonitoredRampUniformVelocityCommand):
    """Ramp command with explicit replay of the verified high-speed skill."""

    def _resample_command(self, env_ids):
        if isinstance(env_ids, slice):
            env_ids = torch.arange(self.num_envs, device=self.device)[env_ids]
        elif not isinstance(env_ids, torch.Tensor):
            env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)

        # Reuse the existing ramp, lateral/yaw sampling, standing handling and
        # timer initialization, then replace only the forward target samples.
        super()._resample_command(env_ids)
        weights = torch.tensor(
            [band[2] for band in MOVING_SPEED_TARGET_BANDS],
            device=self.device,
            dtype=self.target_vel_command_b.dtype,
        )
        band_ids = torch.multinomial(weights, len(env_ids), replacement=True)
        random_values = torch.empty(len(env_ids), device=self.device)
        common_step_counter = getattr(self._env, "common_step_counter", 0)
        if isinstance(common_step_counter, torch.Tensor):
            common_step_counter = float(common_step_counter.item())
        frontier_alpha = min(max(float(common_step_counter) / FRONTIER_RAMP_STEPS, 0.0), 1.0)
        frontier_upper = FRONTIER_SPEED_START + (FRONTIER_SPEED_END - FRONTIER_SPEED_START) * frontier_alpha

        for band_id, (lower, upper, _) in enumerate(MOVING_SPEED_TARGET_BANDS):
            if band_id == len(MOVING_SPEED_TARGET_BANDS) - 1:
                lower, upper = FRONTIER_SPEED_START, frontier_upper
            band_mask = band_ids == band_id
            if torch.any(band_mask):
                selected_env_ids = env_ids[band_mask]
                self.target_vel_command_b[selected_env_ids, 0] = random_values[band_mask].uniform_(lower, upper)

        # The parent sampled standing environments before the forward targets
        # were replaced, so enforce their zero target once more.
        self.target_vel_command_b[env_ids] *= (~self.is_standing_env[env_ids]).unsqueeze(-1)


def _state(env, asset: Articulation):
    """Create per-environment state used only by the robust high-speed task."""
    if not hasattr(env, "_g1_highspeed_anchor_pos_w"):
        env._g1_highspeed_anchor_pos_w = torch.zeros_like(asset.data.root_pos_w)
        env._g1_highspeed_anchor_yaw = torch.zeros_like(asset.data.root_quat_w)
        env._g1_highspeed_lateral_error_integral = torch.zeros(env.num_envs, device=asset.device)
        env._g1_highspeed_prev_root_height = torch.zeros(env.num_envs, device=asset.device)
        env._g1_highspeed_prev_action_delta = torch.zeros(
            env.num_envs, env.action_manager.total_action_dim, device=asset.device
        )
    return env


def reset_motion_anchor(
    env,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Reset the local frame used for drift and heading-error rewards."""
    asset: Articulation = env.scene[asset_cfg.name]
    env = _state(env, asset)
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=asset.device)
    else:
        env_ids = env_ids.to(device=asset.device)

    env._g1_highspeed_anchor_pos_w[env_ids] = asset.data.root_pos_w[env_ids]
    env._g1_highspeed_anchor_yaw[env_ids] = yaw_quat(asset.data.root_quat_w[env_ids])
    env._g1_highspeed_lateral_error_integral[env_ids] = 0.0
    env._g1_highspeed_prev_root_height[env_ids] = asset.data.root_pos_w[env_ids, 2]
    env._g1_highspeed_prev_action_delta[env_ids] = 0.0


def _forward_gate(env, command_name: str) -> torch.Tensor:
    command = env.command_manager.get_command(command_name)
    # Smoothly activate the trajectory terms around walking speed instead of
    # switching them on at a hard threshold.
    return torch.sigmoid((command[:, 0] - 0.35) / 0.15)


def straight_line_position_l2(
    env,
    command_name: str = "base_velocity",
    yaw_weight: float = 0.25,
    integral_weight: float = 0.05,
    lateral_scale: float = 0.75,
    integral_scale: float = 0.5,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize accumulated lateral drift and heading drift in the start frame."""
    asset: Articulation = env.scene[asset_cfg.name]
    env = _state(env, asset)

    anchor_yaw = env._g1_highspeed_anchor_yaw
    current_yaw = yaw_quat(asset.data.root_quat_w)
    displacement_b = quat_apply_inverse(
        anchor_yaw, asset.data.root_pos_w - env._g1_highspeed_anchor_pos_w
    )
    lateral_error = displacement_b[:, 1]

    forward = torch.zeros(env.num_envs, 3, device=asset.device)
    forward[:, 0] = 1.0
    anchor_forward = quat_apply(anchor_yaw, forward)
    current_forward = quat_apply(current_yaw, forward)
    cross_z = anchor_forward[:, 0] * current_forward[:, 1] - anchor_forward[:, 1] * current_forward[:, 0]
    dot = torch.sum(anchor_forward[:, :2] * current_forward[:, :2], dim=1).clamp(-1.0, 1.0)
    yaw_error = torch.atan2(cross_z, dot)

    env._g1_highspeed_lateral_error_integral += torch.abs(lateral_error).detach() * env.step_dt
    integral_error = torch.tanh(env._g1_highspeed_lateral_error_integral / integral_scale)
    gate = _forward_gate(env, command_name)
    # Keep the navigation penalty bounded.  An unbounded squared displacement
    # can exceed the speed-tracking reward during a long high-speed episode and
    # make standing still the cheaper policy.
    lateral_cost = torch.square(torch.tanh(lateral_error / lateral_scale))
    yaw_cost = torch.square(torch.sin(yaw_error))
    return gate * (lateral_cost + yaw_weight * yaw_cost + integral_weight * torch.square(integral_error))


def root_height_rate_l2(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize fast pelvis-height changes without changing the height target."""
    asset: Articulation = env.scene[asset_cfg.name]
    env = _state(env, asset)
    height = asset.data.root_pos_w[:, 2]
    height_rate = (height - env._g1_highspeed_prev_root_height) / env.step_dt
    env._g1_highspeed_prev_root_height = height.detach()
    return torch.square(height_rate)


def action_jerk_l2(env) -> torch.Tensor:
    """Penalize the second difference of actions to suppress command twitching."""
    asset: Articulation = env.scene["robot"]
    env = _state(env, asset)
    action_delta = env.action_manager.action - env.action_manager.prev_action
    action_jerk = action_delta - env._g1_highspeed_prev_action_delta
    env._g1_highspeed_prev_action_delta = action_delta.detach()
    return torch.mean(torch.square(action_jerk), dim=1)


def waist_velocity_l2(
    env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot", joint_names=["waist.*"])
) -> torch.Tensor:
    """Keep waist joints quiet while retaining the existing whole-body velocity cost."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.mean(torch.square(asset.data.joint_vel[:, asset_cfg.joint_ids]), dim=1)


def high_speed_undertracking_huber(
    env,
    command_name: str = "base_velocity",
    speed_threshold: float = 3.5,
    gate_width: float = 0.20,
    huber_delta: float = 0.5,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Keep a non-saturating learning signal when a high-speed policy falls behind."""
    asset: Articulation = env.scene[asset_cfg.name]
    command_x = env.command_manager.get_command(command_name)[:, 0]
    actual_x = asset.data.root_lin_vel_b[:, 0]
    under_speed = torch.clamp_min(command_x - actual_x, 0.0)
    huber = torch.where(
        under_speed < huber_delta,
        0.5 * torch.square(under_speed) / huber_delta,
        under_speed - 0.5 * huber_delta,
    )
    gate = torch.sigmoid((command_x - speed_threshold) / gate_width)
    return gate * huber


def low_speed_height_l2(
    env,
    command_name: str = "base_velocity",
    target_height: float = 0.78,
    speed_threshold: float = 1.4,
    gate_width: float = 0.25,
    height_scale: float = 0.06,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Prevent the crouched local optimum in standing and low-speed walking."""
    asset: Articulation = env.scene[asset_cfg.name]
    command_x = env.command_manager.get_command(command_name)[:, 0]
    low_speed_gate = torch.sigmoid((speed_threshold - command_x) / gate_width)
    height_error = torch.relu(target_height - asset.data.root_pos_w[:, 2]) / height_scale
    return low_speed_gate * torch.square(height_error)


def high_speed_posture_l2(
    env,
    command_name: str = "base_velocity",
    speed_threshold: float = 3.0,
    gate_width: float = 0.45,
    minimum_height: float = 0.70,
    height_scale: float = 0.05,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Keep the high-speed pelvis from collapsing while damping trunk motion."""
    asset: Articulation = env.scene[asset_cfg.name]
    command_x = env.command_manager.get_command(command_name)[:, 0]
    gate = torch.sigmoid((command_x - speed_threshold) / gate_width)
    height_deficit = torch.relu(minimum_height - asset.data.root_pos_w[:, 2]) / height_scale
    trunk_ang_vel = torch.sum(torch.square(asset.data.root_ang_vel_b[:, :2]), dim=1)
    lateral_vel = torch.square(asset.data.root_lin_vel_b[:, 1])
    return gate * (torch.square(height_deficit) + 0.25 * trunk_ang_vel + 0.25 * lateral_vel)
