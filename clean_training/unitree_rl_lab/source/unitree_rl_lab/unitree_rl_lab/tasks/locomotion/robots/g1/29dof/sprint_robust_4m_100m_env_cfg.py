from __future__ import annotations

import math

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass

from unitree_rl_lab.tasks.locomotion import mdp
from unitree_rl_lab.tasks.locomotion.mdp import robust_highspeed

from .sprint_env_cfg import SprintBalancedCommandsCfg
from .sprint_robust_highspeed_env_cfg import (
    RobustHighSpeedEventsCfg,
    RobustHighSpeedRewardsCfg,
    SprintRobustHighSpeedEnvCfg,
    SprintRobustHighSpeedPlayEnvCfg,
)


@configclass
class Fixed4mRampVelocityCommandCfg(mdp.RampUniformLevelVelocityCommandCfg):
    """Keep the verified command generator and replay only a 4 m/s target."""

    class_type: type = robust_highspeed.MonitoredRampUniformVelocityCommand


@configclass
class Robust4m100mCommandsCfg(SprintBalancedCommandsCfg):
    base_velocity = Fixed4mRampVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(4.0, 6.0),
        ramp_time_range=(0.2, 1.0),
        rel_standing_envs=0.15,
        rel_heading_envs=0.0,
        heading_command=False,
        debug_vis=False,
        ranges=Fixed4mRampVelocityCommandCfg.Ranges(
            lin_vel_x=(4.0, 4.0),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(0.0, 0.0),
        ),
        limit_ranges=Fixed4mRampVelocityCommandCfg.Ranges(
            lin_vel_x=(4.0, 4.0),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(0.0, 0.0),
        ),
    )


@configclass
class BaselineRobustHighSpeedRewardsCfg(RobustHighSpeedRewardsCfg):
    """Restore the reward set recorded with model_85800."""

    track_lin_vel_xy = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=3.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.16)},
    )

    # These two terms were added after model_85800 and are intentionally not
    # enabled here so the checkpoint keeps its original reward contract.
    low_speed_height = None
    high_speed_posture = None
    pelvis_height_rate = None


@configclass
class SprintRobust4m100mEnvCfg(SprintRobustHighSpeedEnvCfg):
    """Fine-tune model_85800 for a straight 4 m/s nominal 100 m episode."""

    commands: Robust4m100mCommandsCfg = Robust4m100mCommandsCfg()
    events: RobustHighSpeedEventsCfg = RobustHighSpeedEventsCfg()
    rewards: BaselineRobustHighSpeedRewardsCfg = BaselineRobustHighSpeedRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        # 25 s * 4 m/s is the nominal 100 m training horizon.  All other
        # termination, dynamics-randomization and interface settings remain
        # inherited from the verified Robust HighSpeed task.
        self.episode_length_s = 25.0
        self.commands.base_velocity.class_type = robust_highspeed.MonitoredRampUniformVelocityCommand
        self.commands.base_velocity.ranges.lin_vel_x = (4.0, 4.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        self.commands.base_velocity.limit_ranges.lin_vel_x = (4.0, 4.0)
        self.commands.base_velocity.limit_ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.limit_ranges.ang_vel_z = (0.0, 0.0)


@configclass
class SprintRobust4m100mPlayEnvCfg(SprintRobustHighSpeedPlayEnvCfg):
    """Play configuration; --fixed_lin_vel_x 4.0 makes evaluation deterministic."""

    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = 25.0
