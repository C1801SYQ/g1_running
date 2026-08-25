from __future__ import annotations

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from unitree_rl_lab.tasks.locomotion import mdp

from .sprint_robust_4m_100m_env_cfg import (
    BaselineRobustHighSpeedRewardsCfg,
    SprintRobust4m100mEnvCfg,
    SprintRobust4m100mPlayEnvCfg,
)


@configclass
class Stable4m100mRewardsCfg(BaselineRobustHighSpeedRewardsCfg):
    """Small stability-only correction for the late 4 m/s continuation."""

    # The previous run had intermittent low-height terminations while its
    # velocity and straight-line metrics stayed healthy.  Strengthen the
    # existing height objective and add only a low-weight rate term; the
    # 96/29 policy contract and straight-line terms remain unchanged.
    base_height = RewTerm(
        func=mdp.base_height_l2,
        weight=-12.0,
        params={"target_height": 0.78},
    )
    pelvis_height_rate = RewTerm(
        func=mdp.root_height_rate_l2,
        weight=-0.04,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class SprintRobust4m100mStableEnvCfg(SprintRobust4m100mEnvCfg):
    """Continuation task: stable, straight 4 m/s running."""

    rewards: Stable4m100mRewardsCfg = Stable4m100mRewardsCfg()


@configclass
class SprintRobust4m100mStablePlayEnvCfg(SprintRobust4m100mPlayEnvCfg):
    """Deterministic fixed-speed preview for the stability continuation."""
