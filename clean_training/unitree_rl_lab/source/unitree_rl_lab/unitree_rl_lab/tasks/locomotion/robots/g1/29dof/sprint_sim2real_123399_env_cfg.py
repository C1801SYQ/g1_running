from __future__ import annotations

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from unitree_rl_lab.tasks.locomotion import mdp
from unitree_rl_lab.tasks.locomotion.mdp import sim2real_action, sim2real_observations

from .sprint_env_cfg import SprintObservationsCfg, SprintPlayEnvCfg
from .sprint_robust_highspeed_env_cfg import (
    RobustHighSpeedEventsCfg,
    RobustHighSpeedRewardsCfg,
    SprintRobustHighSpeedEnvCfg,
)


@configclass
class Sim2RealActionsCfg:
    JointPositionAction = sim2real_action.DelayedSmoothedJointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=0.25,
        use_default_offset=True,
        max_delay_steps=1,
        smoothing_alpha=0.95,
    )


@configclass
class Sim2RealObservationsCfg(SprintObservationsCfg):
    @configclass
    class PolicyCfg(SprintObservationsCfg.PolicyCfg):
        base_ang_vel = ObsTerm(
            func=sim2real_observations.base_ang_vel,
            scale=0.2,
            noise=Unoise(n_min=-0.27, n_max=0.27),
        )
        projected_gravity = ObsTerm(
            func=sim2real_observations.projected_gravity,
            noise=Unoise(n_min=-0.09, n_max=0.09),
        )
        joint_pos_rel = ObsTerm(
            func=sim2real_observations.joint_pos_rel,
            noise=Unoise(n_min=-0.0175, n_max=0.0175),
        )
        joint_vel_rel = ObsTerm(
            func=sim2real_observations.joint_vel_rel,
            scale=0.05,
            noise=Unoise(n_min=-0.65, n_max=0.65),
        )

    policy: PolicyCfg = PolicyCfg()


@configclass
class Sim2RealEventsCfg(RobustHighSpeedEventsCfg):
    sensor_bias = EventTerm(
        func=sim2real_observations.reset_sim2real_sensor_bias,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "ang_vel_bias_range": (-0.015, 0.015),
            "gravity_bias_range": (-0.005, 0.005),
            "joint_pos_bias_range": (-0.001, 0.001),
            "joint_vel_bias_range": (-0.03, 0.03),
        },
    )


@configclass
class Sim2RealRewardsCfg(RobustHighSpeedRewardsCfg):
    # Small increases target real actuator smoothness without changing the
    # speed-tracking or high-speed retention terms.
    action_jerk = RewTerm(func=mdp.action_jerk_l2, weight=-0.03)
    waist_velocity = RewTerm(
        func=mdp.waist_velocity_l2,
        weight=-0.008,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["waist.*"])},
    )


@configclass
class SprintSim2Real123399EnvCfg(SprintRobustHighSpeedEnvCfg):
    """Independent actuator/sensor-aware fine-tuning task from model_123399."""

    actions: Sim2RealActionsCfg = Sim2RealActionsCfg()
    observations: Sim2RealObservationsCfg = Sim2RealObservationsCfg()
    events: Sim2RealEventsCfg = Sim2RealEventsCfg()
    rewards: Sim2RealRewardsCfg = Sim2RealRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        # Keep the verified 0--5.1 m/s command mixture unchanged.
        self.scene.num_envs = 8196
        self.events.mass_torso.params["mass_distribution_params"] = (0.97, 1.03)
        self.events.mass_legs_feet.params["mass_distribution_params"] = (0.98, 1.02)
        self.events.actuator_gains.params["stiffness_distribution_params"] = (0.96, 1.04)
        self.events.actuator_gains.params["damping_distribution_params"] = (0.96, 1.04)
        self.events.actuator_effort_limit.params["effort_scale_distribution_params"] = (0.95, 1.05)
        self.events.joint_armature.params["armature_distribution_params"] = (0.95, 1.05)
        self.events.joint_friction.params["friction_distribution_params"] = (0.0, 0.02)
        self.events.push_robot.interval_range_s = (8.0, 12.0)
        self.events.push_robot.params["velocity_range"] = {
            "x": (-0.12, 0.12),
            "y": (-0.12, 0.12),
            "z": (-0.04, 0.04),
            "roll": (-0.12, 0.12),
            "pitch": (-0.12, 0.12),
            "yaw": (-0.18, 0.18),
        }


@configclass
class SprintSim2Real123399PlayEnvCfg(SprintPlayEnvCfg):
    """Clean deterministic preview config; training-only bias/latency is disabled."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 1
