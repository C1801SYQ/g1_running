from __future__ import annotations

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from unitree_rl_lab.tasks.locomotion import mdp
from unitree_rl_lab.tasks.locomotion.mdp import robust_highspeed

from .sprint_env_cfg import (
    SprintBalancedCommandsCfg,
    SprintBalancedEnvCfg,
    SprintBalancedEventsCfg,
    SprintBalancedRewardsCfg,
    SprintPlayEnvCfg,
)


@configclass
class MonitoredRampUniformLevelVelocityCommandCfg(mdp.RampUniformLevelVelocityCommandCfg):
    """Current-task-only command config with speed-band telemetry."""

    class_type: type = robust_highspeed.StratifiedMonitoredRampVelocityCommand


@configclass
class RobustHighSpeedCommandsCfg(SprintBalancedCommandsCfg):
    """Full verified speed coverage with continuous command transitions."""

    base_velocity = MonitoredRampUniformLevelVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(4.0, 6.0),
        ramp_time_range=(0.2, 1.0),
        rel_standing_envs=0.15,
        rel_heading_envs=0.0,
        heading_command=False,
        debug_vis=False,
        ranges=MonitoredRampUniformLevelVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 5.1), lin_vel_y=(-0.08, 0.08), ang_vel_z=(-0.20, 0.20)
        ),
        # The checkpoint already has verified 4.5/5.1 m/s behavior.  Sampling
        # the full range prevents low-speed fine-tuning from erasing it.
        limit_ranges=MonitoredRampUniformLevelVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 5.1), lin_vel_y=(-0.12, 0.12), ang_vel_z=(-0.50, 0.50)
        ),
    )


@configclass
class RobustHighSpeedEventsCfg(SprintBalancedEventsCfg):
    """Reuse the verified dynamics randomization and add only reset anchors."""

    actuator_effort_limit = EventTerm(
        func=mdp.randomize_actuator_effort_limit,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "effort_scale_distribution_params": (0.97, 1.03),
        },
    )
    joint_armature = EventTerm(
        func=mdp.randomize_joint_parameters,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "armature_distribution_params": (0.95, 1.05),
            "operation": "scale",
        },
    )

    motion_anchor = EventTerm(
        func=robust_highspeed.reset_motion_anchor,
        mode="reset",
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class RobustHighSpeedRewardsCfg(SprintBalancedRewardsCfg):
    """Add low-weight smoothness and trajectory-stability terms."""

    straight_line_position = RewTerm(
        func=robust_highspeed.straight_line_position_l2,
        weight=-0.30,
        params={"command_name": "base_velocity", "yaw_weight": 0.25, "integral_weight": 0.05},
    )
    root_height_rate = RewTerm(func=robust_highspeed.root_height_rate_l2, weight=-0.03)
    action_jerk = RewTerm(func=robust_highspeed.action_jerk_l2, weight=-0.02)
    waist_velocity = RewTerm(
        func=robust_highspeed.waist_velocity_l2,
        weight=-0.006,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["waist.*"])},
    )
    high_speed_undertracking = RewTerm(
        func=robust_highspeed.high_speed_undertracking_huber,
        weight=-0.35,
        params={
            "command_name": "base_velocity",
            "speed_threshold": 3.5,
            "gate_width": 0.20,
            "huber_delta": 0.5,
        },
    )
    low_speed_height = RewTerm(
        func=robust_highspeed.low_speed_height_l2,
        weight=-0.20,
        params={"command_name": "base_velocity", "target_height": 0.78, "speed_threshold": 1.4},
    )
    high_speed_posture = RewTerm(
        func=robust_highspeed.high_speed_posture_l2,
        weight=-0.08,
        params={"command_name": "base_velocity", "speed_threshold": 3.0, "minimum_height": 0.70},
    )


@configclass
class SprintRobustHighSpeedEnvCfg(SprintBalancedEnvCfg):
    """Independent high-speed robustness task; the existing task is untouched."""

    commands: RobustHighSpeedCommandsCfg = RobustHighSpeedCommandsCfg()
    events: RobustHighSpeedEventsCfg = RobustHighSpeedEventsCfg()
    rewards: RobustHighSpeedRewardsCfg = RobustHighSpeedRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        # The verified model_80250 is a high-speed checkpoint, so every update
        # must keep seeing the full 0~5.1 m/s range.  Reward-gated expansion
        # previously stalled at 1.5 m/s and caused high-speed forgetting.
        self.scene.num_envs = 8196
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 5.1)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.08, 0.08)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.20, 0.20)
        self.commands.base_velocity.limit_ranges.lin_vel_x = (0.0, 5.1)
        self.curriculum.lin_vel_cmd_levels = None
        self.curriculum.ang_vel_cmd_levels = None

        # Stage 1 preserves meaningful dynamics diversity but deliberately
        # starts narrower than the failed run.  Stronger pushes and parameter
        # ranges are enabled only after the high-speed telemetry remains above
        # its guard thresholds.
        self.events.physics_material.params.update(
            {
                "static_friction_range": (0.65, 1.20),
                "dynamic_friction_range": (0.55, 1.05),
                "restitution_range": (0.0, 0.10),
            }
        )
        self.events.mass_torso.params["mass_distribution_params"] = (0.97, 1.03)
        self.events.mass_legs_feet.params["mass_distribution_params"] = (0.98, 1.02)
        self.events.actuator_gains.params["stiffness_distribution_params"] = (0.97, 1.03)
        self.events.actuator_gains.params["damping_distribution_params"] = (0.97, 1.03)
        self.events.joint_friction.params["friction_distribution_params"] = (0.0, 0.01)
        self.events.actuator_effort_limit.params["effort_scale_distribution_params"] = (0.99, 1.01)
        self.events.joint_armature.params["armature_distribution_params"] = (0.98, 1.02)
        self.events.add_joint_default_pos.params["pos_distribution_params"] = (-0.003, 0.003)
        self.events.base_com.params["com_range"] = {
            "x": (-0.010, 0.010),
            "y": (-0.015, 0.015),
            "z": (-0.015, 0.015),
        }
        self.events.reset_base.params["velocity_range"] = {
            "x": (-0.10, 0.10),
            "y": (-0.10, 0.10),
            "z": (-0.03, 0.03),
            "roll": (-0.08, 0.08),
            "pitch": (-0.08, 0.08),
            "yaw": (-0.12, 0.12),
        }
        self.events.reset_robot_joints.params["position_range"] = (-0.03, 0.03)
        self.events.push_robot.interval_range_s = (6.0, 10.0)
        self.events.push_robot.params["velocity_range"] = {
            "x": (-0.08, 0.08),
            "y": (-0.08, 0.08),
            "z": (-0.02, 0.02),
            "roll": (-0.06, 0.06),
            "pitch": (-0.06, 0.06),
            "yaw": (-0.10, 0.10),
        }


@configclass
class SprintRobustHighSpeedPlayEnvCfg(SprintPlayEnvCfg):
    """Deterministic fixed-speed preview configuration for this task."""

    def __post_init__(self):
        super().__post_init__()
        self.commands.base_velocity.limit_ranges.lin_vel_x = (0.0, 5.1)
