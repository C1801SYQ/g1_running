import math
import os

import isaaclab.sim as sim_utils
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from unitree_rl_lab.tasks.locomotion import mdp
from unitree_rl_lab.tasks.locomotion.mdp import robust_highspeed

from .velocity_env_cfg import ActionsCfg, CommandsCfg, EventCfg, RobotEnvCfg, RobotSceneCfg, TerminationsCfg


ROBUST_PUSH_VELOCITY_RANGE = {
    "x": (-0.5, 0.5),
    "y": (-0.5, 0.5),
    "z": (-0.2, 0.2),
    "roll": (-0.52, 0.52),
    "pitch": (-0.52, 0.52),
    "yaw": (-0.78, 0.78),
}

# Keep the full reset-state randomization above, but use a learnable pulse
# range during high-speed running.  Applying the full reset velocity every
# 1--3 seconds repeatedly destroys the 4.9--5.1 m/s gait before it can recover.
ROBUST_RUNNING_PUSH_VELOCITY_RANGE = {
    "x": (-0.25, 0.25),
    "y": (-0.25, 0.25),
    "z": (-0.10, 0.10),
    "roll": (-0.26, 0.26),
    "pitch": (-0.26, 0.26),
    "yaw": (-0.39, 0.39),
}


def _float_env(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


@configclass
class SprintCommandsCfg(CommandsCfg):
    base_velocity = mdp.UniformLevelVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(4.0, 6.0),
        rel_standing_envs=0.15,
        rel_heading_envs=0.0,
        heading_command=False,
        debug_vis=False,
        ranges=mdp.UniformLevelVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 0.8), lin_vel_y=(-0.03, 0.03), ang_vel_z=(-0.05, 0.05)
        ),
        limit_ranges=mdp.UniformLevelVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 4.5), lin_vel_y=(-0.30, 0.30), ang_vel_z=(-0.50, 0.50)
        ),
    )


@configclass
class SprintObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, scale=0.2, noise=Unoise(n_min=-0.25, n_max=0.25))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.08, n_max=0.08))
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.015, n_max=0.015))
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel, scale=0.05, noise=Unoise(n_min=-0.6, n_max=0.6))
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.history_length = 1
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()

    @configclass
    class CriticCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, scale=0.2)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel, scale=0.05)
        last_action = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.history_length = 1

    critic: CriticCfg = CriticCfg()


@configclass
class SprintRewardsCfg:
    track_lin_vel_xy = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=2.5,
        params={"command_name": "base_velocity", "std": math.sqrt(0.20)},
    )
    track_ang_vel_z = RewTerm(
        func=mdp.track_ang_vel_z_exp,
        weight=0.25,
        params={"command_name": "base_velocity", "std": math.sqrt(0.20)},
    )
    alive = RewTerm(func=mdp.is_alive, weight=0.25)
    base_linear_velocity = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    base_angular_velocity = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.08)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.001)
    joint_acc = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.08)
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-5.0)
    energy = RewTerm(func=mdp.energy, weight=-2e-5)
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-7.0)
    base_height = RewTerm(func=mdp.base_height_l2, weight=-10.0, params={"target_height": 0.78})
    gait = RewTerm(
        func=mdp.feet_gait,
        weight=0.8,
        params={
            "period": 0.62,
            "offset": [0.0, 0.5],
            "threshold": 0.55,
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*ankle_roll.*"),
        },
    )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.3,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*ankle_roll.*"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*ankle_roll.*"),
        },
    )
    feet_clearance = RewTerm(
        func=mdp.foot_clearance_reward,
        weight=1.2,
        params={
            "std": 0.06,
            "tanh_mult": 2.0,
            "target_height": 0.12,
            "asset_cfg": SceneEntityCfg("robot", body_names=".*ankle_roll.*"),
        },
    )
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.5,
        params={
            "threshold": 1,
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["(?!.*ankle.*).*"]),
        },
    )
    joint_deviation_arms = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.05,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_shoulder_.*_joint", ".*_elbow_joint", ".*_wrist_.*"])},
    )
    joint_deviation_waists = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["waist.*"])},
    )
    joint_deviation_legs = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_roll_joint", ".*_hip_yaw_joint"])},
    )


@configclass
class SprintEnvCfg(RobotEnvCfg):
    scene: RobotSceneCfg = RobotSceneCfg(num_envs=9216, env_spacing=2.5)
    commands: SprintCommandsCfg = SprintCommandsCfg()
    observations: SprintObservationsCfg = SprintObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: SprintRewardsCfg = SprintRewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        super().__post_init__()
        self.scene.terrain = TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="plane",
            terrain_generator=None,
            collision_group=-1,
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.0,
                dynamic_friction=1.0,
            ),
        )
        self.scene.num_envs = 9216
        self.episode_length_s = 20.0
        self.events.push_robot = None
        self.events.physics_material = None
        self.events.add_base_mass = None
        self.events.base_external_force_torque = None
        self.terminations.base_height.params["minimum_height"] = 0.55
        self.terminations.bad_orientation.params["limit_angle"] = 0.55
        self.curriculum.terrain_levels = None
        self.curriculum.lin_vel_cmd_levels = CurrTerm(func=mdp.lin_vel_cmd_levels)


@configclass
class SprintBalancedRecoveryVelocityCommandCfg(mdp.RampUniformLevelVelocityCommandCfg):
    """Continuous mixed-speed commands with protected 4.5--5.1 and 5.1--7 m/s bands."""

    class_type: type = mdp.StratifiedMonitoredRampVelocityCommand


@configclass
class SprintBalancedCommandsCfg(SprintCommandsCfg):
    """Concurrent low-speed retention and frontier high-speed training."""

    base_velocity = SprintBalancedRecoveryVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(4.0, 6.0),
        ramp_time_range=(0.6, 1.2),
        rel_standing_envs=0.15,
        rel_heading_envs=0.0,
        heading_command=False,
        debug_vis=False,
        ranges=SprintBalancedRecoveryVelocityCommandCfg.Ranges(
            # Recovery keeps the video-verified 4.5--5.1 m/s range.  The
            # 5.1 m/s frontier is held fixed until stability is revalidated.
            lin_vel_x=(0.0, 5.1), lin_vel_y=(-0.05, 0.05), ang_vel_z=(-0.10, 0.10)
        ),
        limit_ranges=SprintBalancedRecoveryVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 5.1), lin_vel_y=(-0.12, 0.12), ang_vel_z=(-0.50, 0.50)
        ),
    )


@configclass
class SprintBalancedRewardsCfg(SprintRewardsCfg):
    track_lin_vel_xy = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=3.0,
        # Preserve useful gradient while the policy climbs back from the
        # 4.4 m/s verified baseline toward 5.1 m/s.
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    track_ang_vel_z = RewTerm(
        func=mdp.track_ang_vel_z_exp,
        weight=0.6,
        params={"command_name": "base_velocity", "std": math.sqrt(0.16)},
    )
    base_angular_velocity = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.12)
    joint_acc = RewTerm(func=mdp.joint_acc_l2, weight=-5.0e-7)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.12)
    stand_still = RewTerm(
        func=mdp.stand_still,
        weight=-0.25,
        params={"command_name": "base_velocity", "asset_cfg": SceneEntityCfg("robot")},
    )
    straight_line = RewTerm(
        func=mdp.straight_line_l2,
        weight=-0.55,
        params={"forward_threshold": 0.5, "lateral_weight": 1.0, "yaw_weight": 0.75},
    )
    straight_line_position = RewTerm(
        func=robust_highspeed.straight_line_position_l2,
        weight=-0.30,
        params={"command_name": "base_velocity", "yaw_weight": 0.25, "integral_weight": 0.05},
    )
    low_speed_posture = RewTerm(
        func=mdp.low_speed_posture_l1,
        weight=-0.55,
        params={
            "speed_threshold": 1.2,
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=[".*_shoulder_.*_joint", ".*_elbow_joint", ".*_wrist_.*", "waist.*"]
            ),
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
    joint_deviation_arms = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.20,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_shoulder_.*_joint", ".*_elbow_joint", ".*_wrist_.*"])},
    )
    joint_deviation_waists = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.85,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["waist.*"])},
    )
    high_speed_undertracking = RewTerm(
        func=mdp.high_speed_undertracking_huber,
        # Keep an unsaturated forward-progress signal, but do not make a
        # standing local optimum preferable to moving at high speed.
        weight=-0.35,
        params={"command_name": "base_velocity", "speed_threshold": 3.5},
    )
    pelvis_height_rate = RewTerm(
        func=mdp.root_height_rate_l2,
        weight=-0.03,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )
    action_jerk = RewTerm(func=mdp.action_jerk_l2, weight=-0.02)
    waist_velocity = RewTerm(
        func=mdp.waist_velocity_l2,
        weight=-0.006,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["waist.*"])},
    )


@configclass
class SprintBalancedEventsCfg(EventCfg):
    """Stage-1 dynamics randomization focused on sim-to-real robustness."""

    mass_torso = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="torso_link"),
            "mass_distribution_params": (0.92, 1.08),
            "operation": "scale",
            "recompute_inertia": True,
        },
    )
    mass_legs_feet = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", body_names=[".*_hip_.*", ".*_knee_.*", ".*_ankle_.*"]
            ),
            "mass_distribution_params": (0.95, 1.05),
            "operation": "scale",
            "recompute_inertia": True,
        },
    )
    actuator_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "stiffness_distribution_params": (0.90, 1.10),
            "damping_distribution_params": (0.90, 1.10),
            "operation": "scale",
        },
    )
    joint_friction = EventTerm(
        func=mdp.randomize_joint_parameters,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "friction_distribution_params": (0.0, 0.03),
            "operation": "abs",
        },
    )
    motion_anchor = EventTerm(
        func=robust_highspeed.reset_motion_anchor,
        mode="reset",
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class SprintBalancedEnvCfg(SprintEnvCfg):
    commands: SprintBalancedCommandsCfg = SprintBalancedCommandsCfg()
    events: SprintBalancedEventsCfg = SprintBalancedEventsCfg()
    rewards: SprintBalancedRewardsCfg = SprintBalancedRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        # During recovery, performance must be evaluated by speed band rather
        # than advancing a global curriculum whose low-speed samples can mask
        # high-speed regression.
        self.curriculum.lin_vel_cmd_levels = None
        self.curriculum.ang_vel_cmd_levels = None
        # Robustness randomization is enabled only for this training config.  The
        # play config remains disturbance-free so fixed-speed previews stay comparable.
        self.events.physics_material = EventTerm(
            func=mdp.randomize_rigid_body_material,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
                "static_friction_range": (0.65, 1.20),
                "dynamic_friction_range": (0.55, 1.05),
                "restitution_range": (0.0, 0.10),
                "num_buckets": 64,
            },
        )
        self.events.add_joint_default_pos = EventTerm(
            func=mdp.randomize_joint_default_pos,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
                "pos_distribution_params": (-0.003, 0.003),
                "operation": "add",
            },
        )
        self.events.base_com = EventTerm(
            func=mdp.randomize_rigid_body_com,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names="torso_link"),
                "com_range": {"x": (-0.010, 0.010), "y": (-0.015, 0.015), "z": (-0.015, 0.015)},
            },
        )
        self.events.reset_base = EventTerm(
            func=mdp.reset_root_state_uniform,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "pose_range": {
                    "x": (-0.05, 0.05),
                    "y": (-0.05, 0.05),
                    "z": (-0.01, 0.01),
                    "roll": (-0.1, 0.1),
                    "pitch": (-0.1, 0.1),
                    "yaw": (-0.2, 0.2),
                },
                "velocity_range": {
                    "x": (-0.10, 0.10), "y": (-0.10, 0.10), "z": (-0.03, 0.03),
                    "roll": (-0.08, 0.08), "pitch": (-0.08, 0.08), "yaw": (-0.12, 0.12),
                },
            },
        )
        self.events.reset_robot_joints = EventTerm(
            func=mdp.reset_joints_by_offset,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
                "position_range": (-0.03, 0.03),
                "velocity_range": (0.0, 0.0),
            },
        )
        self.events.push_robot = EventTerm(
            func=mdp.push_by_setting_velocity,
            mode="interval",
            interval_range_s=(6.0, 10.0),
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "velocity_range": {
                    "x": (-0.08, 0.08), "y": (-0.08, 0.08), "z": (-0.02, 0.02),
                    "roll": (-0.06, 0.06), "pitch": (-0.06, 0.06), "yaw": (-0.10, 0.10),
                },
            },
        )
        self.events.mass_torso.params["mass_distribution_params"] = (0.97, 1.03)
        self.events.mass_legs_feet.params["mass_distribution_params"] = (0.98, 1.02)
        self.events.actuator_gains.params["stiffness_distribution_params"] = (0.97, 1.03)
        self.events.actuator_gains.params["damping_distribution_params"] = (0.97, 1.03)
        self.events.joint_friction.params["friction_distribution_params"] = (0.0, 0.01)


@configclass
class SprintPlayEnvCfg(SprintEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 1
        # Keep preview commands deterministic so speed-specific videos test the
        # requested velocity instead of a random command or standing episode.
        self.commands.base_velocity.rel_standing_envs = 0.0
        lin_vel_x = _float_env("G1_PREVIEW_LIN_VEL_X", 3.0)
        lin_vel_y = _float_env("G1_PREVIEW_LIN_VEL_Y", 0.0)
        ang_vel_z = _float_env("G1_PREVIEW_ANG_VEL_Z", 0.0)
        self.commands.base_velocity.ranges.lin_vel_x = (lin_vel_x, lin_vel_x)
        self.commands.base_velocity.ranges.lin_vel_y = (lin_vel_y, lin_vel_y)
        self.commands.base_velocity.ranges.ang_vel_z = (ang_vel_z, ang_vel_z)
        self.viewer.origin_type = "asset_root"
        self.viewer.asset_name = "robot"
        self.viewer.eye = (-3.0, -4.0, 2.2)
        self.viewer.lookat = (0.0, 0.0, 0.6)


@configclass
class SprintRefineEnvCfg(SprintEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 8196

        # The first sprint run can stay upright, but the preview shows arm
        # flailing and yaw drift. This phase keeps the observation/action
        # contract unchanged and only tightens the objective around posture.
        self.commands.base_velocity.rel_standing_envs = 0.02
        self.commands.base_velocity.resampling_time_range = (6.0, 8.0)
        self.commands.base_velocity.ranges.lin_vel_x = (1.2, 3.2)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.05, 0.05)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.05, 0.05)
        self.commands.base_velocity.limit_ranges.lin_vel_x = (1.0, 4.5)
        self.commands.base_velocity.limit_ranges.lin_vel_y = (-0.10, 0.10)
        self.commands.base_velocity.limit_ranges.ang_vel_z = (-0.10, 0.10)

        self.rewards.track_lin_vel_xy.weight = 3.0
        self.rewards.track_ang_vel_z.weight = 0.9
        self.rewards.base_angular_velocity.weight = -0.12
        self.rewards.action_rate.weight = -0.12
        self.rewards.energy.weight = -3.0e-5
        self.rewards.flat_orientation_l2.weight = -9.0
        self.rewards.base_height.weight = -12.0
        self.rewards.feet_slide.weight = -0.4
        self.rewards.joint_deviation_arms.weight = -0.35
        self.rewards.joint_deviation_waists.weight = -0.8
        self.rewards.joint_deviation_legs.weight = -0.8


@configclass
class SprintRefinePlayEnvCfg(SprintRefineEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 1
        self.commands.base_velocity.rel_standing_envs = 0.0
        lin_vel_x = _float_env("G1_PREVIEW_LIN_VEL_X", 3.0)
        lin_vel_y = _float_env("G1_PREVIEW_LIN_VEL_Y", 0.0)
        ang_vel_z = _float_env("G1_PREVIEW_ANG_VEL_Z", 0.0)
        self.commands.base_velocity.ranges.lin_vel_x = (lin_vel_x, lin_vel_x)
        self.commands.base_velocity.ranges.lin_vel_y = (lin_vel_y, lin_vel_y)
        self.commands.base_velocity.ranges.ang_vel_z = (ang_vel_z, ang_vel_z)
        self.viewer.origin_type = "asset_root"
        self.viewer.asset_name = "robot"
        self.viewer.eye = (-3.0, -4.0, 2.2)
        self.viewer.lookat = (0.0, 0.0, 0.6)
