# Copyright 2026 Zachary Olkin. All rights reserved.

"""29-dof G1 100 m sprint environment config.

This config inherits ``g1_running_clf_29dof_env_cfg`` (the gait_v2 baseline) and
only overrides the command distribution, the sprint-oriented rewards, the
100 m track target, and the domain-randomization curriculum.  Observation and
action dimensions are identical to gait_v2, so ``running_sprint_29dof`` can be
resumed directly from the ``gait_v2`` checkpoint ``model_175197``.

Stages (selected via the ``stage`` class attribute):

* ``A`` (transition): nominal friction, no pushes, no mass/COM/gain/joint
  friction DR.  Fixes the 0 -> high-speed transition.  Run ~1000-2000 iters.
* ``B`` (speed): high-speed straight samples dominate; light friction/mass/COM/
  gain DR on a subset; push on a small subset; max command 5.5 m/s (reference
  still clamps at 5.1 m/s).
* ``C`` (robustness): full friction/mass/COM/gain DR on subsets, pushes on a
  subset, 25% nominal environments; max command 6.0 m/s (residual-extrapolation
  experiment only).

NOTE: the trajectory library only spans 1.1-5.1 m/s.  Commands above 5.1 m/s
clamp to the fastest reference gait and must be recovered by the CLF residual
action (documented blocker in the sprint deployment notes).
"""

from isaaclab.utils import configclass
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm

from robot_rl.tasks.manager_based.robot_rl import mdp
from robot_rl.tasks.manager_based.robot_rl.mdp.commands.velocity_commands_cfg import (
    SprintProfileCfg,
)
from robot_rl.tasks.manager_based.robot_rl.mdp.speed_gates import stage_dr_spec
from robot_rl.tasks.manager_based.robot_rl.humanoid_env_cfg import HumanoidTerminationCfg

from .g1_running_clf_29dof_env_cfg import (
    G1Running29dofGaitLibraryEnvCfg,
    G1Running29dofCommandsCfg,
    G1Running29dofRewardCfg,
)

# Default sprint curriculum stage: "A", "B", or "C".
SPRINT_STAGE = "A"

# Sprint reward weights.  Kept deliberately modest so they never overpower the
# velocity-tracking / CLF terms that determine top speed.
SPRINT_FORWARD_PROGRESS_WEIGHT = 1.5
SPRINT_LATERAL_ERROR_WEIGHT = 1.0
SPRINT_HEADING_ERROR_WEIGHT = 1.0
SPRINT_STABILITY_WEIGHT = 1.0
SPRINT_TRACK_PROGRESS_WEIGHT = 1.0
SPRINT_TRACK_COMPLETION_WEIGHT = 5.0  # sparse; only on the finish step

# Per-stage maximum commanded forward speed (m/s).
_SPRINT_MAX_VX = {"A": 5.1, "B": 5.5, "C": 6.0}

# Sprint track length (m) and episode horizon (s).  30 s gives a standing start
# enough time to reach 100 m at ~5 m/s (20 s) with margin for acceleration.
SPRINT_TRACK_LENGTH = 100.0
SPRINT_EPISODE_LENGTH_S = 30.0


@configclass
class G1RunningSprint29dofCommandsCfg(G1Running29dofCommandsCfg):
    """Commands for the sprint environment (inherits the gait library)."""
    pass


@configclass
class G1RunningSprint29dofRewardCfg(G1Running29dofRewardCfg):
    """Sprint-oriented rewards on top of the gait_v2 reward set."""

    # Reward forward progress projected onto the track direction, gated high.
    forward_progress = RewTerm(
        func=mdp.forward_progress_reward,
        weight=SPRINT_FORWARD_PROGRESS_WEIGHT,
        params={
            "command_name": "base_velocity",
            "v_scale": 5.0,
            "gate_low": 3.0,
            "gate_high": 4.0,
        },
    )

    # Penalize accumulated lateral offset from the track centerline.
    lateral_position_error = RewTerm(
        func=mdp.lateral_position_error_reward,
        weight=SPRINT_LATERAL_ERROR_WEIGHT,
        params={
            "command_name": "base_velocity",
            "std": 0.25,
            "gate_low": 3.0,
            "gate_high": 4.0,
            "use_sprint_marker": True,
        },
    )

    # Penalize heading error relative to the track direction.
    heading_error = RewTerm(
        func=mdp.heading_error_reward,
        weight=SPRINT_HEADING_ERROR_WEIGHT,
        params={
            "command_name": "base_velocity",
            "std": 0.25,
            "gate_low": 3.0,
            "gate_high": 4.0,
            "use_sprint_marker": True,
        },
    )

    # Survival-oriented upright/height reward at high speed.
    high_speed_stability = RewTerm(
        func=mdp.high_speed_stability_reward,
        weight=SPRINT_STABILITY_WEIGHT,
        params={
            "command_name": "base_velocity",
            "gate_low": 3.0,
            "gate_high": 4.0,
            "roll_std": 0.3,
            "pitch_std": 0.3,
            "height_min": 0.45,
        },
    )

    # Dense track-progress reward (does not saturate).
    track_progress = RewTerm(
        func=mdp.track_progress_reward,
        weight=SPRINT_TRACK_PROGRESS_WEIGHT,
        params={
            "command_name": "base_velocity",
            "track_length": SPRINT_TRACK_LENGTH,
            "speed_min": 0.5,
        },
    )

    # Sparse 100 m completion reward.
    track_completion = RewTerm(
        func=mdp.track_completion_reward,
        weight=SPRINT_TRACK_COMPLETION_WEIGHT,
        params={
            "command_name": "base_velocity",
            "track_length": SPRINT_TRACK_LENGTH,
        },
    )


@configclass
class G1RunningSprint29dofTerminationCfg(HumanoidTerminationCfg):
    """Sprint terminations: inherit fall/time-out, add 100 m completion."""

    track_completion = DoneTerm(
        func=mdp.track_completion,
        time_out=False,
        params={"command_name": "base_velocity", "track_length": SPRINT_TRACK_LENGTH},
    )


@configclass
class G1RunningSprint29dofGaitLibraryEnvCfg(G1Running29dofGaitLibraryEnvCfg):
    """Configuration for the G1 29-dof 100 m sprint environment."""

    commands: G1RunningSprint29dofCommandsCfg = G1RunningSprint29dofCommandsCfg()
    rewards: G1RunningSprint29dofRewardCfg = G1RunningSprint29dofRewardCfg()
    terminations: G1RunningSprint29dofTerminationCfg = G1RunningSprint29dofTerminationCfg()

    # Curriculum stage (plain class attribute; subclasses override it).
    stage: str = SPRINT_STAGE

    def __post_init__(self):
        super().__post_init__()

        max_vx = _SPRINT_MAX_VX.get(self.stage, 5.1)

        # --- Command distribution oriented toward a 100 m sprint --------------
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, max_vx)
        # Tight lateral/yaw/heading ranges: sprint envs run straight.
        self.commands.base_velocity.ranges.lin_vel_y = (-0.30, 0.30)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.50, 0.50)
        self.commands.base_velocity.ranges.heading = (-0.30, 0.30)
        self.commands.base_velocity.ranges.y_pos_offset = (-0.15, 0.15)

        # Sprint profile: 10% zero-stand, 10% low-speed, no static transition,
        # 20% acceleration, 50% sprint, 10% correction.
        accel_targets = (1.0, 3.0, 5.1) if max_vx <= 5.1 else (1.0, 3.0, 5.1, max_vx)
        self.commands.base_velocity.sprint_profile = SprintProfileCfg(
            zero_stand_prob=0.10,
            low_speed_prob=0.10,
            transition_prob=0.0,
            acceleration_prob=0.20,
            sprint_prob=0.50,
            correction_prob=0.10,
            sprint_vx_range=(4.5, max_vx),
            correction_vx_range=(4.5, max_vx),
            low_speed_vx_range=(0.3, 1.2),
            sprint_lat_range=(-0.05, 0.05),
            sprint_yaw_range=(-0.05, 0.05),
            low_speed_lat_range=(-0.05, 0.05),
            low_speed_yaw_range=(-0.05, 0.05),
            accel_targets=accel_targets,
            accel_ramp_duration_range=(2.0, 6.0),
        )
        # Sprint profile fully owns x-velocity sampling.
        self.commands.base_velocity.lin_vel_x_segments = None
        self.commands.base_velocity.lin_vel_x_segment_weights = None

        # --- Allow natural arm swing at high speed ----------------------------
        self.rewards.upper_body_stability.params["command_name"] = "base_velocity"
        self.rewards.upper_body_stability.params["relax_low"] = 3.0
        self.rewards.upper_body_stability.params["relax_high"] = 5.0

        # --- 100 m track horizon ----------------------------------------------
        self.episode_length_s = SPRINT_EPISODE_LENGTH_S

        # --- Stage-specific domain randomization -----------------------------
        self._apply_stage_domain_randomization()

    def _apply_stage_domain_randomization(self):
        """Apply the DR curriculum for the selected stage from ``stage_dr_spec``."""
        spec = stage_dr_spec(self.stage)
        nominal_rel = spec["nominal_rel_envs"]

        # Shared, persistent DR group assignment.  Uses "startup" mode (NOT
        # "prestartup", which EventManager rejects when replicate_physics=True).
        # The events dict is reordered below so this term runs before every
        # other event; nominal environments are never randomized.
        self.events.assign_dr_groups = EventTerm(
            func=mdp.assign_dr_groups,
            mode="startup",
            params={"nominal_rel_envs": nominal_rel, "seed": 0},
        )

        # Ground-contact friction.  Stage A keeps nominal; B/C randomize all
        # NON-nominal environments (rel_envs=1.0 means all non-nominal).
        friction_range = spec["friction_range"]
        if friction_range is None:
            self.events.randomize_ground_contact_friction = None
        else:
            self.events.randomize_ground_contact_friction.func = mdp.subset_randomize_rigid_body_material
            self.events.randomize_ground_contact_friction.params = {
                "asset_cfg": SceneEntityCfg("robot", body_names=[".*_ankle_roll_link"]),
                "static_friction_range": friction_range,
                "dynamic_friction_range": (
                    max(friction_range[0] * 0.8, 0.0),
                    friction_range[1],
                ),
                "restitution_range": (0.0, 0.2),
                "num_buckets": 64,
                "make_consistent": True,
                "rel_envs": 1.0,
            }
            self.events.randomize_ground_contact_friction.mode = "reset"

        # Joint friction / armature.  In Stage C these become subset events so
        # the nominal group keeps its default joint friction/armature.
        if spec["joint_friction"]:
            self.events.joint_friction_params.func = mdp.subset_randomize_joint_parameters_multi_friction
            self.events.joint_friction_params.params = {
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
                "static_friction_distribution_params": (0.3, 1.6),
                "dynamic_friction_distribution_params": (0.3, 1.2),
                "viscous_friction_distribution_params": (0.01, 0.1),
                "operation": "add",
                "rel_envs": 1.0,
            }
            self.events.joint_armature_params.func = mdp.subset_randomize_joint_parameters_multi_friction
            self.events.joint_armature_params.params = {
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
                "armature_distribution_params": (0.95, 1.05),
                "operation": "scale",
                "rel_envs": 1.0,
            }
        else:
            self.events.joint_friction_params = None
            self.events.joint_armature_params = None

        # Base mass DR (non-nominal only).
        if spec["base_mass_range"] is None:
            self.events.add_base_mass = None
        else:
            self.events.add_base_mass.func = mdp.subset_randomize_rigid_body_mass
            self.events.add_base_mass.params = {
                "asset_cfg": SceneEntityCfg("robot", body_names=["waist_yaw_link", "pelvis_link"]),
                "mass_distribution_params": spec["base_mass_range"],
                "operation": "scale",
                "rel_envs": 1.0,
            }

        # COM DR (non-nominal only).
        if spec["com_range"] is None:
            self.events.base_com = None
        else:
            self.events.base_com.func = mdp.subset_randomize_rigid_body_com
            self.events.base_com.params = {
                "asset_cfg": SceneEntityCfg("robot", body_names=["waist_yaw_link", "pelvis_link"]),
                "com_range": spec["com_range"],
                "rel_envs": 1.0,
            }

        # Actuator gain DR (non-nominal only).
        if spec["gain_range"] is None:
            self.events.gain_randomization = None
        else:
            self.events.gain_randomization.func = mdp.subset_randomize_actuator_gains
            self.events.gain_randomization.params = {
                "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
                "stiffness_distribution_params": spec["gain_range"],
                "damping_distribution_params": spec["gain_range"],
                "operation": "scale",
                "distribution": "uniform",
                "rel_envs": 1.0,
            }

        # External force stays disabled in every stage (the base config already
        # sets base_external_force_torque = None).

        # Push: a sub-fraction of the NON-nominal environments.
        push_rel = spec["push_rel_envs"]
        if push_rel <= 0.0:
            self.events.push_robot = None
        else:
            self.events.push_robot.func = mdp.subset_push_by_setting_velocity
            self.events.push_robot.params = {
                "velocity_range": spec["push_velocity_range"],
                "rel_envs": push_rel,
            }

        # Guarantee the DR group mask exists before ANY subset DR event runs:
        # move ``assign_dr_groups`` to the front of the events dict (IsaacLab
        # applies events in config field order).  This is compatible with
        # ``replicate_physics=True`` (no ``prestartup`` term).
        event_items = list(self.events.__dict__.items())
        event_items.sort(key=lambda item: (item[0] != "assign_dr_groups",))
        self.events.__dict__.clear()
        self.events.__dict__.update(event_items)


@configclass
class G1RunningSprint29dofStageBGaitLibraryEnvCfg(G1RunningSprint29dofGaitLibraryEnvCfg):
    """Stage B: speed.  Max command 5.5 m/s, light subset randomization."""

    stage: str = "B"


@configclass
class G1RunningSprint29dofStageCGaitLibraryEnvCfg(G1RunningSprint29dofGaitLibraryEnvCfg):
    """Stage C: robustness.  Max command 6.0 m/s, full subset randomization."""

    stage: str = "C"
