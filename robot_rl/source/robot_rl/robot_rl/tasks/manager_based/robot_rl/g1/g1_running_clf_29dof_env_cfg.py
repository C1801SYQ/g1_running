# Copyright 2026 Zachary Olkin. All rights reserved.

"""29-dof G1 running CLF environment config."""

import torch
from isaaclab.utils import configclass
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import RewardTermCfg as RewTerm

from robot_rl.tasks.manager_based.robot_rl.humanoid_env_cfg import HumanoidEnvCfg, HumanoidCommandsCfg, HumanoidEventsCfg
from robot_rl.tasks.manager_based.robot_rl import mdp
from robot_rl.tasks.manager_based.robot_rl.mdp.commands.traj_tracking.trajectory_cmd_cfg import TrajectoryCommandCfg
from robot_rl.tasks.manager_based.robot_rl.mdp.commands.velocity_commands_cfg import VelocityTrackingCommandCfg
from .g1_trajopt_reward import G1TrajOptCLFRewards
from .g1_trajopt_obs import G1TrajOptObservationsCfg
from robot_rl.assets.robots.g1_29dof import G1_29DOF_MINIMAL_CFG, G1_29DOF_ACTION_SCALE
from .g1_running_clf_env_cfg import heuristic_modification
from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG

import math

REWARD_TYPE = "CLF"
TRACKING_REW_TYPE = "GOAL_ADJ"  # GOAL_ADJ adds xy_vel + yaw_vel rewards for direct velocity tracking
TRAJECTORY_TYPE = "DYNAMIC"
SPEED_RANGE = "ALL"

# Lyapunov weights for running (29-dof)
RUNNING_Q_weights = {}

RUNNING_Q_weights["left_ankle_roll_link:pos_x"] = [1.0, 1.0]
RUNNING_Q_weights["left_ankle_roll_link:pos_y"] = [1.0, 1.0]
RUNNING_Q_weights["left_ankle_roll_link:pos_z"] = [1.0, 1.0]
RUNNING_Q_weights["left_ankle_roll_link:ori_x"] = [1.0, 1.0]
RUNNING_Q_weights["left_ankle_roll_link:ori_y"] = [1.0, 1.0]
RUNNING_Q_weights["left_ankle_roll_link:ori_z"] = [1.0, 1.0]

RUNNING_Q_weights["right_ankle_roll_link:pos_x"] = [1.0, 1.0]
RUNNING_Q_weights["right_ankle_roll_link:pos_y"] = [1.0, 1.0]
RUNNING_Q_weights["right_ankle_roll_link:pos_z"] = [1.0, 1.0]
RUNNING_Q_weights["right_ankle_roll_link:ori_x"] = [1.0, 1.0]
RUNNING_Q_weights["right_ankle_roll_link:ori_y"] = [1.0, 1.0]
RUNNING_Q_weights["right_ankle_roll_link:ori_z"] = [1.0, 1.0]

if REWARD_TYPE == "CLF":
    RUNNING_Q_weights["joint:left_hip_roll_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:left_hip_pitch_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:left_hip_yaw_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:left_knee_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:left_ankle_roll_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:left_ankle_pitch_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:right_hip_roll_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:right_hip_pitch_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:right_hip_yaw_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:right_knee_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:right_ankle_roll_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:right_ankle_pitch_joint"] = [3.0, 0.1]

    RUNNING_Q_weights["joint:waist_yaw_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:waist_roll_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:waist_pitch_joint"] = [3.0, 0.1]
    RUNNING_Q_weights["joint:left_elbow_joint"] = [20.0, 0.1]
    RUNNING_Q_weights["joint:left_shoulder_pitch_joint"] = [20.0, 0.1]
    RUNNING_Q_weights["joint:left_shoulder_roll_joint"] = [20.0, 0.1]
    RUNNING_Q_weights["joint:left_shoulder_yaw_joint"] = [20.0, 0.1]
    RUNNING_Q_weights["joint:right_elbow_joint"] = [20.0, 0.1]
    RUNNING_Q_weights["joint:right_shoulder_pitch_joint"] = [20.0, 0.1]
    RUNNING_Q_weights["joint:right_shoulder_roll_joint"] = [20.0, 0.1]
    RUNNING_Q_weights["joint:right_shoulder_yaw_joint"] = [20.0, 0.1]

    # Wrist joints (low weight - don't need to track them closely)
    RUNNING_Q_weights["joint:left_wrist_roll_joint"] = [1.0, 0.1]
    RUNNING_Q_weights["joint:left_wrist_pitch_joint"] = [1.0, 0.1]
    RUNNING_Q_weights["joint:left_wrist_yaw_joint"] = [1.0, 0.1]
    RUNNING_Q_weights["joint:right_wrist_roll_joint"] = [1.0, 0.1]
    RUNNING_Q_weights["joint:right_wrist_pitch_joint"] = [1.0, 0.1]
    RUNNING_Q_weights["joint:right_wrist_yaw_joint"] = [1.0, 0.1]

    # Pelvis tracking: high pos_x/pos_y/ori_z weights for heading/navigation correction
    RUNNING_Q_weights["pelvis_link:pos_x"] = [1.0, 15.0]
    RUNNING_Q_weights["pelvis_link:pos_y"] = [1.0, 15.0]
    RUNNING_Q_weights["pelvis_link:pos_z"] = [1.0, 1.0]
    RUNNING_Q_weights["pelvis_link:ori_x"] = [1.0, 1.0]
    RUNNING_Q_weights["pelvis_link:ori_y"] = [1.0, 1.0]
    RUNNING_Q_weights["pelvis_link:ori_z"] = [1.0, 15.0]

RUNNING_Q_weights["right_wrist_yaw_link:pos_x"] = [1.0, 1.0]
RUNNING_Q_weights["right_wrist_yaw_link:pos_y"] = [1.0, 1.0]
RUNNING_Q_weights["right_wrist_yaw_link:pos_z"] = [1.0, 1.0]
RUNNING_Q_weights["right_wrist_yaw_link:ori_x"] = [1.0, 1.0]
RUNNING_Q_weights["right_wrist_yaw_link:ori_y"] = [1.0, 1.0]
RUNNING_Q_weights["right_wrist_yaw_link:ori_z"] = [1.0, 1.0]

RUNNING_Q_weights["left_wrist_yaw_link:pos_x"] = [1.0, 1.0]
RUNNING_Q_weights["left_wrist_yaw_link:pos_y"] = [1.0, 1.0]
RUNNING_Q_weights["left_wrist_yaw_link:pos_z"] = [1.0, 1.0]
RUNNING_Q_weights["left_wrist_yaw_link:ori_x"] = [1.0, 1.0]
RUNNING_Q_weights["left_wrist_yaw_link:ori_y"] = [1.0, 1.0]
RUNNING_Q_weights["left_wrist_yaw_link:ori_z"] = [1.0, 1.0]

RUNNING_R_weights = {}

RUNNING_R_weights["left_ankle_roll_link:pos_x"] = [0.05]
RUNNING_R_weights["left_ankle_roll_link:pos_y"] = [0.05]
RUNNING_R_weights["left_ankle_roll_link:pos_z"] = [0.05]
RUNNING_R_weights["left_ankle_roll_link:ori_x"] = [0.05]
RUNNING_R_weights["left_ankle_roll_link:ori_y"] = [0.05]
RUNNING_R_weights["left_ankle_roll_link:ori_z"] = [0.05]

RUNNING_R_weights["right_ankle_roll_link:pos_x"] = [0.05]
RUNNING_R_weights["right_ankle_roll_link:pos_y"] = [0.05]
RUNNING_R_weights["right_ankle_roll_link:pos_z"] = [0.05]
RUNNING_R_weights["right_ankle_roll_link:ori_x"] = [0.05]
RUNNING_R_weights["right_ankle_roll_link:ori_y"] = [0.05]
RUNNING_R_weights["right_ankle_roll_link:ori_z"] = [0.05]

RUNNING_R_weights["joint:left_hip_roll_joint"] = [0.05]
RUNNING_R_weights["joint:left_hip_pitch_joint"] = [0.05]
RUNNING_R_weights["joint:left_hip_yaw_joint"] = [0.05]
RUNNING_R_weights["joint:left_knee_joint"] = [0.05]
RUNNING_R_weights["joint:left_ankle_roll_joint"] = [0.05]
RUNNING_R_weights["joint:left_ankle_pitch_joint"] = [0.05]
RUNNING_R_weights["joint:right_hip_roll_joint"] = [0.05]
RUNNING_R_weights["joint:right_hip_pitch_joint"] = [0.05]
RUNNING_R_weights["joint:right_hip_yaw_joint"] = [0.05]
RUNNING_R_weights["joint:right_knee_joint"] = [0.05]
RUNNING_R_weights["joint:right_ankle_roll_joint"] = [0.05]
RUNNING_R_weights["joint:right_ankle_pitch_joint"] = [0.05]

if REWARD_TYPE == "CLF":
    # Higher R → less aggressive control. Keep pos_x/pos_y/ori_z at moderate level
    # Q/R ratio for pelvis x/y/yaw = 15/1 = 15 for strong tracking
    RUNNING_R_weights["pelvis_link:pos_x"] = [1.0]
    RUNNING_R_weights["pelvis_link:pos_y"] = [1.0]
    RUNNING_R_weights["pelvis_link:pos_z"] = [0.05]
    RUNNING_R_weights["pelvis_link:ori_x"] = [0.05]
    RUNNING_R_weights["pelvis_link:ori_y"] = [0.05]
    RUNNING_R_weights["pelvis_link:ori_z"] = [1.0]
else:
    RUNNING_R_weights["pelvis_link:pos_x"] = [0.05]
    RUNNING_R_weights["pelvis_link:pos_y"] = [0.05]
    RUNNING_R_weights["pelvis_link:pos_z"] = [0.05]
    RUNNING_R_weights["pelvis_link:ori_x"] = [0.05]
    RUNNING_R_weights["pelvis_link:ori_y"] = [0.05]
    RUNNING_R_weights["pelvis_link:ori_z"] = [0.05]

RUNNING_R_weights["joint:waist_yaw_joint"] = [0.05]
RUNNING_R_weights["joint:left_elbow_joint"] = [0.05]
RUNNING_R_weights["joint:left_shoulder_pitch_joint"] = [0.05]
RUNNING_R_weights["joint:left_shoulder_roll_joint"] = [0.05]
RUNNING_R_weights["joint:left_shoulder_yaw_joint"] = [0.05]
RUNNING_R_weights["joint:right_elbow_joint"] = [0.05]
RUNNING_R_weights["joint:right_shoulder_pitch_joint"] = [0.05]
RUNNING_R_weights["joint:right_shoulder_roll_joint"] = [0.05]
RUNNING_R_weights["joint:right_shoulder_yaw_joint"] = [0.05]

RUNNING_R_weights["right_wrist_yaw_link:pos_x"] = [0.05]
RUNNING_R_weights["right_wrist_yaw_link:pos_y"] = [0.05]
RUNNING_R_weights["right_wrist_yaw_link:pos_z"] = [0.05]
RUNNING_R_weights["right_wrist_yaw_link:ori_x"] = [0.05]
RUNNING_R_weights["right_wrist_yaw_link:ori_y"] = [0.05]
RUNNING_R_weights["right_wrist_yaw_link:ori_z"] = [0.05]

RUNNING_R_weights["left_wrist_yaw_link:pos_x"] = [0.05]
RUNNING_R_weights["left_wrist_yaw_link:pos_y"] = [0.05]
RUNNING_R_weights["left_wrist_yaw_link:pos_z"] = [0.05]
RUNNING_R_weights["left_wrist_yaw_link:ori_x"] = [0.05]
RUNNING_R_weights["left_wrist_yaw_link:ori_y"] = [0.05]
RUNNING_R_weights["left_wrist_yaw_link:ori_z"] = [0.05]

traj_path = "trajectories/running"


@configclass
class G1Running29dofCommandsCfg(HumanoidCommandsCfg):
    """Configuration for gait library commands (29-dof)."""
    traj_ref = TrajectoryCommandCfg(
        contact_bodies=[".*_ankle_roll_link"],
        manager_type="library",
        hf_repo="zolkin/robot_rl",
        path=traj_path,
        conditioner_generator_name="base_velocity",
        num_outputs=48,
        Q_weights=RUNNING_Q_weights,
        R_weights=RUNNING_R_weights,
        hold_phi_threshold=0.1,
        heuristic_func=heuristic_modification,
        phasing_boundaries=4,
    )

    base_velocity = VelocityTrackingCommandCfg(
        asset_name="robot",
        resampling_time_range=(7.0, 10.0),
        rel_standing_envs=0.0,
        rel_closed_loop=0.40,
        rel_closed_loop_yaw=0.30,
        rel_open_loop=0.30,
        heading_command=True,       # Enable heading commands
        rel_heading_envs=0.2,       # 20% envs practice turning/heading
        debug_vis=False,
        ranges=VelocityTrackingCommandCfg.VelRanges(
            lin_vel_x=(-1.0, 1.0),
            lin_vel_y=(-1.0, 1.0),
            ang_vel_z=(-1.0, 1.0),
            heading=(-math.pi, math.pi),
            y_pos_offset=(-0.5, 0.5),
            y_kp=(1.2, 1.8),
            y_kd=(0.2, 0.4),
        ),
    )


@configclass
class G1Running29dofObservationCfg(G1TrajOptObservationsCfg):
    """Configuration for running gait library observations (29-dof)."""
    pass


if TRACKING_REW_TYPE == "GOAL_ADJ" or TRACKING_REW_TYPE == "GOAL":
    CLF_WEIGHT = 1.0       # Minimal CLF - let speed tracking dominate
else:
    CLF_WEIGHT = 2.0
EXTRA_JOINT_POS_WEIGHT = 5.0  # Penalize deviation from default for joints without trajectory refs


@configclass
class G1Running29dofRewardCfg(G1TrajOptCLFRewards):
    torque_lims = RewTerm(func=mdp.torque_limits, weight=-1.0)

    if REWARD_TYPE == "CLF":
        clf_reward = RewTerm(func=mdp.clf_reward, weight=CLF_WEIGHT,
                             params={"command_name": "traj_ref", "max_eta_err": 12.0})
        clf_decreasing_condition = RewTerm(func=mdp.clf_decreasing_condition, weight=-5.0,
                                           params={"command_name": "traj_ref", "alpha": 0.5,
                                                   "eta_max": 12, "eta_dot_max": 24})
    else:
        clf_reward = None
        clf_decreasing_condition = None

    # Penalize extra joints (not in trajectory) from drifting away from default positions
    extra_joint_pos = RewTerm(
        func=mdp.joint_pos_default_reward,
        weight=-EXTRA_JOINT_POS_WEIGHT,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[
                "waist_roll_joint", "waist_pitch_joint",
                "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
                "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
            ]),
            "std": 0.3,
        },
    )

    if TRACKING_REW_TYPE == "GOAL" or TRACKING_REW_TYPE == "GOAL_ADJ":
        xy_vel = RewTerm(func=mdp.track_lin_vel_xy_exp, weight=10.0,
                         params={"command_name": "base_velocity", "std": 0.5})
        yaw_vel = RewTerm(func=mdp.track_ang_vel_z_exp, weight=8.0,
                          params={"command_name": "base_velocity", "std": 0.5})


@configclass
class G1Running29dofEventsCfg(HumanoidEventsCfg):
    """Events for 29-dof running."""

    reset_on_ref = EventTerm(func=mdp.reset_on_reference, mode="reset",
                             params={"command_name": "traj_ref",
                                     "base_frame_name": "pelvis_link",
                                     "conditioner_command_name": "base_velocity",
                                     "rel_envs_on_ref": 1.0})

    joint_friction_params = EventTerm(
        func=mdp.randomize_joint_parameters_multi_friction, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
                "static_friction_distribution_params": (0.3, 1.6),
                "dynamic_friction_distribution_params": (0.3, 1.2),
                "viscous_friction_distribution_params": (0.01, 0.1),
                "operation": "add"},
    )

    joint_armature_params = EventTerm(
        func=mdp.randomize_joint_parameters_multi_friction, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
                "armature_distribution_params": (0.95, 1.05), "operation": "scale"},
    )

    gain_randomization = EventTerm(
        func=mdp.randomize_actuator_gains, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
                "stiffness_distribution_params": (0.9, 1.1),
                "damping_distribution_params": (0.9, 1.1),
                "operation": "scale", "distribution": "uniform"},
    )

    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names="waist_yaw_link"),
                "mass_distribution_params": (0.85, 1.15), "operation": "scale"},
    )

    reset_base = None
    reset_robot_joints = None


@configclass
class G1Running29dofGaitLibraryEnvCfg(HumanoidEnvCfg):
    """Configuration for the G1 29-dof running gait library environment."""
    commands: G1Running29dofCommandsCfg = G1Running29dofCommandsCfg()
    observations: G1Running29dofObservationCfg = G1Running29dofObservationCfg()
    rewards: G1Running29dofRewardCfg = G1Running29dofRewardCfg()
    events: G1Running29dofEventsCfg = G1Running29dofEventsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.actions.joint_pos.scale = G1_29DOF_ACTION_SCALE

        # Scene
        self.scene.robot = G1_29DOF_MINIMAL_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        if SPEED_RANGE == "ALL":
            # Speed range: 0 = standing, running up to 5.1 m/s
            self.commands.base_velocity.ranges.lin_vel_x = (0.0, 5.1)
            self.commands.base_velocity.ranges.lin_vel_y = (-0.75, 0.75)
            self.commands.base_velocity.ranges.ang_vel_z = (-1.5, 1.5)    # Turning enabled
            self.commands.base_velocity.ranges.heading = (-3.14, 3.14)     # Full heading range
        else:
            self.commands.base_velocity.ranges.lin_vel_x = (3.6, 3.6)
            self.commands.base_velocity.ranges.lin_vel_y = (0, 0)
            self.commands.base_velocity.ranges.ang_vel_z = (0, 0)

        # Small heading variation for yaw robustness (was (0,0))
        self.commands.base_velocity.ranges.heading = (-0.3, 0.3)

        if self.rewards.holonomic_constraint is not None:
            self.rewards.holonomic_constraint.params["command_name"] = "traj_ref"
            self.rewards.holonomic_constraint_vel.params["command_name"] = "traj_ref"

        self.rewards.dof_acc_l2 = None
        self.rewards.dof_vel_l2 = None

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.scene.height_scanner = None
        self.curriculum.terrain_levels = None

        self.rewards.dof_torques_l2.weight = -1.0e-5

        # Domain randomization
        self.events.base_external_force_torque = None
        self.events.randomize_ground_contact_friction.params['restitution_range'] = (0.0, 0.2)
        self.events.push_robot.params['velocity_range'] = {"x": (-0.75, 0.75), "y": (-0.75, 0.75)}
        self.events.base_com.params['asset_cfg'] = SceneEntityCfg("robot", body_names="waist_yaw_link")

        self.episode_length_s = 20.0


@configclass
class G1Running29dofGaitLibraryEnvCfgPlay(G1Running29dofGaitLibraryEnvCfg):
    """Configuration for the G1 29-dof running gait library play environment."""

    def __post_init__(self):
        super().__post_init__()

        # Single speed for play testing
        self.commands.base_velocity.ranges.lin_vel_x = (3.0, 3.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        self.commands.base_velocity.ranges.resampling_time_range = (10.0, 10.0)

        self.scene.num_envs = 2
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False

        # Keep light randomization for sim2real robustness
        self.events.randomize_ground_contact_friction = None
        self.events.add_base_mass = None
        self.events.base_com = None
        self.events.base_external_force_torque = None
        self.events.push_robot = None
        self.events.gain_randomization = None
        self.events.joint_friction_params = None
        self.events.joint_armature_params = None

        # But keep light joint noise for robustness
        self.observations.policy.enable_corruption = True
