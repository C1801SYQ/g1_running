# Copyright 2026 Zachary Olkin. All rights reserved.

from typing import Tuple
import torch
import tensordict

def symmetric_data_augmentation_episodic(env, obs: tensordict.TensorDict, actions: torch.Tensor) -> Tuple[tensordict.TensorDict, torch.Tensor]:
    """
    Augment the data for the RSL RL data augmentation.

    obs: Tensor of shape [batch, num_obs]
    actions: Tensor of shape [batch, num_actions]
    env: RL vec env

    Flip the observation and actions.
    """

    # Can pull the remapping matrix R from the trajectory manager from the command from the env

    if obs is not None:
        device = obs.device

        batch_size = obs.batch_size[0]

        obs_aug = obs.repeat(2)

        cmd = env.unwrapped.command_manager.get_term("traj_ref")

        # Original observations
        obs_aug["policy"][:batch_size] = obs["policy"][:batch_size]

        for group in ["policy", "critic"]:
            obs_idx = 0
            for i, name in enumerate(env.unwrapped.observation_manager.active_terms[group]):
                obs_size = 0
                if name == "base_ang_vel":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]

                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        obs[group][:, obs_idx:obs_idx + obs_size] * torch.tensor([-1, 1, -1], device=device))
                elif name == "base_lin_vel":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]

                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        obs[group][:, obs_idx:obs_idx + obs_size] * torch.tensor([1, -1, 1], device=device)
                    )
                elif name == "projected_gravity":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]

                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        obs[group][:, obs_idx:obs_idx + obs_size] * torch.tensor([1, -1, 1], device=device)
                    )
                elif name == "velocity_commands":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]

                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        obs[group][:, obs_idx:obs_idx + obs_size] * torch.tensor([1, -1, -1], device=device)
                    )
                elif name == "joint_pos" or name == "joint_vel" or name == "actions":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]
                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        _switch_g1_joints(obs[group][:, obs_idx:obs_idx + obs_size])
                    )
                elif name == "sin_phase" or name == "cos_phase":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]
                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = obs[group][:, obs_idx:obs_idx + obs_size]
                elif name == "ref_traj" or name == "act_traj":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]
                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        cmd.get_symmetric_traj(obs[group][:, obs_idx:obs_idx + obs_size], "pos")
                    )
                elif name == "ref_traj_vel" or name == "act_traj_vel":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]
                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        cmd.get_symmetric_traj(obs[group][:, obs_idx:obs_idx + obs_size], "vel")
                    )
                elif name == "root_quat":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]
                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        obs[group][:, obs_idx:obs_idx + obs_size] * torch.tensor([1, -1, 1, -1], device=device)
                    )
                elif name == "contact_state":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]
                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        cmd.get_symmetric_contacts(obs[group][:, obs_idx:obs_idx + obs_size])
                    )

                # TODO: Add height map support

                obs_idx += obs_size
    else:
        obs_aug = None

    if actions is not None:
        batch_size = actions.shape[0]

        actions_aug = torch.zeros(batch_size * 2, actions.shape[1], device=actions.device)

        # Original actions
        actions_aug[:batch_size] = actions

        actions_aug[batch_size:] = _switch_g1_joints(actions)
    else:
        actions_aug = None

    return (obs_aug, actions_aug)

def symmetric_data_augmentation_half_periodic(env, obs: tensordict.TensorDict, actions: torch.Tensor) -> Tuple[tensordict.TensorDict, torch.Tensor]:
    """
    Augment the data for the RSL RL data augmentation.

    obs: Tensor of shape [batch, num_obs]
    actions: Tensor of shape [batch, num_actions]
    env: RL vec env

    Flip the observation and actions.
    """

    # Can pull the remapping matrix R from the trajectory manager from the command from the env

    if obs is not None:
        device = obs.device

        batch_size = obs.batch_size[0]

        obs_aug = obs.repeat(2)

        cmd = env.unwrapped.command_manager.get_term("traj_ref")

        # Original observations
        obs_aug["policy"][:batch_size] = obs["policy"][:batch_size]

        for group in ["policy", "critic"]:
            obs_idx = 0
            for i, name in enumerate(env.unwrapped.observation_manager.active_terms[group]):
                obs_size = 0
                if name == "base_ang_vel":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]

                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        obs[group][:, obs_idx:obs_idx + obs_size] * torch.tensor([-1, 1, -1], device=device))
                elif name == "base_lin_vel":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]

                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        obs[group][:, obs_idx:obs_idx + obs_size] * torch.tensor([1, -1, 1], device=device)
                    )
                elif name == "projected_gravity":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]

                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        obs[group][:, obs_idx:obs_idx + obs_size] * torch.tensor([1, -1, 1], device=device)
                    )
                elif name == "velocity_commands":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]

                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        obs[group][:, obs_idx:obs_idx + obs_size] * torch.tensor([1, -1, -1], device=device)
                    )
                elif name == "joint_pos" or name == "joint_vel" or name == "actions":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]
                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        _switch_g1_joints(obs[group][:, obs_idx:obs_idx + obs_size])
                    )
                elif name == "sin_phase" or name == "cos_phase":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]
                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = -1*obs[group][:, obs_idx:obs_idx + obs_size]
                elif name == "ref_traj" or name == "act_traj":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]
                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        cmd.get_symmetric_traj(obs[group][:, obs_idx:obs_idx + obs_size], "pos")
                    )
                elif name == "ref_traj_vel" or name == "act_traj_vel":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]
                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        cmd.get_symmetric_traj(obs[group][:, obs_idx:obs_idx + obs_size], "vel")
                    )
                elif name == "root_quat":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]
                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        obs[group][:, obs_idx:obs_idx + obs_size] * torch.tensor([1, -1, 1, -1], device=device)
                    )
                elif name == "contact_state":
                    obs_size = env.unwrapped.observation_manager.group_obs_term_dim[group][i][0]
                    obs_aug[group][batch_size:, obs_idx:obs_idx + obs_size] = (
                        cmd.get_symmetric_contacts(obs[group][:, obs_idx:obs_idx + obs_size])
                    )

                # TODO: Add height map support

                obs_idx += obs_size
    else:
        obs_aug = None

    if actions is not None:
        batch_size = actions.shape[0]

        actions_aug = torch.zeros(batch_size * 2, actions.shape[1], device=actions.device)

        # Original actions
        actions_aug[:batch_size] = actions

        actions_aug[batch_size:] = _switch_g1_joints(actions)
    else:
        actions_aug = None

    return (obs_aug, actions_aug)

def _switch_g1_joints(joints: torch.Tensor) -> torch.Tensor:
    """
    Reflection the joint values about the sagittal plane.

    IsaacSim 29-dof ordering:
    [
        0: left_hip_pitch, 1: right_hip_pitch, 2: waist_yaw,
        3: left_hip_roll, 4: right_hip_roll, 5: waist_roll,
        6: left_hip_yaw, 7: right_hip_yaw, 8: waist_pitch,
        9: left_knee, 10: right_knee,
        11: left_shoulder_pitch, 12: right_shoulder_pitch,
        13: left_ankle_pitch, 14: right_ankle_pitch,
        15: left_shoulder_roll, 16: right_shoulder_roll,
        17: left_ankle_roll, 18: right_ankle_roll,
        19: left_shoulder_yaw, 20: right_shoulder_yaw,
        21: left_elbow, 22: right_elbow,
        23: left_wrist_roll, 24: right_wrist_roll,
        25: left_wrist_pitch, 26: right_wrist_pitch,
        27: left_wrist_yaw, 28: right_wrist_yaw,
    ]

    Map all left -> right and right -> left.
    Negate all roll and yaw joints.
    """
    joints_switched = torch.zeros_like(joints)

    left_leg = [0, 3, 6, 9, 13, 17]     # hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
    right_leg = [1, 4, 7, 10, 14, 18]
    left_arm = [11, 15, 19, 21, 23, 25, 27]   # shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw
    right_arm = [12, 16, 20, 22, 24, 26, 28]
    waist_yaw = [2]
    waist_roll = [5]
    waist_pitch = [8]

    joints_switched[:, left_leg] = joints[:, right_leg]
    joints_switched[:, left_arm] = joints[:, right_arm]
    joints_switched[:, right_leg] = joints[:, left_leg]
    joints_switched[:, right_arm] = joints[:, left_arm]
    joints_switched[:, waist_yaw] = joints[:, waist_yaw]
    joints_switched[:, waist_roll] = joints[:, waist_roll]
    joints_switched[:, waist_pitch] = joints[:, waist_pitch]

    # Negate all roll and yaw joints (waist keeps sign)
    joints_switched[:, [2, 3, 4, 5, 6, 7, 15, 16, 19, 20, 23, 24, 27, 28]] *= -1.0

    return joints_switched