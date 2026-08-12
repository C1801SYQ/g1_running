"""Headless regression checks for G1 stand/walk/run transitions.

The command schedule matches deployment: stand at zero, ramp to 1 m/s, then
ramp to 5.1 m/s.  The report focuses on upper-body shake, inward hip yaw,
base attitude, lateral drift, and the largest target-position step.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation


SIM_ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = SIM_ROOT.parents[2]
sys.path.insert(0, str(SIM_ROOT))

from rl_policy import RLPolicy  # noqa: E402
from robot import Robot  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--policy",
        type=Path,
        default=REPOSITORY_ROOT / "rl_sar/policy/g1/running/policy.pt",
    )
    parser.add_argument(
        "--parameters",
        type=Path,
        default=(
            REPOSITORY_ROOT
            / "rl_sar/policy/g1/running/policy_parameters.yaml"
        ),
    )
    parser.add_argument("--duration", type=float, default=14.0)
    parser.add_argument("--acceleration", type=float, default=3.0)
    parser.add_argument(
        "--max-target-step",
        type=float,
        default=0.0,
        help="Optional experimental per-policy-frame joint target limit.",
    )
    parser.add_argument(
        "--sim-assets-root",
        type=Path,
        default=SIM_ROOT,
        help="Directory containing robots/g1 and its mesh assets.",
    )
    return parser.parse_args()


def scheduled_speed(time_s: float, acceleration: float) -> float:
    if time_s < 3.0:
        return 0.0
    slow_speed = min(1.0, acceleration * (time_s - 3.0))
    if time_s < 7.0:
        return slow_speed
    return min(5.1, 1.0 + acceleration * (time_s - 7.0))


def quaternion_rpy(wxyz: np.ndarray) -> np.ndarray:
    xyzw = np.array([wxyz[1], wxyz[2], wxyz[3], wxyz[0]])
    return Rotation.from_quat(xyzw).as_euler("xyz")


def rms(values: list[float]) -> float:
    return float(np.sqrt(np.mean(np.square(values)))) if values else 0.0


def main() -> None:
    args = parse_args()
    policy = RLPolicy(str(args.parameters.resolve()), str(args.policy.resolve()))
    policy.policy_params.update(
        v_x_min=0.0,
        v_x_max=5.1,
        v_y_min=-0.75,
        v_y_max=0.75,
        w_z_min=-1.5,
        w_z_max=1.5,
    )
    policy.load()

    previous_cwd = Path.cwd()
    try:
        # Robot resolves its XML relative to the current working directory.
        os.chdir(args.sim_assets_root.expanduser().resolve())
        robot = Robot(
            "g1_21j",
            "29dof_basic_scene",
            np.ones(3),
            None,
            False,
            {"kp_y": 1.5, "kd_y": 0.3, "kp_yaw": 0.8, "kd_yaw": 0.3},
        )
    finally:
        os.chdir(previous_cwd)

    robot.set_pd_gains_from_policy(policy)
    robot.input_function = lambda time_s: np.array(
        [scheduled_speed(time_s, args.acceleration), 0.0, 0.0]
    )
    steps_per_action = int(policy.get_dt() / robot.mj_model.opt.timestep)
    joint_index = {name: index for index, name in enumerate(robot.joint_names)}
    arm_indices = [
        index
        for name, index in joint_index.items()
        if "shoulder" in name or "elbow" in name
    ]
    hip_yaw_indices = [
        joint_index["left_hip_yaw_joint"],
        joint_index["right_hip_yaw_joint"],
    ]

    zero_arm_velocity: list[float] = []
    slow_hip_yaw: list[float] = []
    slow_inward_yaw: list[float] = []
    transition_target_steps: list[float] = []
    roll_pitch: list[float] = []
    local_forward_speed: list[float] = []
    max_abs_y = 0.0
    min_pelvis_height = float("inf")
    previous_target: np.ndarray | None = None
    max_target_step = 0.0
    max_target_step_time = 0.0
    max_target_step_joint = ""
    max_target_step_command = 0.0
    max_applied_target_step = 0.0

    while robot.mj_data.time < args.duration:
        time_s = float(robot.mj_data.time)
        command_speed = scheduled_speed(time_s, args.acceleration)
        observation = robot.create_observation(policy)
        raw_target = policy.get_action(observation, robot.joint_names)
        target = raw_target
        if previous_target is not None and args.max_target_step > 0.0:
            target = np.clip(
                raw_target,
                previous_target - args.max_target_step,
                previous_target + args.max_target_step,
            )
        robot.apply_action(target)

        joint_velocity = robot.mj_data.qvel[6:]
        joint_position = robot.mj_data.qpos[7:]
        if time_s >= 1.0 and command_speed < 0.05:
            zero_arm_velocity.extend(np.abs(joint_velocity[arm_indices]))
        if 0.8 <= command_speed <= 1.2:
            left_yaw, right_yaw = joint_position[hip_yaw_indices]
            slow_hip_yaw.extend([abs(left_yaw), abs(right_yaw)])
            slow_inward_yaw.append(abs(right_yaw - left_yaw))
        if previous_target is not None:
            raw_target_delta = np.abs(raw_target - previous_target)
            target_delta = np.abs(target - previous_target)
            step = float(np.max(raw_target_delta))
            transition_target_steps.append(step)
            max_applied_target_step = max(
                max_applied_target_step, float(np.max(target_delta))
            )
            if step > max_target_step:
                max_target_step = step
                max_target_step_time = time_s
                max_target_step_joint = robot.joint_names[int(np.argmax(target_delta))]
                max_target_step_command = command_speed
        previous_target = target.copy()

        rpy = quaternion_rpy(robot.mj_data.qpos[3:7])
        roll_pitch.append(float(np.linalg.norm(rpy[:2])))
        local_forward_speed.append(float(robot.get_local_vel()[0]))
        max_abs_y = max(max_abs_y, abs(float(robot.mj_data.qpos[1])))
        min_pelvis_height = min(min_pelvis_height, float(robot.mj_data.qpos[2]))

        for _ in range(steps_per_action):
            robot.step()

    report = {
        "duration_s": args.duration,
        "fell": min_pelvis_height < 0.45,
        "min_pelvis_height_m": min_pelvis_height,
        "max_abs_lateral_m": max_abs_y,
        "max_roll_pitch_deg": float(np.degrees(max(roll_pitch))),
        "zero_arm_velocity_rms_rad_s": rms(zero_arm_velocity),
        "slow_hip_yaw_rms_rad": rms(slow_hip_yaw),
        "slow_inward_hip_yaw_rms_rad": rms(slow_inward_yaw),
        "max_policy_target_step_rad": max(transition_target_steps, default=0.0),
        "max_applied_target_step_rad": max_applied_target_step,
        "max_policy_target_step_time_s": max_target_step_time,
        "max_policy_target_step_joint": max_target_step_joint,
        "max_policy_target_step_command_mps": max_target_step_command,
        "final_forward_speed_mps": float(
            np.mean(local_forward_speed[-50:])
        ),
    }
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
