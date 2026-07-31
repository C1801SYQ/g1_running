"""Run 29dof MuJoCo: joystick controls, PD auto-straight when stick centered.
Left stick up = accelerate (0~5 m/s), right stick left/right = turn.
PD keeps y-position and yaw straight automatically.
"""
import numpy as np, os, sys, pathlib, glob
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rl_policy import RLPolicy
from robot import Robot
from simulation import Simulation

# Use latest policy
base_log = str(pathlib.Path(__file__).parent.parent.parent /
               'logs/g1_policies/running-clf-29dof/running_clf_29dof')
latest_run = max(glob.glob(os.path.join(base_log, '*')), key=os.path.getmtime)
print(f"[INFO] Using policy from: {latest_run}")
p = RLPolicy(f'{latest_run}/exported/policy_parameters.yaml',
             f'{latest_run}/exported/policy.pt')
p.load()

# Override YAML limits to allow yaw and lateral commands
p.policy_params['w_z_max'] = 2.0
p.policy_params['w_z_min'] = -2.0
p.policy_params['v_y_max'] = 0.5
p.policy_params['v_y_min'] = -0.5

# Track scene
r = Robot('g1_21j', '29dof_track_scene', np.array([1, 1, 1]), None, False,
          {'kp_y': 3.0, 'kd_y': 0.5, 'kp_yaw': 1.5, 'kd_yaw': 0.5})

# Custom command: joystick controls speed + turn, PD keeps y-position
desired_yaw = 0.0  # accumulated desired yaw for smooth turning
def custom_velocity_cmd(policy):
    global desired_yaw
    if r.joystick is not None:
        raw_vx = -r.joystick.get_axis(1)    # left stick up/down
        raw_vy = -r.joystick.get_axis(0)    # left stick left/right
        raw_vyaw = -r.joystick.get_axis(3)  # right stick left/right
    else:
        raw_vx = raw_vy = raw_vyaw = 0.0

    # Forward speed: stick maps to [0, 5.1] linearly
    vx = np.clip(raw_vx, 0.0, 1.0) * policy.get_max_vx()

    # Lateral: use joystick if pushed, else PD correction
    if abs(raw_vy) > 0.05:
        vy = raw_vy * 0.5  # max 0.5 m/s lateral
    else:
        qpos = r.mj_data.qpos
        qvel = r.mj_data.qvel
        vy = np.sign(vx + 1e-6) * np.clip(-3.0 * qpos[1] - 0.5 * qvel[1], -0.5, 0.5)

    # Yaw: accumulate desired yaw from joystick, PD tracks it
    if abs(raw_vyaw) > 0.05:
        desired_yaw += raw_vyaw * 0.15  # accumulate desired heading
    qpos = r.mj_data.qpos
    qvel = r.mj_data.qvel
    siny_cosp = 2 * (qpos[3] * qpos[6] + qpos[4] * qpos[5])
    cosy_cosp = 1 - 2 * (qpos[5]**2 + qpos[6]**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    yaw_err = desired_yaw - yaw
    # Wrap to [-pi, pi]
    yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi
    vyaw = np.clip(2.0 * yaw_err + 0.5 * (-qvel[5]), -2.0, 2.0)

    return np.array([vx, vy, vyaw])

r.get_joystick_command = custom_velocity_cmd

Simulation(p, r, tracking_body_name='torso_link').run(-1)
