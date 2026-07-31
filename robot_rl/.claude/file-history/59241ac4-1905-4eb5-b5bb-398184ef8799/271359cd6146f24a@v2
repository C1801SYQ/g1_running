import numpy as np, csv, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rl_policy import RLPolicy
from robot import Robot
from simulation import Simulation

log_dir = '/home/ubuntu/robot_rl/logs/g1_policies/running-clf-29dof/running_clf_29dof/2026-07-14_21-50-18'
p = RLPolicy(f'{log_dir}/exported/policy_parameters.yaml', f'{log_dir}/exported/policy.pt')
p.load()

for speed in [3.0, 4.0, 5.0]:
    r = Robot('g1_21j','29dof_scene',np.array([1,1,1]),None,False,
              {'kp_y':1.5,'kd_y':0.3,'kp_yaw':0.8,'kd_yaw':0.3})
    s = speed
    r.input_function = lambda t: np.array([s, 0.0, 0.0])
    sim = Simulation(p,r,log=True,log_dir=f'{log_dir}/mujoco_logs',tracking_body_name='torso_link')
    ok = sim.run_headless(total_time=30)
    with open(os.path.join(sim.new_log_folder,'sim_log.csv')) as f:
        rows = list(csv.reader(f))
    vx = sum(float(row[37]) for row in rows[-100:])/min(100,len(rows))
    sp = sum((float(row[37])**2+float(row[38])**2)**0.5 for row in rows[-100:])/min(100,len(rows))
    status = 'FALL' if not ok else 'OK'
    print(f'{status} | cmd={speed:.0f} m/s | actual speed={sp:.2f} m/s | frames={len(rows)}')
