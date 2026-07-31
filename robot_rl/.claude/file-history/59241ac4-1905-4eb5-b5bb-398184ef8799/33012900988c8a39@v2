import numpy as np, csv, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rl_policy import RLPolicy
from robot import Robot
from simulation import Simulation

log_dir = '/home/ubuntu/robot_rl/logs/g1_policies/running-clf-29dof/running_clf_29dof/2026-07-12_15-54-15'

p = RLPolicy(f'{log_dir}/exported/policy_parameters.yaml', f'{log_dir}/exported/policy.pt')
p.load()
r = Robot('g1_21j', '29dof_scene', np.array([1,1,1]), None, False,
          {'kp_y':1.5,'kd_y':0.3,'kp_yaw':0.8,'kd_yaw':0.3})
r.input_function = lambda t: np.array([4.0, 0.0, 0.0])
s = Simulation(p, r, log=True, log_dir=f'{log_dir}/mujoco_logs', tracking_body_name='torso_link')
ok = s.run_headless(total_time=30)
with open(os.path.join(s.new_log_folder, 'sim_log.csv')) as f:
    rows = list(csv.reader(f))
vx = sum(float(row[37]) for row in rows[-100:]) / min(100, len(rows))
sp = sum((float(row[37])**2 + float(row[38])**2)**0.5 for row in rows[-100:]) / min(100, len(rows))
print(f'Fell={not ok} | T={float(rows[-1][0]):.1f}s | Vx={vx:.2f} | Speed={sp:.2f} | Frames={len(rows)}')
