"""Convert official g1_29dof.xml MJCF to USD for IsaacLab."""
import os

from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

from isaaclab.sim.converters import MjcfConverter, MjcfConverterCfg

usd_dir = '/home/ubuntu/robot_rl/robot_assets/g1/g1_29dof'
os.makedirs(usd_dir, exist_ok=True)

# Link meshes
mesh_src = '/home/ubuntu/unitree_mujoco/unitree_robots/g1/meshes'
mesh_dst = os.path.join(usd_dir, 'meshes')
if os.path.exists(mesh_dst):
    os.unlink(mesh_dst)
os.symlink(mesh_src, mesh_dst)

cfg = MjcfConverterCfg(
    asset_path='/home/ubuntu/unitree_mujoco/unitree_robots/g1/g1_29dof.xml',
    usd_dir=usd_dir,
    usd_file_name='g1_29dof.usd',
    fix_base=False,
    self_collision=True,
    make_instanceable=True,
)

print(f'Converting MJCF to USD -> {usd_dir}/g1_29dof.usd ...')
converter = MjcfConverter(cfg)
print('Done!')
print(os.listdir(usd_dir))

simulation_app.close()
