"""Convert 29-dof hand URDF to USD for IsaacLab training."""
from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
import os

# Paths
urdf_path = "/home/ubuntu/robot_rl/robot_assets/g1/g1_29dof/g1_29dof_official.urdf"
usd_dir = "/home/ubuntu/robot_rl/robot_assets/g1/g1_29dof"
os.makedirs(usd_dir, exist_ok=True)

# Link meshes
mesh_src = "/home/ubuntu/unitree_rl_gym/resources/robots/g1_description/meshes"
mesh_dst = os.path.join(usd_dir, "meshes")
if os.path.exists(mesh_dst) and os.path.islink(mesh_dst):
    os.unlink(mesh_dst)
if not os.path.exists(mesh_dst):
    os.symlink(mesh_src, mesh_dst)
    print(f"Linked meshes: {mesh_src} -> {mesh_dst}")

cfg = UrdfConverterCfg(
    asset_path=urdf_path,
    usd_dir=usd_dir,
    usd_file_name="g1_29dof.usd",
    fix_base=False,
    merge_fixed_joints=True,
    make_instanceable=True,
    joint_drive=UrdfConverterCfg.JointDriveCfg(
        drive_type="force",
        target_type="position",
        gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
            stiffness=400.0,
            damping=40.0,
        ),
    ),
)

print(f"Converting {urdf_path} -> {usd_dir}/g1_29dof.usd ...")
converter = UrdfConverter(cfg)
print("Done!")

simulation_app.close()
