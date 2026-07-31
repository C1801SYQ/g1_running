# G1 29-DOF Running RL

Based on [Zolkin1/robot_rl](https://github.com/Zolkin1/robot_rl) and [fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar).

## robot_rl/ - Training
G1 29-dof running policy training using IsaacLab + RSL-RL.
- Speed range: 0-5.1 m/s (standing + running)
- Turning enabled (±1.5 rad/s yaw)
- Speed-first reward weighting (velocity 10x, CLF 1x)
- Symmetric half-periodic data augmentation
- Latest model: speed_turn (4.94 m/s max, standing, turning)

## rl_sar/ - Simulation & Deployment
MuJoCo simulation and real robot deployment framework.
- G1 29-dof running FSM state (keyboard/gamepad control)
- Phase sin/cos observation support
- W/A/S/D speed control, Q/E turning

## Usage
```bash
# Training
cd robot_rl && /path/to/isaaclab.sh -p scripts/rsl_rl/train_policy.py --env_type=running_clf_29dof --headless --num_envs=4096

# MuJoCo simulation
cd rl_sar && ./build.sh -mj && ./cmake_build/bin/rl_sim_mujoco g1 scene_29dof

# Real robot
cd rl_sar && ./cmake_build/bin/rl_real_g1 <NETWORK_INTERFACE>
```
