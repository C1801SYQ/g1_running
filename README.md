# G1 29-DOF Running RL

基于 [Zolkin1/robot_rl](https://github.com/Zolkin1/robot_rl) 和 [fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) 改编。Apache 2.0 协议。

---

## 项目概述

在 G1 21 自由度跑步策略基础上扩展为 **29 自由度**全身跑步，训练策略实现 0~5.1 m/s 变速跑步 + 转向，并通过 rl_sar 框架部署到 MuJoCo 仿真和实体机器人。

**最终成果：** speed_turn 模型实现 4.94 m/s 实际极速，支持站立/变速/转向。

---

## 环境依赖

- Ubuntu 22.04
- NVIDIA RTX 5090 (32GB VRAM)
- IsaacSim 5.0 (IsaacLab)
- Conda env: `env_isaaclab` 或 `isaac_rl_v2`
- MuJoCo 3.2.7
- CUDA 13.0 / Driver 580.173

---

## 目录结构

```
├── robot_rl/                    # RL 训练框架
│   ├── scripts/rsl_rl/          # 训练/测试/导出脚本
│   ├── source/robot_rl/         # 核心代码
│   │   └── robot_rl/tasks/manager_based/robot_rl/
│   │       ├── g1/              # G1 环境配置
│   │       │   ├── g1_running_clf_29dof_env_cfg.py  # 29dof 跑步训练配置
│   │       │   └── agents/      # PPO 策略配置
│   │       └── mdp/             # MDP 组件 (rewards/commands/terminations/events)
│   ├── transfer/sim/            # MuJoCo sim2sim 验证
│   ├── transfer/obelisk/        # Obelisk 实体部署 (ROS2)
│   ├── models/                  # 训练好的策略文件
│   │   ├── speed_turn/          # ★ 最佳模型 (4.94 m/s)
│   │   └── standrun/            # 站立+跑步模型
│   └── trajectories/running/    # 跑步步态轨迹库 (1.2~5.0 m/s)
│
└── rl_sar/                      # 仿真部署框架 (C++/Python)
    ├── policy/g1/running/       # 策略加载配置
    ├── src/rl_sar/              # 核心代码
    │   ├── fsm_robot/fsm_g1.hpp # G1 状态机 (含跑步策略状态)
    │   ├── library/core/rl_sdk/ # RL SDK (观测/输出/PID)
    │   └── src/rl_sim_mujoco.cpp # MuJoCo 仿真器
    └── cmake_build/bin/         # 编译产物
```

---

## 核心修改清单

### 1. 29 自由度适配 (robot_rl)

| 文件 | 修改内容 |
|------|----------|
| `g1_running_clf_29dof_env_cfg.py` | 新建 29dof 训练配置，轨迹库路径、速度范围、随机化参数 |
| `g1_29dof.py` | 新建 29dof 机器人 USD 定义（含 waist_roll/pitch + 6 wrist） |
| `__init__.py` | 注册 `G1-running-clf-29dof` 和 `G1-running-clf-29dof-play` |
| `physical_randomization.py` | 对齐 21dof 随机化范围（原 29dof 配置 10-20x 过宽） |
| `rewards.py` | 新增 `joint_pos_default_reward`（惩罚未跟踪的额外关节漂移） |
| `resets.py` | 修复 extra joints 初始化（waist/wrist 用 default_pos 而非 0） |
| `symmetry_functions.py` | 重写 `_switch_g1_joints` 为 29 关节索引映射（原 21 关节版本导致手臂不对称） |
| `train_policy.py` / `play_policy.py` | 新增 `running_clf_29dof` 环境类型映射 |
| `g1_running_clf_env_cfg.py` | 速度范围 1.0→5.1，轨迹随机化对齐 |

### 2. 速度优先训练策略

| 参数 | 初始值 | 调整值 | 原因 |
|------|--------|--------|------|
| CLF_WEIGHT | 10.0 | **1.0** | 降低轨迹追踪约束，给速度让路 |
| xy_vel weight | 1.0 | **10.0** | 速度跟踪优先于轨迹精度 |
| yaw_vel weight | 1.0 | **8.0** | 强化转向跟踪 |
| arm Q weights | 3.0 | **20.0** | 恢复手臂摆臂（高速时不摆臂影响平衡） |
| speed range | 1.0-5.1 | **0.0-5.1** | 零速也训练，学会站立 |
| heading | 无 | **±3.14 + rel_heading_envs=0.2** | 20% 环境训练转向 |
| ang_vel_z | ±0.75 | **±1.5** | 大角度转向 |

### 3. Sim2Sim MuJoCo 修复 (robot_rl/transfer/sim)

| 文件 | 修改内容 |
|------|----------|
| `robot.py` | **恢复原始 PD 设置** — `gainprm[0]=kp, biasprm[1]=-kp, biasprm[2]=-kd`（原本地修改版误加了 `gainprm[1]=kd` 和 Unitree 增益覆盖，导致 Sim2Sim gap） |
| `robot.py` | 移除机器人名称硬编码限制（添加 29dof 支持） |
| `rl_policy.py` | 修复策略加载路径解析（支持绝对路径） |

### 4. rl_sar 部署适配

| 文件 | 修改内容 |
|------|----------|
| `fsm_g1.hpp` | **新建 `RLFSMStateRLRunning`** — 跑步策略状态机（Enter/Run/Exit/CheckChange） |
| `fsm_g1.hpp` | 注册到工厂 + 按键 Num5/LB_DPadUp 切换 + GetUp→Running 过渡 |
| `rl_sdk.cpp` | **新增 `phase_sin_cos` 观测** — 计算 sin(2πφ)/cos(2πφ) 步态相位（原框架无此观测） |
| `rl_sdk.hpp` | 新增 `base_position` 字段到 RobotState（用于真距离估算） |
| `rl_sim_mujoco.cpp` | 从 `mjData->qpos` 读取 base 位置 |
| `policy/g1/running/config.yaml` | 新建 29dof 跑步策略配置（98 维观测、29 维动作、KP/KD/action_scale/joint_mapping） |

---

## 训练历程

| 阶段 | 模型 | 迭代数 | 速度范围 | 特点 | 极速 |
|------|------|--------|----------|------|------|
| 1 | `2026-07-24_15-42-49` | 3,200 | 1.0-3.7 | 首次成功训练，对齐 21dof 随机化 | ~2.6 m/s |
| 2 | `highspeed` | 9,999 | 1.0-5.1 | 外推轨迹 4.0-5.0（时间缩放，物理不可行） | ~3.0 m/s |
| 3 | `highspeed_v2` | 9,999 | 1.0-5.1 | 修复外推轨迹（只缩放 T，不缩放关节） | ~2.6 m/s |
| 4 | `standrun` | 9,999 | **0.0-5.1** | **首次包含零速站立训练** | 4.75 m/s |
| 5 | `rough_turn` | 5,000 | 0.0-5.1 | 粗糙地形+转向（续训 standrun） | 2.6 m/s (地形太保守) |
| 6 | **`speed_turn`** | 10,000 | 0.0-5.1 | **速度权重 10x + 转向（续训 standrun）** | **4.94 m/s** ★ |
| 7 | `speed_turn_v2` | 训练中 | 0.0-5.1 | 手臂 Q 权重 20x 恢复摆臂（续训 speed_turn） | - |

---

## 使用方法

### 训练

```bash
cd robot_rl
conda activate env_isaaclab
pip install -e source/robot_rl/

# 全新训练
/home/ubuntu/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train_policy.py \
    --env_type=running_clf_29dof --headless --num_envs=4096 --max_iterations=10000 \
    --logger tensorboard

# 续训（从已有 checkpoint 恢复）
/home/ubuntu/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train_policy.py \
    --env_type=running_clf_29dof --headless --num_envs=4096 --max_iterations=10000 \
    --logger tensorboard --resume --load_run=<RUN_DIR> --checkpoint=model_9999
```

### 导出策略

```bash
/home/ubuntu/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play_policy.py \
    --env_type=running_clf_29dof --num_envs=1 --export_policy --headless \
    --load_run=<RUN_DIR> --checkpoint=model_<ITER> --logger tensorboard
```

### MuJoCo 仿真测试 (robot_rl/transfer/sim)

```bash
cd robot_rl
source ~/anaconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
export PYTHONPATH=$PWD/transfer:$PYTHONPATH
python transfer/sim/g1_runner.py --env_type=running_clf_29dof \
    --scene=29dof_basic_scene --load_run=<RUN_DIR> --log
```

### MuJoCo 仿真测试 (rl_sar)

```bash
cd rl_sar
./build.sh -mj                                    # 编译（仅首次）
./cmake_build/bin/rl_sim_mujoco g1 scene_29dof   # 运行

# 键盘控制
0 / A         → 起身站立
5 / LB_DPadUp → 切换到跑步策略
W/S           → 前进/后退
A/D           → 左右平移
Q/E           → 左右转向
P / LB_X      → 被动模式（急停）
```

### 实体部署（Obelisk）

详见 `robot_rl/transfer/obelisk/README.md` 和 `robot_rl/DEPLOY_GUIDE.txt`

---

## 关键 Bug 修复记录

1. **PD 增益 Bug** (robot.py): 本地修改版加了 `gainprm[1]=kd` 和 Unitree 增益覆盖，导致 Sim2Sim gap。恢复原始 Zolkin1 方案解决。

2. **对称数据增强 Bug** (symmetry_functions.py): `_switch_g1_joints` 硬编码 21 关节索引，29dof 训练时手臂映射全错。重写为 29 关节版本。

3. **外推轨迹 Bug** (extrapolate_traj.py): 时间缩放时同时缩放 T 和 qd 系数，导致双重速度缩放。修正为只缩放 T（轨迹管理器自动处理速度缩放）。

4. **GPU 驱动不匹配**: Driver 580.159 (内核) vs 580.173 (用户态)，导致 CUDA 死锁（18h 只跑 2 iter）。重启后对齐解决。

5. **rl_sar 训练稳定后 IsaacSim Play 摔倒**: Play config 禁用随机化，策略依赖训练时的随机化鲁棒性。

6. **rl_sar GetUp→Running 关节状态映射错误**: 两个状态使用不同 `joint_mapping`，切换时状态索引错乱。

---

## 轨迹库

`trajectories/running/` 包含 1.2~5.0 m/s 的跑步步态（5 阶 Bezier 曲线）。4.0~5.0 m/s 为从 3.6 m/s 外推生成（仅缩放 T，保持关节轨迹形状）。

---

## 模型文件

| 文件 | 说明 |
|------|------|
| `models/speed_turn/policy.pt` | ★ 最佳 29dof 跑步策略 (JIT TorchScript) |
| `models/speed_turn/policy_parameters.yaml` | 策略参数（观测/动作/KP/KD/默认关节角） |
| `models/standrun/policy.pt` | 站立+跑步策略（速度优先权重较低版） |

---

## 引用

```
@software{Zolkin_robot_rl,
  author = {Zachary Olkin},
  title = {robot_rl: Reinforcement Learning for Humanoid Robots},
  url = {https://github.com/Zolkin1/robot_rl},
  year = {2026}
}

@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```

---
## 视觉集成模块

来自 [MS-handsome6](https://github.com/MS-handsome6) 的贡献：
-  — 视觉闭环百米冲刺系统
- 机载相机识别跑道白线 + G1 IMU 航向纠偏
- MuJoCo 仿真验证完成
- 详见: https://github.com/MS-handsome6/g1_race_vision
