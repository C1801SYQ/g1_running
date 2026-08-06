# G1 29-DOF Running RL

基于 [Zolkin1/robot_rl](https://github.com/Zolkin1/robot_rl) 和 [fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) 改编。Apache 2.0 协议。

---

## 项目概述

在 G1 21 自由度跑步策略基础上扩展为 **29 自由度**全身跑步，训练策略实现 0~5.1 m/s 变速跑步 + 转向，并通过 rl_sar 框架部署到 MuJoCo 仿真和实体机器人。

**最终成果：** `steady_upper_v2` 模型实现高速跑步 + 视觉循迹，重点优化低速/站立姿态（骨盆直立、相机视角稳定）、上半身稳定（相机防抖）与鲁棒性（COM 偏移、摩擦随机化）。

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
│   │   ├── speed_arm/           # 手臂摆动模型 (4.90 m/s)
│   │   ├── steady_upper_v2/     # ★ 当前最佳 (89996 iter, 视觉循迹优化)
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

### 2b. 视觉循迹优化 (steady_upper_v2)

针对视觉百米冲刺的专项训练优化，核心目标是**相机稳定 + 低速直立 + 平地直线鲁棒**：

| 改动 | 内容 |
|------|------|
| standing 轨迹 | 轨迹库加入官方 standing 轨迹（0 m/s），解决低速/站姿低头导致相机看不到白线 |
| `pelvis_upright_reward` (4.0) | 骨盆 roll/pitch 保持直立，相机前视 |
| `pelvis_height_reward` (3.0) | 骨盆保持 0.65m 高度，防止下蹲看地 |
| `low_speed_upright_reward` (3.0) | 低速/静止时强制直立姿态 |
| `upper_body_stability` (6.0) | 上肢（含腰部）速度/加速度惩罚，抑制相机抖动 |
| `lat_vel` (3.0) | 横向速度跟踪强化，抗横漂（利于视觉循迹） |
| 速度分段采样 | `lin_vel_x_segments` 均匀覆盖 0-1.0/1.0-2.5/2.5-4.0/4.0-4.7/4.7-5.1 五段 |
| COM 偏移 | torso COM `±0.10m`，模拟平地重心偏移下跑直线 |
| 地面摩擦 | 0.3-2.0 每次 reset 随机，适应不同地面 |

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
| 8 | **`steady_upper`** | 29,997 | 0.0-5.1 | 上肢稳定版（续训 speed_turn，arm Q 20x） | ~4.5 m/s |
| 9 | **`steady_upper_v2`** | 89,996 | 0.0-5.1 | **视觉循迹优化**（续训 steady_upper）：骨盆直立/高度、上半身稳定、速度分段采样、COM/质量/摩擦随机化（出现后仰问题） | ★ 当前部署 |

> `steady_upper_v2` 针对视觉循迹做了专项优化：新增 standing 轨迹解决低速/站姿低头（相机看不到白线）、骨盆姿态/高度奖励保持相机前视、上半身稳定惩罚抑制抖动、COM 偏移与摩擦随机化增强平地直线鲁棒性。

---

## 使用方法

### 训练

```bash
cd robot_rl
conda activate env_isaaclab
pip install -e source/robot_rl/

# 全新训练
/home/ubuntu/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train_policy.py \
    --env_type=running_clf_29dof --headless --num_envs=2048 --max_iterations=10000 \
    --logger tensorboard

# 续训（从已有 checkpoint 恢复）
/home/ubuntu/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train_policy.py \
    --env_type=running_clf_29dof --headless --num_envs=2048 --max_iterations=10000 \
    --logger tensorboard --resume --load_run=<RUN_DIR> --checkpoint=model_9999
```

> 注意：`num_envs` 建议用 2048（4096 在部分多 ICD 驱动环境会触发
> omni `carb::tasking Mutex` 递归锁崩溃）。轨迹库包含 `standing` 轨迹
> （0 m/s）用于低速/站姿直立训练。

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

7. **4096 env 续训 GPU 死锁**: 多 ICD 驱动环境下 `num_envs=4096` 触发 omni `carb::tasking Mutex` 递归锁崩溃（从零训练正常、续训必现）。改用 `num_envs=2048` 解决。

8. **IsaacSim GUI 卡死**: 多 ICD 驱动（RTX 5090 + Intel 集显 + 双架构 libcuda）下非 headless 模式环境初始化死循环。训练/play 用 `--headless`；可视化用 MuJoCo（rl_sar）。

9. **Git LFS 推送代理超时**: 环境变量 `HTTPS_PROXY=127.0.0.1:7890` 导致 git LFS 上传超时。禁用代理（`-c http.proxy= -c https.proxy=`）+ 关闭 LFS 锁验证后正常。

---

## 轨迹库

`trajectories/running/` 包含 0~5.1 m/s 的跑步步态（5 阶 Bezier 曲线），含官方 `standing` 轨迹（0 m/s，用于低速/站姿直立）。1.2~3.7 m/s 为官方轨迹，4.0~5.0 m/s 为从 3.6 m/s 外推生成（仅缩放 T，保持关节轨迹形状）。

---

## 模型文件

| 文件 | 说明 |
|------|------|
| `models/steady_upper_v2/policy.pt` | ★ 当前最佳 29dof 跑步策略（89996 iter, 视觉循迹优化） |
| `models/steady_upper_v2/policy_parameters.yaml` | 策略参数（观测/动作/KP/KD/默认关节角） |
| `models/speed_turn/policy.pt` | 速度优先策略 (JIT TorchScript) |
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

`vision/` 目录来自 [MS-handsome6](https://github.com/MS-handsome6) 贡献：
- 视觉闭环百米冲刺系统
- 机载相机严格识别两条跑道白线 + G1 IMU 航向纠偏
- 自适应光照、运动模糊/断线恢复、相机共同抖动补偿和 Huber 鲁棒拟合
- MuJoCo 仿真验证完成

Skill 6 复用原有 `running` policy，不替换 Skill 5。控制器必须按
`0 → 1 → 6` 的顺序操作：先从 Passive 起立，再进入状态 1，最后启动
视觉百米冲刺；按 `6` 不会从 Passive 或 GetUp 自动起立。
MuJoCo 的启动安全支撑会等待 C++ 确认进入 Skill 6 和双线锁定，不再因
固定倒计时结束而让仍在操作状态机的机器人倒地。

安装、DDS/Conda 配置、启动方式和仿真结果见
[`vision/README.md`](vision/README.md)。
