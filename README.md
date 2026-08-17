# G1 29-DOF Running RL

基于 [Zolkin1/robot_rl](https://github.com/Zolkin1/robot_rl) 和 [fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) 改编。Apache 2.0 协议。

---

## 项目概述

在 G1 21 自由度跑步策略基础上扩展为 **29 自由度**全身跑步，训练策略实现 0~5.1 m/s 变速跑步 + 转向，并通过 rl_sar 框架部署到 MuJoCo 仿真和实体机器人。

**已验收成果：** 稳定部署策略来自
`2026-08-12_13-11-17_gait_v2_finish_175198/model_175197.pt`，支持
0~5.1 m/s 速度命令、转向和视觉百米冲刺。最近一次 Skill 6 MuJoCo
实测完成 100 m 用时 **27.36 s**、平均速度 **3.65 m/s**，双线识别率
0.965，最大横向偏移 0.758 m。

当前工作分支已部署 smooth_175600 policy，并新增低中速跑步清理训练任务；回合
并主线前仍需按 `docs/BRANCHES.md` 拆分提交和回归验证。

> checkpoint 编号只在各自运行目录内有意义。当前 `model_175197` 与历史上
> 站姿异常的同编号模型不是同一文件；部署策略 SHA256 为
> `5de41b2e247d44db8c1378cc32367227482ab911a640366bc1fc563fda153b57`。

## 项目导航

| 入口 | 用途 |
|------|------|
| [`robot_rl/README.md`](robot_rl/README.md) | Isaac Lab 训练、策略导出和 sim2sim |
| [`rl_sar/README.md`](rl_sar/README.md) | C++/MuJoCo/实体机器人部署框架 |
| [`vision/README.md`](vision/README.md) | Skill 6/7 视觉循迹、仿真与真机流程 |
| [`docs/REPOSITORY_LAYOUT.md`](docs/REPOSITORY_LAYOUT.md) | 仓库目录职责、生成物边界和新增文件规范 |
| [`docs/BRANCHES.md`](docs/BRANCHES.md) | 分支职责、当前分叉状态和清理建议 |
| [`vision/scripts/README.md`](vision/scripts/README.md) | 视觉脚本索引与安全级别 |
| [`robot_rl/scripts/README.md`](robot_rl/scripts/README.md) | 训练、导出及工具脚本索引 |
| [`docs/REPOSITORY_CLEANUP.md`](docs/REPOSITORY_CLEANUP.md) | 本地生成物、备份和外部克隆的清理建议 |

## 当前状态（2026-08-17）

| 项目 | 状态 |
|------|------|
| 主线稳定基线 | `main` 已合入 Skill 7 视觉更新；历史稳定部署为 gait_v2 `model_175197` |
| 当前工作分支 | `repository-cleanup`，领先 `main` 4 个提交，包含 smooth_175600 policy、低速续训任务和仓库整理文档 |
| 当前部署策略（工作区） | `rl_sar/policy/g1/running/policy.pt`，SHA256 `91c8a527760e91654f6224a2927d6319935cf199abadbf117b16e3b35bcb9adc` |
| 速度范围 | `vx=0~5.1 m/s`、`vy=±0.75 m/s`、`wz=±1.5 rad/s` |
| 已记录视觉实测 | gait_v2：100 m / 27.36 s / 3.65 m/s，`max_abs_y=0.758 m` |
| 当前训练方向 | low/mid-speed cleanup：偏重 0.3~3.0 m/s 低中速跑步，保留少量 3.0~5.1 m/s 高速采样 |
| 机器人自启动 | 本体 user systemd `g1_vision_skill7.service` 已配置为开机启动 Skill 7，目标 `110 m / 260 s`，覆盖 100 m 赛程 |
| 分支整理 | 详见 `docs/BRANCHES.md`；旧视觉审计分支已合入主线，可确认后归档/删除 |

当前工作分支仍有未提交改动；合并回 `main` 前建议把文档整理、policy 部署、
训练配置和视觉脚本改动拆成独立提交，方便回滚和真机行为定位。

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
│   │   ├── speed_turn/          # 历史速度优先模型 (4.94 m/s)
│   │   ├── speed_arm/           # 手臂摆动模型 (4.90 m/s)
│   │   ├── steady_upper_v2/     # 历史归档模型（165198 iter）
│   │   └── standrun/            # 站立+跑步模型
│   └── trajectories/running/    # 跑步步态轨迹库 (1.2~5.0 m/s)
│
├── rl_sar/                      # 仿真部署框架 (C++/Python)
    ├── policy/g1/running/       # ★ 当前部署策略及参数
    ├── src/rl_sar/              # 核心代码
    │   ├── fsm_robot/fsm_g1.hpp # G1 状态机 (含跑步策略状态)
    │   ├── library/core/rl_sdk/ # RL SDK (观测/输出/PID)
    │   └── src/rl_sim_mujoco.cpp # MuJoCo 仿真器
│   └── cmake_build/bin/         # 本地编译产物（不提交）
│
├── vision/                      # Skill 6/7 视觉感知与任务编排
│   ├── scripts/                 # 仿真、真机、审计、安装入口
│   ├── tests/                   # 视觉/安全/任务生命周期测试
│   ├── integrations/            # rl_sar 与审计桥集成
│   └── docs/                    # 真机证据与专题报告
│
└── docs/                        # 仓库级结构和协作规范
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
| standing 轨迹（历史实验） | 曾加入官方 standing 轨迹解决低速低头，但其不对称关节姿态造成后仰，后续已移除 |
| `pelvis_upright_reward` (4.0) | 骨盆 roll/pitch 保持直立，相机前视 |
| `pelvis_height_reward` (3.0) | 骨盆保持 0.65m 高度，防止下蹲看地 |
| `low_speed_upright_reward` (3.0) | 低速/静止时强制直立姿态 |
| `upper_body_stability` (6.0) | 上肢（含腰部）速度/加速度惩罚，抑制相机抖动 |
| `lat_vel` (3.0→1.0) | 横向速度跟踪，后调低以逼策略用 yaw 转向而非横向位移 |
| 速度分段采样 | `lin_vel_x_segments` 均匀覆盖 0-1.0/1.0-2.5/2.5-4.0/4.0-4.7/4.7-5.1 五段 |
| COM 偏移 | torso COM `±0.10m`，模拟平地重心偏移下跑直线 |
| 地面摩擦 | 0.3-2.0 每次 reset 随机，适应不同地面 |

**后续修复迭代**：

| 改动 | 内容 |
|------|------|
| `default_posture_reward` (4.0) | 修复 0 m/s 后仰：低速时驱动关节回对称 default 站姿（移除不对称 standing 轨迹） |
| `torso_roll_rate_reward` (3.0) | 惩罚骨盆横滚角速度，抑制跑步时左右晃动（相机平视） |
| `torso_ang_vel_penalty` (0.05) | 线性惩罚骨盆角速度，强化抗横摆 |
| `rel_heading_envs` 0.2→0.4 | 转向训练比例翻倍，增强 yaw 转向能力 |
| `rel_closed_loop_yaw` 0.30→0.45 | 更多环境用 yaw 闭环跟踪 |
| `yaw_vel` 8.0→12.0 | 强化 yaw 转向跟踪（对应 yaw_vel 奖励从 2.3 提升到 ~5.0） |
| `lat_vel` 3.0→1.0 | 减少横向位移依赖，逼策略用 yaw 旋转纠偏 |

### 2c. gait_v2 / gait_v3 站跑过渡与直线性修复

| 改动 | 内容 |
|------|------|
| 真站立环境掩码 | 站姿、静止和 hip-roll 奖励只作用于 `is_standing_env`，不再误伤 0.3~1.0 m/s 正常步态 |
| 速度分段 | 慢速段改为 0.3~1.0 m/s；精确 0 m/s 由独立 standing 环境采样 |
| 命令斜坡 | `max_acc=(3.0, 3.0, 4.0)`，训练策略适应站立→慢跑→高速的连续命令 |
| 命令模式互斥 | 修复 open-loop、closed-loop-yaw、standing、closed-loop 采样区间重叠 |
| `slow_speed_hip_yaw_reward` | 仅在 0.3~1.5 m/s 直线段约束 hip yaw，改善约 1 m/s 的内八字，同时保留转向和抗扰恢复能力 |
| `straight_line_reward` | 直线命令且速度大于 1 m/s 时抑制横向速度和 yaw rate |
| 动作平滑 | `action_rate_l2=-0.020`，降低站跑切换的关节目标突变 |
| 抗扰随机化 | 每 6~10 s 注入 x/y/yaw 推扰动，并随机化摩擦、质量、COM、增益和关节物理参数 |

部署端不会强制把步态相位重置为 0，也不会在 Skill 6 进入时把全身插值到
默认站姿；实测表明这两种未经训练匹配的处理会导致骨盆下沉和起步险倒。
策略切换仍会清空上一模型遗留的动作队列，避免消费陈旧关节目标。

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
| 7 | `speed_turn_v2` | 历史阶段 | 0.0-5.1 | 手臂 Q 权重 20x 恢复摆臂（续训 speed_turn） | 已合并到后续模型 |
| 8 | **`steady_upper`** | 29,997 | 0.0-5.1 | 上肢稳定版（续训 speed_turn，arm Q 20x） | ~4.5 m/s |
| 9 | **`steady_upper_v2`** | 89,996 | 0.0-5.1 | **视觉循迹优化**（续训 steady_upper）：骨盆直立/高度、上半身稳定、速度分段采样、COM/质量/摩擦随机化（出现后仰问题） | 4.94 m/s |
| 10 | `steady_upper_v2_posture` | 109,995 | 0.0-5.1 | 修复 0 m/s 后仰：移除不对称 standing 轨迹，新增 default_posture 奖励 | 4.92 m/s |
| 11 | `steady_upper_v2_torso` | 139,994 | 0.0-5.1 | **上肢稳定**（续训）：torso roll-rate + ang-vel 奖励，抑制左右晃动 | 4.92 m/s |
| 12 | `steady_upper_v2_turn` | 155,200 | 0.0-5.1 | **转向强化**（续训）：rel_heading_envs 40%、rel_closed_loop_yaw 45%、lat_vel↓、yaw_vel↑，增强 yaw 转向 | 4.95 m/s |
| 13 | `steady_upper_v2_still` | 165,198 | 0.0-5.1 | **0 m/s 关节静止**（续训）：新增 low_speed_joint_stillness 奖励 | 最后一个历史稳定版 |
| 14 | `stand_improve` | 175,197 | 0.0-5.1 | 站姿奖励错误覆盖到 0~0.9 m/s，导致张腿/怪异站姿 | 已弃用，禁止部署 |
| 15 | `gait_v2_finish` | 175,197 | 0.0-5.1 | 修复 standing 掩码、速度段和命令斜坡后重训；当前部署 SHA256 `5de41b2e...3b57` | ★ 当前部署；视觉 100 m 27.36 s |
| 16 | `gait_v3_transition_straight` | 目标约 195,197 | 0.0-5.1 | 从 gait_v2 续训：增加低速 hip-yaw、站立静止、动作平滑和 1 m/s 以上直线奖励 | 训练中，未部署 |

> `steady_upper_v2` 针对视觉循迹做了专项优化：骨盆姿态/高度奖励保持相机前视、上半身稳定惩罚抑制抖动、COM 偏移与摩擦随机化增强平地直线鲁棒性。standing 轨迹只用于早期实验，因姿态不对称已移除。
>
> 后续迭代持续修复问题：`posture` 修复 0 m/s 后仰（移除不对称 standing 轨迹 + default_posture 奖励）、`torso` 抑制跑步时骨盆左右晃动（torso roll-rate 奖励）、`turn` 强化 yaw 转向（减少横向位移依赖，逼策略用旋转纠偏）。
>
> `stand_improve` 与 `gait_v2_finish` 都出现过 `model_175197` 文件名，引用时
> 必须同时写运行目录或 SHA256，不能只看迭代编号。

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
> omni `carb::tasking Mutex` 递归锁崩溃）。当前轨迹库不包含 standing
> 文件；0 m/s 由独立 standing 命令环境、默认站姿和静止奖励训练。

当前 gait_v3 续训由用户级 systemd 托管，关闭终端不会终止训练：

```bash
# 查看服务状态
systemctl --user status g1-running-gait-v3-195198.service

# 查看实时日志
tail -f /home/ubuntu/robot_rl/train_gait_v3_transition_straight.log

# 查看最近一次迭代
tr '\r' '\n' < /home/ubuntu/robot_rl/train_gait_v3_transition_straight.log \
    | grep 'Learning iteration' | tail -1
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

# 0 m/s → 1 m/s → 5.1 m/s 的无头回归诊断
python transfer/sim/diagnose_running_transitions.py \
    --sim-assets-root /home/ubuntu/robot_rl/transfer/sim
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

10. **0 m/s 后仰**: 官方 standing 轨迹双腿不对称（右 hip_pitch=-1.0），0 速度时 CLF 强制跟随导致后仰。移除该轨迹 + 新增 `default_posture_reward` 修复。

11. **上半身左右晃动**: 跑步时骨盆横滚摆动导致相机左右晃、破坏视觉循迹。新增 `torso_roll_rate_reward` + `torso_ang_vel_penalty` 抑制。

12. **视觉寻到相邻跑道**: 跑道身份改为起跑帧不可更新的几何锚点；滑动窗口只处理短时相机抖动，不能靠连续小步累计漂移到邻道。控制器在中心走廊内保持 IMU 直线航向，视觉越界后才做带回差的有界纠偏。

13. **yaw 转向困难**: 策略倾向用横向位移而非 yaw 旋转纠偏（`lat_vel` 权重高、转向训练比例低）。调高 `rel_heading_envs`/`rel_closed_loop_yaw`、`yaw_vel`，降低 `lat_vel`，重训强化转向。

14. **IsaacSim 测速失真**: transfer/sim 测速因 `v_x_max=3.0`（play 配置工件）限幅、场景/PD 增益不对而失真。正确测速需用 `29dof_basic_scene` + `set_pd_gains_from_policy` + 修正 v_x_max，详见 sim2sim 章节。

15. **低速奖励污染走路段**: 旧版用 `vx < 0.9` 判断站立，导致 0~0.9 m/s 环境同时被要求走路和关节静止。改为独立 `is_standing_env` 掩码，慢速采样从 0.3 m/s 起。

16. **速度命令模式重叠**: 旧区间判断会让 open-loop、yaw 闭环和 standing 标志重叠。改为一次随机数划分四个互斥区间，并对目标命令做分轴 slew-rate 限制。

17. **MuJoCo reset 后 Skill 6 不再运行**: viewer reset 只重置 `MjData`，不会通知 Python 任务状态。视觉进程现通过仿真时间回退或 X 位置大跳变重置整场 mission latch、检测器、控制器和计时状态。

18. **策略切换首帧抽搐**: 推理并发队列可能残留上一模型输出。`InitRL()` 在加载新策略前清空 position/velocity/torque 输出队列，避免 Skill 6 消费陈旧动作。

---

## 轨迹库

`trajectories/running/` 包含 1.2~5.0 m/s 的跑步步态（5 阶 Bezier
曲线）。1.2~3.6 m/s 为官方轨迹，4.0/4.5/5.0 m/s 为从 3.6 m/s
外推生成（仅缩放 T，保持关节轨迹形状）。精确 0 m/s 不使用 standing
轨迹，而由独立 standing 命令环境配合默认站姿、关节静止和 hip-roll
奖励学习。

---

## 模型文件

| 文件 | 说明 |
|------|------|
| `rl_sar/policy/g1/running/policy.pt` | ★ 当前部署策略：gait_v2 `model_175197`，SHA256 `5de41b2e...3b57` |
| `rl_sar/policy/g1/running/policy_parameters.yaml` | 当前部署参数（0~5.1 m/s、观测/动作/KP/KD/默认关节角） |
| `models/steady_upper_v2/policy.pt` | 历史归档策略（165198 iter），不是当前部署文件 |
| `models/steady_upper_v2/policy_parameters.yaml` | 历史归档策略参数 |
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

MuJoCo viewer reset 后，视觉任务会检测仿真时间回退/位置跳变并完整重置
Skill 6 mission；无需重启 Python 视觉进程即可再次执行 `0 → 1 → 6`。

当前 gait_v2 策略最近一次视觉回归结果：100 m 用时 27.36 s，平均
3.65 m/s，双线识别率 0.965，最大横向偏移 0.758 m。偏移主要来自策略
本体的低频横摆和终点前短暂丢线，gait_v3 正针对低速过渡和高速直线性续训。

安装、DDS/Conda 配置、启动方式和仿真结果见
[`vision/README.md`](vision/README.md)。
