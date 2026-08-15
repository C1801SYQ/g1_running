# Unitree G1 视觉自主百米冲刺（Skill 6）

这是 `g1_running` 的增量视觉扩展。原有 `robot_rl` 训练框架、29 自由度模型、`rl_sar`、Skill 5 跑步状态和全部策略文件保持不变；本目录新增 D435i 类 RGB-D 相机、双白线识别、视觉/IMU 纠偏和 100 m 比赛状态机。

视觉模块由 [MS-handsome6](https://github.com/MS-handsome6) 贡献，并在本仓库中持续维护。

> 当前完成的是 MuJoCo 联合仿真验证，尚未声明完成真实 G1 百米实机验证。

![G1 双白线识别与机器人视角](assets/g1_camera_dashboard_official_track.png)

## 功能

- 蓝色四跑道直道；目标赛道两条白线中心距 2.1 m，白线各宽 0.10 m，因此两线内侧净宽为 2.0 m。
- RGB 图像经自适应曝光阈值、中性颜色/局部对比度白线分割、抗断裂形态学处理和 Huber 鲁棒直线拟合，同时识别目标跑道两侧白线，可承受暗光、曝光突变、运动模糊和机身抖动。
- 只有两条边界线同时有效时才允许视觉纠偏；起跑帧生成不可移动的跑道几何锚点，短期跟踪窗口不能逐帧漂移到相邻跑道。
- 训练策略和 G1 IMU 负责跑直线；视觉在中心走廊内不转向，只有横向偏差越过进入阈值才做有界纠偏，回到更小的退出阈值后停止纠偏。
- 新增 Skill 6：必须先按 `0` 起立、按 `1` 进入稳定站立/运动状态，再按 `6` 进入视觉冲刺；按 `6` 不会自动起立。原 Skill 5 和按键 `5` 保持不变。
- Skill 5/6 当前共用 gait-v2 `model_175197` 部署策略；安装和编译脚本会校验策略 SHA-256，避免模型与部署配置错配。
- MuJoCo 启动安全支撑不再按固定 16 秒强制消失；C++ 状态机会通过本机 UDP 确认真正进入 Skill 6，再平滑卸掉垂直支撑并等待真实相机姿态稳定，双线锁定后释放水平起跑约束，人工操作慢也不会因超时倒地。
- 越过 100 m 后继续巡线并平滑减速，视觉 UDP 超过 300 ms 无新数据时强制输出零速度。

## 控制结构

```text
MuJoCo G1 + D435i 类 RGB-D 相机
                │
                ▼
       双白线检测与目标跑道锁定 ──┐
                                ├─► 视觉/IMU 控制器
                    G1 IMU ─────┘         │
                                         ▼
                               UDP: vx, vy=0, wz
                                         │
                                         ▼
                            rl_sar Skill 6 状态机
                                         │
                                         ▼
                           原 Skill 5 running policy
```

Skill 6 复用原仓库的 `rl_sar/policy/g1/running/policy.pt`。视觉模式关闭时，UDP 接收器不会覆盖手柄/键盘速度，因此原有运行方式仍然有效。

## 视觉鲁棒性设计

- 每帧根据 ROI 亮度分布自动调整白线亮度阈值；进入阴影时快速降低阈值，离开阴影时缓慢提高，避免自动曝光抖动造成阈值闪烁。
- 同时检查 HSV 饱和度、RGB 色差和局部亮度对比，只保留接近白色的高亮结构，降低蓝色跑道、彩色反光和大面积亮斑的干扰。
- 先用纵向闭运算连接高速模糊形成的断点，再用较小的开运算去噪；直线使用 Huber 损失拟合，减少眩光和模糊尾迹对斜率的影响。
- 锁定跑道后，如果连通轮廓被抖动切碎，会在历史双线附近重新收集**实际白线像素**并分别拟合左右边界；缺少任意一条真实边界仍返回 `LOST`，不会用单线推算另一条线。
- 时间门控把“左右线一起平移的相机抖动”和“线宽/形状改变”分开处理；另用永不更新的起跑锚点限制累计位移，从机制上阻断小步漂移后换到邻道。
- MuJoCo 相机姿态先转换到 OpenCV 坐标系，再用 Skill 6 稳定期内约 30 帧的平均旋转做完整 roll/pitch/yaw 单应补偿；空白边缘使用无效黑像素，不复制出伪白线。
- 锁道时同时标定相机横向零偏、消失点固定偏差和 IMU 平均直行航向；大横向误差、快速横穿、较大航向角或持续转向饱和都会提前降速。
- 跑道中心附近只保持起跑时记录的 IMU 直线航向，不再使用视觉消失点持续引导；大偏差直接进入纠偏，较小但同方向持续外漂会经 0.30 秒确认后提前介入，左右步态摆动不会累计触发；最大纠偏目标为 `0.18 rad`，并通过回差避免反复切换。

调试画面的 `V>=数字` 是当前自适应亮度阈值。实机 RealSense 入口可按现场情况覆盖关键参数，例如：

```bash
python vision/scripts/run_realsense_ros2.py --ros-args \
  -p adaptive_value_floor:=50 \
  -p white_saturation_max:=105 \
  -p local_contrast_min:=12 \
  -p guided_search_margin_ratio:=0.08
```

实机入口默认处于硬性 Dry-run：不会创建 UDP 发送器，
`/g1/race/cmd_vel` 始终发布零速度；算法计算出的未执行命令只发布到
`/g1/race/desired_cmd_vel`。第一阶段相机与感知验收应保持默认值：

```bash
python vision/scripts/run_realsense_ros2.py --ros-args \
  -p command_output_enabled:=false
```

G1 NX 已安装 ROS 2 和源码版 RealSense Wrapper 后，可在仓库根目录用一条
命令同时启动相机与强制 Dry-run 视觉节点；按 `Ctrl+C` 会关闭两个进程：

```bash
bash vision/scripts/run_g1_realsense_dry_run.sh
```

该脚本不会提供启用运动输出的参数，并且发现 `rl_real_g1` 或 `g1_ctrl`
进程时会拒绝启动。

深度安全模块在中央可配置 ROI 内使用有效深度的稳健分位数，默认在 0.80 m
内停车、0.80–1.50 m 线性减速；深度缺失、有效比例不足或超过 0.20 秒未更新
都会失败归零。话题分工为：

- `/g1/race/desired_cmd_vel`：视觉控制器的原始建议；
- `/g1/race/safe_cmd_vel`：经过深度安全门后的建议；
- `/g1/race/cmd_vel`：实际输出，Dry-run 中始终为零；
- `/g1/race/safety_state`：障碍物距离、有效深度比例、帧龄和停止原因。

已编译 audit bridge 后，可运行完整的无运动调用验收链：

```bash
bash vision/scripts/run_g1_realsense_audit.sh
```

audit bridge 只监听 `127.0.0.1:15003`，强制 `vy=0`，把 `vx` 和 `wz`
分别限到 0.10 m/s 和 0.10 rad/s，并在 200 ms 收不到命令时归零。当前
二进制没有速度、起立、停止或 FSM 修改调用，不能驱动机器人。

只有在完成 RGB-D/IMU 标定、仿真闭环、架空测试和独立急停验证后，才允许由
操作员显式设置 `command_output_enabled:=true`。仅把
`cruise_speed_mps` 设为零不能代替 Dry-run 闸门，因为控制器仍有最低跟踪速度。

## 目录

```text
vision/
├── depth_safety.py            # 深度障碍物、数据新鲜度和失效归零
├── line_detector.py           # 双白线识别与跑道身份锁定
├── controller.py              # 视觉/IMU 融合、加速与终点减速
├── rendering.py               # MuJoCo RGB-D 渲染
├── udp_command.py             # 速度指令发送
├── scripts/
│   ├── activate_g1race_dds.sh # 激活 g1race 并统一 DDS 配置
│   ├── install_g1_running.sh  # 应用 rl_sar 集成补丁并首次编译
│   ├── build_skill6.sh        # 只重编译 rl_real_g1，不改原 policy
│   ├── prepare_unitree_scene.py
│   ├── run_g1_realsense_audit.sh   # 深度安全+audit bridge 联调
│   ├── run_g1_realsense_dry_run.sh # 真机相机+感知强制 Dry-run
│   ├── run_unitree_camera_sim.py
│   ├── run_vm_full_demo.sh
│   ├── start_vm_gui.sh
│   └── stop_vm_gui.sh
├── environment.yml            # 可复现的 g1race Conda 环境
├── assets/                    # 独立视觉闭环场景与截图
├── tests/                     # 125 项单元测试
└── integrations/
    ├── g1_loco_bridge/        # 无运动调用的 SDK2 audit bridge
    ├── g1_running/            # rl_sar C++ 集成补丁
    └── unitree_rl_mjlab/      # 早期 ONNX 接入参考
```

Skill 6 的 C++ 接口和状态机改动保存在 `vision/integrations/g1_running/`。
`install_g1_running.sh` 会把这些增量补丁应用到固定版本的 `g1_running`，
不会覆盖仓库中与视觉冲刺无关的训练代码和策略文件。

## 环境

- Ubuntu 22.04
- Python 3.11（推荐 Conda；Anaconda 和 Miniconda 均可）
- MuJoCo Python 3.x
- Unitree `unitree_mujoco`
- Unitree SDK2 / `unitree_sdk2_python`
- CMake、C++17、LibTorch / ONNX Runtime
- 可选：ROS 2 Humble 与 RealSense ROS，用于实机相机接入

## 安装

在本仓库根目录创建名为 `g1race` 的环境：

```bash
conda env create -f vision/environment.yml
conda activate g1race
```

如果 `g1race` 已经存在，则更新它，不需要删除重装：

```bash
conda env update -n g1race -f vision/environment.yml
```

`unitree_sdk2_python` 仍按 `unitree_mujoco` 官方说明安装到同一个 `g1race` 环境中。

安装与 Unitree SDK2 Python 匹配的 CycloneDDS 运行库：

```bash
bash vision/scripts/install_cyclonedds_runtime.sh
```

每次新开终端后，可用下面一条命令同时激活 `g1race` 和本项目的 DDS 配置：

```bash
source vision/scripts/activate_g1race_dds.sh
```

它固定使用 CycloneDDS `0.10.2`、DDS domain `0`、回环网卡 `lo`，并关闭共享内存传输，保证 Python `unitree_mujoco` 桥和 C++ `rl_sar` 使用同一套 DDS 配置。完整启动脚本也会自动执行这一步。

首次安装集成补丁并编译包含 Skill 5 和 Skill 6 的 G1 控制器：

```bash
bash vision/scripts/install_g1_running.sh
```

如果补丁已经安装，只需重新编译控制器：

```bash
bash vision/scripts/build_skill6.sh
```

两个脚本都会校验原 `running` policy 的 SHA-256；不会替换或修改原策略。

## 运行

先确保 `unitree_mujoco` 位于 `~/unitree_ws/unitree_mujoco`，然后在仓库根目录的 Ubuntu 终端执行：

```bash
bash vision/scripts/run_vm_full_demo.sh
```

脚本会打开 MuJoCo 主视角和机器人 RGB-D 视角，并把当前终端保留为 `rl_sar` 键盘控制终端。鼠标可以点击 MuJoCo 窗口查看画面，但按键必须输入到启动命令所在的终端。

严格按下面顺序操作：

1. 启动后控制器处于 `Passive`。在 `rl_sar` 终端按 `0`，进入 `GetUp` 起立过程。
2. 等机器人完全站稳后，在同一终端按 `1`，进入状态 1 `RLFSMStateRLRoboMimicLocomotion`。零速度指令下机器人会保持站立。
3. 确认状态 1 稳定后，在同一终端按 `6`，进入视觉百米冲刺 `Skill 6`。

```text
启动 MuJoCo/DDS → Passive ──0──> GetUp ──1──> 状态 1 ──6──> Skill 6
                                                           │
                              双线巡线冲刺 ←────────────────┘
```

`6` 只在状态 1 有效；从 Passive、GetUp、GetDown 或 Skill 5 按 `6` 都不会启动 Skill 6，也不会自动起立。

仿真安全支撑会一直等待上述状态机流程；C++ 只有在 Skill 6 跑步模型加载完成后才发送模式确认，Python 再收集 0.60 秒相机姿态样本并锁道起步。自动流程以状态 1 确认事件代替固定 3 秒盲等，旧的 17 秒视觉门槛保持取消。

停止全部进程：回到 `rl_sar` 控制终端按 `Ctrl+C`。兼容入口 `start_vm_gui.sh` 也改为相同的前台交互模式：

```bash
bash vision/scripts/start_vm_gui.sh
```

如需手动指定依赖位置：

```bash
UNITREE_ROOT=~/unitree_ws \
G1_RUNNING_ROOT="$PWD" \
bash vision/scripts/run_vm_full_demo.sh
```

## Skill 7（Num7）：1.0 m 视觉行走

Skill 7 复用状态 1 的 `robomimic/locomotion` 策略，通过 D435i 识别白线，以策略训练分布内的 `0.50 m/s` 沿线前进约 1 米，自动停止并返回状态 1。`WALK0P5M` 仅作为兼容旧版本的 UDP 模式名保留。它**不能**与 Unitree 高层 LocoClient 并行使用。

```text
Passive ──0──> GetUp ──1──> 状态 1 ──7──> Skill 7 ──完成/超时/故障──> 零速保持 ──> 状态 1
```

`7` 只在状态 1 有效。进入后 vx/vy/wz 清零，等待第一条正向、安全、新鲜的运动命令（最多 2 秒），随后以命令速度积分估算前进距离。

> **任务计时与 FSM 握手**：Python 视觉可以**先于** `rl_real_g1` 启动，但 Num7 的时间和距离累计**只在收到 C++ FSM 的 `G1_VISION_WALK_0P5M 1` 后才开始**（状态端口 15002，C++ 每 0.5 秒周期心跳重发）。按 7 之前 Python 不累计时间、不累计距离、不输出非零命令、不发送会被下一次任务继承的 hard_stop。收到 `G1_VISION_WALK_0P5M 0` 后立即停止输出并清理任务状态。

> **重复任务与跑道硬锁**：每次真实的 `NONE -> WALK0P5M` 上升沿都会在相机回调线程同步重置 detector、controller 和距离 gate，清除上次任务可能留下的 `lane-identity-lost`；500 ms 心跳不会重复重置。本次任务重新锁定初始居中双白线后，lane anchor 保持不变，仍禁止跳到相邻跑道。仿真启动支撑淡出为 0.60 秒，早于 2 秒首条前进命令超时。

> **重要**：`estimated_distance` 是命令速度的积分估算，**不是**可靠的里程计实测值。真机验收必须以人工测量为准。**修复完成前的旧报告已失效。**

### 停止条件（任一触发即自动回状态 1）

1. 达到目标距离 / 保守停止点（默认 1.00 m，`G1_NUM7_TARGET_M`，限制 0.05～1.00，C++ 与 Python 两端一致）
2. 总时长达到 7 秒（`G1_NUM7_MAX_DURATION_S`）
3. 2 秒内没有第一条有效运动命令（`G1_NUM7_START_TIMEOUT_S`）
4. UDP 超过 300 ms 无新命令
5. Python 报告 hard_stop
6. 深度缺失 / 过期 / 无效 / 检测到障碍
7. 已锁线后丢线
8. 持续收到 NaN、Inf 或非法 UDP 包

> **非法 UDP 包的准确行为**：非法包（NaN/Inf/尾随垃圾/错误 hard_stop 值）会被 C++ 拒绝且**不刷新 watchdog**，也不会改变当前命令、hard_stop、freshness 或 generation 状态——它**不会**单包触发立即 hard-stop。停止由 watchdog 时序决定：
> - 任务已开始：若之后只收到非法包（没有新的合法命令），最迟 300 ms 后触发 `UDP_STALE`；
> - 任务尚未开始：若一直收不到第一条合法正向命令，最迟 2 秒（`G1_NUM7_START_TIMEOUT_S`）触发 start timeout。

停止后 C++ 锁存 force_zero，保持至少 0.5 秒（`G1_NUM7_SETTLE_S`），然后自动返回 `RLFSMStateRLRoboMimicLocomotion`。手动退出：`P`/`LB+X` → Passive，`Num9`/`B` → GetDown，`Num1`/`RB+DPadUp` → 取消并回状态 1。

> **模型复用（行为 A）**：Num7 只在状态 1 可达，进入时若 `robomimic/locomotion` 策略未加载则**拒绝 Num7 并转 Passive**，绝不在 Num7 Enter/控制周期内现场加载模型（避免阻塞控制循环）。Num1 → Num7 → Num1 复用已加载模型，不重复 InitRL。

### 联合仿真（正式 Num7 仿真入口）

```bash
cd ~/g1_running && UNITREE_ROOT=/home/ubuntu G1_RUNNING_ROOT=/home/ubuntu/g1_running \
  bash vision/scripts/run_num7_full_demo.sh
```

按 `0` → `1` → `7`。脚本记录 MuJoCo 实际起始与终止 qpos；Num7 结束后立即停止发送非零命令。

真机放行前必须再运行严格的双任务 headless 门禁：

```bash
cd ~/g1_running
bash vision/scripts/num7_headless_smoke.sh
```

门禁要求两次 Num7 都完成握手、锁线、释放约束、停止并保持零命令，且每次 MuJoCo 实测 qpos 前进必须落在目标 1 m 的开环容差 0.80～1.20 m。只有全部通过才生成
`vision/output/num7_headless_smoke/HARDWARE_RELEASE_READY`。新的门禁运行会先删除旧标记，失败时不会留下可用标记。

> **配置变更（2026-08-15）**：旧版 `0.10 m/s / 0.50 m` 配置落在现有 policy 的低速死区，已改为训练范围内的 `0.50 m/s / 1.00 m`。根据当前 policy 两次 MuJoCo 实测，命令积分采用显式 `G1_NUM7_DISTANCE_SCALE=0.60` 标定；它仍不是里程计，是否允许真机以严格门禁的两次实际 qpos 结果为准。

> **当前仿真放行状态（2026-08-12）**：严格连续两次门禁通过，FSM 停止时 MuJoCo 实际位移分别为 `0.894 m`、`1.121 m`；两次停止后一秒均为零命令，残余约束力为 0。已生成 `HARDWARE_RELEASE_READY`。这只解除软件/仿真门禁，不代替 Jetson 同步编译、D435i audit、吊起测试和地面人工测距。

### 真机 audit-only（不运动）

```bash
bash vision/scripts/run_g1_realsense_audit.sh
```

### 真机吊起测试

```bash
# 1) 先在另一终端启动视觉（默认拒绝运动输出）
bash vision/scripts/run_g1_num7_active.sh
```

该脚本默认 `G1_NUM7_COMMAND_OUTPUT_ENABLED=0`，拒绝启动。只有显式：

```bash
G1_ROBOT_INTERFACE=<已核验的真机DDS网卡> \
G1_NUM7_COMMAND_OUTPUT_ENABLED=1 \
  bash vision/scripts/run_g1_num7_active.sh
```

并且严格仿真门禁标记存在时，才会开启命令输出。脚本拒绝空网卡、`lo` 和不存在的网卡。**机器人必须吊起、现场必须有物理急停**，然后由你单独启动 `rl_real_g1` 并手动按 `0 → 1 → 7`。

### 地面分阶段测试

```bash
G1_ROBOT_INTERFACE=<真机DDS网卡> G1_NUM7_COMMAND_OUTPUT_ENABLED=1 G1_NUM7_TARGET_M=0.10 bash vision/scripts/run_g1_num7_active.sh
G1_ROBOT_INTERFACE=<真机DDS网卡> G1_NUM7_COMMAND_OUTPUT_ENABLED=1 G1_NUM7_TARGET_M=0.25 bash vision/scripts/run_g1_num7_active.sh
G1_ROBOT_INTERFACE=<真机DDS网卡> G1_NUM7_COMMAND_OUTPUT_ENABLED=1 G1_NUM7_TARGET_M=0.50 bash vision/scripts/run_g1_num7_active.sh
G1_ROBOT_INTERFACE=<真机DDS网卡> G1_NUM7_COMMAND_OUTPUT_ENABLED=1 G1_NUM7_TARGET_M=1.00 bash vision/scripts/run_g1_num7_active.sh
```

## 测试

```bash
conda activate g1race
cd vision
python -m unittest discover -s tests -v
```

125 项测试覆盖严格双线模式、单线拒绝、暗光/色偏、运动模糊、完整相机姿态补偿、不可漂移的相邻跑道硬锁、中心走廊直跑、持续外漂提前确认、纠偏回差、任务重置、视觉丢失保护、终点减速、最新策略 SHA-256、Skill 7 检测器生命周期与安全链，以及事件驱动 Skill 6 起步。

## 仿真结果

| 指标 | 联合仿真结果 |
| --- | ---: |
| 连续两次 100 m 用时 | 29.17–44.32 s |
| 平均前进速度 | 2.24–3.43 m/s |
| 最大速度指令 | 4.54–4.58 m/s |
| 双白线有效帧比例 | 99.2%–99.3% |
| 百米段最大骨盆横向偏移 | 0.543–0.874 m |
| 完全停止位置 | 100.67–101.88 m |
| 跌倒 / 相邻跑道切换 / 身份锁丢失 | 0 / 0 / 0 |

上表属于优化前的历史基线，不能作为本次版本的成绩。当前默认配置为 gait-v2 `model_175197`、`0.35 rad/s` 转向上限、`3.00 m/s²` 指令加速度、状态 1 事件确认和 0.60 秒 Skill 6 相机稳定窗；必须重新跑完整 MuJoCo 回归，实机仍须从低速分阶段验证。

## 实机迁移

`scripts/run_realsense_ros2.py` 可订阅 RealSense ROS 2 彩色图与对齐深度，并复用相同控制器。当前完整姿态单应补偿已在 MuJoCo 相机路径验证；实机入口仍需接入 G1 或 D435i 的同步姿态并完成时间戳/外参标定，不能把仿真矩阵直接照搬。真机首次测试不能直接使用 5.10 m/s，应先完成相机外参/曝光标定、架空测试和急停验证，再按 5 m、10 m、20 m、50 m、100 m 分阶段提速，并增加 roll/pitch、通信超时和越线风险的独立安全监控。

## 参考

- [Unitree Robotics / unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
- [MuJoCo Python API](https://mujoco.readthedocs.io/en/stable/python.html)
- [RealSense ROS](https://github.com/realsenseai/realsense-ros)
