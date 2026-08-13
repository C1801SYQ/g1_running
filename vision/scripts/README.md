# Vision 脚本索引

脚本保留在单层目录，避免改变已经用于 Jetson、VM 和文档的稳定路径。本页按用途
分类，并标明是否可能产生机器人运动。

## 日常入口

| 脚本 | 平台 | 作用 | 运动风险 |
|------|------|------|----------|
| `run_vm_full_demo.sh` | x86/VM | Skill 6 联合 MuJoCo + 视觉仿真 | 仅仿真 |
| `run_num7_full_demo.sh` | x86/VM | Skill 7 联合仿真 | 仅仿真 |
| `run_g1_realsense_audit.sh` | Jetson | D435i + 深度安全 + audit bridge | 不发送机器人命令 |
| `run_g1_num7_active.sh` | Jetson | Skill 7 真机视觉输出 | **可运动，需要显式 opt-in** |
| `run_g1_num7_print_controller_cmd.sh` | Jetson | 打印匹配的控制器命令 | 不启动任何进程 |
| `view_num7_camera.py` | Jetson/NoMachine | 实时查看带标注画面 | 只读 |

## 节点与仿真组件

| 脚本 | 说明 |
|------|------|
| `run_realsense_ros2.py` | RealSense ROS2 白线/深度/命令节点 |
| `run_unitree_camera_sim.py` | MuJoCo 相机、视觉和任务编排 |
| `run_standalone_mujoco_demo.py` | 独立 MuJoCo 视觉演示 |
| `prepare_unitree_scene.py` | 准备 Unitree MuJoCo 场景 |
| `num7_pty_driver.py` | Num7 自动按键/PTY 测试驱动 |

## 构建与快速测试

| 脚本 | 说明 |
|------|------|
| `build_skill6.sh` | 校验 policy 后构建 `rl_real_g1` |
| `smoke_test_g1_running.sh` | g1_running 快速回归 |
| `num7_headless_smoke.sh` | Num7 双任务 headless 位移与停止门禁 |
| `run_g1_realsense_dry_run.sh` | 相机和视觉 dry-run，强制关闭机器人输出 |

## 环境与安装

以下脚本会修改系统、网络或运行环境，执行前必须阅读脚本：

| 脚本 | 权限 | 说明 |
|------|------|------|
| `install_g1_running.sh` | 普通用户 | 安装/准备固定版本 g1_running |
| `install_cyclonedds_runtime.sh` | 视环境而定 | 安装 CycloneDDS 运行时 |
| `install_ros_humble_g1.sh` | root | Jetson ARM64 安装 ROS 2 Humble 感知依赖 |
| `robot_internet_nat.sh` | root | 启用、检查或移除机器人网段 NAT 规则 |
| `activate_g1race_dds.sh` | 普通用户 | 配置比赛 DDS 环境 |

`robot_internet_nat.sh` 使用前应显式设置 `ROBOT_LAN_INTERFACE`、
`ROBOT_WAN_INTERFACE` 和 `ROBOT_SUBNET`，不要依赖默认网卡名。

## VM GUI

- `start_vm_gui.sh`
- `stop_vm_gui.sh`

## 安全规则

1. `audit`、`dry_run`、`view` 不得启动真实控制器。
2. 真机 active 脚本必须保留显式 opt-in、release marker 和物理 DDS 网卡检查。
3. `rl_real_g1` 与视觉终端的 Num7 参数必须一致。
4. 首次测试必须吊起机器人，实体急停在手边。
