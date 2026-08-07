# `g1_running` 跑步策略接入

本目录把 [C1801SYQ/g1_running](https://github.com/C1801SYQ/g1_running)
的 G1 29 自由度 `steady_upper_v2` TorchScript 策略接到本项目的视觉循迹命令。

固定上游版本：

```text
commit: d6d13fb0541a94aac9c8e3340ca8c6470de389fa
policy version: steady_upper_v2 / model_89996
policy: rl_sar/policy/g1/running/policy.pt
SHA256: 167b444f7404a21a4751336b8f7e54c5b0b7cd4b7c979b5ad895288867330305
license: Apache-2.0
```

## 改了什么

- `vision_udp_command.hpp`：只监听本机 `127.0.0.1:15001`，接收
  `vx vy wz`。
- `g1_running_vision.patch`：让 `rl_real_g1` 优先使用视觉命令，并补上
  `GetUp -> Running` 的状态切换。
- `g1_running_quiet_getup.patch`：关闭 200 Hz 起身进度条，避免虚拟机终端
  输出阻塞控制循环；不改变起身轨迹、增益或策略权重。
- `g1_running_sim_autostart.patch`：只有设置 `G1_AUTO_RUNNING=1` 时，起身
  完成后自动进入原 Skill 5 Running；视觉 Skill 6 启动脚本不会设置它。
- `g1_running_quiet_run.patch`：默认关闭 200 Hz 的 Running 状态文本，减轻
  虚拟机终端负担；调试时设置 `G1_VERBOSE_RUNNING=1` 可以重新显示。
- `g1_running_skill6.patch`：注册视觉百米冲刺 Skill 6。它只允许从状态 1
  `RLFSMStateRLRoboMimicLocomotion` 按 `6` 进入；Passive、GetUp、
  GetDown 和 Skill 5 中的 `6` 均不会触发自动起立或冲刺。
- Skill 6 真正进入/退出时，C++ 通过本机 UDP 端口 `15002` 通知 MuJoCo；
  仿真安全支撑据此等待操作员完成 `0 → 1 → 6`，随后先卸载垂直支撑，
  等真实相机姿态稳定并锁定双线后才完全释放。
- 收到第一个视觉包后，如果超过 300 ms 没有新包，输出速度自动归零。
- 当前巡线控制器保持 `vy=0`，只通过 `wz` 转向纠偏；策略本身仍保留上游的
  横移与转向训练能力。
- 仿真默认最大前进指令为 5.1 m/s，视觉转向角速度限制为 0.8 rad/s；实机部署前必须从低速重新验证。

## 安装和编译

在 Ubuntu 中：

```bash
cd ~/g1_race_vision
bash scripts/install_g1_running.sh
```

安装脚本会克隆固定版本、应用补丁、校验策略文件，并只编译 G1 控制器
（不编译无关的 A1/Go2 目标）。编译产物是：

```text
~/unitree_ws/g1_running/rl_sar/cmake_build/bin/rl_real_g1
```

需要临时提高仿真速度上限时，可以在启动前设置：

```bash
export G1_VISION_MAX_VX=1.5
```

允许范围最高为策略训练范围 5.1 m/s，但真机不应直接使用高值。
