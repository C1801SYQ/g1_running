# `g1_running` 跑步策略接入

本目录把 [C1801SYQ/g1_running](https://github.com/C1801SYQ/g1_running)
的 G1 29 自由度 `speed_turn` TorchScript 策略接到本项目的视觉循迹命令。

固定上游版本：

```text
commit: 4d06065aa9445b8af4db5d465fa79f67736e5e36
policy: rl_sar/policy/g1/running/policy.pt
SHA256: e3705c5ce94c32c00a4e1900a5e2024deea3a4e22f7b20fbc268f3bcb7e51e57
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
  完成后自动进入 Running；完整仿真脚本会设置它，真机脚本绝不能设置。
- `g1_running_quiet_run.patch`：默认关闭 200 Hz 的 Running 状态文本，减轻
  虚拟机终端负担；调试时设置 `G1_VERBOSE_RUNNING=1` 可以重新显示。
- 收到第一个视觉包后，如果超过 300 ms 没有新包，输出速度自动归零。
- `speed_turn` 训练时横向速度范围为零，因此这里强制 `vy=0`，通过 `wz`
  转向纠偏。
- 默认把视觉前进速度限制在 1.0 m/s、转向角速度限制在 0.5 rad/s。

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
