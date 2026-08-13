# robot_rl 脚本索引

## 主训练流程

当前项目的主要训练和导出入口位于 `rsl_rl/`：

| 脚本 | 作用 |
|------|------|
| `rsl_rl/train_policy.py` | Isaac Lab PPO 训练入口 |
| `rsl_rl/play_policy.py` | 回放、评估和导出 policy |
| `rsl_rl/export_to_hardware.py` | 导出硬件部署资产 |
| `rsl_rl/export_parameters.py` | 导出策略参数 |
| `rsl_rl/plot_trajectories.py` | 轨迹诊断和可视化 |
| `rsl_rl/plot_summary_from_ckpt.py` | checkpoint 摘要 |

## 其他训练后端

- `rl_games/`：RL-Games 训练/回放。
- `skrl/`：SKRL 训练/回放。
- `sb3/`：Stable-Baselines3 训练/回放。

除非实验明确指定其他后端，G1 29-DOF running 使用 `rsl_rl/`。

## 通用工具

| 脚本 | 作用 |
|------|------|
| `list_envs.py` | 列出已注册环境 |
| `play.sh` | 简化回放入口 |
| `mount_remote.sh` | 挂载远端工作目录 |
| `random_agent.py` | 随机动作基线 |
| `zero_agent.py` | 零动作基线 |
| `csv_to_npz_g1.py` | 将 G1 CSV 动作重采样为 NPZ；依赖 Isaac Lab 和 `whole_body_tracking` |
| `env_check/check_g1_rl_env.sh` | 生成本机 GPU/CUDA/Python/Isaac Lab/RealSense 环境报告 |

`env_check/*.txt` 是机器特定输出，已忽略；需要共享时应删去本机路径、进程和硬件
标识，再复制到专题文档目录。

## 新脚本约定

- 训练入口放到对应后端子目录。
- 可复用的数据转换和环境检查工具放在 `scripts/`。
- 临时 notebook、一次性日志和运行报告不要提交。
- 新工具必须在本索引中注明依赖、输入和输出。
