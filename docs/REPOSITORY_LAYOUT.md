# 仓库结构与文件归属

本文约定 `g1_running` 中源码、部署资产、实验工具和生成物的边界。目标是让训练、
仿真、视觉和真机部署可以共存，同时避免日志、dashboard、编译产物和本机报告污染
Git 状态。

## 顶层目录

| 路径 | 职责 | 主要维护内容 |
|------|------|--------------|
| `robot_rl/` | 训练与 sim2sim | Isaac Lab 环境、奖励、命令、训练/导出脚本、轨迹和机器人资产 |
| `rl_sar/` | 策略部署 | C++ FSM、RL SDK、MuJoCo、Unitree 接口和部署 policy |
| `vision/` | 视觉任务 | 白线检测、控制器、深度安全、Skill 6/7 生命周期、仿真及真机脚本 |
| `docs/` | 仓库级文档 | 目录规范、协作说明和跨模块设计文档 |

各模块的使用说明分别位于：

- `robot_rl/README.md`
- `rl_sar/README.md`
- `vision/README.md`
- `vision/scripts/README.md`
- `robot_rl/scripts/README.md`

## 文件应该放在哪里

### 训练代码

- 环境配置：`robot_rl/source/robot_rl/robot_rl/tasks/`
- 训练/导出入口：`robot_rl/scripts/rsl_rl/`
- 一次性但可复用的数据工具：`robot_rl/scripts/`
- sim2sim：`robot_rl/transfer/sim/`
- 训练生成的 checkpoint、TensorBoard 和临时导出不要放到源码目录。

### 部署代码

- FSM 和机器人状态：`rl_sar/src/rl_sar/fsm_robot/`
- 公共运行时：`rl_sar/src/rl_sar/library/core/rl_sdk/`
- 视觉 UDP 协议：`rl_sar/src/rl_sar/include/vision_udp_command.hpp`
- 当前部署策略：`rl_sar/policy/g1/<policy-name>/`
- `rl_sar/cmake_build/`、`install/`、`log/` 和 `backups/` 都是本地产物。

### 视觉代码

- 算法模块：`vision/*.py`
- 包兼容层：`vision/g1_race_vision/`
- 可执行入口：`vision/scripts/`
- 自动测试：`vision/tests/`
- 第三方/部署集成：`vision/integrations/`
- 需要长期保留并进入评审的真机证据：`vision/docs/<topic>/`
- 可再生成的 dashboard：`vision/debug_output/`
- 仿真日志与运行产物：`vision/output/` 或 `vision/logs/`

## 脚本命名约定

- `run_*`：运行任务或节点。
- `start_*` / `stop_*`：管理后台或 GUI 生命周期。
- `install_*`：安装依赖，会修改系统；必须手动执行并阅读脚本。
- `build_*`：构建已有源码。
- `smoke_test_*` / `*_smoke`：无完整验收含义的快速回归。
- `view_*`：只读查看工具。

新增脚本后，应同时加入对应的 `scripts/README.md`，注明运行平台、是否需要 root、
是否可能发送机器人命令。

## Git 与生成物规则

以下内容不应提交：

- `__pycache__`、`.pytest_cache`；
- `cmake_build`、`build`、`install`、运行日志；
- `vision/debug_output`、`vision/output`；
- 本机环境审计报告；
- 手工备份压缩包；
- 本地工具配置和上下文缓存。

例外：需要作为问题证据进入代码评审的图片和日志，应复制到
`vision/docs/<topic>/`，附 README、采集背景和哈希清单，而不是直接提交整个输出
目录。

## Policy 管理

Policy 文件名容易重复，引用时必须同时记录：

1. 训练运行目录；
2. checkpoint 迭代号；
3. SHA256；
4. 对应 config/parameter 文件；
5. MuJoCo 与真机验收状态。

部署 policy 的替换应作为独立提交，不要与大范围代码整理混在同一个提交中。

## 推荐的提交边界

- 训练奖励/命令修改；
- policy 更新；
- C++ 部署修改；
- 视觉算法修改；
- 真机证据和审计报告；
- 文档与仓库整理。

这些类别尽量分别提交，便于回滚和定位真机行为变化。
