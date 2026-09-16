# Release workflow / 工程验证入口

本仓库包含训练、部署和视觉任务三条链路。首先选择验证层级，避免仅为阅读项目而安装全部 GPU 依赖。

## CPU 软件回归

Python 3.11+，无需连接机器人。

```bash
python -m pip install pytest pyyaml "numpy>=1.24,<2" "opencv-python>=4.8,<4.12"
python -m pytest -q
```

根目录 pytest 配置只收集 `vision/tests`，避免误运行第三方 SDK 或训练脚本中的测试入口。

## 发布文件审计

```bash
git lfs pull
python tools/check_release.py
```

工具只读取部署 policy、source 元数据与配置，输出 SHA256、维度和关节映射检查结果。
缺失文件、LFS 指针或维度不一致返回非零退出码。它不会载入或执行 policy，不连接硬件。
SHA256 用于留存与比对，不是对策略行为的认证。
`source.txt` 中训练 checkpoint / deploy 的来源路径可能指向未随公开仓库发布的本地文件；
公开策略可用不等于能完整复现原训练，应先检查来源资产是否齐全。

## 仿真与实机

- 训练依赖和命令：[`../robot_rl/README.md`](../robot_rl/README.md)。
- 部署构建：[`../rl_sar/README.md`](../rl_sar/README.md)。
- Skill 6/7 相机、视觉与状态机流程：[`../vision/README.md`](../vision/README.md)。
- 在相同配置下留存 sim2sim 日志，再按既有吊装、急停、低速流程进行实机验证。

本轮仅验证 CPU 视觉软件回归与发布文件一致性；未重新训练、未进行 C++ 全量构建或真机验收。
主页的历史速度/时间指标仍属于原有实验记录，不能视为本轮新结果。
