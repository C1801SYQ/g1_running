# 仓库整理清单

本文记录当前仓库的整理边界和日常清理建议。目标是让源码、部署资产和可复现实验材料留在 Git 中，把本机运行产物、临时备份和外部克隆留在工作区外。

## 当前建议

| 类型 | 当前示例 | 建议处理 |
|------|----------|----------|
| 外部仓库克隆 | 工作区中的外部仓库目录 | 保持未跟踪；如需长期引用，改为文档链接、submodule 或单独 workspace |
| Office 临时锁 | `.~lock.附件1-参赛队伍系统架构与通信链路信息备案表.xlsx#` | 可删除；已加入 `.gitignore` |
| 评测输出 | `robot_rl/evaluations_skill5_battery_20260816/` | 保持未跟踪；只把关键 JSON 摘要或结论复制进 `docs/` |
| policy 备份 | `rl_sar/policy/g1/running/backup_*/`、`policy.pt.bak_*` | 保持未跟踪；部署 policy 替换应独立提交并记录 SHA256 |
| 编译/运行产物 | `rl_sar/cmake_build/`、`rl_sar/install/`、`rl_sar/log/`、`vision/output/` | 保持未跟踪；需要复盘时归档到专题 docs |
| 顶层日志 | `sim_vision.log` | 保持未跟踪；若是证据，移动到 `vision/docs/<topic>/` 并附采集说明 |

## 推荐工作流

1. 新增源码、脚本或文档前，先确认归属模块：`robot_rl/`、`rl_sar/`、`vision/` 或 `docs/`。
2. 生成的 checkpoint、`.npz`、日志、dashboard 和临时备份默认不提交。
3. 真机或仿真证据需要进入评审时，建立 `vision/docs/<topic>/README.md`，只提交压缩后的关键图片、摘要和哈希清单。
4. 替换 `rl_sar/policy/g1/running/policy.pt` 时，单独提交 policy、配置、SHA256 和验收记录。
5. 提交前运行：

```bash
git status --short
git diff --stat
pytest vision/tests
```

## 本次未自动处理的项目

为了避免误删用户资料，本次只更新忽略规则和文档，没有删除未跟踪文件。确认不需要后，可手动清理：

```bash
rm '.~lock.附件1-参赛队伍系统架构与通信链路信息备案表.xlsx#'
```

较大的外部仓库目录、`robot_rl/evaluations_skill5_battery_20260816/` 和 `rl_sar/policy/g1/running/backup_*/` 建议先确认是否仍需本地保留，再移动到仓库外部或删除。
