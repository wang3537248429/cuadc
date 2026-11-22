# 本地视觉模型

公开源码包含两套经过审核的运行权重压缩包（`.pt.gz`），安装脚本会先校验解压后的 SHA-256，再生成本地 `.pt` 文件。训练数据、训练记录和 `yolov8n_pretraining.pt.gz` 不进入公开发布。

| 文件 | 用途 |
| --- | --- |
| `task1_v4m_best.pt` | Task 1 图形类别识别 |
| `task2_sim_digits_v1.pt` | Task 2 十位、个位识别 |
| `task1_v4m_best.pt.gz` | 公开发布的 Task 1 权重归档 |
| `task2_sim_digits_v1.pt.gz` | 公开发布的 Task 2 权重归档 |
| `MODEL_SHA256SUMS` | 运行权重校验值 |
| `ARCHIVE_SHA256SUMS` | 本地历史压缩副本的校验值 |

如替换为自己的模型，请保留原始归档并更新清单、类别和值映射。完整仓库根目录执行：

```bash
python3 scripts/check_local_resources.py --project gazebo_sim --require
```

校验清单用于确认版本，不构成许可证明。替换模型后应同步更新清单并重新完成在线识别和完整仿真验收。准备方法见完整源码包中的 `docs/资源与许可.md`。
