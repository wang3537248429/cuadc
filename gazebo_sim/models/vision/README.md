# 本地视觉模型

公开源码仅保留说明与 SHA-256 清单，不包含权重、压缩权重或训练记录。现有权重涉及尚未确认再分发许可的素材，本次发布不提供下载附件。

| 文件 | 用途 |
| --- | --- |
| `task1_v4m_best.pt` | Task 1 图形类别识别 |
| `task2_sim_digits_v1.pt` | Task 2 十位、个位识别 |
| `MODEL_SHA256SUMS` | 运行权重校验值 |
| `ARCHIVE_SHA256SUMS` | 本地历史压缩副本的校验值 |

自行取得有权使用的模型后放到本目录，并核对类别和值映射。完整仓库根目录执行：

```bash
python3 scripts/check_local_resources.py --project gazebo_sim --require
```

校验清单用于确认版本，不构成许可证明。替换模型后应同步更新清单并重新完成在线识别和完整仿真验收。准备方法见完整源码包中的 `docs/资源与许可.md`。
