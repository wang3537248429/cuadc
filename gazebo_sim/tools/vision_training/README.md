# 视觉训练工具

本目录保留当前 YOLO 训练与离线评估流程。工具依赖仿真模块的类别映射、相机模型和像素预处理，默认路径相对 `gazebo_sim/`。数据、基础模型和靶标图片需自行准备，许可要求见完整源码包中的 `docs/资源与许可.md`。

| 工具 | 职责 |
| --- | --- |
| `prepare_private_panels.py` | 从本地素材准备场景靶板 |
| `gazebo_dataset.py`、`camera_pose.cc` | Gazebo 相机数据采集与相机位姿插件 |
| `pentagon_dataset.py` | 生成五边井视角数据 |
| `augment_dataset.py` | 训练划分内的图像增强 |
| `flight_scale_dataset.py` | 飞行尺度与像素候选区域处理 |
| `prepare_oblique_dataset.py` | 斜视样本处理 |
| `train_pentagon.py` | 当前训练入口，支持 Task 1/2 |
| `evaluate_pixel_pipeline.py` | 离线检查与在线相同的像素推理链路 |
| `evaluate_flight_video.py` | 对录像、时间戳和遥测做离线评估 |
| `publish_models.py` | 清理可信检查点的本地元数据并更新模型文件 |

在仓库根目录查看具体参数：

```bash
gazebo_sim/.venv/bin/python gazebo_sim/tools/vision_training/train_pentagon.py --help
gazebo_sim/.venv/bin/python gazebo_sim/tools/vision_training/evaluate_pixel_pipeline.py --help
```

训练环境使用仿真目录固定依赖。Task 1 对应类别表中的图形类别，Task 2 对应十位与个位类别；旧版整板数字识别模型不能用于当前视觉入口。准备数据时按原始样本来源分离训练、验证和测试划分，保留数据与模型版本记录。

`publish_models.py` 的名称沿用既有工具接口，只执行本地文件整理。源码导出只包含明确列入发布清单的两套运行权重归档，其他输出和训练记录仍会排除；工具不向任何平台上传文件，也不替代授权审核。
