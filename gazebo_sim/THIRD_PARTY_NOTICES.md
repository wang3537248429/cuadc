# 第三方软件与资产声明

本文件说明仓库中直接包含或由安装脚本获取的第三方内容。根目录
`LICENSE` 只授权项目原创代码，不会替代下列第三方内容的许可证。

## ArduPilot Mini Talon V-Tail 模型

- 来源：<https://github.com/ArduPilot/SITL_Models>
- 固定提交：`25bc38ed8c6c0345840159a8cbc0b02781d52f3c`
- 上游路径：`Gazebo/models/mini_talon_vtail`
- 许可证声明：GPL-3.0
- 上游作者：Alessandro Bacchini、Seth Schaffer、Rhys Mainwaring

仓库只包含运行所需的 DAE 网格和 ArduPilot 标志图片。项目统一缩放网格，
并另行增加双电机、固定翼物理、传感器、载荷与简化碰撞体。详细修改见
[`OFFICIAL_ASSETS.md`](OFFICIAL_ASSETS.md)。

## ArduPilot Gazebo 跑道资产与插件

- 来源：<https://github.com/ArduPilot/ardupilot_gazebo>
- 固定提交：`082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
- 许可证：GPL-3.0

仓库包含由上游 `models/runway` 整理的机场网格和草地颜色贴图。安装脚本还会
临时获取固定提交的插件源码，应用 `patches/` 中的补丁后构建，
并把许可证随运行产物保存在 `.deps/runtime/`。

## ArduPilot / ArduPlane

- 来源：<https://github.com/ArduPilot/ardupilot>
- 固定版本：ArduPlane 4.6.3
- 固定提交：`3fc7011a7d3dc047cbb17d8bd98ee94577d144c6`
- 许可证：GPL-3.0

ArduPilot 源码和二进制不直接提交到本仓库。`scripts/bootstrap.sh` 获取固定
提交、构建 SITL、保留许可证与运行二进制，再清理临时源码树。

GPL-3.0 许可证全文保存在 [`LICENSES/GPL-3.0.txt`](LICENSES/GPL-3.0.txt)。

## 视觉训练子项目

`tools/vision_training/` 来源于用户提供的 CUADC Vision System，其代码保留
独立的 MIT 许可证和以下去标识化版权声明：

```text
Copyright (c) 2026 CUADC Vision Contributors
```

详见 [`tools/vision_training/LICENSE`](tools/vision_training/LICENSE)。

## YOLO、Ultralytics 与 Python 依赖

当前公开源码分发两套项目运行权重归档；训练记录和由竞赛图案派生的数据集仍不分发。训练工具保留在 `tools/vision_training/`，其代码许可不自动覆盖输入素材或输出权重。

Ultralytics、PyTorch、OpenCV、PySide6、MAVProxy、pymavlink、NumPy、PyYAML 和 pytest 通过包索引安装，不作为第三方源码复制到仓库。版本见 `pyproject.toml` 与 `requirements.lock.txt`。Ultralytics 8.4.106 包元数据标注 AGPL-3.0；使用、修改和分发时应遵守对应条款，项目 MIT 许可不替代这些义务。AGPL 全文见 [`LICENSES/AGPL-3.0.txt`](LICENSES/AGPL-3.0.txt)。

## Gazebo、ROS 2 与 Mission Planner

Gazebo Garden、ROS 2 Humble 和 ROS/Gazebo 桥通过系统软件源安装，不直接
包含在仓库中。Mission Planner 由用户单独安装，本项目只通过 MAVLink UDP
与其通信。

## 竞赛规则与识别素材

官方竞赛规则 PDF 不随仓库分发；项目仅记录规则版本并实现仿真侧的验收代理。
目标图像和训练样本的再分发许可尚未确认，当前源码和 Release 附件不包含这些资源。公开的运行权重由项目发布者提供；替换权重时应保留来源、许可和修改记录。
