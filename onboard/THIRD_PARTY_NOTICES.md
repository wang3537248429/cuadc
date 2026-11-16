# 实机项目第三方软件与模型声明

本文件只说明 `onboard` 实机项目直接包含或安装的第三方内容。`LICENSE`
以 MIT 授权项目原创代码，不替代第三方软件、训练数据和模型的许可证。

## YOLO、Ultralytics 与模型权重

运行时从 `models/vision/` 加载使用者自行准备的模型，当前源码与 Release 附件不分发权重。Ultralytics、PyTorch、OpenCV、
pymavlink、pyserial、NumPy 和 PyYAML 由安装脚本通过 Python 包索引安装，
不作为第三方源码复制到本项目。

当前锁定的 Ultralytics `8.4.106` 包元数据标明为 AGPL-3.0；使用和再发布
实机项目及模型时必须同时遵守该许可。许可证全文见
[`LICENSES/AGPL-3.0.txt`](LICENSES/AGPL-3.0.txt)。

替换本地权重时需检查
PyTorch 检查点的 `train_args`、Git 信息、自定义元数据和数据集再分发授权。

## ArduPilot 与 Mission Planner

实机项目通过 MAVLink 与用户已有的 ArduPlane 飞控和 Mission Planner 通信，
不包含 ArduPilot、Mission Planner、Gazebo、ROS 或 SITL 源码和二进制。
相关软件仍受各自上游许可证约束。

## 识别标签清单

`assets/targets/catalog.json` 仅保存运行时需要的类别和值映射。识别图片、训练
数据和竞赛资料不进入实机项目；模型发布者仍应确认自己拥有对应训练数据与
权重的使用和再分发权。
