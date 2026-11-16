# CUADC 仿真模块

Gazebo Garden 物理与相机仿真，配合 ArduPlane SITL 执行 Mission Planner 上传的航线。源码在 `src/cuadc_sim/`，配置在 `config/`，安装和启动入口在 `scripts/`。

适配环境：Ubuntu 22.04 amd64、Python 3.10、ROS 2 Humble、Gazebo Garden 7.9.x。固定 Python 依赖见 `requirements.lock.txt`。

本目录下执行：

```bash
sudo ./scripts/install_system_deps.sh
# 先按 models/vision/README.md 与 assets/targets/README.md 准备合法本地资源
./scripts/bootstrap.sh
./scripts/run_sim.sh --official-route A
```

Mission Planner 使用 UDP 14550，上传 `config/missions/official_route_a.waypoints`；B 区需同时更换场地参数和航线。公开航线使用 SITL 坐标。按 `Ctrl+C` 关闭本次启动的进程。

完整使用说明在完整源码包的 `docs/操作文档.md`。第三方资产与依赖说明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。当前源码不分发靶标图片、训练数据与 YOLO 权重；资源齐全后才能运行完整视觉仿真。
