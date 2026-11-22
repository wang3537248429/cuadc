# CUADC

固定翼任务仿真、机载视觉程序与中文遥测监控工具。仿真由 Gazebo Garden 提供物理与相机画面，ArduPlane SITL 执行地面站上传的任务，YOLO 处理相机帧。

本项目用于 CUADC 竞赛任务验证，仓库发布的是仿真代码：场景、任务流程与视觉处理都在仿真环境中运行，不包含实机参数、现场配置或队伍内部资料。

## 目录与入口

| 目录 | 职责 | 主要入口 |
| --- | --- | --- |
| `gazebo_sim/` | 场景、物理仿真、SITL、视觉与任务记录 | `scripts/run_sim.sh` |
| `onboard/` | 树莓派相机、YOLO、MAVLink 路由与部署 | `scripts/install_onboard.sh` |
| `ground_station/` | 中文遥测监控与 TLOG 回放 | `run.sh` |
| `scripts/` | 跨模块校验、隐私审计和源码导出 | `verify_release.sh` |
| `docs/` | 安装、操作、资源、航线与发布说明 | 下列文档索引 |

三个模块分别管理源码、配置、依赖和测试。生成的日志、虚拟环境和训练数据不进入公开源码；两套经过审核的运行权重归档例外。

## 使用前须知

公开包提供原创代码、配置示例、测试、有明确许可的上游仿真资产和两套运行用 YOLO 权重归档。竞赛靶标图片、衍生数据和训练记录仍不随源码发布，完整视觉仿真还需要使用者按[资源与许可](docs/资源与许可.md)准备靶标图片。

A/B 示例航线已迁移至 ArduPilot 的公开 SITL 原点，保留局部航线形状、相对高度和任务命令。文件名 `official_route_a/b.waypoints` 为兼容原有入口保留，不表示其中包含真实比赛场地位置。

源码测试、完整仿真和实机验收分别记录。源码检查通过不能代替 YOLO 识别、投水落点或实际飞行验收。

## 安装与运行

图形仿真的适配环境为 Ubuntu 22.04 amd64、Python 3.10、ROS 2 Humble 和 Gazebo Garden 7.9.x。Python 完整依赖记录在 `gazebo_sim/requirements.lock.txt`，SITL 与插件提交固定在安装脚本中。脚本按自身文件位置定位仓库，因此可以从任意当前目录调用。

在仓库根目录执行：

```bash
# 安装系统依赖，需要管理员权限
sudo ./gazebo_sim/scripts/install_system_deps.sh

# 完整视觉模式：先按资源说明准备本地靶标，再执行校验与构建
python3 scripts/check_local_resources.py --project gazebo_sim --require
./gazebo_sim/scripts/bootstrap.sh

# 默认显示 Gazebo 和视觉窗口；A、B 分别启动
./gazebo_sim/scripts/run_sim.sh --official-route A
```

公开源码不包含尚未确认再分发许可的靶标图片，但包含两套经过审核的 `.pt.gz` 运行权重。安装脚本会校验归档并解压到本地；若只是要在干净下载上验证 Gazebo、ArduPlane、MAVLink 和场景生成，可以先运行无视觉模式：

```bash
./gazebo_sim/scripts/bootstrap.sh --no-vision
CUADC_HEADLESS=1 ./gazebo_sim/scripts/run_sim.sh --no-vision --seed 1
```

这个模式会用无纹理白板生成可加载的场景，不启动 YOLO，也不代表识别或完整飞行验收通过。准备好有权使用的靶标图片后，去掉 `--no-vision`，脚本会在启动前校验图片和模型的 SHA-256。

Mission Planner 选择 UDP 14550，载入并上传对应的 A/B 航线。任务控制、起飞和航线执行方法见[操作文档](docs/操作文档.md)。结束测试时，在启动终端按 `Ctrl+C`，由 supervisor 关闭本次启动的进程。

## 源码检查

不安装 Gazebo 或视觉模型也可运行源码测试：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e './gazebo_sim[test]' pyserial==3.5
python -m pip install --no-deps -e ./onboard -e ./ground_station
CUADC_SKIP_SDF=1 ./scripts/verify_release.sh
```

资源缺席的专用测试会显示 `skipped`。已有完整环境时执行 `./scripts/verify_release.sh`，同时检查本地资源和 SDF 格式。检查结果保存在 `dist/validation/`。本次整理的验证范围与计数见 [validation.json](docs/validation.json)。

## 文档

- [操作文档](docs/操作文档.md)：环境安装、可视化运行、地面站、完整任务检查与关闭。
- [项目结构](docs/项目结构.md)：目录职责、代码入口、生成文件位置与维护约定。
- [航线配置](docs/航线配置.md)：公开坐标、A/B 场地与本地航线配置。
- [资源与许可](docs/资源与许可.md)：模型、靶标、数据准备与分发范围。
- [开源发布说明](docs/开源发布说明.md)：审计、干净导出与 GitHub 发布步骤。
- [贡献指南](CONTRIBUTING.md)及[第三方声明](THIRD_PARTY_NOTICES.md)。

## 许可证

原创代码和项目运行权重按仓库许可发布；第三方模型、网格、贴图和依赖适用各自许可证；详见各模块的 `THIRD_PARTY_NOTICES.md` 与 `LICENSES/`。本仓库的许可不授予竞赛图案、训练数据或第三方权重的再分发权。
