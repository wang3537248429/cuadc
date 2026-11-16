# CUADC 机载模块

树莓派 4B 机载视觉、任务记录与 MAVLink 路由程序。源码在 `src/cuadc_onboard/`，设备与算法配置在 `config/`，部署服务在 `systemd/` 与 `udev/`。

安装器适配 Ubuntu 22.04 aarch64，使用普通用户在本目录执行：

```bash
./scripts/install_onboard.sh
# 重新登录，使 dialout/video 用户组生效
.venv-onboard/bin/cuadc-onboard --config config/onboard.yaml check
```

安装前先准备有权使用的本地模型并核对 `models/vision/MODEL_SHA256SUMS`。源码不附带模型或训练数据。设备串口、相机标定和执行机构参数需在本地配置，不能沿用示例推定硬件合格。

开发方法见 [DEVELOPMENT.md](DEVELOPMENT.md)，依赖许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。完整源码包的 `docs/操作文档.md` 提供统一环境与验证说明。
