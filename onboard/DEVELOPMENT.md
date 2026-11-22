# 机载程序开发说明

机载目录面向树莓派 4B 和 Ubuntu 22.04 aarch64。桌面环境只能用于单元测试和离线检查，不能据此判断相机、串口或舵机已经满足实机要求。

## 开发流程

在仓库根目录创建机载虚拟环境并安装依赖后，先运行配置检查：

```bash
onboard/.venv-onboard/bin/python -m pytest onboard/tests -q
onboard/.venv-onboard/bin/cuadc-onboard --config onboard/config/onboard.yaml check --skip-camera
```

连接硬件前核对相机和飞控的 udev 别名、串口波特率、模型 SHA-256 和释放机构通道。生产程序不会负责解锁、切换飞行模式或启动任务；起飞、航线和返航由地面站与飞控执行。

不得提交 TLOG、相机原片、设备序列号、真实坐标或包含训练机路径的检查点。公开发布包含两套已审核运行权重，但不包含训练数据。自行替换模型时，在 `MODEL_SHA256SUMS` 中记录校验值，并保留来源与许可记录。
