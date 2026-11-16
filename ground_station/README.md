# CUADC 中文监控

读取 MAVLink 遥测与 TLOG，显示任务状态。该工具与 Mission Planner 分别运行；Mission Planner 负责飞控任务上传和飞行控制。

需要 Python 3.10 或以上版本、Tk、pymavlink 与 pyserial。在本目录执行：

```bash
python3 -m pip install -e .
./run.sh
```

Ubuntu 缺少 Tk 时安装 `python3-tk`。Windows 可用 `build_windows.bat` 或 `build_windows.ps1` 构建，构建依赖见 `requirements-build.txt`。测试位于 `tests/`，完整项目操作见完整源码包的 `docs/操作文档.md`。
