# Gazebo 仿真开发说明

## 修改前

先在仓库根目录运行 `python3 scripts/privacy_audit.py`，确认工作区没有待发布的本地配置、日志或模型运行产物。仿真依赖和生成文件必须留在 `.venv/`、`.deps/`、`.runtime/` 或 `runs/`，不要提交这些目录。

## 修改后

从仓库根目录运行：

```bash
gazebo_sim/.venv/bin/python -m pytest gazebo_sim/tests -q
gazebo_sim/.venv/bin/python -m pytest scripts/tests/test_release_packages.py -q
```

涉及场景、航线或模型加载时，再执行 `./scripts/verify_release.sh`。涉及真实航线文件时，保留原始文件的 SHA-256 记录，并在提交说明中写明变更原因。

仿真代码只负责模拟和验证，不会自动修改真实飞控参数，也不会把本地场景答案传给 YOLO 推理进程。新增配置应提供公开模板；个人坐标、设备路径和运行日志放在 `.local` 或被忽略的运行目录。
