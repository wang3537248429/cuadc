# 仓库工具

本目录只保存跨模块工具；仿真与硬件专用脚本分别留在 `gazebo_sim/scripts/` 和 `onboard/scripts/`。以下命令从仓库根目录执行。

| 工具 | 用途 |
| --- | --- |
| `verify_release.sh` | 隐私、资源、Shell、Python 测试和 SDF 统一检查 |
| `privacy_audit.py` | 检查公开文件中的路径、凭据、位置与图片元数据 |
| `release_policy.py` | 定义可公开的文件范围，供审计与导出共用 |
| `build_release_packages.py` | 生成无 Git 历史的目录或源码压缩包 |
| `check_local_resources.py` | 校验本地权重与靶标；`--require` 要求完整资源 |
| `anonymize_mission.py` | 将 WPL 航线迁移到公开 SITL 原点 |
| `verify_precision.py` | 对参数误差预算做离线蒙特卡洛检查 |
| `e2e_reports.py`、`render_validation.py` | 读取或整理本地验收结果 |
| `clean_generated.sh` | 清理运行、构建、缓存与发布产物 |
| `tests/` | 上述仓库工具的测试 |

常用操作：

```bash
python3 scripts/privacy_audit.py
python3 scripts/check_local_resources.py --require
./scripts/verify_release.sh
python3 scripts/build_release_packages.py public --directory dist/cuadc-public
python3 scripts/build_release_packages.py public
```

导出命令拒绝覆盖已有目录。需再次构建时指定新的目录名，或保存旧产物后移走旧目录。发布操作与可用资源范围见[开源发布说明](../docs/开源发布说明.md)。

新下载的仓库可先执行 `./gazebo_sim/scripts/bootstrap.sh --no-vision` 和 `CUADC_HEADLESS=1 ./gazebo_sim/scripts/run_sim.sh --no-vision --seed 1` 验证基础仿真；完整视觉运行仍按资源说明准备本地图片与模型。
