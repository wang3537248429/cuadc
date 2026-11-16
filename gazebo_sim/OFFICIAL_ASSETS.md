# 官方仿真资产

## 固定翼外观模型

渲染使用的机体基于官方 `ArduPilot/SITL_Models` 仓库发布的
**Mini Talon V-Tail**：

- 仓库：`https://github.com/ArduPilot/SITL_Models`
- 固定提交：`25bc38ed8c6c0345840159a8cbc0b02781d52f3c`
- 上游路径：`Gazebo/models/mini_talon_vtail`
- ROS 包许可证声明：`GPL-3.0`
- 上游模型作者：Alessandro Bacchini、Seth Schaffer、Rhys Mainwaring
- 原始 CAD 归属与链接保存在上游 `model.config`

上游 1.14 米翼展网格按 `160 / 57`（`2.807017544`）统一缩放到本项目的
3.2 米翼展。项目另外增加双电机、标定质量与惯量、简化碰撞体、ArduPlane
接口、传感器、水瓶和红色飘带。本项目不会把上游资产表述为 CUADC 原创网格。

项目只在 `models/cuadc_twin/meshes` 中保留运行所需的 `.dae`
外观文件和 ArduPilot 标志。物理碰撞体有意采用基本几何体，以保持 500 Hz
求解稳定。项目不保留完整 `SITL_Models` 仓库；固定提交、归属说明和逐文件
哈希可用于审计并复现所选资产，无需附带无关的 Gazebo 与 RealFlight 模型。

## 世界材质

机场 UV 网格和单张草地颜色贴图基于 `ArduPilot/ardupilot_gazebo` 中的官方
`runway` 模型：

- 仓库：`https://github.com/ArduPilot/ardupilot_gazebo`
- 固定提交：`082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
- 上游路径：`models/runway`
- 仓库许可证：`GPL-3.0`

项目有意不附带法线、位移、粗糙度和环境光遮蔽贴图。单张 2K 颜色贴图在
改善地面观感的同时显著降低显存和纹理采样开销。

`scripts/bootstrap.sh` 会校验 `OFFICIAL_ASSET_SHA256SUMS`，避免残缺或意外
替换的资产被误认为固定官方版本。
