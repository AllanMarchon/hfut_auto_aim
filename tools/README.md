# 工具模块说明

本目录提供普通自瞄和打符链路共用的基础组件，由根目录 `CMakeLists.txt` 中的 `tools` 静态库统一构建。

| 文件 | 作用 |
| --- | --- |
| `ballistic_solver.*`、`trajectory.*` | 计算弹道解、弹着高度和飞行时间。 |
| `camera2gimbal_extrinsic.*` | 从 YAML 读取相机到云台的旋转和平移外参。 |
| `extended_kalman_filter.*` | 提供扩展卡尔曼滤波及创新统计。 |
| `ransac_sine_fitter.*` | 使用 RANSAC 拟合正弦运动参数。 |
| `math_tools.*` | 提供角度、坐标变换、旋转和时间差等数学函数。 |
| `pid.*` | 提供带输出限幅的 PID 控制器。 |
| `img_tools.*` | 提供图像调试绘制函数。 |
| `logger.*` | 提供项目统一的 spdlog 日志器。 |
| `yaml.hpp` | 提供 YAML 文件加载和必需字段读取辅助函数。 |

新增或调整公共组件时，请同步更新根目录 `CMakeLists.txt` 中 `tools` 目标的源文件列表，并保持调用方使用的单位和坐标约定一致。
