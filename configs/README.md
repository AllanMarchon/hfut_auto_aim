# 配置说明

本目录保存实车运行需要的主要配置。硬件参数、相机内参、视觉算法参数和控制输出参数分开维护，避免上车调参时互相覆盖。

| 文件 | 作用 |
| --- | --- |
| `hardware.yaml` | 实车硬件配置：相机、串口、弹速、安全开火、相机到枪管外参 |
| `camera_info.yaml` | 相机内参、畸变参数和图像尺寸 |
| `standard3.yaml` | 视觉算法配置：模型、OpenVINO device、颜色、ROI、传统检测、Tracker、Aimer、Shooter |
| `controller.yaml` | 控制输出配置：Planner/MPC、速度和加速度语义、反馈对齐、限幅、fire gate |

## 运行时配置

`src/standard.cpp` 启动后会读取 `hardware.yaml` 和 `camera_info.yaml`，再把相机内参、畸变、相机到枪管外参、敌方颜色和可选 OpenVINO device 写入 `build/sp25_runtime.yaml`。视觉算法模块实际读取这个运行时 YAML。

这样做的目的是把上车经常变化的硬件参数集中在 `hardware.yaml`，让 `standard3.yaml` 主要负责算法参数。换相机、换串口、改外参时，一般不需要到算法配置里到处同步。

## 常调顺序

1. 相机打不开或 FPS 异常：先看 `hardware.yaml` 的 `camera` 段、MVS SDK 路径、USB3 连接、曝光和分辨率。
2. 图像识别效果差：先看 `standard3.yaml` 的 `enemy_color`、`device`、`yolo_name`、`min_confidence`、`threshold`、`use_roi`。
3. PnP 距离或角度明显不对：先看 `camera_info.yaml`、图像分辨率、`flip_image` 和 `hardware.camera.camera_to_barrel`。
4. 云台速度或加速度语义不对：先看 `controller.yaml` 的 `planner`、`mpc_planner`、`serial_command` 和 `command_limiter`。
5. 串口无反馈或云台不动：先看 `hardware.serial` 的端口、波特率、收发协议、角度单位和 timeout。
6. 不开火：先确认 `hardware.safety.enable_fire`、启动命令是否带 `--allow-fire`，再看 `controller.yaml` 的 `fire_gate` 和日志里的 `sp_fire/fire/gate`。

## 控制输出参数

`controller.yaml` 是上车调试最容易误改的文件，建议按现象分组理解：

- `planner.mode`：选择主控制链路。`mpc` 使用 TinyMPC 轨迹规划；Aimer 相关模式可用于回退对照。
- `mpc_planner`：MPC 目标偏置、开火阈值、预测延迟、最大角加速度和 Q/R 权重。
- `serial_command`：定义下发给电控的速度语义。当前常用 `feedback_error`，表示速度前馈包含 `cmd - fb` 的追踪误差补偿。
- `command_limiter` / `output_filter`：限制异常跳变，避免检测或 PnP 抖动直接打到下位机。
- `feedback_alignment`：用图像时间戳对齐下位机反馈，运动中可以减少 world 坐标抖动。
- `fire_gate`：最终开火门控，要求下发命令和反馈角度已经足够接近。

串口收发单位固定为 rad / rad/s / rad/s^2。Web 可视化和控制台日志会把角度类数据转成度制，方便上车观察。

## 校验命令

```bash
python3 scripts/validate_configs.py
python3 scripts/start.py --mode check
```

`--mode check` 会额外打印系统环境、OpenCV、CMake、MVS 和 USB 相机检测信息，适合上车前快速排查。
