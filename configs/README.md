# 配置文件说明

上车调试时优先从 `hardware.yaml` 开始看。它负责真实硬件入口，其他文件按算法模块拆分。

| 文件 | 什么时候改 |
|---|---|
| `hardware.yaml` | 实车相机来源、串口端口、相机到枪管外参、弹速、安全开火开关 |
| `camera_info.yaml` | 相机内参和畸变；每次重新标定相机后更新这里 |
| `detector.yaml` | 装甲板识别、置信度、PnP 和传统灯条检测参数 |
| `tracker.yaml` | 3D 目标跟踪、关联、丢失保持、车体结构估计参数 |
| `controller.yaml` | 云台控制、弹道补偿、延迟补偿、开火判定参数 |
| `gimbal_pipeline.yaml` | 总控开关，例如 tracker/controller 策略和仿真输入模式 |
| `simulation.yaml` | Webots/Gestalt 仿真专用配置；实车调试通常不用改 |

## 实车调试顺序

1. 在 `hardware.yaml` 里确认 `camera.camera_info`、`serial.port`、`controller.bullet_speed`。
2. 把真实相机标定结果写入 `camera_info.yaml`。
3. 先保持 `hardware.yaml` 的 `safety.enable_fire: false`。
4. 识别不稳改 `detector.yaml`，跟踪不稳改 `tracker.yaml`，控制和开火判定改 `controller.yaml`。
5. 所有链路稳定后，再考虑打开 `safety.enable_fire`。

## 容易混淆的点

- `detector.yaml` 里的 tracker 是检测器内部的 2D 跟踪，不是 `tracker.yaml` 的 3D 目标跟踪。
- `hardware.yaml` 的 `controller.bullet_speed` 是实车弹速覆盖值。
- `simulation.yaml` 的 `controller.bullet_speed` 只服务仿真，不用于实车入口。
