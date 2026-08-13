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

1. 在 `hardware.yaml` 里确认 `camera.backend`、`camera.camera_info`、`serial.port`、`serial.protocol`、`controller.bullet_speed`。
2. 把真实相机标定结果写入 `camera_info.yaml`。
3. 先保持 `hardware.yaml` 的 `safety.enable_fire: false`。
4. 修改配置后先运行 `python scripts/validate_configs.py`。
5. 识别不稳改 `detector.yaml`，跟踪不稳改 `tracker.yaml`，控制和开火判定改 `controller.yaml`。
6. 所有链路稳定后，再考虑打开 `safety.enable_fire`。

## 配置校验

```bash
python scripts/validate_configs.py
```

脚本会检查 YAML 解析、实车分辨率和 `camera_info.yaml` 是否一致、相机外参是否仍为全 0、常见拼写错误（例如 `out_post` / `negtive`）、以及 `tracker.implementation` 是否误写在 `tracker.yaml`。

如果希望警告也导致失败：

```bash
python scripts/validate_configs.py --strict
```

## 容易混淆的点

- `detector.yaml` 里的 tracker 是检测器内部的 2D 跟踪，不是 `tracker.yaml` 的 3D 目标跟踪。
- 普通车跟踪器实现只在 `gimbal_pipeline.yaml` 的 `tracker.implementation` 切换，`tracker.yaml` 只调具体滤波/结构参数。
- `hardware.yaml` 的 `controller.bullet_speed` 是实车弹速覆盖值。
- `hardware.yaml` 的 `camera.backend` 决定实车相机来源：`opencv` 不需要工业相机 SDK；`hik` / `mindvision` 需要编译时打开对应 CMake 开关。
- `hardware.yaml` 的 `serial.protocol` 默认用旧实车 `infantry` 24 字节协议；如果电控确认使用 32 字节，再切到 `infantry_32`。
- `hardware.yaml` 的 `serial.infantry32_tail_fields` 默认用 `duplicate_velocity`，对齐旧实车 32 字节协议；只有电控明确要角加速度时再改成 `acceleration`。
- `simulation.yaml` 的 `controller.bullet_speed` 只服务仿真，不用于实车入口。
- `tracker.yaml` 和 `controller.yaml` 顶部有“常调区 / 进阶区”索引；先按常调区改，不要一上来动后端内部参数。

## 调参速查

| 症状 | 优先看哪里 |
|---|---|
| 相机打不开或选错相机 | `hardware.yaml`：`camera.backend`、`camera.camera_sn`、SDK 编译开关 |
| 串口有数据但解析不到反馈 | `hardware.yaml`：`serial.protocol`、`serial.port`、`serial.baudrate` |
| 画面识别不到装甲 | `detector.yaml`：敌方颜色、置信度、传统灯条阈值、模型路径 |
| PnP 距离或姿态明显错 | `camera_info.yaml`、`detector.yaml` 的 `pose` |
| 目标丢失或轨迹漂 | `tracker.yaml`：生命周期、观测噪声、运动模型、门控 |
| 想切普通车跟踪器实现 | `gimbal_pipeline.yaml` 的 `tracker.implementation` |
| 想切控制策略 | `gimbal_pipeline.yaml` 的 `controller.strategy` |
| 云台跟随慢或提前量不对 | `controller.yaml` 的 `delay` / `solver` |
| 输出角度抖动 | `controller.yaml` 的 `output_filter` |
| 明明选了不该打的目标 | `gimbal_pipeline.yaml` 的 `selector` |
