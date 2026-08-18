# 配置说明

当前 `test` 分支已经切到 HFUT 适配版 SP25 主链，配置入口只保留实车需要的三类文件。

| 文件 | 作用 |
| --- | --- |
| `standard3.yaml` | SP25 算法配置：模型、OpenVINO device、颜色、ROI、传统检测、tracker、aimer、shooter |
| `hardware.yaml` | HFUT 实车硬件配置：海康相机、OpenCV 备用相机、infantry 串口、弹速、安全开火 |
| `controller.yaml` | 串口输出前的云台命令限幅：最大角速度、最大角加速度、相对反馈角限幅、限幅后开火门控 |
| `camera_info.yaml` | 相机内参、畸变参数和图像尺寸 |

## 运行时配置

`src/standard.cpp` 启动后会读取 `hardware.yaml` 和 `camera_info.yaml`，再把相机内参、畸变、相机到枪管外参、敌方颜色和可选 OpenVINO device 写入 `build/sp25_runtime.yaml`。SP25 的 `YOLO/Solver/Tracker/Aimer/Shooter` 实际读取这个运行时 YAML。

这样做的目的：SP25 算法参数保持原版风格，HFUT 实车硬件参数集中放在 `hardware.yaml`，上车换相机/串口时不需要到算法配置里到处改。

## 常调顺序

1. 相机打不开或 FPS 异常：先看 `hardware.yaml` 的 `camera` 段和 MVS SDK 路径。
2. 图像识别效果差：先看 `standard3.yaml` 的 `enemy_color`、`device`、`yolo_name`、`min_confidence`、`threshold`、`use_roi`。
3. PnP 距离或角度明显不对：先看 `camera_info.yaml` 和 `hardware.camera.camera_to_barrel`。
4. 云台速度、加速度或输出跳变不对：先看 `controller.yaml` 的 `output_filter` 和 `aim_planner`。
5. 串口无反馈或云台不动：先看 `hardware.serial` 的端口、波特率、收发协议和角度单位。
6. 不开火：先确认 `hardware.safety.enable_fire`、启动命令是否带 `--allow-fire`，再看 `controller.yaml` 的 `fire_gate`。

串口收发单位固定为 rad / rad/s / rad/s²。Web 可视化和控制台日志会把角度类数据转成度制，方便上车观察。

## 校验命令

```bash
python3 scripts/validate_configs.py
python3 scripts/start.py --mode check
```

`--mode check` 会额外打印系统环境、OpenCV、CMake、MVS 和 USB 相机检测信息，适合上车前快速排查。
