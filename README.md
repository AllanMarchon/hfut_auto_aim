# HFUT Auto Aim

HFUT Auto Aim 是面向实车的装甲板自瞄工程。项目使用 SP25 风格的视觉核心完成装甲板识别、PnP、目标跟踪、弹道预测和轨迹规划，并接入 HFUT 实车侧的海康相机、infantry 串口协议、安全开火控制和 Web 调试界面。

当前主链路可以概括为：

```text
相机取流
  -> YOLO / 传统灯条角点修正
  -> PnP 位姿解算
  -> Tracker + 11 维 EKF 目标估计
  -> Aimer 或 MPC 轨迹规划
  -> Shooter / fire gate 开火门控
  -> infantry_32 串口下发
  -> 下位机云台控制
```

## 功能特点

- 支持海康 MVS 相机和 OpenCV 相机后端。
- 支持 OpenVINO YOLOv5/YOLOv8/YOLO11 装甲板识别。
- 支持传统灯条角点修正，提高 PnP 输入稳定性。
- 使用 Tracker 和 11 维 EKF 估计目标整车状态。
- 默认可使用 TinyMPC 轨迹规划，也保留 Aimer 弹道预测链路作为 fallback。
- 支持 infantry 串口反馈、图像时间戳对齐、速度/加速度前馈和安全开火门控。
- 内置 Web MJPEG 调试界面，便于上车观察识别、跟踪、命令和开火建议。

## 目录结构

| 路径 | 作用 |
| --- | --- |
| `src/standard.cpp` | 实车主入口，连接相机、串口、Web、视觉算法和控制输出 |
| `tasks/auto_aim/` | 自瞄核心算法：检测、PnP、Tracker、Aimer、Shooter、Planner |
| `tools/` | 数学工具、EKF、弹道、日志和通用工具 |
| `assets/` | OpenVINO IR 模型和分类模型 |
| `io/camera/` | 海康 MVS 与 OpenCV 相机适配 |
| `io/serial/` | infantry 串口收发协议 |
| `io/web/` | Web 调试和 MJPEG 推流 |
| `configs/` | 硬件、算法、控制和相机内参配置 |
| `apps/` | 串口测试、手动云台、标定采图等上车前工具 |
| `calibration/` | 相机内参标定说明和脚本 |
| `scripts/start.py` | 构建、检查、启动和常用工具入口 |

## 小电脑默认环境

目标小电脑按以下环境适配：Ubuntu 22.04.5、x86_64、Intel i9-12900H、约 7.5 GiB 内存、Intel 集成显卡、OpenCV 4.5.4、CMake 3.26、G++ 11.4、OpenVINO 2025.3.0，海康 MVS SDK 安装在 `/opt/MVS`。

小电脑没有 NVIDIA 独显，因此不依赖 CUDA/TensorRT/ONNX Runtime GPU。模型推理通过 OpenVINO 运行，当前实车配置通常使用 `CPU`；如果 Intel GPU 插件稳定，也可以通过配置或启动参数切到 `GPU`。

## 构建

```bash
cd ~/hfut_auto_aim-main
python3 scripts/start.py --mode build
```

默认查找：

- MVS 头文件：`/opt/MVS/include/MvCameraControl.h`
- MVS 动态库：`/opt/MVS/lib/64/libMvCameraControl.so`
- OpenVINO CMake：`/usr/lib/openvino-2025.3.0/cmake` 或系统 CMake 包路径

如果路径不同，可以显式传入：

```bash
python3 scripts/start.py --mode build \
  --hik-include /opt/MVS/include \
  --hik-library /opt/MVS/lib/64/libMvCameraControl.so \
  --openvino-dir /usr/lib/openvino-2025.3.0/cmake
```

也可以手动 CMake 构建：

```bash
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

## 启动

第一次启动时，如果还没有 `build/standard`，`scripts/start.py` 会自动先构建一次。

不发串口的干跑检查：

```bash
python3 scripts/start.py --mode dry --max-frames 200
```

实车运行，默认禁用真实开火：

```bash
python3 scripts/start.py --mode live
```

只接收反馈、不发送云台命令，适合检查相机和串口反馈：

```bash
python3 scripts/start.py --mode live --no-serial-send
```

确认安全后，允许把开火建议发送给下位机：

```bash
python3 scripts/start.py --mode live --allow-fire
```

Web 可视化默认开启，地址为：

```text
http://<小电脑IP>:8080/
```

如果只想查看远端状态，可以在调试电脑上运行：

```bash
python3 scripts/visualize.py --host <小电脑IP>
```

需要排查内部细节时，可以临时提高日志级别：

```bash
HFUT_LOG_LEVEL=debug python3 scripts/start.py --mode live
```

## 开机自启动

小电脑推荐使用 systemd 注册为 `auto_aim.service`，不依赖桌面登录或 `screen`：

```bash
cd ~/hfut_auto_aim-main
sudo bash scripts/install_auto_aim_service.sh
```

默认服务命令等价于：

```bash
python3 scripts/start.py --mode live --allow-fire
```

常用维护命令：

```bash
systemctl status auto_aim
journalctl -u auto_aim -f
sudo systemctl restart auto_aim
sudo systemctl stop auto_aim
sudo systemctl disable --now auto_aim
```

启动参数集中在 `/etc/default/auto_aim`。默认值为：

```bash
AUTO_AIM_ARGS="--allow-fire"
```

比赛或调试时如果要关闭 Web 推流，可改为：

```bash
AUTO_AIM_ARGS="--no-web-view --allow-fire"
```

取消注册：

```bash
sudo bash scripts/uninstall_auto_aim_service.sh
```

## 配置文件

| 文件 | 作用 |
| --- | --- |
| `configs/hardware.yaml` | 实车硬件参数：相机、串口、弹速、安全开火、相机到枪管外参 |
| `configs/camera_info.yaml` | 相机内参、畸变参数和图像尺寸 |
| `configs/standard3.yaml` | 视觉算法参数：模型、OpenVINO device、颜色、ROI、传统检测、Tracker、Aimer、Shooter |
| `configs/controller.yaml` | 控制输出参数：Planner/MPC、速度/加速度语义、反馈对齐、限幅和 fire gate |

启动时 `src/standard.cpp` 会把 `hardware.yaml` 和 `camera_info.yaml` 中的实车内外参同步到 `build/sp25_runtime.yaml`，视觉算法模块实际读取这个运行时配置。

当前串口下发单位保持为 rad / rad/s / rad/s^2；Web 可视化和控制台日志为了观察方便显示为 deg / deg/s / deg/s^2。

校验配置：

```bash
python3 scripts/start.py --mode check
python3 scripts/validate_configs.py
```

## 上车前工具

单独检查串口收发，不启动视觉链路：

```bash
python3 scripts/start.py --mode serial-test \
  --serial-port /dev/ttyACM0 \
  --serial-tx-protocol infantry_32 \
  --serial-rx-protocol infantry
```

手动给云台下发 yaw/pitch/vel/acc，确认方向、符号和限幅：

```bash
python3 scripts/start.py --mode manual-gimbal --serial-port /dev/ttyACM0
```

生成固定串口名 `/dev/gimbal`：

```bash
python3 scripts/start.py --mode install-udev --udev-device /dev/ttyACM0 --udev-name gimbal
```

采集棋盘格并生成内参：

```bash
python3 scripts/start.py --mode capture-calibration --display
python3 scripts/start.py --mode calibrate-camera \
  --calibration-images 'calibration/images/*.png' \
  --pattern-cols 9 --pattern-rows 6 --square-size 0.025
```

## 上车注意

- `hardware.safety.enable_fire` 默认用于配置层安全保护；实车允许开火还需要启动命令带 `--allow-fire`。
- 海康相机建议接 USB3；FPS 不足时优先检查曝光、分辨率、OpenVINO device、Web 推流步长和磁盘剩余空间。
- `planner.mode: mpc` 使用 TinyMPC 轨迹规划；需要快速回退时可以切到 Aimer 链路做对照测试。
- 调参数时一次只改一个变量，并记录 `raw/stable/cmd/fb/cmd_vel/cmd_acc/lim_err`、弹着点和实际工况。
- `yaw_offset/pitch_offset` 用于修固定弹着偏差；运动目标滞后优先检查延迟参数和下位机跟随，不要用固定 offset 掩盖动态问题。
