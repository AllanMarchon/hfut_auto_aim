# HFUT SP Vision 25 实车版

这个分支是基于 TongjiSuperPower `sp_vision_25` 重构的 HFUT 实车适配版本。项目结构按 SP25 根目录展开，核心自瞄算法直接放在 `tasks/auto_aim/`、`tools/`、`assets/`、`src/standard.cpp`，不再额外套 `sp25/` 目录。

## 当前取舍

- 全面使用 SP25 的检测、PnP、跟踪、瞄准和射击判定主链。
- 保留 HFUT 已经实车跑通过的海康 MVS 相机、infantry 串口协议、相机参数、开火安全开关和 Web MJPEG 可视化。
- 删除旧自瞄 detector/pipeline、Webots/Gestalt 仿真、旧测试程序和旧第三方兼容层。
- 保留 `python3 scripts/start.py` 一键启动思路，入口目标统一为 `build/standard`。

## 目录结构

| 路径 | 作用 |
| --- | --- |
| `src/standard.cpp` | 实车入口：连接 HFUT 相机/串口/Web，调用 SP25 主链 |
| `tasks/auto_aim/` | SP25 自瞄核心算法 |
| `tools/` | SP25 数学、滤波、日志和弹道工具 |
| `assets/` | SP25 OpenVINO IR 模型和分类模型 |
| `io/camera/` | HFUT 海康 MVS 与 OpenCV 相机适配 |
| `io/serial/` | HFUT infantry 串口收发协议 |
| `io/web/` | 实车调试 Web 可视化服务 |
| `configs/` | SP25 算法、硬件和相机内参配置 |
| `scripts/start.py` | 构建、检查和实车启动脚本 |

## 小电脑默认环境

目标机器按以下环境适配：Ubuntu 22.04.5、x86_64、Intel i9-12900H、约 7.5 GiB 内存、Intel 集成显卡、OpenCV 4.5.4、CMake 3.26、G++ 11.4、OpenVINO 2025.3.0、海康 MVS SDK 安装在 `/opt/MVS`。

没有 NVIDIA 独显，因此默认不走 CUDA/TensorRT/ONNX Runtime GPU，SP25 模型通过 OpenVINO `GPU` 或 `CPU` 运行。当前默认 `configs/standard3.yaml` 使用 `device: GPU`，如果 Intel GPU 插件异常，可启动时加 `--sp25-device CPU`。

## 构建

```bash
cd /path/to/hfut_auto_aim-main
python3 scripts/start.py --mode build
```

默认查找：

- MVS 头文件：`/opt/MVS/include/MvCameraControl.h`
- MVS 动态库：`/opt/MVS/lib/64/libMvCameraControl.so`
- OpenVINO CMake：`/usr/lib/openvino-2025.3.0/cmake` 或系统 CMake 包路径

如果路径不同，显式传入：

```bash
python3 scripts/start.py --mode build \
  --hik-include /opt/MVS/include \
  --hik-library /opt/MVS/lib/64/libMvCameraControl.so \
  --openvino-dir /usr/lib/openvino-2025.3.0/cmake
```

## 启动

第一次直接启动时，如果还没有 `build/standard`，脚本会自动先构建一次。先做不发串口的干跑检查：

```bash
python3 scripts/start.py --mode dry --max-frames 200
```

实车运行但默认禁用开火：

```bash
python3 scripts/start.py --mode live
```

确认安全后才允许把 SP25 开火判定发给下位机：

```bash
python3 scripts/start.py --mode live --allow-fire
```

Web 可视化默认开启，地址为 `http://<小电脑IP>:8080/`。只想看状态或打开页面，可以在调试电脑上运行：

```bash
python3 scripts/visualize.py --host <小电脑IP>
```

## 配置

- `configs/standard3.yaml`：SP25 算法参数、模型路径、OpenVINO device、ROI、传统检测、tracker/aimer/shooter 参数。
- `configs/hardware.yaml`：HFUT 实车硬件参数，包括海康相机、串口协议、弹速、安全开火开关。
- `configs/camera_info.yaml`：相机内参和畸变参数。

启动时 `src/standard.cpp` 会把 `hardware.yaml` 和 `camera_info.yaml` 中的实车内外参同步到 `build/sp25_runtime.yaml`，SP25 各模块实际读取这个运行时配置。

校验配置：

```bash
python3 scripts/start.py --mode check
python3 scripts/validate_configs.py
```

## 上车注意

- 当前分支目标是“先把 SP25 主链接进实车 IO”，不是继续维护旧自瞄 pipeline。
- `hardware.safety.enable_fire` 默认是 `false`，启动脚本也要求额外传 `--allow-fire` 才会发开火建议。
- 海康相机必须挂 USB3；如果 FPS 不够，先确认曝光、分辨率、OpenVINO device、Web 推流步长和磁盘剩余空间。
- 根分支 `main` 保留旧工程；本分支可以继续按 SP25 结构演进。
