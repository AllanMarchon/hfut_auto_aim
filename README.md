# hfut_auto_aim

`hfut_auto_aim` 是从 `hfut_rm_auto_aim_ws` 拆出的 ROS-free 自瞄主链。它使用普通 CMake 构建，在单进程中完成装甲板检测、位姿估计、目标跟踪、选板、运动预测、弹道补偿、云台控制和开火判断。

项目当前主要用于仿真闭环和算法调试。Webots/Unity 使用原子文件桥；运行在 Windows 的
Gestalt 使用共享内存加 WSL interop pipe（TCP 可回退）与 WSL2 通信。两条路径都不需要
ROS2 节点、DDS、TF 服务或虚拟串口。

## 数据流

完整视觉模式：

```text
camera_frame.bin
  -> image + intrinsics + camera pose + gimbal feedback
  -> NN detection
  -> SolvePnP / pose refinement
  -> tracker
  -> target selector
  -> prediction + ballistic compensation
  -> gimbal command + fire advice
  -> gimbal_command.bin
```

直接位姿模式：

```text
armor_pose_frame.bin
  -> noisy armor positions + radial yaw
  -> tracker
  -> target selector
  -> prediction + ballistic compensation
  -> gimbal_command.bin

camera_frame.bin -> debug visualization only
```

默认使用完整视觉模式（`vision`）；直接位姿模式（`armor_pose`）用于隔离图像
链路、单独验证状态估计/预测/控制（难度矩阵可用 `HFUT_MATRIX_INPUT_MODE`
分别回归两种模式）。

Gestalt 路径复用完整视觉链：Windows 代理读取游戏进程的共享帧 v3，经 pipe/TCP 发送同一
渲染提交的 raw/LZ4 无损像素、相机位姿和 FOV；WSL2 完成检测到控制的全部算法，再由
代理把命令转换为 `RBExtAim`。目标位置、转速、装甲相位等真值不会进入该路径。

## 功能范围

- ONNX Runtime 装甲板检测，包含预处理、关键点解码、NMS 和质量过滤。
- NN+传统灯条混合角点精化（`corner_refine`）：在每个 NN 检测的局部 ROI 内
  重跑灯条检测（OTSU 自适应阈值 + minAreaRect + 几何校验），通过才替换
  关键点，失败保留 NN 关键点（极端光照/模糊下行为与未开启一致）；灯条
  内陷按 2D track 的 NN板幅/精化板幅比值 EMA 补偿，无环境常数。
- 传统灯条检测回退链（`armor_detector_traditional`）：灯条配对 + LeNet 数字
  分类 + PCA 角点精化，产出与 NN 链相同的 `ArmorDetection`，位姿估计复用共享
  适配器，不引入 g2o/Sophus。`detector.impl` 选择 `nn`/`traditional`/`auto`
  （NN 初始化失败自动回退）。
- 小/大装甲板 SolvePnP（IPPE 取正深度候选中重投影最小解，孪生解歧义度
  等效 σ 随观测上报供下游诊断），以及 single-yaw、滑动窗口等位姿细化路径。
- 整车跟踪器 `VehicleArmorTracker` + **InEKF（左不变误差状态 EKF，默认后端）**；
  备选后端 `ukf_v1`/`ukf_v2` 保留注册（`backend_config.backend_type` 可切回）。
  InEKF 与 ukf_v1 共用朝向/距离自适应观测噪声模型
  （`trackers/vehicle/adaptive_measurement_noise.hpp`）。
- 旋转见证与自适应过程噪声：无模型装甲方位见证驱动旋转 Q 自适应缩放，
  静止目标抑制虚假旋转、旋转目标全带宽，并对称板陷阱时有界纠偏。
- 观测宽容度：后验中心跳变限幅提交（歪预测器不再持续拒准观测）、
  TRACKING 持续冲突放宽门（连续 2 帧严格门失败但放宽门通过即采信观测）、
  拒止连击 6 帧结构化重置（NIS 拒止保留已学结构、重建拒止回退标称结构）。
- 跟踪稳定化：观测 commit 状态透出（门控拒止帧按丢失处理）、运动 guard
  （线速度/加速度/整车 yaw 角速度钳制、静止死区、丢帧半衰，且协方差随状态
  同步缩放/重建）、输出 OneEuro 稳定器、TEMP_LOST 预测限时发布
  （静止 0.15s / 移动目标滑行 0.5s）。
- 选板时间滞回：上一块装甲板仍有效时，仅当新板云台运动量明显更小才切换，
  抑制整车观测抖动引起的相邻板跳选；跟踪器面板假设另有
  `panel_switch_hysteresis` 滞回机制（评估后默认关闭，见 VALIDATION.md）。
- 当前位姿、预测位姿和 MPC 控制策略；预测外推使用限幅匀加速度模型，并
  将单帧实测算法耗时计入预测时域。
- 进程内弹道补偿、开火窗口和命中概率判断。
- OpenCV 调试叠加层和 PlotJuggler UDP 数据输出。
- 与仿真真值按帧序号对齐的 JSONL 诊断和统计工具。

当前不包含真实硬件串口发送层、ROS2 节点封装和 ROS2 服务式弹道接口。仿真弹道默认使用进程内 `LocalTrajectoryCompensator`。

## 运行要求

构建依赖：

- CMake 3.16 以上和 C++17 编译器。
- OpenCV 4.5、Eigen3、yaml-cpp、fmt。
- qpOASES，默认从 `/usr/local/include` 和 `/usr/local/lib` 查找。
- ONNX Runtime，默认位于 `/opt/onnxruntime-gpu`。

运行还需要一个实现相同文件协议的仿真数据源。当前配套仓库为：

- `/root/hfut_auto_aim_sim`：Webots 仿真和命中评分。
- `/root/hfut_auto_aim_unity_sim`：兼容相同桥协议的 Unity 仿真。
- `reference/sp_vision_25_gestalt_system_bridge` 所使用的 Gestalt shared-frame v3 游戏构建。

跟踪器、选择器和控制器读取本仓库自带的 pipeline 配置：

```text
/root/hfut_auto_aim/configs/gimbal_pipeline.yaml
```

该文件最初复制自 `hfut_rm_auto_aim_ws`（含其未提交调参），此后两个仓库的配置
**各自独立演化、互不影响**：ros-free 的调参只改本仓库副本，ROS2 工作区不再被
引用。临时使用其他 pipeline 配置时，把路径作为 `run.sh` 的第二个位置参数传入。

## 构建

```bash
cd /root/hfut_auto_aim
./scripts/build.sh
```

默认构建目录是 `build/`，构建类型为 `Release`，并行任务数为 2。大型 UKF/MPC 编译单元内存占用较高，可以显式调整：

```bash
CMAKE_BUILD_TYPE=RelWithDebInfo BUILD_JOBS=1 ./scripts/build.sh
```

ONNX Runtime 不在默认路径时使用原始 CMake 命令配置：

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT=/path/to/onnxruntime
cmake --build build --parallel 2
```

主要构建目标：

| 目标 | 作用 |
| --- | --- |
| `bringup_sim` | 完整检测、跟踪、控制和 Webots/Gestalt IO 主程序 |
| `detector_test` | 检测器和位姿估计验证程序 |
| `video_test` | 真实相机视频离线识别、PnP、跟踪和预测验证程序 |
| `hfut_detector` | ROS-free 检测静态库 |
| `hfut_pipeline` | 跟踪、选择和控制静态库 |
| `hfut_io` | 相机文件读取、云台命令写入和 PlotJuggler 输出 |

单元测试共 13 个（跟踪、检测、选板、控制、标定等），应全部通过：

```bash
ctest --test-dir build --output-on-failure
```

仿真回归使用难度矩阵（4 场景 × L1-L4 共 16 格，原地旋转/横移旋转/环绕旋转/
随机走位），在仿真仓运行：

```bash
cd /root/hfut_auto_aim_sim
./run_difficulty_matrix.sh /tmp/matrix 60          # 全 16 格，vision 默认
HFUT_MATRIX_INPUT_MODE=armor_pose ./run_difficulty_matrix.sh /tmp/matrix_ap 60
HFUT_MATRIX_STRATEGY=predicted ./run_difficulty_matrix.sh /tmp/matrix_pred 60
python3 tools/matrix_report.py /tmp/matrix
```

矩阵脚本默认 `HFUT_MATRIX_STRATEGY=mpc`；`matrix_report.py` 汇总 TRACKING%、
当前/预测中心误差、转速估计、命中率与有效 DPS。任何参数改动以全矩阵回归
为准，单场景数字会过拟合。

## 真实视频离线测试

`video_test` 使用视频帧序号和源 FPS 生成时间戳，逐帧运行与在线 vision 模式相同的
NN 检测、PnP、整车状态估计和预测，但不会发送云台命令。默认读取
`test_video/output.avi` 与配套的 `test_video/camera_info.yaml`：

```bash
./scripts/build.sh
./scripts/run_video_test.sh
```

输出包括带检测/PnP/整车状态/预测叠加层的视频、逐帧 JSONL 和汇总 JSON：

```text
/tmp/hfut_video_test_overlay.avi
/tmp/hfut_video_test.jsonl
/tmp/hfut_video_test_summary.json
```

配套标定为 `1280x1024`，当前视频为 `1440x1080`。默认
`--calibration-mode=center_crop` 按等比放大后居中裁剪解释这一区别，有效内参为
`fx=fy=1125, cx=720, cy=540`。如果实际采集链使用了拉伸缩放，改用 `scale`；
`strict` 会在分辨率不一致时直接报错，避免静默使用错误内参。

```bash
./scripts/run_video_test.sh --display --frame-step=2
CALIBRATION_MODE=strict ./scripts/run_video_test.sh
```

视频没有同步运动捕捉真值，因此汇总只报告检测连续率、有效 PnP 比例、重投影误差、
帧间位姿跳变、跟踪接受率和预测输出率，不把这些指标冒充绝对位置精度。JSONL 保留
每块观测装甲板、整车 `x/y/z`、速度、加速度、姿态、`r1/r2/dza/yaw/vyaw` 以及当前
和预测装甲板坐标，可直接用于三维调试回放。

## 3D 可视化调试器

> **当前状态：无法正常工作，待修理（低优先级）**。修理前不要依赖其显示
> 结果下算法结论（见 docs/HANDOVER.md §6）。

先生成诊断 JSONL，再启动本地服务：

```bash
./scripts/run_video_test.sh --no-overlay
./scripts/run_debug_3d.sh
```

浏览器打开 `http://127.0.0.1:8765`。场景支持鼠标拖动旋转、平移和滚轮缩放，底部
时间轴可逐帧、变速和循环回放。右侧图层分别显示本车 `gimbal_link`、
`camera_link` 与相机视锥，识别装甲板、当前整车、预测整车、模拟弹道、线速度和
整车旋转速度。绿色线速度箭头长度按 `0.48 m/(m/s)` 映射；紫色小陀螺箭头位于
目标 z 轴，向上/向下表示旋转方向，长度按 `0.08 m/(rad/s)` 映射。

可通过环境变量回放其他诊断文件：

```bash
DIAGNOSTICS_PATH=/tmp/another_run.jsonl ./scripts/run_debug_3d.sh --port 8877
```

页面也可用 `LOAD JSONL` 直接载入本机文件。Three.js 已随工具放在
`tools/debug_3d/vendor/`，运行时不依赖外网；服务端只使用 Python 标准库。
可选的浏览器回归脚本为 `python3 tools/debug_3d/verify_playwright.py`，需要本机另行安装
Playwright、Pillow 及其 Chromium；也可用 `--browser` 指定已有 Chrome。

## 快速开始

终端 1，启动 Webots：

```bash
cd /root/hfut_auto_aim_sim
./run_stationary_spin_target_test.sh
```

终端 2，启动自瞄：

```bash
cd /root/hfut_auto_aim
./scripts/run.sh --debug
```

脚本默认桥目录为 `~/hfut_auto_aim_webots`。使用自定义目录时，两端必须设置相同值：

```bash
export WEBOTS_ROS_FREE_BRIDGE_DIR=~/my_auto_aim_run
```

snap 版 Webots 可能把 controller 放进独立 `/tmp` 命名空间，所以不要把
ROS-free 文件桥默认放在 `/tmp` 下。

程序启动后会等待新帧；按 `Ctrl+C` 停止，调试窗口中也可按 `q` 退出。

## Gestalt + WSL2

Gestalt 模拟器与共享内存必须留在 Windows；CUDA/ONNX、检测和跟踪控制继续运行在
WSL2。首选路径由 WSL 直接启动 Windows Python 代理，使用 WSL interop stdin/stdout
二进制管道传输 raw 帧和命令，不经过 TCP/IP 或 `/mnt/c` 文件轮询。

Windows PowerShell 中先安装一次依赖：

```powershell
cd \\wsl.localhost\Ubuntu22.04\root\hfut_auto_aim
py -m pip install -r .\tools\requirements-gestalt-windows.txt
```

游戏以 prep 参数启动后，在 WSL2 中一条命令同时启动 Windows 代理和算法：

```bash
cd /root/hfut_auto_aim
./scripts/run_gestalt_stdio.sh --debug
```

该命令当前**默认允许开火**（`run_gestalt_stdio.sh` 无条件追加 `--allow-fire`，
launcher 中的 `--no-fire` 追加逻辑已被注释）——README 此前"默认禁止开火"的
描述已过时，HANDOVER §6 将其列为下次 Gestalt 运行前必须先统一的安全项。
需要显式禁火时请检查脚本行为后再操作。interop 被禁用时才使用 TCP 回退：

```powershell
py .\tools\gestalt_bridge_windows.py auto `
  --prepare-match --start-match --listen 127.0.0.1 --frame-codec lz4 --no-fire
```
```bash
./scripts/run_gestalt.sh --debug
```

TCP 回退推荐使用 mirrored networking 和 `127.0.0.1`；stdio 主路径与 WSL 网络模式无关。

使用 `--prepare-match --start-match` 时，代理还会在 prep 中生成并验证红方 HACHISEN，
在帧契约通过后开赛。代理自动执行 `RBTakeOver`、应用相机参数、维护 `ExtAimClaim`
心跳，并校验 shared-frame 中的 player/map/view/epoch 身份。WS、pipe/TCP 或算法命令超时会
清除开火锁存；进程退出时释放 claim。逐步操作见
[docs/GESTALT_WSL2_TUTORIAL.md](docs/GESTALT_WSL2_TUTORIAL.md)，协议和坐标说明见
[docs/GESTALT_BRIDGE.md](docs/GESTALT_BRIDGE.md)。

Gestalt path 在目标完全消失 0.5 秒等待后执行巡扫：yaw 120 deg/s，pitch 30 deg/s，
范围 `[-10,+10]` deg（源码实际值；旧文档中的 3s、60/20 deg/s 已过时）。参数位于
`bridge.gestalt.idle_scan`；Webots/Unity path 不实例化该
扫描器。检测器出现原始装甲结果时先冻结扫描；`TRACKING` 由自瞄直接接管，
`TEMP_LOST` 预测最多对外保留 `0.15s`，随后清空控制目标并开始计算等待时间。

## 输入模式

transport path 和输入模式由总控 [configs/gimbal_pipeline.yaml](configs/gimbal_pipeline.yaml)
的 `global:` 段选择（`bridge_path` / `input_mode`），传输细节在
[configs/simulation.yaml](configs/simulation.yaml)。通信时，算法端和仿真端应选择相同模式。

| `global.bridge_path` | 传输 | 可用输入模式 |
| --- | --- | --- |
| `webots` | 本地原子文件桥（Webots/Unity） | `vision`、`armor_pose` |
| `gestalt` | Windows shared-frame/WebSocket + interop pipe（TCP 回退） | `vision`（强制） |

| 模式 | 算法路径 | 输入文件 |
| --- | --- | --- |
| `vision`（默认） | 图像 -> 检测 -> SolvePnP -> 跟踪控制 | `camera_frame.bin` 或 Gestalt pipe/TCP frame |
| `armor_pose` | 带噪声三维装甲板位姿 -> 跟踪控制 | `armor_pose_frame.bin`，同序号图像只用于显示 |

## 检测器选择

`vision` 模式下由 `global.detector_impl` 选择检测链：

| 值 | 含义 |
| --- | --- |
| `nn`（默认） | NN 检测器（ONNX Runtime），失败即退出 |
| `traditional` | 传统灯条检测器，无需 NN 模型 |
| `auto` | NN 优先；初始化失败自动回退到灯条检测器 |

## 目标选择与屏蔽

目标筛选器在总控 `gimbal_pipeline.yaml` 的 `selector:` 段配置：

- `strategy`：`priority_list`（按优先度列表顺序选板）等；`priority_robot_ids`
  只是**优先度排序**，不是白名单——未列出的目标仍可能经最小偏航回退被选中。
- `blocked_robot_ids`：**屏蔽列表**，列表内目标永不参与选板（跟踪器内部仍
  运行），且控制器"无选中时取第一个目标"的回退路径同样跳过被屏蔽目标。

当前 outpost 的特殊处理未完成，已在默认配置中屏蔽（`blocked_robot_ids:
["outpost"]`）；恢复时从列表移除即可。

传统链参数在 `detector.yaml` 的 `detector_traditional:`（阈值、灯条/装甲几何约束、
分类置信度、`enemy_color`）。注意仿真灯条亮度低于真车过曝 LED，仿真用
`binary_thres: 90`，真车恢复 160。传统链的角点来自灯条几何与 PCA 精化，
不施加 gestalt 角点外扩。运行时也可命令行覆盖：`./scripts/run.sh --detector=traditional`。

## 命名约定

实验迭代遗留的 `norm4_v2/norm4_v3` 命名已规范化为功能命名：

- 代码：`trackers/vehicle/`（原 `trackers/norm4_v3/`），`VehicleArmorTracker`
  （原 `Norm4ArmorTrackerV2`），命名空间 `fyt::auto_aim::vehicle`。
- 配置段：规范段名 `vehicle_tracker.*`；`norm4_v3.*` 段作为遗留别名继续生效
  （参数层自动回退）。
- `tracker.implementation`：规范值 `"vehicle"`；遗留值 `"norm4"`/`"norm4_v2"`
  自动映射为 `"vehicle"`。
- 配置资产路径 `config/norm4_v3/profiles/**` 保持原名：它们是
  `package://gimbal_pipeline/config/norm4_v3/...` 公开 URL。
- `trackers/norm4_v2/`（旧运行时）、`binder/`、`adaptive`/`outpost` 各历史版本
  为实验保留代码，不在当前链路，维持原名。

## 配置文件

配置按影响面拆分为 5 个文件，全部位于 `configs/`：

| 文件 | 负责内容 |
| --- | --- |
| `gimbal_pipeline.yaml` | **总控开关**：`global:`（传输路径/输入模式/检测器实现）、基础运行参数、跟踪器实现与后端（`tracker.implementation`、`norm4_v3.backend_config`）、目标选择（`selector:`）、控制策略（`controller.strategy/ballistic_mode`） |
| `simulation.yaml` | 仿真桥接（`bridge:`/gestalt/巡扫）、相机外参（`camera_to_barrel:`）、按传输路径的弹速（`controller.bullet_speed`） |
| `detector.yaml` | NN 检测器全参数（`detector:`，含 `corner_refine` 角点精化）、传统灯条检测器（`detector_traditional:`） |
| `tracker.yaml` | 跟踪/滤波/结构/绑定/输出平滑主参数（ROS 风格）+ ros-free 跟踪防抖覆盖（`tracking:` 段） |
| `controller.yaml` | 解算、延时链（`delay.prediction_extra_s`）、开火、MPC、日志 |

参数层按 `tracker.yaml -> controller.yaml -> gimbal_pipeline.yaml` 顺序深合并，
后加载者覆盖先加载者（总控文件优先）。已合并去重的历史覆盖项：
`prediction_extra_s`（→ `controller.yaml` 的 `delay.prediction_extra_s`）、
`output_stabilizer`（→ `tracker.yaml` 的 `smoother:`）、
`selector.blocked_robot_ids`（→ 总控 `selector:`）。

Webots 端对应配置位于 `/root/hfut_auto_aim_sim/config/camera_robot.env`：

```bash
WEBOTS_AUTO_AIM_OUTPUT_MODE=armor_pose
```

也可以临时覆盖算法模式与检测链：

```bash
./scripts/run.sh --input-mode=vision
./scripts/run.sh --armor-pose
./scripts/run.sh --gestalt
./scripts/run.sh --detector=traditional
```

`bridge.dir` 为空时使用 `WEBOTS_ROS_FREE_BRIDGE_DIR`。启动脚本会把该环境变量默认设为 `~/hfut_auto_aim_webots`；直接运行二进制且环境变量也为空时，底层库才会回退到 `/tmp/hfut_auto_aim_webots`。

默认模型为：

```text
tasks/auto_aim/detector/model/RobotDetectionModel/0708.onnx
```

默认后端为 ONNX Runtime CUDA、FP16，并使用 `640x640` RGB 输入。相机内参随每个 `camera_frame.bin` 或 `armor_pose_frame.bin` 到达；`camera_to_barrel.xyz/rpy` 是所有 bridge 和输入模式共用的外参接口。Gestalt 默认值来自当前 HACHISEN 机型，其余车型按实际安装位置填写。

使用自定义配置集（整套目录）：

```bash
./scripts/run.sh --config-dir=/path/to/configs
```

## 命令行参数

| 参数 | 作用 |
| --- | --- |
| `--debug` | 打开 OpenCV 叠加画面，并向 `127.0.0.1:9870` 发送 PlotJuggler JSON |
| `--diagnostics` | 写入桥目录下的 `tracking_diagnostics.jsonl` |
| `--diagnostics=/path/file.jsonl` | 写入指定诊断文件 |
| `--input-mode=vision` | 临时使用完整视觉模式 |
| `--input-mode=armor_pose` | 临时使用直接位姿模式 |
| `--input-mode=gestalt` | 使用 Gestalt transport path 和完整视觉模式 |
| `--armor-pose` | `--input-mode=armor_pose` 的简写 |
| `--gestalt` | `--input-mode=gestalt` 的简写 |
| `--bridge-path=webots\|gestalt` | 只覆盖 transport path |

常用入口：

```bash
./scripts/run.sh
./scripts/run.sh --debug
./scripts/run_diagnostics.sh
./scripts/run.sh --debug --diagnostics=/tmp/tracking.jsonl
```

## Debug 画面

`./scripts/run.sh --debug` 打开名为 `hfut_auto_aim` 的窗口。主要标记：

| 标记 | 含义 |
| --- | --- |
| 黄色小圆圈 | 相机光轴中心 |
| 普通细框与标签 | 视觉模式下的检测装甲板、类别和置信度 |
| 紫色粗框 `LOCK` | 当前锁定的检测装甲板 |
| 绿色细框与连线 | 跟踪器当前估计的整车四块装甲板位置 |
| 青色 `CENTER` 和箭头 | 当前车辆中心及线速度方向 |
| 红色/绿色菱形 `AIM` | 最终命令方向；绿色允许开火，红色禁止开火 |
| 黄色圆圈 `TARGET` | 弹丸到达时预测的目标装甲板位置 |

左上角显示开火建议、跟踪状态、云台和命令角度、角度差、距离、平移/旋转速度、`r1/r2`、预测与飞行时间、数据年龄、开火误差和处理耗时。所有显示角度为弧度。

保存叠加截图：

```bash
HFUT_DEBUG_SNAPSHOT=/tmp/hfut_overlay.png ./scripts/run.sh --debug
```

程序每 30 帧覆盖一次该图片。

PlotJuggler 配置为 `Streaming -> UDP Server`，端口 `9870`，协议选择 JSON。

## 诊断与真值对齐

先运行仿真，再采集逐帧算法诊断：

```bash
cd /root/hfut_auto_aim
./scripts/run_diagnostics.sh
```

Webots 同时在桥目录生成 `target_truth.jsonl`。结束采集后汇总：

```bash
./tools/diagnose_tracking.py \
  --bridge-dir ~/hfut_auto_aim_webots \
  --output /tmp/diagnostics_summary.json
```

汇总内容包括：

- 检测深度与真值比例、重投影误差。
- 直接位姿位置/yaw 噪声误差。
- 当前中心、预测中心和控制目标误差。
- 命令 yaw 误差和预测时间。
- tracker 的 `r1/r2/dza` 结构收敛误差。
- 跟踪状态、命令模式和模式切换次数。

历史验证结果见 [docs/VALIDATION.md](docs/VALIDATION.md)。

ROS2 工作空间（hfut_rm_auto_aim_ws）功能迁移盘点见
[docs/UNMIGRATED_FEATURES.md](docs/UNMIGRATED_FEATURES.md)：仿真闭环无缺失；上实车需补
串口链路与相机驱动；打符/多相机/补盲为 ROS2 侧未完成方向，维持现状。

## 文件桥协议

默认桥目录下的主要文件：

| 文件 | 方向 | 布局 |
| --- | --- | --- |
| `camera_frame.bin` | 仿真 -> 自瞄 | 216 B header + 原始像素 |
| `armor_pose_frame.bin` | 仿真 -> 自瞄 | 184 B header + N x 128 B 装甲板记录（协议 v3） |
| `gimbal_command.bin` | 自瞄 -> 仿真 | 112 B 固定命令包 |

写入方先生成 `.tmp` 文件，再通过原子重命名替换正式文件；读取方使用递增的 `seq` 判断新数据。协议约定：

- 角度为 `rad`，角速度为 `rad/s`，角加速度为 `rad/s^2`。
- 位置和距离为 `m`，时间为 `s`。
- 控制帧轴与 odom 对齐，默认原点位于云台/射手中心。
- 相机四元数顺序为 `(w, x, y, z)`。

[io/bridge_protocol.hpp](io/bridge_protocol.hpp) 是 C++ 协议唯一来源，仿真端和算法端必须使用同一版本。

Gestalt pipe/TCP 使用 32 B envelope 分帧；上行是 148 B 帧元数据加 raw 或 LZ4 无损的
4-byte 像素，下行复用 112 B `CommandPacket`。WSL 使用 latest-frame 后台接收器与推理
并行，积压时覆盖旧帧。其 C++ 唯一来源为
[io/gestalt/gestalt_protocol.hpp](io/gestalt/gestalt_protocol.hpp)，回环测试同时检查 Python
端的相同尺寸。

## 项目结构

| 路径 | 作用 |
| --- | --- |
| `apps/bringup_sim.cpp` | 主循环、模式切换、诊断和调试显示入口 |
| `apps/detector_test.cpp` | 检测器独立验证入口 |
| `configs/gimbal_pipeline.yaml` | 总控开关（global 模式选择、跟踪器实现与后端、selector、控制策略） |
| `configs/simulation.yaml` | 仿真桥接、相机外参、按传输路径的弹速 |
| `configs/detector.yaml` | NN/传统检测器与位姿估计配置 |
| `configs/tracker.yaml` | 跟踪/滤波/输出平滑主参数 + 跟踪防抖覆盖 |
| `configs/controller.yaml` | 解算、延时链、开火、MPC、日志 |
| `io/gestalt/` | Gestalt pipe/TCP 协议、WSL 客户端和坐标转换 |
| `tools/gestalt_bridge_windows.py` | Windows shared-frame/WebSocket 代理 |
| `io/` | 二进制协议、相机读取、云台写入和 PlotJuggler |
| `tasks/auto_aim/detector/` | 检测、关键点、PnP 和调试绘制 |
| `tasks/auto_aim/pipeline/` | 跟踪、选板、预测、控制和开火判断 |
| `third_party/compat/` | 让移植算法脱离 ROS2 编译的消息和 API 兼容层 |
| `scripts/` | 构建、运行和诊断脚本 |
| `tools/diagnose_tracking.py` | 仿真真值与算法输出对齐统计 |

## 常见问题

- 持续显示 `no frame`：确认仿真正在运行、两端桥目录相同，并检查对应 `.bin` 文件是否持续更新。
- Gestalt 持续 `no camera frame`：stdio 先检查 `WSL interop stdio`；TCP 回退再检查
  `WSL client connected`。若一直等待 identity，检查 `--player-id`、游戏 shared-frame v3
  构建和 `RBTakeOver` 是否生效。
- stdio 没有帧：确认 `/proc/sys/fs/binfmt_misc/WSLInterop` 为 `enabled`，并从 WSL 直接
  执行 `/mnt/c/Windows/py.exe -c "print('ok')"`。只有 TCP 回退模式才检查 `47000`、
  mirrored/NAT 地址和 Windows 防火墙。
- `armor_pose` 模式只有图像或只有位姿：确认两端模式一致，并检查两个文件的 `seq` 是否同步。
- 找不到 ONNX Runtime：使用 `-DONNXRUNTIME_ROOT=...` 重新配置，同时运行时设置相应 `LD_LIBRARY_PATH`。
- 找不到 pipeline 配置：传入正确的第二个位置参数，或恢复默认工作空间路径。
- Debug 窗口无法打开：确认存在可用图形会话；无界面运行时去掉 `--debug`，仅使用诊断 JSONL。
- PlotJuggler 没有数据：确认 UDP Server 使用 `9870` 端口和 JSON 协议。
