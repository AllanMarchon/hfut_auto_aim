# SP25 实车基线接入说明

## 当前状态

本分支已把 `sp_vision_25-main` 作为独立实战基线快照放入
`reference/sp_vision_25/`。该目录用于后续抽取 SP25 的轻量自瞄主链，
暂不参与当前 `hfut_auto_aim` 默认 CMake 构建，也不覆盖现有
`apps/bringup_real.cpp`、`tasks/auto_aim/` 或硬件适配代码。

为避免提交无关大文件，`assets/demo/demo.avi` 没有导入；其余源码、配置、模型
和原仓自带 SDK 适配文件均保留，方便对照 SP25 原始行为。

## 接入目标

目标不是整仓替换，而是在当前 `test` 分支形成一条并行的 SP25 风格实车主链：

```text
当前相机输入 / 串口反馈
  -> SP25 OpenVINO 检测
  -> SP25 Solver / Tracker / Aimer / Shooter
  -> 当前串口命令输出
```

现有复杂链路（`Pipeline`、整车 InEKF、selector、MPC、诊断大链路）先保留在主仓
原位置，作为对照和后续可选模块，不作为第一版 SP25 实车链路依赖。

## 相机替换分析

第一版建议优先复用当前 `HikCameraSource` 打通 MV-CS016-10UC、MVS SDK 路径和
`configs/hardware.yaml`，但必须在 SP25 入口里单独计时：取帧、Bayer/BGR 转换、
flip/clone、源 FPS。若源 FPS 或转换耗时异常，再把 SP25 的后台采集线程和最新帧队列
模型迁入当前相机层。

需要确认的点：

- 曝光和增益是否先贴近 SP25 的 `exposure_ms=2.5`、`gain=16.9` 起步；
- 当前 `flip_image` 是否真实需要，避免图像方向和 SP25 检测/外参假设不一致；
- OpenVINO 输入路径是否尽量保持 SP25 的 u8 tensor + OpenVINO preprocess，避免
  在 CPU 上额外做大段 float NCHW 预处理。

## 串口替换分析

第一版建议继续复用当前 `InfantrySerialTransport`，因为它已经跑通 `/dev/ttyACM0`、
`infantry` / `infantry_32` 和 rad/deg 配置；不要直接搬 SP25 的 CBoard CAN。

需要确认的点：

- SP25 输出命令语义和当前下位机期望是否一致：绝对 yaw/pitch、增量角、速度、加速度、
  开火位分别是什么；
- 当前串口反馈只有 yaw/pitch/roll 时，是否需要做带时间戳的反馈缓存，用图像时间戳取最近
  或插值姿态，避免直接用“当前反馈”造成延迟误差；
- 第一版建议只发送 yaw、pitch、distance、shoot/control 等最小字段，先不要混入 MPC
  速度/加速度。

## 可视化替换分析

可视化可以保留，但不能污染 SP25 性能判断。SP25 入口中建议默认低频或关闭 Web/MJPEG、
OpenCV overlay；打开时必须输出 visual 耗时。若关闭可视化仍低 FPS，再看相机或推理；
若关闭后 FPS 明显上升，说明当前可视化不适合作为实车默认路径。

## 启动脚本替换分析

`python3 scripts/start.py` 的一键启动思路建议保留，但后续应增加 pipeline 选择，而不是新造
一套启动方式：

```bash
python3 scripts/start.py --mode live --pipeline current
python3 scripts/start.py --mode live --pipeline sp25
```

`current` 保持现有 `bringup_real`，`sp25` 指向后续新增的 SP25 实车入口。这样便于同一套
硬件配置下 A/B 对比 FPS、延迟和瞄准效果。

## 需要后续确认

- 下位机当前命令协议的精确定义：yaw/pitch 是绝对角还是误差角，单位和符号约定是什么；
- 串口反馈中的 roll/pitch/yaw 对应哪个坐标系，是否能提供四元数或更精确时间戳；
- 实车是否必须保留 Web 调试，还是默认只保留低频日志和可选窗口；
- 第一版 SP25 检测使用 `yolov5.xml`、`yolo11.xml` 还是当前模型转换后的 OpenVINO IR。
