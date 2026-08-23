# 标定工具

本目录用于相机内参标定。内参负责把图像像素转换成相机坐标下的视线方向；外参和弹着 offset 需要结合实车机械位置、枪管轴线和实弹结果继续校准。

## 采集棋盘格图片

先编译工具：

```bash
python3 scripts/start.py --mode build
```

使用当前 `configs/hardware.yaml` 的相机配置采图：

```bash
python3 scripts/start.py --mode capture-calibration --display \
  --calibration-output-dir calibration/images \
  --calibration-max-images 40
```

窗口中按 `s` 可手动保存，按 `q` 或 `Esc` 退出。不开窗口时会按 `--calibration-save-interval` 自动保存。

## 生成内参

默认棋盘格为 9x6 内角点，单格边长 25 mm。按实际标定板修改参数：

```bash
python3 scripts/start.py --mode calibrate-camera \
  --calibration-images 'calibration/images/*.png' \
  --pattern-cols 9 \
  --pattern-rows 6 \
  --square-size 0.025 \
  --calibration-output configs/camera_info.yaml
```

生成的 `configs/camera_info.yaml` 会被 `configs/hardware.yaml` 引用。`standard` 启动时会把内参、畸变和图像尺寸同步到 `build/sp25_runtime.yaml`。

## 采图要求

- 标定图片要覆盖画面中心、四角、不同距离和轻微倾斜角度。
- 有效图片建议至少 8-12 张，实车建议 25-40 张。
- 标定采图时的分辨率、裁剪、翻转和镜头焦距必须和实车运行一致。
- 如果改了 `width/height`、ROI、`flip_image`、镜头焦距或相机安装方式，需要重新标定或重新确认内外参。

## 外参和弹着偏差

`hardware.camera.camera_to_barrel` 描述相机坐标系相对枪管/云台坐标系的位置和姿态，不由本脚本自动求解。

当前工程按常用机器人坐标约定理解外参：

- `x`：向前。
- `y`：向左。
- `z`：向上。
- `rpy`：相机相对枪管/云台坐标系的 roll、pitch、yaw，单位 rad。

如果 PnP 距离和角度明显不对，优先检查内参、畸变、图像翻转和 `camera_to_barrel`。如果视觉和云台已经稳定对准，但实弹落点存在固定偏差，再调 `controller.yaml` 中当前链路使用的 `yaw_offset/pitch_offset`。

不要用固定 offset 修动态问题。静止目标固定偏差通常对应外参、弹速、枪管机械轴或零位；运动目标滞后或旋转时偏差，通常还要检查预测延迟、MPC 参数和下位机跟随。
