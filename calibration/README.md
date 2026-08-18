# 标定工具

这个目录只保留上车前最需要的标定工具，不接入旧自瞄算法链路。

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

默认棋盘格为 9x6 内角点，单格边长 25mm。按实际标定板修改参数：

```bash
python3 scripts/start.py --mode calibrate-camera \
  --calibration-images 'calibration/images/*.png' \
  --pattern-cols 9 \
  --pattern-rows 6 \
  --square-size 0.025 \
  --calibration-output configs/camera_info.yaml
```

生成的 `configs/camera_info.yaml` 会被 `configs/hardware.yaml` 引用，`standard` 启动时会同步到 `build/sp25_runtime.yaml`。

## 上车注意

- 图片要覆盖画面中心、四角、不同距离和轻微倾斜角度。
- 有效图片建议至少 8-12 张，实车建议 25-40 张。
- 如果改了分辨率、裁剪或镜头焦距，需要重新标定。
- 枪管外参 `hardware.camera.camera_to_barrel` 不由本脚本求解；如果 PnP 准但云台偏，需要继续手动校准外参或 yaw/pitch offset。
