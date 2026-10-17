# 模型资源说明

本目录保存视觉算法使用的神经网络模型。OpenVINO IR 模型由 `.xml` 结构文件和同名 `.bin` 权重文件组成，部署时需要同时保留。

| 文件 | 格式 | 用途与状态 |
| --- | --- | --- |
| `buff_repvgg.xml`、`buff_repvgg.bin` | OpenVINO IR | 打符检测模型，由 `configs/buff.yaml` 默认引用。 |
| `tiny_resnet.onnx` | ONNX | 分类模型，由 `configs/standard3.yaml` 的 `classify_model` 引用。 |
| `yolo11.xml`、`yolo11.bin` | OpenVINO IR | YOLO11 模型，由 `configs/standard3.yaml` 配置。 |
| `yolov8.xml`、`yolov8.bin` | OpenVINO IR | YOLOv8 模型，由 `configs/standard3.yaml` 配置。 |
| `yolov5.xml`、`yolov5.bin` | OpenVINO IR | YOLOv5 模型，由 `configs/standard3.yaml` 配置。 |
| `yolo11_buff_int8.xml`、`yolo11_buff_int8.bin` | OpenVINO IR | 打符模型资源，当前默认配置未引用。 |
| `szu_rune.onnx`、`szu_rune_model_0624.onnx` | ONNX | 符目标模型备份，当前默认配置未引用；两个文件内容相同。 |

更换模型时，请同步更新对应 YAML 中的模型路径，并确认模型格式与当前推理后端兼容。
