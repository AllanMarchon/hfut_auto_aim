#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校验 HFUT 适配版 SP25 的实车配置。"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError as exc:
    print(f"[ERROR] 缺少 PyYAML，无法解析 YAML：{exc}")
    print("        安装方式：python3 -m pip install pyyaml")
    sys.exit(2)


REQUIRED_CONFIGS = ["hardware.yaml", "camera_info.yaml", "standard3.yaml", "controller.yaml"]
YOLO_MODELS = {
    "yolov5": "yolov5_model_path",
    "yolov8": "yolov8_model_path",
    "yolo11": "yolo11_model_path",
}
SERIAL_PROTOCOLS = {"infantry", "infantry_16", "infantry_24", "infantry_32", "16", "24", "32"}


class Report:
    def __init__(self) -> None:
        self.errors: list[str] = []
        self.warnings: list[str] = []
        self.infos: list[str] = []

    def error(self, message: str) -> None:
        self.errors.append(message)

    def warn(self, message: str) -> None:
        self.warnings.append(message)

    def info(self, message: str) -> None:
        self.infos.append(message)

    def print(self) -> None:
        for message in self.errors:
            print(f"[ERROR] {message}")
        for message in self.warnings:
            print(f"[WARN]  {message}")
        for message in self.infos:
            print(f"[INFO]  {message}")


def load_yaml(path: Path, report: Report) -> Any:
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except FileNotFoundError:
        report.error(f"缺少配置文件：{path}")
        return None
    except yaml.YAMLError as exc:
        report.error(f"YAML 解析失败：{path}：{exc}")
        return None
    except OSError as exc:
        report.error(f"无法读取配置文件：{path}：{exc}")
        return None
    return {} if data is None else data


def is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def require_number(report: Report, value: Any, name: str, *, positive: bool = False) -> None:
    if not is_finite_number(value):
        report.error(f"{name} 必须是有限数字")
        return
    if positive and float(value) <= 0.0:
        report.error(f"{name} 必须大于 0")


def require_int(report: Report, value: Any, name: str, *, positive: bool = False) -> None:
    if not isinstance(value, int) or isinstance(value, bool):
        report.error(f"{name} 必须是整数")
        return
    if positive and value <= 0:
        report.error(f"{name} 必须大于 0")


def require_bool(report: Report, value: Any, name: str) -> None:
    if not isinstance(value, bool):
        report.error(f"{name} 必须是 true/false")


def require_sign(report: Report, value: Any, name: str) -> None:
    if not is_finite_number(value):
        report.error(f"{name} 必须是有限数字")
        return
    if abs(abs(float(value)) - 1.0) > 1e-6:
        report.error(f"{name} 必须是 1 或 -1")


def require_false(report: Report, value: Any, name: str, reason: str) -> None:
    require_bool(report, value, name)
    if isinstance(value, bool) and value:
        report.error(f"{name} 必须为 false：{reason}")


def require_choice(report: Report, value: Any, name: str, choices: set[str]) -> None:
    if not isinstance(value, str) or not value:
        report.error(f"{name} 必须是非空字符串，可选值：{sorted(choices)}")
        return
    if value not in choices:
        report.error(f"{name}={value!r} 不在允许范围内：{sorted(choices)}")


def require_vector(report: Report, value: Any, name: str, length: int) -> None:
    if not isinstance(value, list) or len(value) != length:
        report.error(f"{name} 必须是长度为 {length} 的数组")
        return
    if any(not is_finite_number(item) for item in value):
        report.error(f"{name} 必须只包含有限数字")


def require_path(report: Report, repo_root: Path, raw_path: Any, name: str) -> Path | None:
    if not isinstance(raw_path, str) or not raw_path:
        report.error(f"{name} 必须是非空路径")
        return None
    path = Path(raw_path)
    resolved = path if path.is_absolute() else repo_root / path
    if not resolved.exists():
        report.error(f"{name} 指向的文件不存在：{resolved}")
    return resolved


def validate_camera_info(report: Report, camera_info: Any, path: Path) -> tuple[int | None, int | None]:
    if not isinstance(camera_info, dict):
        report.error(f"{path.name} 必须是 YAML 对象")
        return None, None

    width = camera_info.get("image_width")
    height = camera_info.get("image_height")
    require_number(report, width, f"{path.name}.image_width", positive=True)
    require_number(report, height, f"{path.name}.image_height", positive=True)

    matrix = camera_info.get("camera_matrix", {}).get("data")
    require_vector(report, matrix, f"{path.name}.camera_matrix.data", 9)
    dist = camera_info.get("distortion_coefficients", {}).get("data")
    if not isinstance(dist, list) or len(dist) not in {4, 5, 8, 12, 14}:
        report.warn(f"{path.name}.distortion_coefficients.data 长度通常应为 4/5/8/12/14")
    elif any(not is_finite_number(item) for item in dist):
        report.error(f"{path.name}.distortion_coefficients.data 必须只包含有限数字")

    return int(width) if isinstance(width, int) else None, int(height) if isinstance(height, int) else None


def validate_hardware(report: Report, repo_root: Path, hardware: Any, camera_info: Any) -> None:
    root = hardware.get("hardware") if isinstance(hardware, dict) else None
    if not isinstance(root, dict):
        report.error("hardware.yaml 缺少 hardware 根节点")
        return

    camera = root.get("camera")
    if not isinstance(camera, dict):
        report.error("hardware.camera 缺失或不是对象")
        return
    require_choice(report, camera.get("backend"), "hardware.camera.backend", {"hik", "opencv"})
    if not isinstance(camera.get("camera_sn", ""), str):
        report.error("hardware.camera.camera_sn 必须是字符串")
    require_int(report, camera.get("device_index"), "hardware.camera.device_index")
    require_number(report, camera.get("width"), "hardware.camera.width", positive=True)
    require_number(report, camera.get("height"), "hardware.camera.height", positive=True)
    require_number(report, camera.get("fps"), "hardware.camera.fps", positive=True)
    require_number(report, camera.get("exposure_time_us"), "hardware.camera.exposure_time_us", positive=True)
    require_number(report, camera.get("gain"), "hardware.camera.gain")
    require_bool(report, camera.get("flip_image"), "hardware.camera.flip_image")
    require_choice(report, camera.get("calibration_mode"), "hardware.camera.calibration_mode", {"strict", "scale", "center_crop"})

    camera_info_path = require_path(report, repo_root, camera.get("camera_info"), "hardware.camera.camera_info")
    if camera_info_path is not None and camera_info_path.name == "camera_info.yaml":
        info_width, info_height = validate_camera_info(report, camera_info, camera_info_path)
        if info_width is not None and camera.get("width") != info_width:
            report.error(f"hardware.camera.width 与 camera_info.image_width 不一致：{camera.get('width')} != {info_width}")
        if info_height is not None and camera.get("height") != info_height:
            report.error(f"hardware.camera.height 与 camera_info.image_height 不一致：{camera.get('height')} != {info_height}")

    extrinsics = camera.get("camera_to_barrel")
    if not isinstance(extrinsics, dict):
        report.error("hardware.camera.camera_to_barrel 缺失或不是对象")
    else:
        require_vector(report, extrinsics.get("xyz"), "hardware.camera.camera_to_barrel.xyz", 3)
        require_vector(report, extrinsics.get("rpy"), "hardware.camera.camera_to_barrel.rpy", 3)

    serial = root.get("serial")
    if not isinstance(serial, dict):
        report.error("hardware.serial 缺失或不是对象")
    else:
        if not isinstance(serial.get("port"), str) or not serial.get("port"):
            report.error("hardware.serial.port 不能为空")
        require_number(report, serial.get("baudrate"), "hardware.serial.baudrate", positive=True)
        require_choice(report, serial.get("protocol"), "hardware.serial.protocol", SERIAL_PROTOCOLS)
        require_choice(report, serial.get("tx_protocol", serial.get("protocol")), "hardware.serial.tx_protocol", SERIAL_PROTOCOLS)
        require_choice(report, serial.get("rx_protocol", serial.get("protocol")), "hardware.serial.rx_protocol", SERIAL_PROTOCOLS)
        require_choice(report, serial.get("infantry32_tail_fields"), "hardware.serial.infantry32_tail_fields", {"duplicate_velocity", "acceleration"})
        require_false(
            report,
            serial.get("command_angles_in_degrees"),
            "hardware.serial.command_angles_in_degrees",
            "电控串口下发统一使用 rad / rad/s / rad/s²",
        )
        require_false(
            report,
            serial.get("feedback_angles_in_degrees"),
            "hardware.serial.feedback_angles_in_degrees",
            "电控串口反馈统一按 rad 解析",
        )
        require_number(report, serial.get("read_timeout_ms"), "hardware.serial.read_timeout_ms")
        require_number(report, serial.get("feedback_timeout_ms"), "hardware.serial.feedback_timeout_ms", positive=True)
        require_bool(report, serial.get("require_feedback"), "hardware.serial.require_feedback")

    detector = root.get("detector", {})
    if not isinstance(detector, dict):
        report.error("hardware.detector 必须是对象")
    else:
        require_choice(report, detector.get("enemy_color"), "hardware.detector.enemy_color", {"red", "blue"})

    controller = root.get("controller", {})
    if not isinstance(controller, dict):
        report.error("hardware.controller 必须是对象")
    else:
        require_number(report, controller.get("bullet_speed"), "hardware.controller.bullet_speed", positive=True)

    safety = root.get("safety", {})
    if not isinstance(safety, dict):
        report.error("hardware.safety 必须是对象")
    else:
        require_bool(report, safety.get("dry_run"), "hardware.safety.dry_run")
        require_bool(report, safety.get("enable_fire"), "hardware.safety.enable_fire")
        if safety.get("enable_fire"):
            report.warn("hardware.safety.enable_fire 当前为 true；上车前确认安全边界")


def validate_standard3(report: Report, repo_root: Path, config: Any) -> None:
    if not isinstance(config, dict):
        report.error("standard3.yaml 必须是 YAML 对象")
        return

    require_choice(report, config.get("enemy_color"), "standard3.enemy_color", {"red", "blue"})
    yolo_name = config.get("yolo_name")
    require_choice(report, yolo_name, "standard3.yolo_name", set(YOLO_MODELS))
    if isinstance(yolo_name, str) and yolo_name in YOLO_MODELS:
        require_path(report, repo_root, config.get(YOLO_MODELS[yolo_name]), f"standard3.{YOLO_MODELS[yolo_name]}")
    require_path(report, repo_root, config.get("classify_model"), "standard3.classify_model")
    for key in YOLO_MODELS.values():
        if key in config:
            require_path(report, repo_root, config.get(key), f"standard3.{key}")
    if not isinstance(config.get("device"), str) or not config.get("device"):
        report.error("standard3.device 必须是 OpenVINO device 字符串，例如 GPU 或 CPU")
    require_number(report, config.get("min_confidence"), "standard3.min_confidence", positive=True)
    require_bool(report, config.get("use_traditional"), "standard3.use_traditional")
    require_bool(report, config.get("use_roi"), "standard3.use_roi")

    roi = config.get("roi")
    if not isinstance(roi, dict):
        report.error("standard3.roi 缺失或不是对象")
    else:
        for key in ["x", "y", "width", "height"]:
            require_int(report, roi.get(key), f"standard3.roi.{key}", positive=key in {"width", "height"})

    numeric_fields = [
        "threshold", "max_angle_error", "min_lightbar_ratio", "max_lightbar_ratio",
        "min_lightbar_length", "min_armor_ratio", "max_armor_ratio", "max_side_ratio",
        "max_rectangular_error", "yaw_offset", "pitch_offset", "comming_angle",
        "leaving_angle", "decision_speed", "high_speed_delay_time", "low_speed_delay_time",
        "first_tolerance", "second_tolerance", "judge_distance",
    ]
    for field in numeric_fields:
        require_number(report, config.get(field), f"standard3.{field}")
    for field in ["min_detect_count", "max_temp_lost_count", "outpost_max_temp_lost_count"]:
        require_int(report, config.get(field), f"standard3.{field}", positive=True)
    require_choice(report, config.get("aim_point_mode"), "standard3.aim_point_mode", {"observed", "sp25"})
    require_bool(report, config.get("auto_fire"), "standard3.auto_fire")
    require_int(report, config.get("image_width"), "standard3.image_width", positive=True)
    require_int(report, config.get("image_height"), "standard3.image_height", positive=True)
    require_vector(report, config.get("R_gimbal2imubody"), "standard3.R_gimbal2imubody", 9)
    require_vector(report, config.get("R_camera2gimbal"), "standard3.R_camera2gimbal", 9)
    require_vector(report, config.get("t_camera2gimbal"), "standard3.t_camera2gimbal", 3)
    require_vector(report, config.get("camera_matrix"), "standard3.camera_matrix", 9)
    require_vector(report, config.get("distort_coeffs"), "standard3.distort_coeffs", 5)


def controller_root(config: Any) -> Any:
    if not isinstance(config, dict):
        return None
    nested = config.get("gimbal_pipeline", {}).get("ros__parameters", {}).get("controller")
    if isinstance(nested, dict):
        return nested
    return config.get("controller", config)


def validate_controller(report: Report, config: Any) -> None:
    controller = controller_root(config)
    if not isinstance(controller, dict):
        report.error("controller.yaml 缺少 controller 配置")
        return

    planner = controller.get("aim_planner")
    if not isinstance(planner, dict):
        report.error("controller.aim_planner 缺失或不是对象")
    else:
        require_bool(report, planner.get("enable"), "controller.aim_planner.enable")
        require_number(report, planner.get("max_yaw_acc"), "controller.aim_planner.max_yaw_acc", positive=True)
        require_number(report, planner.get("max_pitch_acc"), "controller.aim_planner.max_pitch_acc", positive=True)

    output = controller.get("output_filter")
    if not isinstance(output, dict):
        report.error("controller.output_filter 缺失或不是对象")
    else:
        require_bool(report, output.get("enable_clamping"), "controller.output_filter.enable_clamping")
        require_number(report, output.get("max_yaw_diff"), "controller.output_filter.max_yaw_diff", positive=True)
        require_number(report, output.get("max_pitch_diff"), "controller.output_filter.max_pitch_diff", positive=True)
        require_bool(report, output.get("enable_rate_limiter"), "controller.output_filter.enable_rate_limiter")
        require_number(report, output.get("max_yaw_rate"), "controller.output_filter.max_yaw_rate", positive=True)
        require_number(report, output.get("max_pitch_rate"), "controller.output_filter.max_pitch_rate", positive=True)
        require_number(report, output.get("one_euro_freq"), "controller.output_filter.one_euro_freq", positive=True)

    limiter = controller.get("command_limiter")
    if not isinstance(limiter, dict):
        report.error("controller.command_limiter 缺失或不是对象")
    else:
        require_bool(report, limiter.get("enable"), "controller.command_limiter.enable")
        require_number(report, limiter.get("reset_timeout_s"), "controller.command_limiter.reset_timeout_s", positive=True)

    feedback_alignment = controller.get("feedback_alignment")
    if not isinstance(feedback_alignment, dict):
        report.error("controller.feedback_alignment 缺失或不是对象")
    else:
        require_bool(report, feedback_alignment.get("enable"), "controller.feedback_alignment.enable")
        require_number(report, feedback_alignment.get("timestamp_offset_ms"), "controller.feedback_alignment.timestamp_offset_ms")
        if is_finite_number(feedback_alignment.get("timestamp_offset_ms")) and float(feedback_alignment["timestamp_offset_ms"]) < 0.0:
            report.error("controller.feedback_alignment.timestamp_offset_ms 必须大于等于 0")
        require_number(report, feedback_alignment.get("max_sample_age_ms"), "controller.feedback_alignment.max_sample_age_ms", positive=True)
        require_int(report, feedback_alignment.get("history_size"), "controller.feedback_alignment.history_size", positive=True)
        if isinstance(feedback_alignment.get("history_size"), int) and feedback_alignment["history_size"] < 2:
            report.error("controller.feedback_alignment.history_size 必须大于等于 2")

    stabilizer = controller.get("target_stabilizer")
    if stabilizer is not None:
        if not isinstance(stabilizer, dict):
            report.error("controller.target_stabilizer 必须是对象")
        else:
            require_bool(report, stabilizer.get("enable"), "controller.target_stabilizer.enable")
            require_number(report, stabilizer.get("max_yaw_jump"), "controller.target_stabilizer.max_yaw_jump", positive=True)
            require_number(report, stabilizer.get("max_yaw_rate"), "controller.target_stabilizer.max_yaw_rate", positive=True)
            require_int(report, stabilizer.get("hold_frames"), "controller.target_stabilizer.hold_frames")
            require_number(report, stabilizer.get("lost_target_hold_s"), "controller.target_stabilizer.lost_target_hold_s")
            if is_finite_number(stabilizer.get("lost_target_hold_s")) and float(stabilizer["lost_target_hold_s"]) < 0.0:
                report.error("controller.target_stabilizer.lost_target_hold_s 必须大于等于 0")
            require_bool(
                report,
                stabilizer.get("freeze_on_unstable_state"),
                "controller.target_stabilizer.freeze_on_unstable_state",
            )

    fire_gate = controller.get("fire_gate")
    if not isinstance(fire_gate, dict):
        report.error("controller.fire_gate 缺失或不是对象")
    else:
        require_bool(report, fire_gate.get("enable"), "controller.fire_gate.enable")
        require_number(report, fire_gate.get("yaw_tolerance"), "controller.fire_gate.yaw_tolerance", positive=True)
        require_number(report, fire_gate.get("pitch_tolerance"), "controller.fire_gate.pitch_tolerance", positive=True)

    output_adapter = controller.get("output_adapter")
    if not isinstance(output_adapter, dict):
        report.error("controller.output_adapter 缺失或不是对象")
    else:
        require_sign(
            report,
            output_adapter.get("sp_pitch_to_command_sign"),
            "controller.output_adapter.sp_pitch_to_command_sign",
        )
        require_sign(
            report,
            output_adapter.get("feedback_yaw_to_world_sign"),
            "controller.output_adapter.feedback_yaw_to_world_sign",
        )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="校验 HFUT 适配版 SP25 配置。")
    parser.add_argument("--config-dir", type=Path, default=Path("configs"))
    parser.add_argument("--strict", action="store_true", help="把警告也视为失败。")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    repo_root = Path(__file__).resolve().parents[1]
    config_dir = args.config_dir if args.config_dir.is_absolute() else repo_root / args.config_dir
    report = Report()

    configs: dict[str, Any] = {}
    for name in REQUIRED_CONFIGS:
        configs[name] = load_yaml(config_dir / name, report)

    if configs.get("hardware.yaml") is not None and configs.get("camera_info.yaml") is not None:
        validate_hardware(report, repo_root, configs["hardware.yaml"], configs["camera_info.yaml"])
    if configs.get("standard3.yaml") is not None:
        validate_standard3(report, repo_root, configs["standard3.yaml"])
    if configs.get("controller.yaml") is not None:
        validate_controller(report, configs["controller.yaml"])

    report.info("配置校验目标：SP25 核心 + HFUT 海康相机/串口/Web 可视化适配")
    report.print()
    if report.errors:
        return 1
    if args.strict and report.warnings:
        return 1
    print("[OK] SP25 实车配置校验通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
