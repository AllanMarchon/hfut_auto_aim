#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校验自瞄配置文件的常见错误。

用法：
  python scripts/validate_configs.py
  python scripts/validate_configs.py --strict
"""

from __future__ import annotations

import argparse
import difflib
import math
import sys
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError as exc:  # pragma: no cover - 本机环境依赖提示
    print(f"[ERROR] 缺少 PyYAML，无法解析 YAML：{exc}")
    print("        安装方式：python -m pip install pyyaml")
    sys.exit(2)


REQUIRED_CONFIGS = [
    "hardware.yaml",
    "camera_info.yaml",
    "detector.yaml",
    "tracker.yaml",
    "controller.yaml",
    "gimbal_pipeline.yaml",
    "simulation.yaml",
]

KNOWN_ROBOT_IDS = {
    "1",
    "2",
    "3",
    "4",
    "5",
    "sentry",
    "outpost",
    "base",
    "negative",
}

COMMON_SELECTOR_TYPOS = {
    "out_post": "outpost",
    "negtive": "negative",
}


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


def get_in(data: Any, keys: list[str]) -> Any:
    node = data
    for key in keys:
        if not isinstance(node, dict) or key not in node:
            return None
        node = node[key]
    return node


def is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def require_number(report: Report, value: Any, name: str, *, positive: bool = False) -> None:
    if not is_finite_number(value):
        report.error(f"{name} 必须是有限数字")
        return
    if positive and float(value) <= 0.0:
        report.error(f"{name} 必须大于 0")


def require_bool(report: Report, value: Any, name: str) -> None:
    if not isinstance(value, bool):
        report.error(f"{name} 必须是 true/false")


def require_choice(report: Report, value: Any, name: str, choices: set[str]) -> None:
    if not isinstance(value, str):
        report.error(f"{name} 必须是字符串，可选值：{sorted(choices)}")
        return
    if value not in choices:
        report.error(f"{name}={value!r} 不在允许范围内：{sorted(choices)}")


def require_vector3(report: Report, value: Any, name: str) -> bool:
    if not isinstance(value, list) or len(value) != 3:
        report.error(f"{name} 必须是长度为 3 的数组")
        return False
    bad = [v for v in value if not is_finite_number(v)]
    if bad:
        report.error(f"{name} 必须只包含有限数字")
        return False
    return True


def all_zero(values: Any) -> bool:
    return isinstance(values, list) and all(is_finite_number(v) and abs(float(v)) < 1e-12 for v in values)


def resolve_repo_path(repo_root: Path, raw_path: Any) -> Path | None:
    if not isinstance(raw_path, str) or not raw_path:
        return None
    if raw_path.startswith("package://"):
        return None
    path = Path(raw_path)
    return path if path.is_absolute() else repo_root / path


def ros_params(data: Any) -> Any:
    return get_in(data, ["gimbal_pipeline", "ros__parameters"])


def validate_camera_info(report: Report, data: Any, path: Path) -> None:
    width = data.get("image_width") if isinstance(data, dict) else None
    height = data.get("image_height") if isinstance(data, dict) else None
    require_number(report, width, f"{path.name}.image_width", positive=True)
    require_number(report, height, f"{path.name}.image_height", positive=True)

    matrix_data = get_in(data, ["camera_matrix", "data"])
    if not isinstance(matrix_data, list) or len(matrix_data) != 9:
        report.error(f"{path.name}.camera_matrix.data 必须是 9 个数")
    elif any(not is_finite_number(v) for v in matrix_data):
        report.error(f"{path.name}.camera_matrix.data 必须只包含有限数字")

    dist_data = get_in(data, ["distortion_coefficients", "data"])
    if not isinstance(dist_data, list) or len(dist_data) not in {4, 5, 8, 12, 14}:
        report.warn(f"{path.name}.distortion_coefficients.data 长度通常应为 4/5/8/12/14")
    elif any(not is_finite_number(v) for v in dist_data):
        report.error(f"{path.name}.distortion_coefficients.data 必须只包含有限数字")

    projection_data = get_in(data, ["projection_matrix", "data"])
    if projection_data is not None and (not isinstance(projection_data, list) or len(projection_data) != 12):
        report.warn(f"{path.name}.projection_matrix.data 通常应为 12 个数")


def validate_hardware(report: Report, repo_root: Path, configs: dict[str, Any]) -> None:
    data = configs.get("hardware.yaml")
    root = data.get("hardware") if isinstance(data, dict) else None
    if not isinstance(root, dict):
        report.error("hardware.yaml 缺少 hardware 根节点")
        return

    camera = root.get("camera")
    if not isinstance(camera, dict):
        report.error("hardware.camera 缺失或不是对象")
        return

    require_choice(report, camera.get("backend"), "hardware.camera.backend", {"opencv", "hik", "mindvision"})
    if not isinstance(camera.get("camera_sn", ""), str):
        report.error("hardware.camera.camera_sn 必须是字符串")
    require_number(report, camera.get("width"), "hardware.camera.width", positive=True)
    require_number(report, camera.get("height"), "hardware.camera.height", positive=True)
    require_number(report, camera.get("fps"), "hardware.camera.fps", positive=True)
    require_number(report, camera.get("exposure_time_us"), "hardware.camera.exposure_time_us", positive=True)
    require_number(report, camera.get("gain"), "hardware.camera.gain")
    require_number(report, camera.get("analog_gain"), "hardware.camera.analog_gain")
    require_number(report, camera.get("frame_speed"), "hardware.camera.frame_speed")
    frame_speed = camera.get("frame_speed")
    if is_finite_number(frame_speed) and int(frame_speed) not in {0, 1, 2, 3}:
        report.warn("hardware.camera.frame_speed 通常应为 0/1/2/3")
    require_bool(report, camera.get("flip_image"), "hardware.camera.flip_image")
    require_choice(
        report,
        camera.get("calibration_mode"),
        "hardware.camera.calibration_mode",
        {"strict", "scale", "center_crop"},
    )

    camera_info_raw = camera.get("camera_info")
    camera_info_path = resolve_repo_path(repo_root, camera_info_raw)
    if camera_info_path is None:
        report.error("hardware.camera.camera_info 必须指向本仓库内的 camera_info YAML")
    elif not camera_info_path.exists():
        report.error(f"hardware.camera.camera_info 指向的文件不存在：{camera_info_path}")
    else:
        camera_info = load_yaml(camera_info_path, report)
        if camera_info:
            validate_camera_info(report, camera_info, camera_info_path)
            if camera_info.get("image_width") != camera.get("width"):
                report.error(
                    "hardware.camera.width 与 camera_info.image_width 不一致："
                    f"{camera.get('width')} != {camera_info.get('image_width')}"
                )
            if camera_info.get("image_height") != camera.get("height"):
                report.error(
                    "hardware.camera.height 与 camera_info.image_height 不一致："
                    f"{camera.get('height')} != {camera_info.get('image_height')}"
                )

    extrinsics = camera.get("camera_to_barrel")
    if not isinstance(extrinsics, dict):
        report.error("hardware.camera.camera_to_barrel 缺失或不是对象")
    else:
        xyz_ok = require_vector3(report, extrinsics.get("xyz"), "hardware.camera.camera_to_barrel.xyz")
        rpy_ok = require_vector3(report, extrinsics.get("rpy"), "hardware.camera.camera_to_barrel.rpy")
        if xyz_ok and rpy_ok and all_zero(extrinsics.get("xyz")) and all_zero(extrinsics.get("rpy")):
            report.warn("hardware.camera.camera_to_barrel 仍为全 0；上车前需要填真实外参")

    serial = root.get("serial")
    if not isinstance(serial, dict):
        report.error("hardware.serial 缺失或不是对象")
    else:
        if not isinstance(serial.get("port"), str) or not serial.get("port"):
            report.error("hardware.serial.port 不能为空")
        require_number(report, serial.get("baudrate"), "hardware.serial.baudrate", positive=True)
        require_choice(
            report,
            serial.get("protocol"),
            "hardware.serial.protocol",
            {"infantry", "infantry_16", "infantry_24", "infantry_32", "16", "24", "32"},
        )
        require_choice(
            report,
            serial.get("infantry32_tail_fields"),
            "hardware.serial.infantry32_tail_fields",
            {"acceleration", "accel", "duplicate_velocity", "velocity"},
        )
        require_bool(report, serial.get("command_angles_in_degrees"), "hardware.serial.command_angles_in_degrees")
        require_bool(report, serial.get("feedback_angles_in_degrees"), "hardware.serial.feedback_angles_in_degrees")
        require_number(report, serial.get("read_timeout_ms"), "hardware.serial.read_timeout_ms")
        require_number(report, serial.get("feedback_timeout_ms"), "hardware.serial.feedback_timeout_ms")
        require_bool(report, serial.get("require_feedback"), "hardware.serial.require_feedback")

    detector = root.get("detector")
    if isinstance(detector, dict):
        require_choice(report, detector.get("enemy_color"), "hardware.detector.enemy_color", {"red", "blue", "white"})
    else:
        report.error("hardware.detector 缺失或不是对象")

    controller = root.get("controller")
    if isinstance(controller, dict):
        require_number(report, controller.get("bullet_speed"), "hardware.controller.bullet_speed", positive=True)
        strategy = controller.get("strategy", "")
        if strategy:
            require_choice(
                report,
                strategy,
                "hardware.controller.strategy",
                {"current", "predicted", "mpc", "state_machine"},
            )
    else:
        report.error("hardware.controller 缺失或不是对象")

    safety = root.get("safety")
    if isinstance(safety, dict):
        require_bool(report, safety.get("dry_run"), "hardware.safety.dry_run")
        require_bool(report, safety.get("enable_fire"), "hardware.safety.enable_fire")
        if safety.get("enable_fire") is True:
            report.warn("hardware.safety.enable_fire=true；确认上车闭环稳定后再开火")
    else:
        report.error("hardware.safety 缺失或不是对象")


def validate_master(report: Report, configs: dict[str, Any]) -> None:
    data = configs.get("gimbal_pipeline.yaml")
    if not isinstance(data, dict):
        report.error("gimbal_pipeline.yaml 不是对象")
        return

    global_cfg = data.get("global")
    if not isinstance(global_cfg, dict):
        report.error("gimbal_pipeline.yaml 缺少 global 段")
    else:
        require_choice(report, global_cfg.get("bridge_path"), "global.bridge_path", {"webots", "gestalt"})
        require_choice(report, global_cfg.get("input_mode"), "global.input_mode", {"vision", "armor_pose"})
        require_choice(report, global_cfg.get("detector_impl"), "global.detector_impl", {"nn", "traditional", "auto"})

    params = ros_params(data)
    if not isinstance(params, dict):
        report.error("gimbal_pipeline.yaml 缺少 gimbal_pipeline.ros__parameters")
        return

    require_number(report, params.get("predict_rate"), "gimbal_pipeline.predict_rate", positive=True)
    require_number(report, params.get("tracker_timeout"), "gimbal_pipeline.tracker_timeout", positive=True)

    tracker_impl = get_in(params, ["tracker", "implementation"])
    require_choice(report, tracker_impl, "tracker.implementation", {"vehicle", "norm4", "norm4_v2", "norm4_v3"})

    backend_type = get_in(params, ["norm4_v3", "backend_config", "backend_type"])
    require_choice(report, backend_type, "norm4_v3.backend_config.backend_type", {"inekf", "ukf_v1", "ukf_v2"})

    selector = params.get("selector")
    if isinstance(selector, dict):
        priority = selector.get("priority_robot_ids")
        blocked = selector.get("blocked_robot_ids")
        if not isinstance(priority, list) or not all(isinstance(v, str) for v in priority):
            report.error("selector.priority_robot_ids 必须是字符串数组")
        if not isinstance(blocked, list) or not all(isinstance(v, str) for v in blocked):
            report.error("selector.blocked_robot_ids 必须是字符串数组")
        else:
            validate_robot_id_list(report, blocked, "selector.blocked_robot_ids")
    else:
        report.error("gimbal_pipeline.selector 缺失或不是对象")

    controller = params.get("controller")
    if isinstance(controller, dict):
        require_choice(report, controller.get("strategy"), "controller.strategy", {"current", "predicted", "mpc", "state_machine"})
        require_choice(report, controller.get("ballistic_mode"), "controller.ballistic_mode", {"local", "service"})
    else:
        report.error("gimbal_pipeline.controller 缺失或不是对象")


def validate_robot_id_list(report: Report, values: list[str], name: str) -> None:
    for value in values:
        if value in COMMON_SELECTOR_TYPOS:
            report.error(f"{name} 中的 {value!r} 疑似拼写错误，应为 {COMMON_SELECTOR_TYPOS[value]!r}")
            continue
        if value not in KNOWN_ROBOT_IDS:
            suggestion = difflib.get_close_matches(value, sorted(KNOWN_ROBOT_IDS), n=1)
            if suggestion:
                report.warn(f"{name} 中的 {value!r} 不是常见目标名，是否想写 {suggestion[0]!r}？")
            else:
                report.warn(f"{name} 中的 {value!r} 不是常见目标名，请确认是否为新增目标")


def validate_tracker(report: Report, configs: dict[str, Any]) -> None:
    data = configs.get("tracker.yaml")
    params = ros_params(data)
    if not isinstance(params, dict):
        report.error("tracker.yaml 缺少 gimbal_pipeline.ros__parameters")
        return

    tracker = params.get("tracker")
    if not isinstance(tracker, dict):
        report.error("tracker.yaml 缺少 tracker 段")
    else:
        if "implementation" in tracker:
            report.error("tracker.implementation 不应写在 tracker.yaml；请只在 gimbal_pipeline.yaml 中切换")
        require_number(report, tracker.get("tracking_thres"), "tracker.tracking_thres", positive=True)
        require_number(report, tracker.get("lost_thres"), "tracker.lost_thres", positive=True)

    tracking = data.get("tracking") if isinstance(data, dict) else None
    if isinstance(tracking, dict):
        scale = tracking.get("observation_noise_scale")
        if isinstance(scale, dict):
            require_number(report, scale.get("vision"), "tracking.observation_noise_scale.vision", positive=True)
            require_number(report, scale.get("armor_pose"), "tracking.observation_noise_scale.armor_pose", positive=True)
        else:
            report.error("tracking.observation_noise_scale 缺失或不是对象")
        require_number(report, tracking.get("max_temp_lost_prediction_s"), "tracking.max_temp_lost_prediction_s")
        require_number(report, tracking.get("temp_lost_coast_max_s"), "tracking.temp_lost_coast_max_s")
        require_number(report, tracking.get("temp_lost_coast_min_speed_mps"), "tracking.temp_lost_coast_min_speed_mps")
        require_number(report, tracking.get("id_association_max_distance_m"), "tracking.id_association_max_distance_m")


def validate_controller(report: Report, configs: dict[str, Any]) -> None:
    data = configs.get("controller.yaml")
    params = ros_params(data)
    if not isinstance(params, dict):
        report.error("controller.yaml 缺少 gimbal_pipeline.ros__parameters")
        return
    controller = params.get("controller")
    if not isinstance(controller, dict):
        report.error("controller.yaml 缺少 controller 段")
        return

    solver = controller.get("solver")
    if isinstance(solver, dict):
        require_number(report, solver.get("shooting_range_width"), "controller.solver.shooting_range_width", positive=True)
        require_number(report, solver.get("shooting_range_height"), "controller.solver.shooting_range_height", positive=True)
        require_number(report, solver.get("gravity"), "controller.solver.gravity", positive=True)
        require_number(report, solver.get("iteration_times"), "controller.solver.iteration_times", positive=True)
    else:
        report.error("controller.solver 缺失或不是对象")

    delay = controller.get("delay")
    if isinstance(delay, dict):
        for key in ["prediction_extra_s", "control_latency_s", "trigger_to_muzzle_s", "max_processing_delay_s"]:
            require_number(report, delay.get(key), f"controller.delay.{key}")
    else:
        report.warn("controller.delay 缺失，调延迟时会不直观")


def validate_detector(report: Report, configs: dict[str, Any]) -> None:
    data = configs.get("detector.yaml")
    if not isinstance(data, dict):
        report.error("detector.yaml 不是对象")
        return
    detector = data.get("detector")
    if not isinstance(detector, dict):
        report.error("detector.yaml 缺少 detector 段")
        return
    tracker = detector.get("tracker")
    if not isinstance(tracker, dict):
        report.error("detector.tracker 缺失；这是检测器内部 2D 关联器")
    else:
        require_number(report, tracker.get("iou_threshold"), "detector.tracker.iou_threshold")
        require_number(report, tracker.get("max_missed"), "detector.tracker.max_missed", positive=True)

    traditional = data.get("detector_traditional")
    if isinstance(traditional, dict):
        require_number(report, traditional.get("binary_thres"), "detector_traditional.binary_thres", positive=True)
        ignore_classes = traditional.get("ignore_classes")
        if isinstance(ignore_classes, list):
            for value in ignore_classes:
                if value in COMMON_SELECTOR_TYPOS:
                    report.error(f"detector_traditional.ignore_classes 中的 {value!r} 疑似拼写错误")
        else:
            report.warn("detector_traditional.ignore_classes 缺失或不是数组")


def validate_simulation(report: Report, configs: dict[str, Any]) -> None:
    data = configs.get("simulation.yaml")
    if not isinstance(data, dict):
        report.error("simulation.yaml 不是对象")
        return
    extrinsics = data.get("camera_to_barrel")
    if isinstance(extrinsics, dict):
        for path_name in ["webots", "gestalt"]:
            node = extrinsics.get(path_name)
            if isinstance(node, dict):
                require_vector3(report, node.get("xyz"), f"simulation.camera_to_barrel.{path_name}.xyz")
                require_vector3(report, node.get("rpy"), f"simulation.camera_to_barrel.{path_name}.rpy")
            else:
                report.error(f"simulation.camera_to_barrel.{path_name} 缺失或不是对象")
    else:
        report.error("simulation.camera_to_barrel 缺失或不是对象")

    bullet_speed = get_in(data, ["controller", "bullet_speed"])
    if isinstance(bullet_speed, dict):
        require_number(report, bullet_speed.get("webots"), "simulation.controller.bullet_speed.webots", positive=True)
        require_number(report, bullet_speed.get("gestalt"), "simulation.controller.bullet_speed.gestalt", positive=True)
    else:
        report.error("simulation.controller.bullet_speed 缺失或不是对象")


def validate_required_files(report: Report, config_dir: Path) -> dict[str, Any]:
    configs: dict[str, Any] = {}
    for name in REQUIRED_CONFIGS:
        path = config_dir / name
        if not path.exists():
            report.error(f"缺少必要配置文件：{path}")
            continue
        configs[name] = load_yaml(path, report)
    return configs


def main() -> int:
    parser = argparse.ArgumentParser(description="校验 HFUT 自瞄配置文件")
    parser.add_argument(
        "--config-dir",
        default="configs",
        help="配置目录，默认 configs",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="有警告也返回失败",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    config_dir = Path(args.config_dir)
    if not config_dir.is_absolute():
        config_dir = repo_root / config_dir

    report = Report()
    configs = validate_required_files(report, config_dir)
    if len(configs) == len(REQUIRED_CONFIGS):
        validate_hardware(report, repo_root, configs)
        validate_master(report, configs)
        validate_tracker(report, configs)
        validate_controller(report, configs)
        validate_detector(report, configs)
        validate_simulation(report, configs)

    report.print()
    if report.errors:
        print(f"\n配置校验失败：{len(report.errors)} 个错误，{len(report.warnings)} 个警告")
        return 1
    if args.strict and report.warnings:
        print(f"\n严格模式失败：0 个错误，{len(report.warnings)} 个警告")
        return 1
    print(f"\n配置校验通过：0 个错误，{len(report.warnings)} 个警告")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
