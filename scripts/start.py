#!/usr/bin/env python3
"""HFUT 适配版 SP25 实车启动脚本。

默认启动根目录 SP25 入口 `standard`。相机、串口和开火安全开关仍从
`configs/hardware.yaml` 读取，算法参数从 `configs/standard3.yaml` 读取。
"""

from __future__ import annotations

import argparse
import os
import pathlib
import re
import shutil
import subprocess
import sys


PROJECT_DIR = pathlib.Path(__file__).resolve().parents[1]
BUILD_DIR = PROJECT_DIR / "build"

# 这里只是清理可能仍在后台占用相机/串口的旧进程名，不代表当前分支继续依赖旧链路。
CONFLICT_PATTERNS = [
    "ros2_hik_camera_node",
    "armor_detector_node",
    "gimbal_pipeline_node",
    "rm_serial_driver_node",
    "ros2 launch rm_bringup",
    "MvViewer",
    "bringup_real",
    "bringup_sp25_real",
    "standard",
    "serial_transport_test",
    "manual_gimbal_test",
    "capture_calibration_images",
]


def command_exists(name: str) -> bool:
    return shutil.which(name) is not None


def run(cmd: list[str], *, env: dict[str, str] | None = None,
        check: bool = False, capture: bool = False) -> subprocess.CompletedProcess[str]:
    print("+", " ".join(cmd), flush=True)
    return subprocess.run(
        cmd,
        cwd=PROJECT_DIR,
        env=env,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
        check=check,
    )


def require_file(path: pathlib.Path, hint: str) -> None:
    if not path.exists():
        raise SystemExit(f"缺少文件：{path}\n{hint}")


def default_openvino_dir() -> pathlib.Path:
    env_root = os.environ.get("OpenVINO_DIR") or os.environ.get("OPENVINO_DIR")
    if env_root:
        return pathlib.Path(env_root).expanduser()
    candidates = [
        pathlib.Path("/usr/lib/openvino-2025.3.0/cmake"),
        pathlib.Path("/usr/lib/x86_64-linux-gnu/cmake/openvino"),
        pathlib.Path("/opt/intel/openvino_2025.3.0/runtime/cmake"),
        pathlib.Path("/opt/intel/openvino/runtime/cmake"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def default_openvino_lib_dir() -> pathlib.Path:
    candidates = [
        pathlib.Path("/usr/lib/openvino-2025.3.0"),
        pathlib.Path("/usr/lib/x86_64-linux-gnu"),
        pathlib.Path("/opt/intel/openvino_2025.3.0/runtime/lib/intel64"),
        pathlib.Path("/opt/intel/openvino/runtime/lib/intel64"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def make_env(mvs_lib_dir: pathlib.Path, openvino_lib_dir: pathlib.Path) -> dict[str, str]:
    env = os.environ.copy()
    paths = [str(path) for path in (mvs_lib_dir, openvino_lib_dir) if path.exists()]
    current = env.get("LD_LIBRARY_PATH", "")
    if current:
        paths.append(current)
    env["LD_LIBRARY_PATH"] = ":".join(paths)
    return env


def find_hik_usb_paths() -> list[pathlib.Path]:
    if not command_exists("lsusb"):
        return []
    result = run(["lsusb"], capture=True)
    paths: list[pathlib.Path] = []
    pattern = re.compile(r"Bus\s+(\d+)\s+Device\s+(\d+):.*(?:2bdf|Hikrobot)", re.I)
    for line in (result.stdout or "").splitlines():
        match = pattern.search(line)
        if match:
            paths.append(pathlib.Path("/dev/bus/usb") / match.group(1) / match.group(2))
    return paths


def print_camera_owners() -> None:
    usb_paths = find_hik_usb_paths()
    if not usb_paths:
        print("[start] lsusb 未发现 Hikrobot/2bdf 相机。")
        return
    for usb_path in usb_paths:
        print(f"[start] Hikrobot USB 路径：{usb_path}")
        if command_exists("fuser"):
            run(["fuser", "-v", str(usb_path)])


def stop_conflicting_processes() -> None:
    print("[start] 清理可能占用相机或串口的旧进程...")
    for pattern in CONFLICT_PATTERNS:
        subprocess.run(["pkill", "-f", pattern], cwd=PROJECT_DIR,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def check_environment(args: argparse.Namespace) -> int:
    checks = [
        ["uname", "-m"],
        ["bash", "-lc", "cat /etc/os-release | head -5"],
        ["bash", "-lc", "g++ --version | head -1"],
        ["bash", "-lc", "cmake --version | head -1"],
        ["bash", "-lc", "pkg-config --modversion opencv4 || true"],
        ["bash", "-lc", "lsusb | grep -Ei 'Hikrobot|2bdf|camera' || true"],
        ["bash", "-lc", "find /opt /usr /usr/local -name MvCameraControl.h 2>/dev/null | head"],
        ["bash", "-lc", "find /opt /usr /usr/local -name 'libMvCameraControl.so*' 2>/dev/null | head"],
    ]
    for cmd in checks:
        run(cmd)
    print(f"OpenVINO_DIR={args.openvino_dir}")
    print(f"OpenVINO lib dir={args.openvino_lib_dir}")
    print_camera_owners()
    return run([sys.executable, "scripts/validate_configs.py"]).returncode


def build_project(args: argparse.Namespace, env: dict[str, str]) -> int:
    require_file(args.hik_include / "MvCameraControl.h", "请检查 --hik-include 或 /opt/MVS/include。")
    require_file(args.hik_library, "请检查 --hik-library 或 /opt/MVS/lib/64/libMvCameraControl.so。")

    configure = [
        "cmake",
        "-S", str(PROJECT_DIR),
        "-B", str(BUILD_DIR),
        "-DHFUT_ENABLE_HIK_CAMERA=ON",
        f"-DHFUT_HIK_INCLUDE_DIR={args.hik_include}",
        f"-DHFUT_HIK_LIBRARY={args.hik_library}",
        f"-DCMAKE_BUILD_TYPE={args.build_type}",
    ]
    if args.openvino_dir.exists():
        configure.append(f"-DOpenVINO_DIR={args.openvino_dir}")

    status = run(configure, env=env).returncode
    if status != 0:
        return status
    return run(["cmake", "--build", str(BUILD_DIR), f"-j{args.jobs}"], env=env).returncode


def build_standard_command(args: argparse.Namespace) -> list[str]:
    exe = BUILD_DIR / "standard"
    require_file(exe, "请先运行：python3 scripts/start.py --mode build")
    cmd = [
        str(exe),
        "--hardware-config", str(args.hardware_config),
        "--sp25-config", str(args.config),
        "--controller-config", str(args.controller_config),
        "--camera-backend", args.camera_backend,
        "--web-port", str(args.web_port),
    ]
    if args.mode == "dry":
        cmd.append("--dry-run")
    if args.web_view:
        cmd.append("--web-view")
        cmd += ["--web-host", args.web_host]
        cmd += ["--web-frame-step", str(args.web_frame_step)]
    if args.max_frames >= 0:
        cmd += ["--max-frames", str(args.max_frames)]
    if args.enemy_color:
        cmd += ["--enemy-color", args.enemy_color]
    if args.exposure_time_us is not None:
        cmd += ["--exposure-time-us", str(args.exposure_time_us)]
    if args.gain is not None:
        cmd += ["--gain", str(args.gain)]
    if args.sp25_device:
        cmd += ["--sp25-device", args.sp25_device]
    if args.display:
        cmd.append("--display")
    if args.allow_fire:
        cmd.append("--enable-fire")
    if not args.serial_send:
        cmd.append("--no-serial-send")
    cmd += args.extra
    return cmd


def run_standard(args: argparse.Namespace, env: dict[str, str]) -> int:
    if not (BUILD_DIR / "standard").exists():
        print("[start] 未找到 build/standard，先自动构建一次。")
        status = build_project(args, env)
        if status != 0:
            return status
    if args.stop_conflicts:
        stop_conflicting_processes()
    print_camera_owners()
    if args.web_view:
        print(f"[start] Web 可视化：http://{args.viewer_host}:{args.web_port}/")
    if args.mode == "live" and not args.allow_fire:
        print("[start] live 模式：串口开启，开火强制关闭。")
    if args.mode == "dry":
        print("[start] dry 模式：不打开串口、不下发控制。")
    return run(build_standard_command(args), env=env).returncode


def run_serial_test(args: argparse.Namespace, env: dict[str, str]) -> int:
    exe = BUILD_DIR / "serial_transport_test"
    if not exe.exists():
        status = build_project(args, env)
        if status != 0:
            return status
    require_file(exe, "请先运行：python3 scripts/start.py --mode build")
    cmd = [
        str(exe),
        args.serial_port,
        str(args.baudrate),
        args.serial_tx_protocol,
        args.serial_rx_protocol,
        args.infantry32_tail_fields,
    ]
    return run(cmd, env=env).returncode


def run_manual_gimbal_test(args: argparse.Namespace, env: dict[str, str]) -> int:
    exe = BUILD_DIR / "manual_gimbal_test"
    if not exe.exists():
        status = build_project(args, env)
        if status != 0:
            return status
    require_file(exe, "请先运行：python3 scripts/start.py --mode build")
    if args.stop_conflicts:
        stop_conflicting_processes()
    cmd = [
        str(exe),
        "--port", args.serial_port,
        "--baudrate", str(args.baudrate),
        "--yaw-range-deg", str(args.manual_yaw_range_deg),
        "--pitch-range-deg", str(args.manual_pitch_range_deg),
        "--hz", str(args.manual_hz),
        "--distance", str(args.manual_distance),
        "--max-rate-rad-s", str(args.manual_max_rate_rad_s),
        "--max-acc-rad-s2", str(args.manual_max_acc_rad_s2),
    ]
    if args.manual_no_feedback:
        cmd.append("--no-feedback")
    return run(cmd, env=env).returncode


def run_capture_calibration(args: argparse.Namespace, env: dict[str, str]) -> int:
    exe = BUILD_DIR / "capture_calibration_images"
    if not exe.exists():
        status = build_project(args, env)
        if status != 0:
            return status
    require_file(exe, "请先运行：python3 scripts/start.py --mode build")
    if args.stop_conflicts:
        stop_conflicting_processes()
    print_camera_owners()
    cmd = [
        str(exe),
        "--hardware-config", str(args.hardware_config),
        "--camera-backend", args.camera_backend,
        "--output-dir", str(args.calibration_output_dir),
        "--prefix", args.calibration_prefix,
        "--save-interval", str(args.calibration_save_interval),
        "--max-images", str(args.calibration_max_images),
    ]
    if args.exposure_time_us is not None:
        cmd += ["--exposure-time-us", str(args.exposure_time_us)]
    if args.gain is not None:
        cmd += ["--gain", str(args.gain)]
    if args.display:
        cmd.append("--display")
    return run(cmd, env=env).returncode


def run_calibrate_camera(args: argparse.Namespace, env: dict[str, str]) -> int:
    script = PROJECT_DIR / "calibration" / "calibrate_camera.py"
    require_file(script, "缺少内参标定脚本。")
    cmd = [
        sys.executable,
        str(script),
        "--images", args.calibration_images,
        "--pattern-cols", str(args.pattern_cols),
        "--pattern-rows", str(args.pattern_rows),
        "--square-size", str(args.square_size),
        "--output", str(args.calibration_output),
    ]
    if args.show_corners:
        cmd.append("--show")
    return run(cmd, env=env).returncode


def run_install_udev(args: argparse.Namespace, env: dict[str, str]) -> int:
    script = PROJECT_DIR / "scripts" / "install_udev.py"
    require_file(script, "缺少 udev 安装脚本。")
    cmd = [
        sys.executable,
        str(script),
        "--device", args.udev_device,
        "--name", args.udev_name,
    ]
    if args.udev_vendor_id:
        cmd += ["--vendor-id", args.udev_vendor_id]
    if args.udev_product_id:
        cmd += ["--product-id", args.udev_product_id]
    if args.udev_serial:
        cmd += ["--serial", args.udev_serial]
    if args.udev_dry_run:
        cmd.append("--dry-run")
    return run(cmd, env=env).returncode


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="启动 HFUT 适配版 SP25 实车主链。")
    parser.add_argument(
        "--mode",
        choices=(
            "dry", "live", "build", "check",
            "serial-test", "manual-gimbal",
            "capture-calibration", "calibrate-camera", "install-udev",
        ),
        default="live",
    )
    parser.add_argument("--config", type=pathlib.Path, default=PROJECT_DIR / "configs" / "standard3.yaml")
    parser.add_argument("--hardware-config", type=pathlib.Path, default=PROJECT_DIR / "configs" / "hardware.yaml")
    parser.add_argument("--controller-config", type=pathlib.Path, default=PROJECT_DIR / "configs" / "controller.yaml")
    parser.add_argument("--camera-backend", default="hik", choices=("hik", "opencv"))
    parser.add_argument("--sp25-device", default="", help="覆盖 OpenVINO device，例如 GPU 或 CPU。")
    parser.add_argument("--openvino-dir", type=pathlib.Path, default=default_openvino_dir())
    parser.add_argument("--openvino-lib-dir", type=pathlib.Path, default=default_openvino_lib_dir())
    parser.add_argument("--hik-include", type=pathlib.Path, default=pathlib.Path("/opt/MVS/include"))
    parser.add_argument("--hik-library", type=pathlib.Path, default=pathlib.Path("/opt/MVS/lib/64/libMvCameraControl.so"))
    parser.add_argument("--mvs-lib-dir", type=pathlib.Path, default=pathlib.Path("/opt/MVS/lib/64"))
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--build-type", default="Release")
    parser.add_argument("--max-frames", type=int, default=-1)
    parser.add_argument("--enemy-color", choices=("red", "blue"), default=None)
    parser.add_argument("--exposure-time-us", type=float, default=None)
    parser.add_argument("--gain", type=float, default=None)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--no-web-view", dest="web_view", action="store_false")
    parser.set_defaults(web_view=True)
    parser.add_argument("--web-host", default="0.0.0.0")
    parser.add_argument("--viewer-host", default=os.environ.get("HFUT_AUTO_AIM_HOST", "192.168.137.44"))
    parser.add_argument("--web-port", type=int, default=8080)
    parser.add_argument("--web-frame-step", type=int, default=2)
    parser.add_argument("--no-stop-conflicts", dest="stop_conflicts", action="store_false")
    parser.set_defaults(stop_conflicts=True)
    parser.add_argument("--allow-fire", action="store_true")
    parser.add_argument("--no-serial-send", dest="serial_send", action="store_false",
                        help="live 时打开串口收反馈，但不向下位机发送控制包。")
    parser.set_defaults(serial_send=True)
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--serial-tx-protocol", default="infantry_32")
    parser.add_argument("--serial-rx-protocol", default="infantry")
    parser.add_argument("--infantry32-tail-fields", default="acceleration")
    parser.add_argument("--manual-yaw-range-deg", type=float, default=20.0)
    parser.add_argument("--manual-pitch-range-deg", type=float, default=15.0)
    parser.add_argument("--manual-hz", type=float, default=100.0)
    parser.add_argument("--manual-distance", type=float, default=1.0)
    parser.add_argument("--manual-max-rate-rad-s", type=float, default=5.0)
    parser.add_argument("--manual-max-acc-rad-s2", type=float, default=80.0)
    parser.add_argument("--manual-no-feedback", action="store_true")
    parser.add_argument("--calibration-output-dir", type=pathlib.Path, default=PROJECT_DIR / "calibration" / "images")
    parser.add_argument("--calibration-prefix", default="calib")
    parser.add_argument("--calibration-save-interval", type=int, default=20)
    parser.add_argument("--calibration-max-images", type=int, default=40)
    parser.add_argument("--calibration-images", default="calibration/images/*.png")
    parser.add_argument("--pattern-cols", type=int, default=9)
    parser.add_argument("--pattern-rows", type=int, default=6)
    parser.add_argument("--square-size", type=float, default=0.025)
    parser.add_argument("--calibration-output", type=pathlib.Path, default=PROJECT_DIR / "configs" / "camera_info.yaml")
    parser.add_argument("--show-corners", action="store_true")
    parser.add_argument("--udev-device", default="/dev/ttyACM0")
    parser.add_argument("--udev-name", default="gimbal")
    parser.add_argument("--udev-vendor-id", default="")
    parser.add_argument("--udev-product-id", default="")
    parser.add_argument("--udev-serial", default="")
    parser.add_argument("--udev-dry-run", action="store_true")
    parser.add_argument("extra", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.extra and args.extra[0] == "--":
        args.extra = args.extra[1:]
    args.config = args.config.expanduser().resolve()
    args.hardware_config = args.hardware_config.expanduser().resolve()
    args.controller_config = args.controller_config.expanduser().resolve()
    args.openvino_dir = args.openvino_dir.expanduser().resolve()
    args.openvino_lib_dir = args.openvino_lib_dir.expanduser().resolve()
    args.hik_include = args.hik_include.expanduser().resolve()
    args.hik_library = args.hik_library.expanduser().resolve()
    args.mvs_lib_dir = args.mvs_lib_dir.expanduser().resolve()
    args.calibration_output_dir = args.calibration_output_dir.expanduser().resolve()
    args.calibration_output = args.calibration_output.expanduser().resolve()
    if args.allow_fire and args.mode != "live":
        raise SystemExit("--allow-fire 只能配合 --mode live 使用")
    if args.web_port <= 0 or args.web_port > 65535:
        raise SystemExit("--web-port 必须在 1..65535")
    if args.web_frame_step <= 0:
        raise SystemExit("--web-frame-step 必须大于 0")
    if args.calibration_save_interval < 0:
        raise SystemExit("--calibration-save-interval 必须 >= 0")
    if args.calibration_max_images == 0:
        raise SystemExit("--calibration-max-images 不能为 0")
    if args.pattern_cols <= 0 or args.pattern_rows <= 0 or args.square_size <= 0.0:
        raise SystemExit("棋盘格参数必须大于 0")
    if args.manual_hz <= 0.0:
        raise SystemExit("--manual-hz 必须大于 0")
    return args


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    env = make_env(args.mvs_lib_dir, args.openvino_lib_dir)
    if args.mode == "check":
        return check_environment(args)
    if args.mode == "build":
        return build_project(args, env)
    if args.mode == "serial-test":
        return run_serial_test(args, env)
    if args.mode == "manual-gimbal":
        return run_manual_gimbal_test(args, env)
    if args.mode == "capture-calibration":
        return run_capture_calibration(args, env)
    if args.mode == "calibrate-camera":
        return run_calibrate_camera(args, env)
    if args.mode == "install-udev":
        return run_install_udev(args, env)
    return run_standard(args, env)


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        print("\n[start] 已中断。")
        raise SystemExit(130)
