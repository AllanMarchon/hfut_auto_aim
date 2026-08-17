#!/usr/bin/env python3
"""实车一键启动脚本。

test 分支默认启动 SP25 主链：相机和串口沿用本项目适配层，检测、PnP、
跟踪和瞄准使用 SP25。需要回到旧主链时传 --pipeline current。
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

CONFLICT_PATTERNS = [
    "ros2_hik_camera_node",
    "armor_detector_node",
    "gimbal_pipeline_node",
    "rm_serial_driver_node",
    "ros2 launch rm_bringup",
    "MvViewer",
    "bringup_real",
    "bringup_sp25_real",
    "manual_gimbal_test",
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
        raise SystemExit(f"missing: {path}\n{hint}")


def default_onnx_root() -> pathlib.Path:
    env_root = os.environ.get("ONNXRUNTIME_ROOT")
    if env_root:
        return pathlib.Path(env_root).expanduser()
    hfut_root = pathlib.Path.home() / "opt" / "onnxruntime-linux-x64-gpu-1.18.1"
    if hfut_root.exists():
        return hfut_root
    home_root = pathlib.Path.home() / "opt" / "onnxruntime"
    if home_root.exists():
        return home_root
    return pathlib.Path("/opt/onnxruntime-gpu")


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


def make_env(onnx_root: pathlib.Path, mvs_lib_dir: pathlib.Path,
             openvino_lib_dir: pathlib.Path) -> dict[str, str]:
    env = os.environ.copy()
    paths = [
        str(path)
        for path in (onnx_root / "lib", mvs_lib_dir, openvino_lib_dir)
        if path.exists()
    ]
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
        print("[start] Hik camera USB device not found by lsusb.")
        return
    for usb_path in usb_paths:
        print(f"[start] Hik camera USB path: {usb_path}")
        if command_exists("fuser"):
            run(["fuser", "-v", str(usb_path)])


def stop_conflicting_processes() -> None:
    print("[start] stopping known camera/serial conflicts if present...")
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

    print(f"ONNXRUNTIME_ROOT={args.onnx_root}")
    print(f"ONNX header: {args.onnx_root / 'include' / 'onnxruntime_cxx_api.h'}")
    print(f"ONNX library: {args.onnx_root / 'lib' / 'libonnxruntime.so'}")
    print(f"OpenVINO_DIR={args.openvino_dir}")
    print(f"OpenVINO lib dir: {args.openvino_lib_dir}")
    print_camera_owners()
    return run([sys.executable, "scripts/validate_configs.py"]).returncode


def build_project(args: argparse.Namespace, env: dict[str, str]) -> int:
    require_file(args.hik_include / "MvCameraControl.h", "Hik MVS include path is wrong.")
    require_file(args.hik_library, "Hik MVS library path is wrong.")

    if args.pipeline == "current":
        require_file(
            args.onnx_root / "include" / "onnxruntime_cxx_api.h",
            "Install ONNX Runtime first, or pass --onnx-root /path/to/onnxruntime.",
        )
        require_file(
            args.onnx_root / "lib" / "libonnxruntime.so",
            "Install ONNX Runtime first, or pass --onnx-root /path/to/onnxruntime.",
        )

    configure = [
        "cmake",
        "-S",
        str(PROJECT_DIR),
        "-B",
        str(BUILD_DIR),
        "-DHFUT_ENABLE_REAL_IO=ON",
        "-DHFUT_ENABLE_HIK_CAMERA=ON",
        f"-DHFUT_ENABLE_DETECTOR={'ON' if args.pipeline == 'current' else 'OFF'}",
        f"-DHFUT_ENABLE_PIPELINE={'ON' if args.pipeline == 'current' else 'OFF'}",
        f"-DHFUT_ENABLE_SP25={'ON' if args.pipeline == 'sp25' else 'OFF'}",
        f"-DHFUT_HIK_INCLUDE_DIR={args.hik_include}",
        f"-DHFUT_HIK_LIBRARY={args.hik_library}",
        "-DBUILD_TESTING=ON",
        f"-DCMAKE_BUILD_TYPE={args.build_type}",
    ]
    if args.pipeline == "current":
        configure.append(f"-DONNXRUNTIME_ROOT={args.onnx_root}")
    if args.pipeline == "sp25" and args.openvino_dir.exists():
        configure.append(f"-DOpenVINO_DIR={args.openvino_dir}")
    status = run(configure, env=env).returncode
    if status != 0:
        return status
    return run(["cmake", "--build", str(BUILD_DIR), f"-j{args.jobs}"], env=env).returncode


def build_bringup_command(args: argparse.Namespace) -> list[str]:
    exe = BUILD_DIR / ("bringup_sp25_real" if args.pipeline == "sp25" else "bringup_real")
    require_file(exe, f"Run: python3 scripts/start.py --mode build --pipeline {args.pipeline}")

    cmd = [
        str(exe),
        "--hardware-config",
        str(args.hardware_config),
        "--camera-backend",
        args.camera_backend,
        "--web-port",
        str(args.web_port),
    ]
    if args.pipeline == "sp25":
        cmd += ["--sp25-config", str(args.sp25_config)]
        if args.sp25_device:
            cmd += ["--sp25-device", args.sp25_device]
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
    if args.display:
        cmd.append("--display")
    if args.allow_fire:
        cmd.append("--enable-fire")
    cmd += args.extra
    return cmd


def run_bringup(args: argparse.Namespace, env: dict[str, str]) -> int:
    if args.stop_conflicts:
        stop_conflicting_processes()
    print_camera_owners()
    if args.web_view:
        print(f"[start] web viewer URL: http://{args.viewer_host}:{args.web_port}/")
    if args.mode == "live" and not args.allow_fire:
        print("[start] live mode: serial enabled, fire forced OFF.")
    if args.mode == "dry":
        print("[start] dry mode: camera/detector/pipeline enabled, serial disabled.")
    return run(build_bringup_command(args), env=env).returncode


def run_serial_test(args: argparse.Namespace, env: dict[str, str]) -> int:
    exe = BUILD_DIR / "serial_transport_test"
    require_file(exe, "Run: python3 scripts/start.py --mode build")
    cmd = [
        str(exe),
        args.serial_port,
        str(args.baudrate),
        args.serial_tx_protocol,
        args.serial_rx_protocol,
        args.infantry32_tail_fields,
        args.command_unit,
    ]
    return run(cmd, env=env).returncode


def run_manual_gimbal_test(args: argparse.Namespace, env: dict[str, str]) -> int:
    exe = BUILD_DIR / "manual_gimbal_test"
    require_file(exe, "Run: python3 scripts/start.py --mode build")
    if args.stop_conflicts:
        stop_conflicting_processes()
    cmd = [
        str(exe),
        "--port",
        args.serial_port,
        "--baudrate",
        str(args.baudrate),
        "--yaw-range-deg",
        str(args.manual_yaw_range_deg),
        "--pitch-range-deg",
        str(args.manual_pitch_range_deg),
        "--hz",
        str(args.manual_hz),
        "--distance",
        str(args.manual_distance),
        "--max-rate-rad-s",
        str(args.manual_max_rate_rad_s),
        "--max-acc-rad-s2",
        str(args.manual_max_acc_rad_s2),
    ]
    if args.manual_no_feedback:
        cmd.append("--no-feedback")
    return run(cmd, env=env).returncode


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="启动 HFUT 实车自瞄主链。")
    parser.add_argument("--pipeline", choices=("sp25", "current"), default="sp25",
                        help="sp25 为 test 分支默认主链；current 保留旧主链。")
    parser.add_argument(
        "--mode",
        choices=("dry", "live", "build", "check", "serial-test", "manual-gimbal"),
        default="live",
        help="dry 不发串口；live 打开串口，但未传 --allow-fire 时仍关闭开火。",
    )
    parser.add_argument("--camera-backend", default="hik", choices=("hik", "opencv", "mindvision"))
    parser.add_argument("--hardware-config", type=pathlib.Path, default=PROJECT_DIR / "configs" / "hardware.yaml")
    parser.add_argument("--sp25-config", type=pathlib.Path, default=PROJECT_DIR / "configs" / "sp25_real.yaml")
    parser.add_argument("--sp25-device", default="", help="覆盖 SP25 OpenVINO device，例如 GPU 或 CPU。")
    parser.add_argument("--onnx-root", type=pathlib.Path, default=default_onnx_root())
    parser.add_argument("--openvino-dir", type=pathlib.Path, default=default_openvino_dir())
    parser.add_argument("--openvino-lib-dir", type=pathlib.Path, default=default_openvino_lib_dir())
    parser.add_argument("--hik-include", type=pathlib.Path, default=pathlib.Path("/opt/MVS/include"))
    parser.add_argument("--hik-library", type=pathlib.Path, default=pathlib.Path("/opt/MVS/lib/64/libMvCameraControl.so"))
    parser.add_argument("--mvs-lib-dir", type=pathlib.Path, default=pathlib.Path("/opt/MVS/lib/64"))
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--build-type", default="Release")
    parser.add_argument("--max-frames", type=int, default=-1, help="-1 表示一直运行直到 Ctrl+C。")
    parser.add_argument("--enemy-color", choices=("red", "blue", "white"), default=None)
    parser.add_argument("--exposure-time-us", type=float, default=None)
    parser.add_argument("--gain", type=float, default=None)
    parser.add_argument("--display", action="store_true", help="打开 OpenCV 调试窗口，SSH 下通常不要开。")
    parser.add_argument("--no-web-view", dest="web_view", action="store_false", help="关闭 MJPEG Web 调试流。")
    parser.set_defaults(web_view=True)
    parser.add_argument("--web-host", default="0.0.0.0")
    parser.add_argument("--viewer-host", default=os.environ.get("HFUT_AUTO_AIM_HOST", "192.168.137.44"))
    parser.add_argument("--web-port", type=int, default=8080)
    parser.add_argument("--web-frame-step", type=int, default=2)
    parser.add_argument("--no-stop-conflicts", dest="stop_conflicts", action="store_false")
    parser.set_defaults(stop_conflicts=True)
    parser.add_argument("--allow-fire", action="store_true", help="危险开关：允许把开火建议发给下位机。")
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--serial-tx-protocol", default="infantry_32")
    parser.add_argument("--serial-rx-protocol", default="infantry")
    parser.add_argument("--infantry32-tail-fields", default="acceleration")
    parser.add_argument("--command-unit", choices=("radians", "degrees"), default="radians")
    parser.add_argument("--manual-yaw-range-deg", type=float, default=20.0)
    parser.add_argument("--manual-pitch-range-deg", type=float, default=15.0)
    parser.add_argument("--manual-hz", type=float, default=100.0)
    parser.add_argument("--manual-distance", type=float, default=1.0)
    parser.add_argument("--manual-max-rate-rad-s", type=float, default=5.0)
    parser.add_argument("--manual-max-acc-rad-s2", type=float, default=80.0)
    parser.add_argument("--manual-no-feedback", action="store_true")
    parser.add_argument("extra", nargs=argparse.REMAINDER, help="-- 后面的额外参数会传给当前主链入口。")
    args = parser.parse_args(argv)
    if args.extra and args.extra[0] == "--":
        args.extra = args.extra[1:]
    args.onnx_root = args.onnx_root.expanduser().resolve()
    args.hardware_config = args.hardware_config.expanduser().resolve()
    args.sp25_config = args.sp25_config.expanduser().resolve()
    args.openvino_dir = args.openvino_dir.expanduser().resolve()
    args.openvino_lib_dir = args.openvino_lib_dir.expanduser().resolve()
    args.hik_include = args.hik_include.expanduser().resolve()
    args.hik_library = args.hik_library.expanduser().resolve()
    args.mvs_lib_dir = args.mvs_lib_dir.expanduser().resolve()
    if args.pipeline == "sp25" and args.camera_backend == "mindvision":
        raise SystemExit("SP25 entrypoint currently supports --camera-backend hik or opencv")
    if args.pipeline == "sp25" and args.enemy_color == "white":
        raise SystemExit("SP25 entrypoint currently supports --enemy-color red or blue")
    if args.allow_fire and args.mode != "live":
        raise SystemExit("--allow-fire is only valid with --mode live")
    if args.web_port <= 0 or args.web_port > 65535:
        raise SystemExit("--web-port must be in 1..65535")
    if args.web_frame_step <= 0:
        raise SystemExit("--web-frame-step must be > 0")
    return args


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    env = make_env(args.onnx_root, args.mvs_lib_dir, args.openvino_lib_dir)
    if args.mode == "check":
        return check_environment(args)
    if args.mode == "build":
        return build_project(args, env)
    if args.mode == "serial-test":
        return run_serial_test(args, env)
    if args.mode == "manual-gimbal":
        return run_manual_gimbal_test(args, env)
    return run_bringup(args, env)


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        print("\n[start] interrupted.")
        raise SystemExit(130)
