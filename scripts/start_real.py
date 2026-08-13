#!/usr/bin/env python3
"""One-command helper for real-vehicle bringup on the NUC.

Default mode is intentionally safe: camera + detector + pipeline only,
with bringup_real --dry-run. Use --mode live to open serial. This script
does not enable fire unless --allow-fire is passed explicitly.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import shutil
import subprocess
import sys


PROJECT_DIR = pathlib.Path(__file__).resolve().parents[1]
BUILD_DIR = PROJECT_DIR / "build"


def existing_path(*candidates: pathlib.Path) -> pathlib.Path | None:
    for path in candidates:
        if path.exists():
            return path
    return None


def prepend_ld_library_path(env: dict[str, str], *paths: pathlib.Path) -> None:
    entries = [str(path) for path in paths if path.exists()]
    current = env.get("LD_LIBRARY_PATH", "")
    if current:
        entries.append(current)
    env["LD_LIBRARY_PATH"] = ":".join(entries)


def run(cmd: list[str], env: dict[str, str] | None = None) -> int:
    print("+", " ".join(cmd), flush=True)
    return subprocess.call(cmd, cwd=PROJECT_DIR, env=env)


def require_file(path: pathlib.Path, hint: str) -> None:
    if not path.exists():
        raise SystemExit(f"missing: {path}\n{hint}")


def default_onnx_root() -> pathlib.Path:
    env_root = os.environ.get("ONNXRUNTIME_ROOT")
    if env_root:
        return pathlib.Path(env_root).expanduser()
    home_root = pathlib.Path.home() / "opt" / "onnxruntime"
    if home_root.exists():
        return home_root
    return pathlib.Path("/opt/onnxruntime-gpu")


def make_env(onnx_root: pathlib.Path, mvs_lib_dir: pathlib.Path) -> dict[str, str]:
    env = os.environ.copy()
    prepend_ld_library_path(env, onnx_root / "lib", mvs_lib_dir)
    return env


def check_environment(args: argparse.Namespace) -> int:
    print("== system ==")
    run(["uname", "-m"])
    run(["bash", "-lc", "cat /etc/os-release | head -5"])
    run(["bash", "-lc", "g++ --version | head -1"])
    run(["bash", "-lc", "cmake --version | head -1"])
    run(["bash", "-lc", "pkg-config --modversion opencv4 || true"])

    print("== camera / MVS ==")
    run(["bash", "-lc", "lsusb | grep -Ei 'Hikrobot|2bdf|camera' || true"])
    run(["bash", "-lc", "find /opt /usr /usr/local -name MvCameraControl.h 2>/dev/null | head"])
    run(["bash", "-lc", "find /opt /usr /usr/local -name 'libMvCameraControl.so*' 2>/dev/null | head"])

    print("== ONNX Runtime ==")
    print(f"ONNXRUNTIME_ROOT={args.onnx_root}")
    print("header:", args.onnx_root / "include" / "onnxruntime_cxx_api.h")
    print("library:", args.onnx_root / "lib" / "libonnxruntime.so")

    print("== repo config ==")
    return run([sys.executable, "scripts/validate_configs.py"])


def build_project(args: argparse.Namespace, env: dict[str, str]) -> int:
    require_file(
        args.onnx_root / "include" / "onnxruntime_cxx_api.h",
        "Install ONNX Runtime first, or pass --onnx-root /path/to/onnxruntime.",
    )
    require_file(
        args.onnx_root / "lib" / "libonnxruntime.so",
        "Install ONNX Runtime first, or pass --onnx-root /path/to/onnxruntime.",
    )
    require_file(args.hik_include / "MvCameraControl.h", "Hik MVS SDK include path is wrong.")
    require_file(args.hik_library, "Hik MVS SDK library path is wrong.")

    configure = [
        "cmake",
        "-S",
        str(PROJECT_DIR),
        "-B",
        str(BUILD_DIR),
        "-DHFUT_ENABLE_REAL_IO=ON",
        "-DHFUT_ENABLE_HIK_CAMERA=ON",
        f"-DHFUT_HIK_INCLUDE_DIR={args.hik_include}",
        f"-DHFUT_HIK_LIBRARY={args.hik_library}",
        f"-DONNXRUNTIME_ROOT={args.onnx_root}",
        "-DBUILD_TESTING=ON",
        f"-DCMAKE_BUILD_TYPE={args.build_type}",
    ]
    status = run(configure, env)
    if status != 0:
        return status
    return run(["cmake", "--build", str(BUILD_DIR), f"-j{args.jobs}"], env)


def run_bringup(args: argparse.Namespace, env: dict[str, str]) -> int:
    exe = BUILD_DIR / "bringup_real"
    require_file(exe, "Run: python3 scripts/start_real.py --mode build")

    cmd = [
        str(exe),
        "--hardware-config",
        str(args.hardware_config),
        "--camera-backend",
        args.camera_backend,
    ]
    if args.mode == "dry":
        cmd.append("--dry-run")
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
    return run(cmd, env)


def run_serial_test(args: argparse.Namespace, env: dict[str, str]) -> int:
    exe = BUILD_DIR / "serial_transport_test"
    require_file(exe, "Run: python3 scripts/start_real.py --mode build")
    cmd = [
        str(exe),
        args.serial_port,
        str(args.baudrate),
        args.serial_tx_protocol,
        args.serial_rx_protocol,
        args.infantry32_tail_fields,
        args.command_unit,
    ]
    return run(cmd, env)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build and run hfut_auto_aim real-vehicle entry."
    )
    parser.add_argument(
        "--mode",
        choices=("dry", "live", "build", "check", "serial-test"),
        default="dry",
        help="dry: camera/detector only; live: also open serial; build/check helpers.",
    )
    parser.add_argument("--camera-backend", default="hik", choices=("hik", "opencv", "mindvision"))
    parser.add_argument("--hardware-config", type=pathlib.Path, default=PROJECT_DIR / "configs" / "hardware.yaml")
    parser.add_argument("--onnx-root", type=pathlib.Path, default=default_onnx_root())
    parser.add_argument("--hik-include", type=pathlib.Path, default=pathlib.Path("/opt/MVS/include"))
    parser.add_argument("--hik-library", type=pathlib.Path, default=pathlib.Path("/opt/MVS/lib/64/libMvCameraControl.so"))
    parser.add_argument("--mvs-lib-dir", type=pathlib.Path, default=pathlib.Path("/opt/MVS/lib/64"))
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--build-type", default="Release")
    parser.add_argument("--max-frames", type=int, default=300, help="Use -1 to run until Ctrl+C.")
    parser.add_argument("--enemy-color", choices=("red", "blue", "white"), default=None)
    parser.add_argument("--exposure-time-us", type=float, default=None)
    parser.add_argument("--gain", type=float, default=None)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--allow-fire", action="store_true", help="Pass --enable-fire to bringup_real. Dangerous.")
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--serial-tx-protocol", default="infantry_32")
    parser.add_argument("--serial-rx-protocol", default="infantry")
    parser.add_argument("--infantry32-tail-fields", default="acceleration")
    parser.add_argument("--command-unit", choices=("radians", "degrees"), default="radians")
    parser.add_argument("extra", nargs=argparse.REMAINDER, help="Extra args after -- are passed to bringup_real.")
    args = parser.parse_args(argv)
    if args.extra and args.extra[0] == "--":
        args.extra = args.extra[1:]
    args.onnx_root = args.onnx_root.expanduser().resolve()
    args.hardware_config = args.hardware_config.expanduser().resolve()
    args.hik_include = args.hik_include.expanduser().resolve()
    args.hik_library = args.hik_library.expanduser().resolve()
    args.mvs_lib_dir = args.mvs_lib_dir.expanduser().resolve()
    return args


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    env = make_env(args.onnx_root, args.mvs_lib_dir)
    if args.allow_fire and args.mode != "live":
        raise SystemExit("--allow-fire is only valid with --mode live")
    if args.mode == "check":
        return check_environment(args)
    if args.mode == "build":
        return build_project(args, env)
    if args.mode == "serial-test":
        return run_serial_test(args, env)
    if args.mode in ("dry", "live"):
        return run_bringup(args, env)
    raise SystemExit(f"unsupported mode: {args.mode}")


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
