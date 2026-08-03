#!/usr/bin/env python3
"""Launch the Windows Gestalt proxy over WSL interop stdio, without TCP."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


PROJECT_DIR = Path(__file__).resolve().parents[1]


def windows_path(path: Path) -> str:
    return subprocess.check_output(
        ["wslpath", "-w", str(path)], text=True).strip()


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Run Gestalt Windows proxy and hfut_auto_aim over WSL interop pipes")
    parser.add_argument(
        "--windows-python", default=os.environ.get(
            "GESTALT_WINDOWS_PYTHON", "/mnt/c/Windows/py.exe"))
    parser.add_argument("--debug", action="store_true")
    parser.add_argument(
        "--allow-fire", action="store_true",
        help="allow fire commands; the default forces fire=0")
    parser.add_argument("--max-fps", type=float, default=60.0)
    parser.add_argument(
        "--frame-codec", choices=("raw", "lz4"), default="raw",
        help="stdio is fast enough for raw frames; LZ4 remains available")
    parser.add_argument("--game-pid", type=int, default=0)
    parser.add_argument("--endpoint-timeout", type=float, default=60.0)
    parser.add_argument("--entity-id", type=int, default=66000005)
    parser.add_argument("--team-id", type=int, default=0)
    parser.add_argument("--allowance", type=int, default=400)
    parser.add_argument(
        "--existing-match", action="store_true",
        help="skip prepare-match/start-match and attach to an existing controllable player")
    parser.add_argument(
        "bringup_args", nargs=argparse.REMAINDER,
        help="arguments after -- are passed to scripts/run.sh")
    return parser.parse_args()


def stop_process(process: subprocess.Popen | None, timeout_s: float = 3.0) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=timeout_s)


def main() -> int:
    arguments = parse_arguments()
    windows_python = Path(arguments.windows_python)
    if not windows_python.exists():
        raise SystemExit(f"Windows Python not found: {windows_python}")

    proxy_command = [
        str(windows_python), "-3",
        windows_path(PROJECT_DIR / "tools" / "gestalt_bridge_windows.py"),
        "auto", "--transport", "stdio",
        "--player-id", "0",
        "--entity-id", str(arguments.entity_id),
        "--team-id", str(arguments.team_id),
        "--allowance", str(arguments.allowance),
        "--frame-width", "1280", "--frame-height", "720",
        "--fov", "25", "--shutter-speed", "120", "--iso", "600",
        "--arm-length", "0",
        "--max-fps", str(arguments.max_fps),
        "--frame-codec", arguments.frame_codec,
        "--endpoint-timeout", str(arguments.endpoint_timeout),
    ]
    if arguments.game_pid > 0:
        proxy_command += ["--game-pid", str(arguments.game_pid)]
    if not arguments.existing_match:
        proxy_command += ["--prepare-match", "--start-match"]
    # if not arguments.allow_fire:
    #     proxy_command.append("--no-fire")

    proxy = subprocess.Popen(
        proxy_command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=None, bufsize=0)
    if proxy.stdin is None or proxy.stdout is None:
        stop_process(proxy)
        raise SystemExit("failed to create Windows proxy interop pipes")

    frame_fd = proxy.stdout.fileno()
    command_fd = proxy.stdin.fileno()
    bringup_command = [
        str(PROJECT_DIR / "scripts" / "run.sh"),
        "--gestalt",
        f"--gestalt-read-fd={frame_fd}",
        f"--gestalt-write-fd={command_fd}",
    ]
    if arguments.debug:
        bringup_command.append("--debug")
    bringup_args = arguments.bringup_args
    if bringup_args and bringup_args[0] == "--":
        bringup_args = bringup_args[1:]
    bringup_command.extend(bringup_args)

    bringup = None
    try:
        bringup = subprocess.Popen(
            bringup_command, pass_fds=(frame_fd, command_fd), close_fds=True)
        # Only bringup_sim should keep the Linux ends. Closing the launcher's
        # copies ensures Windows observes EOF immediately when bringup exits.
        proxy.stdout.close()
        proxy.stdin.close()

        while True:
            bringup_result = bringup.poll()
            proxy_result = proxy.poll()
            if bringup_result is not None:
                try:
                    proxy.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    pass
                return bringup_result
            if proxy_result is not None:
                bringup.send_signal(signal.SIGTERM)
                bringup.wait(timeout=3.0)
                return proxy_result or 1
            time.sleep(0.1)
    except KeyboardInterrupt:
        if bringup is not None and bringup.poll() is None:
            bringup.send_signal(signal.SIGINT)
            try:
                bringup.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                stop_process(bringup)
        try:
            proxy.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            pass
        return 130
    finally:
        stop_process(bringup)
        stop_process(proxy)


if __name__ == "__main__":
    raise SystemExit(main())
