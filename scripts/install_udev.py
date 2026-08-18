#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""为下位机串口安装固定设备名 udev 规则。"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path


def run(cmd: list[str], *, capture: bool = False) -> subprocess.CompletedProcess[str]:
    print("+", " ".join(cmd), flush=True)
    return subprocess.run(
        cmd,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
        check=False,
    )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="安装 /dev/gimbal 固定串口名。")
    parser.add_argument("--device", default="/dev/ttyACM0", help="当前下位机串口设备。")
    parser.add_argument("--name", default="gimbal", help="生成的 /dev/<name> 名称。")
    parser.add_argument("--vendor-id", default="", help="手动指定 USB vendor id。")
    parser.add_argument("--product-id", default="", help="手动指定 USB product id。")
    parser.add_argument("--serial", default="", help="可选：额外匹配 ID_SERIAL_SHORT。")
    parser.add_argument("--rule-path", default="/etc/udev/rules.d/99-hfut-gimbal.rules")
    parser.add_argument("--dry-run", action="store_true", help="只打印规则，不写入系统。")
    return parser.parse_args(argv)


def udev_properties(device: str) -> dict[str, str]:
    if not shutil.which("udevadm"):
        return {}
    result = run(["udevadm", "info", "-q", "property", "-n", device], capture=True)
    props: dict[str, str] = {}
    for line in (result.stdout or "").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        props[key] = value
    return props


def clean_id(value: str, name: str) -> str:
    value = value.strip().lower()
    if not re.fullmatch(r"[0-9a-f]{4}", value):
        raise SystemExit(f"{name} 必须是 4 位十六进制，例如 1a86")
    return value


def build_rule(args: argparse.Namespace) -> str:
    props = udev_properties(args.device)
    vendor = args.vendor_id or props.get("ID_VENDOR_ID", "")
    product = args.product_id or props.get("ID_MODEL_ID", "")
    serial = args.serial or props.get("ID_SERIAL_SHORT", "")
    if not vendor or not product:
        raise SystemExit(
            "无法自动识别串口 USB VID/PID。请确认设备存在，或手动传 "
            "--vendor-id XXXX --product-id YYYY。"
        )
    vendor = clean_id(vendor, "vendor-id")
    product = clean_id(product, "product-id")

    parts = [
        'SUBSYSTEM=="tty"',
        f'ATTRS{{idVendor}}=="{vendor}"',
        f'ATTRS{{idProduct}}=="{product}"',
    ]
    if serial:
        parts.append(f'ATTRS{{serial}}=="{serial}"')
    parts += [f'SYMLINK+="{args.name}"', 'MODE="0666"', 'GROUP="dialout"']
    return ", ".join(parts) + "\n"


def write_rule(path: Path, content: str) -> None:
    if os.name != "posix":
        raise SystemExit("udev 规则只能在 Linux 上安装。")
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        path.write_text(content, encoding="utf-8")
        return
    if not shutil.which("sudo"):
        raise SystemExit("当前不是 root，且找不到 sudo。请用 sudo 重新运行。")
    proc = subprocess.run(["sudo", "tee", str(path)], input=content, text=True, check=False)
    if proc.returncode != 0:
        raise SystemExit("写入 udev 规则失败。")


def reload_udev() -> None:
    if os.name != "posix":
        return
    prefix = [] if hasattr(os, "geteuid") and os.geteuid() == 0 else ["sudo"]
    run(prefix + ["udevadm", "control", "--reload-rules"])
    run(prefix + ["udevadm", "trigger"])


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    content = build_rule(args)
    print("将安装以下规则：")
    print(content, end="")
    if args.dry_run:
        return 0
    path = Path(args.rule_path)
    write_rule(path, content)
    reload_udev()
    print(f"[OK] 已写入 {path}")
    print(f"[OK] 重新插拔下位机后可使用 /dev/{args.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
