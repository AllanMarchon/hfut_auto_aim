#!/usr/bin/env python3
"""Open the auto-aim web visualization without touching the camera.

This script is intentionally a viewer only. The camera and detector must already
be running from scripts/start.py with --web-view enabled.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
import webbrowser


DEFAULT_HOST = os.environ.get("HFUT_AUTO_AIM_HOST", "192.168.137.44")


def fetch_text(url: str, timeout: float) -> str:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.read().decode("utf-8", errors="replace")


def wait_for_status(url: str, timeout_s: float, poll_s: float) -> tuple[bool, str]:
    deadline = time.monotonic() + timeout_s
    last_error = ""
    while True:
        try:
            return True, fetch_text(url, timeout=min(2.0, poll_s))
        except (OSError, urllib.error.URLError, urllib.error.HTTPError) as error:
            last_error = str(error)
            if time.monotonic() >= deadline:
                return False, last_error
            time.sleep(poll_s)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open HFUT auto-aim web visualization.")
    parser.add_argument("--host", default=DEFAULT_HOST, help="NUC address running start.py.")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--wait", type=float, default=3.0, help="Seconds to wait for status.json.")
    parser.add_argument("--no-open", action="store_true", help="Only print/check URLs.")
    parser.add_argument("--status", action="store_true", help="Print status.json and exit.")
    parser.add_argument("--snapshot", action="store_true", help="Open snapshot.jpg instead of the main page.")
    parser.add_argument("--stream", action="store_true", help="Open stream.mjpg directly.")
    args = parser.parse_args(argv)
    if args.port <= 0 or args.port > 65535:
        raise SystemExit("--port must be in 1..65535")
    return args


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    base_url = f"http://{args.host}:{args.port}"
    status_url = f"{base_url}/status.json"
    page_url = f"{base_url}/"
    snapshot_url = f"{base_url}/snapshot.jpg"
    stream_url = f"{base_url}/stream.mjpg"

    ok, payload = wait_for_status(status_url, args.wait, 0.5)
    if not ok:
        print(f"[visualize] web stream is not reachable: {status_url}")
        print(f"[visualize] last error: {payload}")
        print("[visualize] start it first on the NUC, for example:")
        print("  python3 scripts/start.py")
        return 2

    if args.status:
        try:
            print(json.dumps(json.loads(payload), indent=2, ensure_ascii=False))
        except json.JSONDecodeError:
            print(payload)
        return 0

    target_url = page_url
    if args.snapshot:
        target_url = snapshot_url
    elif args.stream:
        target_url = stream_url

    print(f"[visualize] status: {status_url}")
    print(f"[visualize] page:   {page_url}")
    print(f"[visualize] stream: {stream_url}")
    if not args.no_open:
        webbrowser.open(target_url)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
