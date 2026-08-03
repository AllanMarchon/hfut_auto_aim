#!/usr/bin/env python3
"""Windows-side Gestalt shared-memory/WebSocket to WSL2 TCP proxy.

The game publishes final viewport pixels and the exact FSceneView metadata in
process-scoped shared memory. This proxy forwards that frame-atomic packet to
hfut_auto_aim and translates returned right-handed radians into RBExtAim's UE
left-handed degree convention. No target ground truth crosses this bridge.
"""

from __future__ import annotations

import argparse
import ctypes
from ctypes import wintypes
import json
import math
import os
import queue
import socket
import struct
import subprocess
import sys
import threading
import time


ENVELOPE_MAGIC = b"HFGNET1\0"
ENVELOPE_VERSION = 1
MESSAGE_FRAME = 1
MESSAGE_COMMAND = 2
ENVELOPE = struct.Struct("<8sIIQQ")
FRAME_METADATA = struct.Struct("<QddIIIIQdd3d4dIIIiiII")
COMMAND_PACKET = struct.Struct("<8sIIQd9dBbHI")

FRAME_MAPPING_MAGIC = 0x314D52464E565347
FRAME_MAPPING_VERSION = 3
REGION_HEADER_BYTES = 4096
SLOT_HEADER_BYTES = 256
REGION_PREFIX = struct.Struct("<Q7I4xQQQ")

FILE_MAP_READ = 0x0004
AF_INET = 2
TCP_TABLE_OWNER_PID_LISTENER = 3
NO_ERROR = 0
ERROR_INSUFFICIENT_BUFFER = 122
PROCESS_QUERY_LIMITED_INFORMATION = 0x1000

GESTALT_PROCESS_NAMES = {
    "robotbridgedemo-win64-shipping.exe",
    "robotbridgedemo-win64-development.exe",
    "robotbridgedemo.exe",
}

A_MAP_PTR = 1000001
A_HEALTH = 10000003
A_ALLOWANCE_17MM = 10000033
A_PLAYER_ID = 10000035
A_TEAM_ID = 10000036
A_CONNECTION_ENTITY_CONFIG_ID = 10000064
A_CLASS = 60000002
A_IS_AI = 50000088
A_MATCH_STATUS = 80000005

PIXEL_FORMAT_BGRA8 = 1
PIXEL_FORMAT_RGBA8 = 2
PIXEL_FORMAT_A2B10G10R10 = 3
PIXEL_FORMAT_LZ4_OFFSET = 100


def require_windows() -> None:
    if os.name != "nt":
        raise SystemExit("gestalt_bridge_windows.py must run with Windows Python")


def ipv4_listeners() -> list[tuple[str, int, int]]:
    """Return loopback/any-address IPv4 listeners as (address, port, pid)."""
    iphlpapi = ctypes.WinDLL("iphlpapi", use_last_error=True)
    get_table = iphlpapi.GetExtendedTcpTable
    get_table.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(wintypes.ULONG),
        wintypes.BOOL,
        wintypes.ULONG,
        ctypes.c_int,
        wintypes.ULONG,
    ]
    get_table.restype = wintypes.DWORD

    size = wintypes.ULONG(0)
    result = get_table(None, ctypes.byref(size), False, AF_INET,
                       TCP_TABLE_OWNER_PID_LISTENER, 0)
    if result != ERROR_INSUFFICIENT_BUFFER or size.value == 0:
        return []
    storage = ctypes.create_string_buffer(size.value)
    result = get_table(storage, ctypes.byref(size), False, AF_INET,
                       TCP_TABLE_OWNER_PID_LISTENER, 0)
    if result != NO_ERROR:
        return []

    count = struct.unpack_from("<I", storage, 0)[0]
    row = struct.Struct("<6I")
    listeners = []
    for index in range(count):
        state, local_addr, local_port, _, _, pid = row.unpack_from(
            storage, 4 + index * row.size)
        del state
        decoded_port = socket.ntohs(local_port & 0xFFFF)
        if local_addr in (0, 0x0100007F):
            address = "0.0.0.0" if local_addr == 0 else "127.0.0.1"
            listeners.append((address, decoded_port, int(pid)))
    return listeners


def listener_pid_for_port(port: int) -> int:
    """Return the exact process owning an IPv4 loopback game listener."""
    for _, candidate_port, pid in ipv4_listeners():
        if candidate_port == port:
            return pid
    return 0


def process_image_name(process_id: int) -> str:
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    kernel32.OpenProcess.restype = wintypes.HANDLE
    kernel32.QueryFullProcessImageNameW.argtypes = [
        wintypes.HANDLE, wintypes.DWORD, wintypes.LPWSTR,
        ctypes.POINTER(wintypes.DWORD),
    ]
    kernel32.QueryFullProcessImageNameW.restype = wintypes.BOOL
    kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel32.CloseHandle.restype = wintypes.BOOL

    process = kernel32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, False, process_id)
    if not process:
        return ""
    try:
        buffer = ctypes.create_unicode_buffer(32768)
        size = wintypes.DWORD(len(buffer))
        if not kernel32.QueryFullProcessImageNameW(
                process, 0, buffer, ctypes.byref(size)):
            return ""
        return os.path.basename(buffer.value)
    finally:
        kernel32.CloseHandle(process)


def is_gestalt_process_name(name: str) -> bool:
    lowered = name.replace("\\", "/").rsplit("/", 1)[-1].lower()
    return lowered in GESTALT_PROCESS_NAMES or (
        lowered.startswith("robotbridgedemo-win64-") and lowered.endswith(".exe"))


def parse_ws_port(value: str) -> int:
    if value.lower() == "auto":
        return 0
    try:
        port = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("ws_port must be 'auto' or an integer") from error
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("ws_port must be in [1, 65535]")
    return port


def resolve_game_endpoint(requested_port: int, requested_pid: int,
                          timeout_s: float) -> tuple[int, int]:
    """Resolve and WebSocket-probe the current Gestalt game listener."""
    from websockets.sync.client import connect as ws_connect

    deadline = time.monotonic() + timeout_s
    last_candidates = []
    while time.monotonic() < deadline:
        listeners = ipv4_listeners()
        if requested_port:
            candidates = [entry for entry in listeners if entry[1] == requested_port]
        else:
            names = {}
            candidates = []
            for entry in listeners:
                _, _, pid = entry
                if requested_pid and pid != requested_pid:
                    continue
                name = names.setdefault(pid, process_image_name(pid))
                if requested_pid or is_gestalt_process_name(name):
                    candidates.append(entry)
        last_candidates = candidates
        for _, port, pid in candidates:
            if requested_pid and pid != requested_pid:
                continue
            try:
                probe = ws_connect(
                    f"ws://127.0.0.1:{port}/", open_timeout=0.5,
                    close_timeout=0.2, max_size=None)
                probe.close()
                return port, pid
            except Exception:
                continue
        time.sleep(0.25)

    target = f"port {requested_port}" if requested_port else "the Gestalt process"
    detail = ", ".join(f"{port}/pid={pid}" for _, port, pid in last_candidates)
    suffix = f"; listener candidates: {detail}" if detail else ""
    raise RuntimeError(f"no usable loopback WebSocket found for {target}{suffix}")


class SharedFrameCapture:
    def __init__(self, process_id: int, timeout_s: float):
        self.process_id = process_id
        self.last_commit = 0
        self._mapping = None
        self._base = None
        self._region = None
        self._open(timeout_s)

    def _open(self, timeout_s: float) -> None:
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.OpenFileMappingW.argtypes = [wintypes.DWORD, wintypes.BOOL,
                                               wintypes.LPCWSTR]
        kernel32.OpenFileMappingW.restype = wintypes.HANDLE
        kernel32.MapViewOfFile.argtypes = [wintypes.HANDLE, wintypes.DWORD,
                                           wintypes.DWORD, wintypes.DWORD,
                                           ctypes.c_size_t]
        kernel32.MapViewOfFile.restype = ctypes.c_void_p
        kernel32.UnmapViewOfFile.argtypes = [ctypes.c_void_p]
        kernel32.UnmapViewOfFile.restype = wintypes.BOOL
        kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
        kernel32.CloseHandle.restype = wintypes.BOOL
        self._kernel32 = kernel32

        name = "{47534652-414D-4501-0000-%012X}" % self.process_id
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            mapping = kernel32.OpenFileMappingW(FILE_MAP_READ, False, name)
            if mapping:
                self._mapping = mapping
                break
            time.sleep(0.025)
        if not self._mapping:
            raise RuntimeError(
                f"Gestalt frame mapping {name} is unavailable; use a build with shared-frame v3")

        self._base = kernel32.MapViewOfFile(
            self._mapping, FILE_MAP_READ, 0, 0, 0)
        if not self._base:
            self.close()
            raise RuntimeError("MapViewOfFile failed")

        prefix = ctypes.string_at(self._base, REGION_PREFIX.size)
        values = REGION_PREFIX.unpack(prefix)
        (magic, version, header_bytes, slot_count, slot_header_bytes,
         slot_stride, max_width, max_height, region_bytes, writer_pid,
         qpc_frequency) = values
        if (magic != FRAME_MAPPING_MAGIC or version != FRAME_MAPPING_VERSION or
                header_bytes != REGION_HEADER_BYTES or
                slot_header_bytes != SLOT_HEADER_BYTES or
                not 0 < slot_count <= 16 or writer_pid != self.process_id or
                qpc_frequency == 0 or region_bytes < header_bytes):
            self.close()
            raise RuntimeError("Gestalt shared-frame v3 header is invalid")
        self._region = {
            "header_bytes": header_bytes,
            "slot_count": slot_count,
            "slot_stride": slot_stride,
            "max_width": max_width,
            "max_height": max_height,
            "writer_pid": writer_pid,
            "qpc_frequency": qpc_frequency,
        }

    def close(self) -> None:
        if self._base:
            self._kernel32.UnmapViewOfFile(ctypes.c_void_p(self._base))
            self._base = None
        if self._mapping:
            self._kernel32.CloseHandle(self._mapping)
            self._mapping = None

    def __del__(self):
        self.close()

    def grab(self, timeout_s: float = 0.1):
        if not self._base or not self._region:
            return None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            best = None
            best_commit = self.last_commit
            for index in range(self._region["slot_count"]):
                slot = (self._base + self._region["header_bytes"] +
                        index * self._region["slot_stride"])
                commit = ctypes.c_int64.from_address(slot).value
                if commit <= 0 or commit & 1 or commit <= best_commit:
                    continue
                header = ctypes.string_at(slot, SLOT_HEADER_BYTES)
                width = struct.unpack_from("<I", header, 76)[0]
                height = struct.unpack_from("<I", header, 80)[0]
                row_bytes = struct.unpack_from("<I", header, 84)[0]
                pixel_bytes = struct.unpack_from("<I", header, 88)[0]
                if (width == 0 or height == 0 or
                        width > self._region["max_width"] or
                        height > self._region["max_height"] or
                        row_bytes != width * 4 or
                        pixel_bytes != row_bytes * height or
                        pixel_bytes > self._region["slot_stride"] - SLOT_HEADER_BYTES):
                    continue
                best = (slot, header, width, height, row_bytes, pixel_bytes)
                best_commit = commit
            if best is None:
                time.sleep(0.001)
                continue

            slot, header, width, height, row_bytes, pixel_bytes = best
            pixels = ctypes.string_at(slot + SLOT_HEADER_BYTES, pixel_bytes)
            if ctypes.c_int64.from_address(slot).value != best_commit:
                continue

            engine_frame = struct.unpack_from("<Q", header, 8)[0]
            capture_qpc = struct.unpack_from("<q", header, 16)[0]
            world_time = struct.unpack_from("<d", header, 24)[0]
            camera_position = struct.unpack_from("<3f", header, 36)
            camera_quaternion = struct.unpack_from("<4f", header, 48)
            fov_degrees = struct.unpack_from("<f", header, 64)[0]
            pixel_format = struct.unpack_from("<I", header, 92)[0]
            arm_length = struct.unpack_from("<f", header, 96)[0]
            view_actor = struct.unpack_from("<I", header, 100)[0]
            takeover_target = struct.unpack_from("<I", header, 104)[0]
            takeover_player = struct.unpack_from("<i", header, 108)[0]
            takeover_map = struct.unpack_from("<i", header, 112)[0]
            takeover_epoch = struct.unpack_from("<I", header, 116)[0]
            identity_flags = struct.unpack_from("<I", header, 120)[0]
            self.last_commit = best_commit
            return {
                "seq": engine_frame,
                "capture_time_s": capture_qpc / self._region["qpc_frequency"],
                "world_time_s": world_time,
                "width": width,
                "height": height,
                "row_bytes": row_bytes,
                "pixel_bytes": pixel_bytes,
                "pixel_format": pixel_format,
                "fov_degrees": fov_degrees,
                "camera_arm_length_cm": arm_length,
                "camera_position": camera_position,
                "camera_quaternion": camera_quaternion,
                "writer_pid": self._region["writer_pid"],
                "view_actor": view_actor,
                "takeover_target": takeover_target,
                "takeover_player": takeover_player,
                "takeover_map": takeover_map,
                "takeover_epoch": takeover_epoch,
                "identity_flags": identity_flags,
                "pixels": pixels,
            }
        return None


class GameControl:
    """Single-threaded WS owner with latest-command replacement and claim lease."""

    def __init__(self, ws_port: int, player_id: int, camera: dict,
                 claim_interval_s: float = 1.0, prepare_match: bool = False,
                 entity_id: int = 66000005, team_id: int = 0,
                 allowance: int = 400, max_fps: float = 60.0):
        self.ws_port = ws_port
        self.player_id = player_id
        self.camera = camera
        self.claim_interval_s = claim_interval_s
        self.prepare_match = prepare_match
        self.entity_id = entity_id
        self.team_id = team_id
        self.allowance = allowance
        self.max_fps = max_fps
        self._lock = threading.Lock()
        self._latest = None
        self._latest_generation = 0
        self._sent_generation = 0
        self._last_aim = None
        self._failure = None
        self._maps = {}
        self._watched = set()
        self._prepared = threading.Event()
        self._frame_contract = threading.Event()
        self._start_match_requested = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="gestalt-ws", daemon=True)
        self._thread.start()

    def aim(self, yaw_degrees: float, pitch_degrees: float, fire: bool,
            yaw_velocity_deg_s: float = 0.0,
            pitch_velocity_deg_s: float = 0.0) -> None:
        with self._lock:
            self._latest_generation += 1
            self._latest = (
                yaw_degrees, pitch_degrees, bool(fire),
                yaw_velocity_deg_s, pitch_velocity_deg_s)

    def clear_fire(self) -> None:
        with self._lock:
            if self._last_aim is None:
                return
            yaw, pitch, _, _, _ = self._last_aim
            self._latest_generation += 1
            self._latest = (yaw, pitch, False, 0.0, 0.0)

    def frame_contract_passed(self) -> None:
        self._frame_contract.set()

    def start_match(self) -> None:
        self._start_match_requested.set()

    def failure_reason(self) -> str | None:
        with self._lock:
            return self._failure

    def _fail(self, reason: str) -> None:
        with self._lock:
            if self._failure is None:
                self._failure = reason
        print(f"[gestalt-win] match preparation failed: {reason}",
              file=sys.stderr, flush=True)
        self._stop.set()

    def _update_telemetry(self, payload: dict) -> set[int]:
        if (payload.get("type") != 0 or
                payload.get("method") != "watchAttributeMaps.result"):
            return set()
        discovered = set()
        results = payload.get("params", {}).get("watch_attribute_maps_results", [])
        if not isinstance(results, list):
            return discovered
        for result in results:
            if not isinstance(result, dict):
                continue
            map_id = result.get("attribute_map_id")
            attributes = result.get("attributes") or {}
            if not isinstance(map_id, int) or not isinstance(attributes, dict):
                continue
            target = self._maps.setdefault(map_id, {})
            for key, value in attributes.items():
                try:
                    attribute_id = int(key)
                except (TypeError, ValueError):
                    continue
                if not isinstance(value, (int, float)):
                    continue
                target[attribute_id] = value
                if ((attribute_id == A_MAP_PTR or 0 <= attribute_id < 100000) and
                        value > 0):
                    discovered.add(int(value))
        return discovered

    def _match_status(self) -> float | None:
        for attributes in self._maps.values():
            if A_MATCH_STATUS in attributes:
                return float(attributes[A_MATCH_STATUS])
        return None

    def _player_entity_config(self) -> int | None:
        for attributes in self._maps.values():
            pointer = attributes.get(self.player_id)
            if not isinstance(pointer, (int, float)) or pointer <= 0:
                continue
            player_map = self._maps.get(int(pointer), {})
            value = player_map.get(A_CONNECTION_ENTITY_CONFIG_ID)
            if isinstance(value, (int, float)):
                return int(value)
        return None

    def _current_player_map(self) -> int | None:
        for attributes in self._maps.values():
            player_pointer = attributes.get(self.player_id)
            if not isinstance(player_pointer, (int, float)) or player_pointer <= 0:
                continue
            player_map = self._maps.get(int(player_pointer), {})
            battle_pointer = player_map.get(A_MAP_PTR)
            if not isinstance(battle_pointer, (int, float)) or battle_pointer <= 0:
                continue
            battle_map_id = int(battle_pointer)
            battle_map = self._maps.get(battle_map_id, {})
            if battle_map.get(A_PLAYER_ID) == self.player_id:
                return battle_map_id

        candidates = []
        for map_id, attributes in self._maps.items():
            if (attributes.get(A_PLAYER_ID) == self.player_id and
                    attributes.get(A_HEALTH, 0) > 0):
                candidates.append(map_id)
        return max(candidates) if candidates else None

    def _expected_spawn_map(self, previous_map: int) -> int | None:
        map_id = self._current_player_map()
        if map_id is None or map_id <= previous_map:
            return None
        attributes = self._maps.get(map_id, {})
        if (attributes.get(A_HEALTH, 0) <= 0 or
                attributes.get(A_PLAYER_ID) != self.player_id or
                attributes.get(A_TEAM_ID) != self.team_id or
                attributes.get(A_CLASS) != 1004 or
                self._player_entity_config() != self.entity_id):
            return None
        return map_id

    @staticmethod
    def _message(request_id: int, method: str, params: dict) -> str:
        return json.dumps({"type": 0, "id": request_id,
                           "method": method, "params": params},
                          separators=(",", ":"))

    def _run(self) -> None:
        try:
            from websockets.sync.client import connect as ws_connect
        except ImportError:
            print("[gestalt-win] missing dependency: py -m pip install websockets>=12",
                  file=sys.stderr, flush=True)
            return

        request_id = 990001
        ws = None
        next_claim = 0.0
        next_rearm = 0.0
        armed = False
        setup_phase = "idle"
        setup_deadline = 0.0
        previous_map = -1
        spawn_attempts = 0
        match_start_attempts = 0
        next_match_start = 0.0
        match_start_confirmed = False

        def send(method: str, params: dict) -> None:
            nonlocal request_id
            request_id += 1
            ws.send(self._message(request_id, method, params))

        def execute(command: str) -> None:
            send("console.exec", {"command": command})

        def watch(ids) -> None:
            fresh = [int(map_id) for map_id in ids
                     if int(map_id) > 0 and int(map_id) not in self._watched]
            if not fresh:
                return
            self._watched.update(fresh)
            send("attribute.watchAttributeMaps", {
                "attribute_map_ids": fresh,
                "watch_type": 2,
            })

        def arm(include_allowance: bool) -> None:
            nonlocal armed, next_claim, next_rearm
            execute(f"ExtAimClaim {self.player_id} 1")
            execute(f"UEExec RBTakeOver {self.player_id}")
            if include_allowance:
                execute(f"SetAttribute {self.player_id} {A_ALLOWANCE_17MM} "
                        f"{self.allowance}")
            for command in (
                    "UEExec r.VisionBridge.Enable 1",
                    "UEExec r.MotionBlurQuality 0",
                    "UEExec r.AntiAliasingMethod 1",
                    "UEExec r.RobotNav.DebugDraw 0",
                    f"UEExec t.MaxFPS {self.max_fps:g}"):
                execute(command)
            send("rgbCamera.applySettings", {"camera": self.camera})
            now = time.monotonic()
            next_claim = now + self.claim_interval_s
            next_rearm = now + 2.0
            armed = True

        while not self._stop.is_set():
            try:
                if ws is None:
                    ws = ws_connect(
                        f"ws://127.0.0.1:{self.ws_port}/", open_timeout=2.0,
                        close_timeout=0.5, max_size=None)
                    self._sent_generation = -1
                    self._maps.clear()
                    self._watched.clear()
                    watch(range(1, 257))
                    if self.prepare_match and not self._prepared.is_set():
                        setup_phase = "wait-status"
                        setup_deadline = time.monotonic() + 60.0
                        armed = False
                        print(f"[gestalt-win] game WS connected :{self.ws_port}; "
                              "waiting for prep telemetry", flush=True)
                    else:
                        arm(include_allowance=False)
                        self._prepared.set()
                        setup_phase = "ready"
                        print(f"[gestalt-win] game WS connected :{self.ws_port}; "
                              f"takeover player={self.player_id}", flush=True)

                try:
                    raw = ws.recv(timeout=0.01)
                    if isinstance(raw, bytes):
                        raw = raw.decode("utf-8", errors="replace")
                    payload = json.loads(raw)
                    discovered = self._update_telemetry(payload)
                    if discovered:
                        watch(discovered)
                except TimeoutError:
                    pass
                except (json.JSONDecodeError, UnicodeDecodeError):
                    pass

                now = time.monotonic()
                if setup_phase == "wait-status":
                    status = self._match_status()
                    if status is not None and status >= 1:
                        self._fail(
                            f"MatchStatus={status:g}; restart the game in prep without "
                            "-exec=SetMatchStatus")
                        continue
                    if status is not None:
                        previous_map = self._current_player_map() or -1
                        spawn_attempts = 1
                        execute(f"Respawn {self.player_id} {self.entity_id} {self.team_id}")
                        setup_phase = "wait-spawn"
                        setup_deadline = now + 15.0
                        print(f"[gestalt-win] prep: Respawn {self.player_id} "
                              f"{self.entity_id} {self.team_id}", flush=True)
                    elif now >= setup_deadline:
                        self._fail("MatchStatus telemetry was unavailable for 60 seconds")
                        continue
                elif setup_phase == "wait-spawn":
                    spawn_map = self._expected_spawn_map(previous_map)
                    if spawn_map is not None:
                        execute(f"SetAttribute {self.player_id} {A_IS_AI} 1")
                        arm(include_allowance=True)
                        self._prepared.set()
                        setup_phase = "ready"
                        print(f"[gestalt-win] prep passed: player={self.player_id} "
                              f"map={spawn_map} team={self.team_id} class=1004 "
                              f"entity={self.entity_id}; takeover armed", flush=True)
                    elif now >= setup_deadline:
                        if spawn_attempts >= 3:
                            self._fail("red HACHISEN spawn/identity gate failed after 3 attempts")
                            continue
                        spawn_attempts += 1
                        execute(f"Respawn {self.player_id} {self.entity_id} {self.team_id}")
                        setup_deadline = now + 15.0
                        print(f"[gestalt-win] prep: retrying Respawn "
                              f"({spawn_attempts}/3)", file=sys.stderr, flush=True)

                if armed and now >= next_claim:
                    execute(f"ExtAimClaim {self.player_id} 1")
                    next_claim = now + self.claim_interval_s

                if (armed and not self._frame_contract.is_set() and
                        now >= next_rearm):
                    execute(f"UEExec RBTakeOver {self.player_id}")
                    send("rgbCamera.applySettings", {"camera": self.camera})
                    next_rearm = now + 2.0

                if (self._start_match_requested.is_set() and
                        self._frame_contract.is_set() and self._prepared.is_set()):
                    status = self._match_status()
                    if status is not None and status >= 1:
                        if not match_start_confirmed:
                            match_start_confirmed = True
                            print(f"[gestalt-win] match start confirmed: "
                                  f"MatchStatus={status:g}", flush=True)
                    elif now >= next_match_start:
                        if match_start_attempts >= 15:
                            self._fail("SetMatchStatus 1 had no observable effect")
                            continue
                        execute("SetMatchStatus 1")
                        match_start_attempts += 1
                        next_match_start = now + 2.0
                        if match_start_attempts == 1:
                            print("[gestalt-win] frame gate passed; sending "
                                  "SetMatchStatus 1", flush=True)

                with self._lock:
                    latest = self._latest
                    generation = self._latest_generation
                if latest is not None and generation != self._sent_generation:
                    yaw, pitch, fire, yaw_velocity, pitch_velocity = latest
                    request_id += 1
                    ws.send(self._message(request_id, "console.exec", {
                        "command": (f"UEExec RBExtAim {self.player_id} "
                                    f"{yaw:.5f} {pitch:.5f} {1 if fire else 0} "
                                    f"{yaw_velocity:.5f} {pitch_velocity:.5f}")}))
                    with self._lock:
                        self._sent_generation = generation
                        self._last_aim = latest
                time.sleep(0.002)
            except Exception as error:
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass
                ws = None
                armed = False
                print(f"[gestalt-win] game WS reconnecting: {error}",
                      file=sys.stderr, flush=True)
                self._stop.wait(1.0)

        if ws is not None:
            try:
                if self._last_aim is not None:
                    yaw, pitch, _, _, _ = self._last_aim
                    request_id += 1
                    ws.send(self._message(request_id, "console.exec", {
                        "command": (f"UEExec RBExtAim {self.player_id} "
                                    f"{yaw:.5f} {pitch:.5f} 0 0 0")}))
                request_id += 1
                ws.send(self._message(request_id, "console.exec", {
                    "command": f"ExtAimClaim {self.player_id} 0"}))
                request_id += 1
                ws.send(self._message(request_id, "console.exec", {
                    "command": "UEExec RBTakeOver release"}))
                ws.close()
            except Exception:
                pass

    def close(self) -> None:
        self.clear_fire()
        self._stop.set()
        self._thread.join(timeout=3.0)


def consume_command_buffer(buffered: bytearray, control: GameControl,
                           allow_fire: bool) -> int:
    consumed = 0
    while len(buffered) >= ENVELOPE.size:
        magic, version, message_type, payload_size, seq = ENVELOPE.unpack_from(buffered)
        if (magic != ENVELOPE_MAGIC or version != ENVELOPE_VERSION or
                message_type != MESSAGE_COMMAND or
                payload_size != COMMAND_PACKET.size):
            raise ConnectionError("invalid command envelope")
        message_size = ENVELOPE.size + payload_size
        if len(buffered) < message_size:
            break
        payload = bytes(buffered[ENVELOPE.size:message_size])
        del buffered[:message_size]
        fields = COMMAND_PACKET.unpack(payload)
        if fields[0] != b"HFUTCMD1" or fields[1] != 1 or fields[3] != seq:
            raise ConnectionError("invalid command packet")
        yaw_rad = fields[5]
        yaw_velocity_rad_s = fields[7]
        pitch_rad = fields[9]
        pitch_velocity_rad_s = fields[11]
        fire = allow_fire and bool(fields[14])
        mode = fields[15]
        if (mode == 1 and math.isfinite(yaw_rad) and
                math.isfinite(pitch_rad) and
                math.isfinite(yaw_velocity_rad_s) and
                math.isfinite(pitch_velocity_rad_s)):
            # hfut control: RH yaw positive left. UE: LH yaw positive right.
            control.aim(
                -math.degrees(yaw_rad), math.degrees(pitch_rad), fire,
                -math.degrees(yaw_velocity_rad_s),
                math.degrees(pitch_velocity_rad_s))
        else:
            control.clear_fire()
        consumed += 1
    return consumed


def command_loop(connection: socket.socket, control: GameControl,
                 stop: threading.Event, timeout_s: float,
                 allow_fire: bool = True) -> None:
    last_command = time.monotonic()
    timed_out = False
    buffered = bytearray()
    connection.settimeout(0.1)
    try:
        while not stop.is_set():
            try:
                chunk = connection.recv(65536)
                if not chunk:
                    raise ConnectionError("WSL client disconnected")
                buffered.extend(chunk)
            except socket.timeout:
                if time.monotonic() - last_command > timeout_s and not timed_out:
                    control.clear_fire()
                    timed_out = True
                continue
            if consume_command_buffer(buffered, control, allow_fire) > 0:
                last_command = time.monotonic()
                timed_out = False
    except (ConnectionError, OSError) as error:
        if not stop.is_set():
            print(f"[gestalt-win] command channel closed: {error}",
                  file=sys.stderr, flush=True)
    finally:
        control.clear_fire()
        stop.set()


def stdio_command_loop(input_stream, control: GameControl,
                       stop: threading.Event, timeout_s: float,
                       allow_fire: bool = True) -> None:
    chunks: queue.Queue[bytes | None] = queue.Queue()

    def read_stdin() -> None:
        read_available = getattr(input_stream, "read1", input_stream.read)
        try:
            while not stop.is_set():
                chunk = read_available(65536)
                chunks.put(chunk if chunk else None)
                if not chunk:
                    return
        except OSError:
            chunks.put(None)

    reader = threading.Thread(target=read_stdin, name="gestalt-stdio-read", daemon=True)
    reader.start()
    last_command = time.monotonic()
    timed_out = False
    buffered = bytearray()
    try:
        while not stop.is_set():
            try:
                chunk = chunks.get(timeout=0.1)
            except queue.Empty:
                if time.monotonic() - last_command > timeout_s and not timed_out:
                    control.clear_fire()
                    timed_out = True
                continue
            if chunk is None:
                raise ConnectionError("WSL stdio command pipe closed")
            buffered.extend(chunk)
            if consume_command_buffer(buffered, control, allow_fire) > 0:
                last_command = time.monotonic()
                timed_out = False
    except (ConnectionError, OSError) as error:
        if not stop.is_set():
            print(f"[gestalt-win] command channel closed: {error}",
                  file=sys.stderr, flush=True)
    finally:
        control.clear_fire()
        stop.set()


def frame_rejection_reason(frame: dict, arguments) -> str | None:
    if not arguments.no_identity_gate:
        if (frame["identity_flags"] & 0x7) != 0x7:
            return f"identity flags=0x{frame['identity_flags']:X}"
        if frame["takeover_player"] != arguments.player_id:
            return f"player={frame['takeover_player']} expected={arguments.player_id}"
        if frame["takeover_map"] < 0:
            return f"invalid takeover map={frame['takeover_map']}"
        if frame["view_actor"] == 0 or frame["view_actor"] != frame["takeover_target"]:
            return f"view={frame['view_actor']} target={frame['takeover_target']}"
        if frame["takeover_epoch"] == 0:
            return "takeover epoch is zero"
    if frame["width"] != arguments.frame_width or frame["height"] != arguments.frame_height:
        return (f"frame={frame['width']}x{frame['height']} expected="
                f"{arguments.frame_width}x{arguments.frame_height}")
    if abs(frame["fov_degrees"] - arguments.fov) > 0.1:
        return f"fov={frame['fov_degrees']:.2f} expected={arguments.fov:.2f}"
    if abs(frame["camera_arm_length_cm"] - arguments.arm_length) > 0.1:
        return (f"arm={frame['camera_arm_length_cm']:.2f}cm "
                f"expected={arguments.arm_length:.2f}cm")
    return None


def allowed_client_addresses(value: str) -> set[str]:
    if value != "auto":
        addresses = {item.strip() for item in value.split(",") if item.strip()}
        if not addresses:
            raise SystemExit("--allow-client must contain at least one IPv4 address")
        return addresses

    addresses = {"127.0.0.1"}
    try:
        result = subprocess.run(
            ["wsl.exe", "hostname", "-I"], check=True, capture_output=True,
            text=True, timeout=5.0)
        for candidate in result.stdout.split():
            try:
                socket.inet_aton(candidate)
            except OSError:
                continue
            addresses.add(candidate)
    except (OSError, subprocess.SubprocessError):
        pass
    return addresses


def frame_packet_parts(frame: dict, codec: str = "raw") -> tuple[bytes, bytes, bytes]:
    pixels = frame["pixels"]
    pixel_format = frame["pixel_format"]
    if codec == "lz4":
        import lz4.block
        compressed = lz4.block.compress(pixels, mode="fast", store_size=False)
        if len(compressed) < len(pixels):
            pixels = compressed
            pixel_format += PIXEL_FORMAT_LZ4_OFFSET
    metadata = FRAME_METADATA.pack(
        frame["seq"], frame["capture_time_s"], frame["world_time_s"],
        frame["width"], frame["height"], frame["row_bytes"],
        pixel_format, len(pixels), frame["fov_degrees"],
        frame["camera_arm_length_cm"], *frame["camera_position"],
        *frame["camera_quaternion"], frame["writer_pid"], frame["view_actor"],
        frame["takeover_target"], frame["takeover_player"], frame["takeover_map"],
        frame["takeover_epoch"], frame["identity_flags"])
    envelope = ENVELOPE.pack(
        ENVELOPE_MAGIC, ENVELOPE_VERSION, MESSAGE_FRAME,
        len(metadata) + len(pixels), frame["seq"])
    return envelope, metadata, pixels


def frame_packet(frame: dict, codec: str = "raw") -> bytes:
    return b"".join(frame_packet_parts(frame, codec))


def forward_frame_loop(capture: SharedFrameCapture, control: GameControl,
                       arguments, stop: threading.Event, send_parts) -> None:
    rejected_frames = 0
    accepted_consecutive = 0
    accepted_epoch = None
    gate_reported = False
    sent_frames = 0
    first_sent_wall = None
    first_sent_capture = None
    first_sent_sequence = None
    encoded_bytes = 0
    raw_bytes = 0
    encode_time_s = 0.0
    send_time_s = 0.0

    while not stop.is_set():
        failure = control.failure_reason()
        if failure:
            raise RuntimeError(failure)
        frame = capture.grab(0.1)
        if frame is None:
            continue
        rejection = frame_rejection_reason(frame, arguments)
        if rejection:
            accepted_consecutive = 0
            accepted_epoch = None
            rejected_frames += 1
            if rejected_frames == 1 or rejected_frames % 120 == 0:
                print("[gestalt-win] waiting for frame contract: " + rejection,
                      file=sys.stderr, flush=True)
            continue
        if accepted_epoch == frame["takeover_epoch"]:
            accepted_consecutive += 1
        else:
            accepted_epoch = frame["takeover_epoch"]
            accepted_consecutive = 1
        if accepted_consecutive < 3:
            continue
        if not gate_reported:
            gate_reported = True
            control.frame_contract_passed()
            print(
                "[gestalt-win] frame contract passed: "
                f"{frame['width']}x{frame['height']} "
                f"fov={frame['fov_degrees']:.2f} "
                f"arm={frame['camera_arm_length_cm']:.2f}cm "
                f"pid={frame['takeover_player']} "
                f"map={frame['takeover_map']} epoch={frame['takeover_epoch']}",
                flush=True)
            if arguments.start_match:
                control.start_match()

        encode_started = time.perf_counter()
        packet_parts = frame_packet_parts(frame, arguments.frame_codec)
        encode_time_s += time.perf_counter() - encode_started
        encoded_bytes += len(packet_parts[2])
        raw_bytes += frame["pixel_bytes"]
        send_started = time.perf_counter()
        send_parts(packet_parts)
        send_time_s += time.perf_counter() - send_started
        sent_frames += 1
        if first_sent_wall is None:
            first_sent_wall = time.monotonic()
            first_sent_capture = frame["capture_time_s"]
            first_sent_sequence = frame["seq"]
        if sent_frames % 300 == 0:
            wall_elapsed = time.monotonic() - first_sent_wall
            capture_elapsed = frame["capture_time_s"] - first_sent_capture
            forwarded_fps = ((sent_frames - 1) / wall_elapsed
                             if wall_elapsed > 0 else 0.0)
            publisher_fps = ((frame["seq"] - first_sent_sequence) / capture_elapsed
                             if capture_elapsed > 0 else 0.0)
            compression_ratio = raw_bytes / encoded_bytes if encoded_bytes > 0 else 1.0
            print(f"[gestalt-win] frames={sent_frames} "
                  f"last={frame['seq']} {frame['width']}x{frame['height']} "
                  f"fov={frame['fov_degrees']:.2f} "
                  f"publisher_fps={publisher_fps:.1f} "
                  f"forwarded_fps={forwarded_fps:.1f} "
                  f"ratio={compression_ratio:.2f}x "
                  f"encode={1000.0 * encode_time_s / sent_frames:.1f}ms "
                  f"send={1000.0 * send_time_s / sent_frames:.1f}ms",
                  flush=True)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Gestalt Windows shared-frame/WebSocket proxy for hfut_auto_aim in WSL2")
    parser.add_argument(
        "ws_port", nargs="?", type=parse_ws_port, default=0,
        help="Gestalt loopback WebSocket port, or 'auto' (default: auto)")
    parser.add_argument(
        "--transport", choices=("tcp", "stdio"), default="tcp",
        help="WSL frame/command transport (default: tcp)")
    parser.add_argument("--player-id", type=int, default=0,
                        help="player controlled by RBTakeOver/RBExtAim (default: 0)")
    parser.add_argument("--game-pid", type=int, default=0,
                        help="restrict automatic listener discovery to this game PID")
    parser.add_argument("--endpoint-timeout", type=float, default=60.0,
                        help="seconds to wait for the game WebSocket (default: 60)")
    parser.add_argument(
        "--prepare-match", action="store_true",
        help="in prep, spawn and verify player 0 as red HACHISEN before takeover")
    parser.add_argument(
        "--start-match", action="store_true",
        help="send SetMatchStatus 1 only after the shared-frame contract passes")
    parser.add_argument("--entity-id", type=int, default=66000005,
                        help="vehicle entity used by --prepare-match")
    parser.add_argument("--team-id", type=int, default=0,
                        help="vehicle team used by --prepare-match")
    parser.add_argument("--allowance", type=int, default=400,
                        help="initial 17mm allowance used by --prepare-match")
    parser.add_argument("--listen", default="0.0.0.0",
                        help="TCP bind address (0.0.0.0 is required for WSL2 NAT)")
    parser.add_argument("--tcp-port", type=int, default=47000)
    parser.add_argument(
        "--allow-client", default="auto",
        help="comma-separated WSL IPv4 allowlist; auto uses localhost + `wsl hostname -I`")
    parser.add_argument("--frame-width", type=int, default=1280)
    parser.add_argument("--frame-height", type=int, default=720)
    parser.add_argument("--fov", type=float, default=25.0)
    parser.add_argument("--shutter-speed", type=float, default=120.0)
    parser.add_argument("--iso", type=float, default=600.0)
    parser.add_argument("--arm-length", type=float, default=0.0,
                        help="camera arm length in cm; sentry first-person view requires 0")
    parser.add_argument(
        "--max-fps", type=float, default=60.0,
        help="UE render/shared-frame rate cap; 0 disables the cap (default: 60)")
    parser.add_argument(
        "--frame-codec", choices=("raw", "lz4"), default="lz4",
        help="lossless pixel transport encoding (default: lz4)")
    parser.add_argument("--no-fire", action="store_true",
                        help="force all forwarded RBExtAim commands to fire=0")
    parser.add_argument("--mapping-timeout", type=float, default=15.0)
    parser.add_argument("--command-timeout", type=float, default=0.5)
    parser.add_argument("--no-identity-gate", action="store_true",
                        help="allow frames without verified takeover identity (unsafe)")
    return parser.parse_args()


def main() -> int:
    require_windows()
    arguments = parse_arguments()
    stdio_output_fd = -1
    if arguments.transport == "stdio":
        stdio_output_fd = sys.stdout.buffer.fileno()
        # Protocol bytes own stdout in stdio mode; all human-readable logs go
        # to the inherited stderr stream.
        sys.stdout = sys.stderr
    try:
        import websockets.sync.client  # noqa: F401
    except ImportError as error:
        raise SystemExit(
            "missing dependency; run: py -m pip install -r "
            "tools\\requirements-gestalt-windows.txt") from error
    if not 1 <= arguments.tcp_port <= 65535:
        raise SystemExit("--tcp-port must be in [1, 65535]")
    if arguments.endpoint_timeout <= 0:
        raise SystemExit("--endpoint-timeout must be positive")
    if arguments.start_match and not arguments.prepare_match:
        raise SystemExit("--start-match requires --prepare-match")
    if arguments.prepare_match and arguments.player_id != 0:
        raise SystemExit("--prepare-match currently requires --player-id 0")
    if arguments.allowance < 0:
        raise SystemExit("--allowance must not be negative")
    if not 1.0 < arguments.fov < 179.0:
        raise SystemExit("--fov must be in (1, 179) degrees")
    if arguments.frame_width <= 0 or arguments.frame_height <= 0:
        raise SystemExit("--frame-width and --frame-height must be positive")
    if not math.isfinite(arguments.max_fps) or arguments.max_fps < 0:
        raise SystemExit("--max-fps must be finite and >= 0")
    if arguments.frame_codec == "lz4":
        try:
            import lz4.block  # noqa: F401
        except ImportError as error:
            raise SystemExit(
                "missing lz4; rerun the Windows requirements install command") from error

    try:
        ws_port, game_pid = resolve_game_endpoint(
            arguments.ws_port, arguments.game_pid, arguments.endpoint_timeout)
    except RuntimeError as error:
        raise SystemExit(str(error)) from error
    arguments.ws_port = ws_port
    print(f"[gestalt-win] discovered game WebSocket: 127.0.0.1:{ws_port} "
          f"pid={game_pid}", flush=True)
    capture = SharedFrameCapture(game_pid, arguments.mapping_timeout)
    control = GameControl(
        arguments.ws_port, arguments.player_id,
        {"enabled": 1, "fovDegrees": arguments.fov,
         "shutterSpeed": arguments.shutter_speed, "iso": arguments.iso,
         "armLength": arguments.arm_length},
        prepare_match=arguments.prepare_match,
        entity_id=arguments.entity_id, team_id=arguments.team_id,
        allowance=arguments.allowance, max_fps=arguments.max_fps)

    if arguments.transport == "stdio":
        print(f"[gestalt-win] game pid={game_pid}; WSL interop stdio; fire="
              f"{'disabled' if arguments.no_fire else 'enabled'}; "
              f"max_fps={arguments.max_fps:g}; codec={arguments.frame_codec}", flush=True)
        stop = threading.Event()
        receiver = threading.Thread(
            target=stdio_command_loop,
            args=(sys.stdin.buffer, control, stop, arguments.command_timeout,
                  not arguments.no_fire),
            name="gestalt-command", daemon=True)
        receiver.start()

        def send_stdio_parts(packet_parts) -> None:
            for packet_part in packet_parts:
                remaining = memoryview(packet_part)
                while remaining:
                    written = os.write(stdio_output_fd, remaining)
                    if written <= 0:
                        raise BrokenPipeError("WSL stdio frame pipe closed")
                    remaining = remaining[written:]

        try:
            forward_frame_loop(capture, control, arguments, stop, send_stdio_parts)
        except (BrokenPipeError, ConnectionError, OSError, RuntimeError) as error:
            if not stop.is_set():
                print(f"[gestalt-win] stdio frame channel closed: {error}",
                      file=sys.stderr, flush=True)
        finally:
            stop.set()
            control.clear_fire()
            receiver.join(timeout=1.0)
            control.close()
            capture.close()
        return 0

    allowed_clients = allowed_client_addresses(arguments.allow_client)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((arguments.listen, arguments.tcp_port))
    server.listen(1)
    server.settimeout(1.0)
    print(f"[gestalt-win] game pid={game_pid}; listening on "
          f"{arguments.listen}:{arguments.tcp_port}; clients="
          f"{','.join(sorted(allowed_clients))}; fire="
          f"{'disabled' if arguments.no_fire else 'enabled'}; "
          f"max_fps={arguments.max_fps:g}; codec={arguments.frame_codec}", flush=True)

    try:
        while True:
            failure = control.failure_reason()
            if failure:
                raise RuntimeError(failure)
            try:
                connection, address = server.accept()
            except socket.timeout:
                continue
            if address[0] not in allowed_clients:
                print(f"[gestalt-win] rejected TCP client {address[0]}:{address[1]}",
                      file=sys.stderr, flush=True)
                connection.close()
                continue
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"[gestalt-win] WSL client connected from {address[0]}:{address[1]}",
                  flush=True)
            stop = threading.Event()
            receiver = threading.Thread(
                target=command_loop,
                args=(connection, control, stop, arguments.command_timeout,
                      not arguments.no_fire),
                name="gestalt-command", daemon=True)
            receiver.start()

            def send_tcp_parts(packet_parts) -> None:
                for packet_part in packet_parts:
                    connection.sendall(packet_part)

            try:
                forward_frame_loop(capture, control, arguments, stop, send_tcp_parts)
            except (ConnectionError, OSError, RuntimeError) as error:
                print(f"[gestalt-win] frame channel closed: {error}",
                      file=sys.stderr, flush=True)
            finally:
                stop.set()
                control.clear_fire()
                try:
                    connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                connection.close()
                receiver.join(timeout=1.0)
    except KeyboardInterrupt:
        print("\n[gestalt-win] stopping", flush=True)
    except RuntimeError as error:
        print(f"[gestalt-win] stopping: {error}", file=sys.stderr, flush=True)
        return 2
    finally:
        server.close()
        control.close()
        capture.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
