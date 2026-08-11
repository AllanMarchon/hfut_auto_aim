#!/usr/bin/env python3
"""Audit Webots armor-pose shots by selected armor and final score result."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


ARMOR_ORDER = ("front", "left", "rear", "right")
LOWER_ARMORS = {"front", "rear"}
UPPER_ARMORS = {"left", "right"}


def expand_path(value: str | Path) -> Path:
    return Path(os.path.expandvars(str(value))).expanduser()


def finite(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists() or path.stat().st_size <= 0:
        return []
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8-sig", errors="replace") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(record, dict):
                records.append(record)
    return records


def nested(record: dict[str, Any], *keys: str, default: Any = None) -> Any:
    value: Any = record
    for key in keys:
        if not isinstance(value, dict):
            return default
        value = value.get(key, default)
    return value


def armor_layer(name: str, index: int = -1) -> str:
    if name in LOWER_ARMORS:
        return "lower"
    if name in UPPER_ARMORS:
        return "upper"
    if index >= 0:
        return "lower" if index % 2 == 0 else "upper"
    return "unknown"


def nearest(records: list[dict[str, Any]], times: list[float], target: float) -> dict[str, Any] | None:
    if not records:
        return None
    pos = bisect.bisect_left(times, target)
    candidates = []
    if pos < len(records):
        candidates.append(records[pos])
    if pos > 0:
        candidates.append(records[pos - 1])
    return min(candidates, key=lambda item: abs(finite(item.get("sim_time_s"), 0.0) - target))


def parse_simple_yaml_number(path: Path, key: str) -> float | None:
    if not path.exists():
        return None
    pattern = re.compile(rf"^\s*{re.escape(key)}\s*:\s*([-+0-9.eE]+)")
    for line in path.read_text(encoding="utf-8-sig", errors="replace").splitlines():
        match = pattern.match(line)
        if match:
            return finite(match.group(1), math.nan)
    return None


def parse_env_default(path: Path, name: str) -> float | None:
    if not path.exists():
        return None
    pattern = re.compile(rf"{re.escape(name)}:=([-+0-9.eE]+)")
    for line in path.read_text(encoding="utf-8-sig", errors="replace").splitlines():
        match = pattern.search(line)
        if match:
            return finite(match.group(1), math.nan)
    return None


def parse_camera_to_barrel_xyz(path: Path) -> tuple[float, float, float] | None:
    if not path.exists():
        return None
    text = path.read_text(encoding="utf-8-sig", errors="replace")
    match = re.search(r"webots:\s*(?:\n\s*#.*)*\n\s*xyz:\s*\[([^\]]+)\]", text)
    if not match:
        return None
    parts = [finite(part.strip(), math.nan) for part in match.group(1).split(",")]
    if len(parts) != 3 or any(not math.isfinite(part) for part in parts):
        return None
    return (parts[0], parts[1], parts[2])


def build_shots(diagnostics: list[dict[str, Any]], events: list[dict[str, Any]]) -> dict[int, dict[str, Any]]:
    diagnostics = sorted(diagnostics, key=lambda item: finite(item.get("sim_time_s"), 0.0))
    times = [finite(item.get("sim_time_s"), 0.0) for item in diagnostics]
    shots: dict[int, dict[str, Any]] = {}

    for event in events:
        shot_id = event.get("shot")
        if shot_id is None:
            continue
        try:
            shot = int(shot_id)
        except (TypeError, ValueError):
            continue
        event_type = str(event.get("event", ""))
        if event_type == "fired":
            event_time = finite(event.get("sim_time_s"), 0.0)
            diag = nearest(diagnostics, times, event_time) or {}
            control = nested(diag, "control_target", default={}) or {}
            command = nested(diag, "command", default={}) or {}
            selected_index = int(finite(control.get("real_selected_index"), -1))
            if selected_index < 0:
                selected_index = int(finite(control.get("selected_index"), -1))
            selected_name = str(control.get("selected_name") or "unknown")
            selected_layer = str(control.get("selected_layer") or armor_layer(selected_name, selected_index))
            shots[shot] = {
                "shot": shot,
                "time": event_time,
                "selected_name": selected_name,
                "selected_layer": selected_layer,
                "selected_index": selected_index,
                "yaw_err_deg": finite(command.get("yaw_diff_deg"), math.nan),
                "pitch_err_deg": finite(command.get("pitch_diff_deg"), math.nan),
                "distance_m": finite(command.get("distance_m"), math.nan),
                "target_error_m": math.nan,
                "result": "active",
                "hit_armor": "",
            }
        elif event_type in {"hit", "miss"}:
            shots.setdefault(shot, {"shot": shot})
            shots[shot]["result"] = event_type
            shots[shot]["hit_armor"] = str(event.get("armor", "") or "")
            shots[shot]["flight_time_s"] = finite(event.get("flight_time_s"), math.nan)
    return shots


def ratio(part: int, total: int) -> float:
    return part / total if total > 0 else 0.0


def format_pct(value: float) -> str:
    return f"{value * 100.0:.1f}%"


def summarize_by_selection(shots: dict[int, dict[str, Any]]) -> tuple[Counter, Counter, Counter, dict]:
    selected = Counter()
    results = Counter()
    mismatches = Counter()
    errors: dict[tuple[str, str, int], list[float]] = defaultdict(lambda: [0, 0.0, 0.0, 0.0])

    for shot in shots.values():
        name = str(shot.get("selected_name", "unknown"))
        layer = str(shot.get("selected_layer", armor_layer(name)))
        index = int(finite(shot.get("selected_index"), -1))
        key = (name, layer, index)
        result = str(shot.get("result", "active"))
        hit_armor = str(shot.get("hit_armor", ""))
        selected[key] += 1
        results[(key, result)] += 1
        if result == "hit" and hit_armor and hit_armor != name:
            mismatches[(key, hit_armor)] += 1
        bucket = errors[key]
        bucket[0] += 1
        yaw_err = finite(shot.get("yaw_err_deg"), math.nan)
        pitch_err = finite(shot.get("pitch_err_deg"), math.nan)
        if math.isfinite(yaw_err):
            bucket[1] += abs(yaw_err)
        if math.isfinite(pitch_err):
            bucket[2] += abs(pitch_err)
            bucket[3] += pitch_err
    return selected, results, mismatches, errors


def print_config_checks(root: Path) -> None:
    controller = root / "configs" / "controller.yaml"
    simulation = root / "configs" / "simulation.yaml"
    camera_env = root / "sim" / "webots" / "config" / "camera_robot.env"

    shooting_w = parse_simple_yaml_number(controller, "shooting_range_width")
    shooting_h = parse_simple_yaml_number(controller, "shooting_range_height")
    score_w = parse_env_default(camera_env, "WEBOTS_SCORE_ARMOR_WIDTH")
    score_h = parse_env_default(camera_env, "WEBOTS_SCORE_ARMOR_HEIGHT")
    shooter_x = parse_env_default(camera_env, "WEBOTS_SCORE_SHOOTER_OFFSET_X")
    shooter_y = parse_env_default(camera_env, "WEBOTS_SCORE_SHOOTER_OFFSET_Y")
    shooter_z = parse_env_default(camera_env, "WEBOTS_SCORE_SHOOTER_OFFSET_Z")
    camera_xyz = parse_camera_to_barrel_xyz(simulation)

    print("配置一致性检查:")
    print(f"  fire window: width={shooting_w} height={shooting_h}")
    print(f"  score armor: width={score_w} height={score_h}")
    print(f"  camera_to_barrel.webots.xyz={camera_xyz}")
    print(f"  score shooter offset=({shooter_x}, {shooter_y}, {shooter_z})")

    if shooting_h and score_h and shooting_h > score_h * 1.25:
        print("  [WARN] 开火高度窗口明显大于计分高度：会出现 fire=1 但高板实际 miss。")
    if shooting_w and score_w and shooting_w < score_w * 0.85:
        print("  [INFO] 开火宽度小于计分宽度：偏保守，不是命中率虚高来源。")
    if camera_xyz is not None and shooter_x is not None and shooter_z is not None:
        cx, _cy, cz = camera_xyz
        if abs(cx) > 0.02 or abs(cz) > 0.02:
            if abs(shooter_x) < 1e-6 and abs(shooter_z) < 1e-6:
                print("  [WARN] 自瞄使用了相机到枪管外参，但 Webots 计分弹丸从相机节点发出；高板 pitch 可能系统偏。")
    print()


def print_diagnosis(selected: Counter, results: Counter, mismatches: Counter) -> None:
    lower_fired = sum(count for (name, layer, _index), count in selected.items() if layer == "lower")
    upper_fired = sum(count for (name, layer, _index), count in selected.items() if layer == "upper")
    lower_hit = sum(results[(key, "hit")] for key in selected if key[1] == "lower")
    upper_hit = sum(results[(key, "hit")] for key in selected if key[1] == "upper")
    lower_done = sum(results[(key, "hit")] + results[(key, "miss")] for key in selected if key[1] == "lower")
    upper_done = sum(results[(key, "hit")] + results[(key, "miss")] for key in selected if key[1] == "upper")

    total_fired = lower_fired + upper_fired
    upper_select_ratio = ratio(upper_fired, total_fired)
    lower_hit_rate = ratio(lower_hit, lower_done)
    upper_hit_rate = ratio(upper_hit, upper_done)

    print("诊断结论:")
    print(f"  upper selected ratio={format_pct(upper_select_ratio)}")
    print(f"  lower hit_rate={format_pct(lower_hit_rate)} upper hit_rate={format_pct(upper_hit_rate)}")

    if total_fired == 0:
        print("  [BLOCKED] 没有 fired 事件，先确认 Webots 和自瞄端都在运行。")
        return
    if upper_select_ratio < 0.20:
        print("  [LIKELY] 选板逻辑偏低板：优先看 selection_method / facing_* / switch_movement_margin。")
    elif upper_done > 20 and upper_hit_rate + 0.20 < lower_hit_rate:
        print("  [LIKELY] 高板被选中但打不中：优先看 shooting_range_height、pitch_offset、枪口偏置、弹速/延时。")
    else:
        print("  [MIXED] 选板和弹道都可能有影响，需要结合下面的分板统计看。")

    mismatch_total = sum(mismatches.values())
    if mismatch_total > 0:
        print(f"  [WARN] 存在 {mismatch_total} 次选中 A 但命中 B，优先查装甲板相位/命名/计分几何。")
    print()


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit HFUT Webots shot selection and hit results.")
    parser.add_argument("--bridge-dir", default=str(Path.home() / "hfut_auto_aim_webots"))
    parser.add_argument("--diagnostics", default="tracking_diagnostics.jsonl")
    parser.add_argument("--events", default="score_events.jsonl")
    parser.add_argument("--repo-root", default=".")
    args = parser.parse_args()

    bridge_dir = expand_path(args.bridge_dir)
    diagnostics_path = expand_path(args.diagnostics)
    events_path = expand_path(args.events)
    if not diagnostics_path.is_absolute():
        diagnostics_path = bridge_dir / diagnostics_path
    if not events_path.is_absolute():
        events_path = bridge_dir / events_path
    repo_root = expand_path(args.repo_root)

    diagnostics = read_jsonl(diagnostics_path)
    events = read_jsonl(events_path)
    if not diagnostics:
        print(f"没有读取到诊断日志: {diagnostics_path}")
        return 1
    if not events:
        print(f"没有读取到计分事件: {events_path}")
        return 1

    print_config_checks(repo_root)

    shots = build_shots(diagnostics, events)
    selected, results, mismatches, errors = summarize_by_selection(shots)
    print(f"样本: diagnostics={len(diagnostics)} events={len(events)} shots={len(shots)}")
    print()

    print("按发射时选中板统计:")
    for key, total in selected.most_common():
        hit = results[(key, "hit")]
        miss = results[(key, "miss")]
        active = results[(key, "active")]
        done = hit + miss
        err_count, yaw_abs_sum, pitch_abs_sum, pitch_sum = errors[key]
        avg_yaw = yaw_abs_sum / err_count if err_count else 0.0
        avg_pitch = pitch_abs_sum / err_count if err_count else 0.0
        avg_signed_pitch = pitch_sum / err_count if err_count else 0.0
        print(
            f"  {key}: fired={total} hit={hit} miss={miss} active={active} "
            f"hit_rate={format_pct(ratio(hit, done))} "
            f"avg_abs_err=({avg_yaw:.3f}deg yaw, {avg_pitch:.3f}deg pitch) "
            f"avg_pitch_err={avg_signed_pitch:.3f}deg"
        )
    print()

    hit_boards = Counter(str(event.get("armor", "unknown")) for event in events if event.get("event") == "hit")
    print("按最终命中板统计:")
    for name in ARMOR_ORDER:
        print(f"  {name}: {hit_boards[name]}")
    print(f"  lower={hit_boards['front'] + hit_boards['rear']} upper={hit_boards['left'] + hit_boards['right']}")
    print()

    if mismatches:
        print("选中 A 但实际命中 B:")
        for key, count in mismatches.most_common(12):
            print(f"  {key}: {count}")
        print()

    print_diagnosis(selected, results, mismatches)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
