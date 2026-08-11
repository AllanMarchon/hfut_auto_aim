#!/usr/bin/env python3
"""Print a compact live status line for the ROS-free Webots auto-aim bridge."""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from collections import Counter, deque
from pathlib import Path
from typing import Any


K_RAD_TO_DEG = 180.0 / math.pi


def expand_path(value: str | Path) -> Path:
    return Path(os.path.expandvars(str(value))).expanduser()


def default_bridge_dir() -> Path:
    env_dir = os.environ.get("WEBOTS_ROS_FREE_BRIDGE_DIR", "")
    return expand_path(env_dir) if env_dir else Path.home() / "hfut_auto_aim_webots"


def finite(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def vector3(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) < 3:
        return None
    out = [finite(item, math.nan) for item in value[:3]]
    return out if all(math.isfinite(item) for item in out) else None


def distance(left: list[float], right: list[float]) -> float:
    return math.sqrt(sum((a - b) * (a - b) for a, b in zip(left, right)))


def get_nested(record: dict[str, Any] | None, keys: list[str], default: Any = None) -> Any:
    value: Any = record or {}
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def read_tail_jsonl(path: Path, max_bytes: int = 262144, max_records: int = 512) -> list[dict[str, Any]]:
    if not path.exists() or path.stat().st_size <= 0:
        return []
    with path.open("rb") as handle:
        size = handle.seek(0, os.SEEK_END)
        handle.seek(max(0, size - max_bytes))
        data = handle.read().decode("utf-8-sig", errors="replace")
    if data and not data.startswith("{"):
        data = data[data.find("\n") + 1 :]
    records: list[dict[str, Any]] = []
    for line_number, line in enumerate(data.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(record, dict):
            record.setdefault("seq", line_number)
            records.append(record)
    return records[-max_records:]


def read_latest_json(path: Path) -> dict[str, Any] | None:
    records = read_tail_jsonl(path, max_bytes=65536, max_records=1)
    return records[-1] if records else None


def parse_score(path: Path) -> dict[str, float | int | None]:
    score = {
        "shots": None,
        "hits": None,
        "misses": None,
        "hit_rate": None,
        "dps": None,
        "fired": None,
        "fire_advice": None,
    }
    if not path.exists():
        return score
    tokens = path.read_text(encoding="utf-8-sig", errors="replace").replace("\n", " ").split()
    values: dict[str, str] = {}
    for token in tokens:
        if "=" in token:
            key, value = token.split("=", 1)
            values[key.strip()] = value.strip()
    for key in score:
        if key not in values:
            continue
        number = finite(values[key], math.nan)
        if math.isfinite(number):
            score[key] = int(number) if key in {"shots", "hits", "misses", "fired", "fire_advice"} else number
    return score


def armor_name_by_index(index: int) -> str:
    return {
        0: "front",
        1: "left",
        2: "rear",
        3: "right",
    }.get(index, "unknown")


def armor_layer_by_index(index: int) -> str:
    if index < 0:
        return "unknown"
    return "lower" if index % 2 == 0 else "upper"


def parse_score_events(path: Path) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "fired_events": 0,
        "hit_events": 0,
        "miss_events": 0,
        "hits_by_armor": {},
        "lower_hits": 0,
        "upper_hits": 0,
        "last_hit_armor": None,
        "last_event": None,
    }
    if not path.exists() or path.stat().st_size <= 0:
        return summary

    hits_by_armor: Counter[str] = Counter()
    try:
        with path.open("r", encoding="utf-8-sig", errors="replace") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(record, dict):
                    continue
                event = str(record.get("event", ""))
                summary["last_event"] = event or None
                if event == "fired":
                    summary["fired_events"] += 1
                elif event == "miss":
                    summary["miss_events"] += 1
                elif event == "hit":
                    armor = str(record.get("armor", "unknown") or "unknown")
                    hits_by_armor[armor] += 1
                    summary["hit_events"] += 1
                    summary["last_hit_armor"] = armor
                    if armor in {"front", "rear"}:
                        summary["lower_hits"] += 1
                    elif armor in {"left", "right"}:
                        summary["upper_hits"] += 1
    except OSError:
        return summary

    summary["hits_by_armor"] = dict(hits_by_armor)
    return summary


def nearest_by_time(records: list[dict[str, Any]], target_time: float) -> dict[str, Any] | None:
    if not records:
        return None
    return min(records, key=lambda record: abs(finite(record.get("sim_time_s"), 0.0) - target_time))


def truth_center_control(truth_record: dict[str, Any]) -> list[float] | None:
    shooter = vector3(truth_record.get("shooter_position")) or [0.0, 0.0, 0.0]
    positions = []
    for armor in truth_record.get("armors", []):
        if not isinstance(armor, dict):
            continue
        position = vector3(armor.get("position"))
        if position is not None:
            positions.append(position)
    if positions:
        return [statistics.fmean(position[axis] for position in positions) - shooter[axis] for axis in range(3)]
    target = vector3(truth_record.get("target_position"))
    return [target[axis] - shooter[axis] for axis in range(3)] if target is not None else None


def truth_position_for_control(diagnostic: dict[str, Any], truth_record: dict[str, Any]) -> list[float] | None:
    control = diagnostic.get("control_target", {})
    if not isinstance(control, dict) or not control.get("valid", False):
        return None
    if control.get("tracks_center", False) or control.get("virtual_target", False):
        return truth_center_control(truth_record)
    command_target = vector3(control.get("control_target_position"))
    shooter = vector3(truth_record.get("shooter_position")) or [0.0, 0.0, 0.0]
    candidates = []
    for armor in truth_record.get("armors", []):
        if not isinstance(armor, dict):
            continue
        position = vector3(armor.get("position"))
        if position is not None:
            candidates.append([position[axis] - shooter[axis] for axis in range(3)])
    if command_target is None or not candidates:
        return None
    return min(candidates, key=lambda candidate: distance(command_target, candidate))


def command_mode_name(mode: int) -> str:
    return {
        -2: "BLIND",
        -1: "NO_MEAS",
        0: "UNKNOWN",
        1: "NORMAL",
    }.get(mode, str(mode))


def track_state_name(state: int) -> str:
    return {
        -1: "LOST",
        0: "DETECTING",
        1: "TRACKING",
        2: "TEMP_LOST",
    }.get(state, str(state))


class RollingStats:
    def __init__(self, window: int):
        self.window = max(1, window)
        self.records: deque[dict[str, Any]] = deque(maxlen=self.window)

    def add(self, record: dict[str, Any]) -> None:
        self.records.append(record)

    def ratio(self, predicate) -> float | None:
        if not self.records:
            return None
        return sum(1 for record in self.records if predicate(record)) / len(self.records)


def make_status(
    diagnostic: dict[str, Any],
    truth_records: list[dict[str, Any]],
    score: dict[str, float | int | None],
    score_events: dict[str, Any],
    rolling: RollingStats,
    previous: dict[str, float] | None,
) -> tuple[dict[str, Any], dict[str, float]]:
    seq = int(finite(diagnostic.get("seq"), 0))
    sim_time = finite(diagnostic.get("sim_time_s"), 0.0)
    command = get_nested(diagnostic, ["command"], {}) or {}
    control = get_nested(diagnostic, ["control_target"], {}) or {}
    delay = get_nested(diagnostic, ["delay"], {}) or {}
    rl = get_nested(diagnostic, ["rl_action"], {}) or {}
    selected_index = int(finite(control.get("real_selected_index"), -1))
    if selected_index < 0:
        selected_index = int(finite(control.get("selected_index"), -1))
    selected_name = str(control.get("selected_name") or armor_name_by_index(selected_index))
    selected_layer = str(control.get("selected_layer") or armor_layer_by_index(selected_index))
    truth_record = nearest_by_time(truth_records, sim_time)
    truth_lag_ms = None
    target_error_m = None
    if truth_record is not None:
        truth_lag_ms = abs(finite(truth_record.get("sim_time_s"), 0.0) - sim_time) * 1000.0
        command_target = vector3(control.get("control_target_position"))
        truth_target = truth_position_for_control(diagnostic, truth_record)
        if command_target is not None and truth_target is not None:
            target_error_m = distance(command_target, truth_target)

    now = time.monotonic()
    fps = None
    wall_hz = None
    if previous is not None:
        seq_delta = seq - int(previous.get("seq", seq))
        sim_delta = sim_time - previous.get("sim_time", sim_time)
        wall_delta = now - previous.get("wall_time", now)
        if sim_delta > 1e-9:
            fps = seq_delta / sim_delta
        if wall_delta > 1e-9:
            wall_hz = seq_delta / wall_delta

    status = {
        "seq": seq,
        "time_s": sim_time,
        "fps": fps,
        "wall_hz": wall_hz,
        "mode": int(finite(diagnostic.get("command_mode"), 0)),
        "track_state": int(finite(diagnostic.get("track_state"), -1)),
        "target_id": str(diagnostic.get("selected_id", "")),
        "tracked_count": int(finite(diagnostic.get("tracked_count"), 0)),
        "armor_count": len(diagnostic.get("direct_armors", [])),
        "selected_index": selected_index,
        "selected_name": selected_name,
        "selected_layer": selected_layer,
        "yaw_deg": finite(command.get("yaw_deg"), math.nan),
        "pitch_deg": finite(command.get("pitch_deg"), math.nan),
        "yaw_err_deg": finite(command.get("yaw_diff_deg"), math.nan),
        "pitch_err_deg": finite(command.get("pitch_diff_deg"), math.nan),
        "distance_m": finite(command.get("distance_m"), math.nan),
        "fire": int(finite(command.get("fire_advice"), 0)),
        "target_error_m": target_error_m,
        "truth_lag_ms": truth_lag_ms,
        "proc_ms": finite(delay.get("processing_delay_s"), math.nan) * 1000.0,
        "flight_ms": finite(delay.get("flight_time_s"), math.nan) * 1000.0,
        "pred_ms": finite(delay.get("prediction_s"), math.nan) * 1000.0,
        "rl_enabled": bool(rl.get("enabled", False)),
        "rl_valid": bool(rl.get("valid", False)),
        "rl_dyaw_deg": finite(rl.get("delta_yaw_rad"), 0.0) * K_RAD_TO_DEG,
        "rl_dpitch_deg": finite(rl.get("delta_pitch_rad"), 0.0) * K_RAD_TO_DEG,
        "rl_gate": int(finite(rl.get("fire_gate"), 1)),
        "score": score,
        "score_events": score_events,
        "fire_ratio_window": rolling.ratio(lambda record: finite(get_nested(record, ["command", "fire_advice"], 0)) > 0),
        "track_ratio_window": rolling.ratio(lambda record: finite(record.get("tracked_count"), 0) > 0),
    }
    next_previous = {"seq": float(seq), "sim_time": sim_time, "wall_time": now}
    return status, next_previous


def fmt_float(value: Any, precision: int = 2, suffix: str = "") -> str:
    if value is None:
        return "n/a"
    number = finite(value, math.nan)
    if not math.isfinite(number):
        return "n/a"
    return f"{number:.{precision}f}{suffix}"


def format_status(status: dict[str, Any]) -> str:
    score = status["score"]
    score_events = status.get("score_events", {}) or {}
    score_text = "score=n/a"
    if score.get("shots") is not None or score.get("hits") is not None:
        hit_rate = score.get("hit_rate")
        hit_rate_pct = None if hit_rate is None else finite(hit_rate, 0.0) * 100.0
        score_text = (
            f"shots={score.get('shots', 0)} hits={score.get('hits', 0)} "
            f"miss={score.get('misses', 0)} hit={fmt_float(hit_rate_pct, 1, '%')} "
            f"dps={fmt_float(score.get('dps'), 1)}"
        )
    hits_by_armor = score_events.get("hits_by_armor", {})
    if isinstance(hits_by_armor, dict) and hits_by_armor:
        board_hits = ",".join(
            f"{name}:{int(hits_by_armor.get(name, 0))}" for name in ["front", "left", "rear", "right"]
        )
        score_text += (
            f" boards={board_hits} lower={int(score_events.get('lower_hits', 0))}"
            f" upper={int(score_events.get('upper_hits', 0))}"
            f" last={score_events.get('last_hit_armor') or 'n/a'}"
        )
    target_error = status.get("target_error_m")
    target_error_text = "target_err=n/a" if target_error is None else f"target_err={target_error * 100.0:.1f}cm"
    if status.get("selected_index", -1) >= 0:
        board_text = f"board={status['selected_name']}/{status['selected_layer']}#{status['selected_index']}"
    else:
        board_text = "board=n/a"
    return (
        f"[SIM] t={fmt_float(status['time_s'], 2, 's')} seq={status['seq']} "
        f"fps={fmt_float(status.get('fps'), 1)} wall={fmt_float(status.get('wall_hz'), 1)} "
        f"truth_lag={fmt_float(status.get('truth_lag_ms'), 1, 'ms')}\n"
        f"[TRACK] state={track_state_name(status['track_state'])}({status['track_state']}) "
        f"mode={command_mode_name(status['mode'])} target={status['target_id'] or 'n/a'} "
        f"tracked={status['tracked_count']} armors={status['armor_count']} "
        f"track_win={fmt_float((status.get('track_ratio_window') or 0.0) * 100.0, 0, '%')}\n"
        f"[AIM] yaw={fmt_float(status['yaw_deg'], 2, 'deg')} pitch={fmt_float(status['pitch_deg'], 2, 'deg')} "
        f"dist={fmt_float(status['distance_m'], 2, 'm')} fire={status['fire']} "
        f"fire_win={fmt_float((status.get('fire_ratio_window') or 0.0) * 100.0, 0, '%')} {board_text}\n"
        f"[ERR] yaw={fmt_float(status['yaw_err_deg'], 3, 'deg')} "
        f"pitch={fmt_float(status['pitch_err_deg'], 3, 'deg')} {target_error_text}\n"
        f"[TIME] proc={fmt_float(status['proc_ms'], 1, 'ms')} "
        f"flight={fmt_float(status['flight_ms'], 1, 'ms')} pred={fmt_float(status['pred_ms'], 1, 'ms')}\n"
        f"[RL] enabled={int(status['rl_enabled'])} valid={int(status['rl_valid'])} "
        f"dyaw={fmt_float(status['rl_dyaw_deg'], 3, 'deg')} "
        f"dpitch={fmt_float(status['rl_dpitch_deg'], 3, 'deg')} gate={status['rl_gate']}\n"
        f"[SCORE] {score_text}"
    )


def wait_message(paths: dict[str, Path]) -> str:
    missing = [name for name, path in paths.items() if not path.exists() or path.stat().st_size <= 0]
    return "waiting for " + ", ".join(missing) if missing else "waiting for new diagnostics frame"


def main() -> int:
    parser = argparse.ArgumentParser(description="Watch HFUT ROS-free Webots auto-aim status.")
    parser.add_argument("--bridge-dir", default=str(default_bridge_dir()))
    parser.add_argument("--diagnostics", default="tracking_diagnostics.jsonl")
    parser.add_argument("--truth", default="target_truth.jsonl")
    parser.add_argument("--score", default="score.txt")
    parser.add_argument("--score-events", default="score_events.jsonl")
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--window", type=int, default=120, help="rolling frame window for fire/track ratios")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--json", action="store_true", help="print one JSON object per update")
    args = parser.parse_args()

    bridge_dir = expand_path(args.bridge_dir)
    diagnostics_path = expand_path(args.diagnostics)
    truth_path = expand_path(args.truth)
    score_path = expand_path(args.score)
    score_events_path = expand_path(args.score_events)
    if not diagnostics_path.is_absolute():
        diagnostics_path = bridge_dir / diagnostics_path
    if not truth_path.is_absolute():
        truth_path = bridge_dir / truth_path
    if not score_path.is_absolute():
        score_path = bridge_dir / score_path
    if not score_events_path.is_absolute():
        score_events_path = bridge_dir / score_events_path

    rolling = RollingStats(args.window)
    previous: dict[str, float] | None = None
    last_seq: int | None = None
    paths = {"diagnostics": diagnostics_path, "truth": truth_path}
    printed_wait = False
    while True:
        diagnostic = read_latest_json(diagnostics_path)
        truth_records = read_tail_jsonl(truth_path)
        if diagnostic is None or not truth_records:
            if not printed_wait:
                print(f"[live_status] bridge={bridge_dir} {wait_message(paths)}", flush=True)
                printed_wait = True
            if args.once:
                return 1
            time.sleep(max(0.1, args.interval))
            continue

        printed_wait = False
        seq = int(finite(diagnostic.get("seq"), 0))
        if seq != last_seq:
            rolling.add(diagnostic)
            score = parse_score(score_path)
            score_events = parse_score_events(score_events_path)
            status, previous = make_status(diagnostic, truth_records, score, score_events, rolling, previous)
            if args.json:
                print(json.dumps(status, ensure_ascii=False, separators=(",", ":")), flush=True)
            else:
                print(format_status(status), flush=True)
            last_seq = seq
        if args.once:
            return 0
        time.sleep(max(0.1, args.interval))


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[live_status] stopped", file=sys.stderr)
        raise SystemExit(130)
