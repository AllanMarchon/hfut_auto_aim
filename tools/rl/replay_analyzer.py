#!/usr/bin/env python3
"""Summarize one HFUT simulator episode for RL baseline/reward work."""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
from bisect import bisect_left
from collections import Counter
from pathlib import Path
from typing import Any

from hfut_env import ScoreStats, finite, get_nested, iter_jsonl, read_score


K_DAMAGE_PER_HIT = 20.0


def expand_path(value: str | Path) -> Path:
    return Path(os.path.expandvars(str(value))).expanduser()


def default_bridge_dir() -> Path:
    env_dir = os.environ.get("WEBOTS_ROS_FREE_BRIDGE_DIR", "")
    return expand_path(env_dir) if env_dir else Path.home() / "hfut_auto_aim_webots"


def is_vector3(value: Any) -> bool:
    return isinstance(value, list) and len(value) >= 3


def vector3(value: Any) -> list[float] | None:
    if not is_vector3(value):
        return None
    out = [finite(item, math.nan) for item in value[:3]]
    return out if all(math.isfinite(item) for item in out) else None


def distance(left: list[float], right: list[float]) -> float:
    return math.sqrt(sum((a - b) * (a - b) for a, b in zip(left, right)))


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * q))))
    return ordered[index]


def metric_summary(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {
            "samples": 0,
            "mean": None,
            "mean_abs": None,
            "median_abs": None,
            "p90_abs": None,
            "p99_abs": None,
            "max_abs": None,
        }
    abs_values = [abs(value) for value in values]
    return {
        "samples": len(values),
        "mean": statistics.fmean(values),
        "mean_abs": statistics.fmean(abs_values),
        "median_abs": statistics.median(abs_values),
        "p90_abs": percentile(abs_values, 0.90),
        "p99_abs": percentile(abs_values, 0.99),
        "max_abs": max(abs_values),
    }


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return list(iter_jsonl(path))


def nearest_by_time(records: list[dict[str, Any]], times: list[float], target_time: float) -> dict[str, Any] | None:
    if not records:
        return None
    index = bisect_left(times, target_time)
    if index >= len(records):
        index = len(records) - 1
    if index > 0 and abs(times[index - 1] - target_time) <= abs(times[index] - target_time):
        index -= 1
    return records[index]


def truth_center_control(truth_record: dict[str, Any]) -> list[float] | None:
    shooter = vector3(truth_record.get("shooter_position")) or [0.0, 0.0, 0.0]
    armors = truth_record.get("armors", [])
    positions = [
        position
        for armor in armors
        if isinstance(armor, dict)
        for position in [vector3(armor.get("position"))]
        if position is not None
    ]
    if positions:
        return [
            statistics.fmean(position[axis] for position in positions) - shooter[axis]
            for axis in range(3)
        ]
    target = vector3(truth_record.get("target_position"))
    if target is None:
        return None
    return [target[axis] - shooter[axis] for axis in range(3)]


def truth_position_for_control(
    diagnostic: dict[str, Any],
    truth_record: dict[str, Any],
) -> list[float] | None:
    control = diagnostic.get("control_target", {})
    if not isinstance(control, dict) or not control.get("valid", False):
        return None

    if control.get("tracks_center", False) or control.get("virtual_target", False):
        return truth_center_control(truth_record)

    command_target = vector3(control.get("control_target_position"))
    shooter = vector3(truth_record.get("shooter_position")) or [0.0, 0.0, 0.0]
    truth_positions = []
    for armor in truth_record.get("armors", []):
        if not isinstance(armor, dict):
            continue
        position = vector3(armor.get("position"))
        if position is not None:
            truth_positions.append([position[axis] - shooter[axis] for axis in range(3)])
    if command_target is None or not truth_positions:
        return None
    return min(truth_positions, key=lambda candidate: distance(command_target, candidate))


def summarize_rewards(
    diagnostics: list[dict[str, Any]],
    score: ScoreStats,
    reward_cfg: dict[str, float],
) -> dict[str, float]:
    yaw_errors = [
        abs(finite(get_nested(record, ["command", "yaw_diff_deg"], 0.0))) / 10.0
        for record in diagnostics
    ]
    pitch_errors = [
        abs(finite(get_nested(record, ["command", "pitch_diff_deg"], 0.0))) / 10.0
        for record in diagnostics
    ]
    lost_frames = sum(1 for record in diagnostics if finite(record.get("tracked_count"), 0.0) <= 0)
    hit_reward = reward_cfg.get("hit", 10.0) * score.hits
    miss_reward = reward_cfg.get("miss", -4.0) * score.misses
    aim_error_penalty = reward_cfg.get("aim_error", -1.0) * sum(
        yaw + pitch for yaw, pitch in zip(yaw_errors, pitch_errors)
    )
    lost_penalty = reward_cfg.get("lost_target", -0.5) * lost_frames
    total = hit_reward + miss_reward + aim_error_penalty + lost_penalty
    return {
        "hit_reward": hit_reward,
        "miss_reward": miss_reward,
        "aim_error_penalty": aim_error_penalty,
        "lost_target_penalty": lost_penalty,
        "zero_action_smoothness_penalty": 0.0,
        "total_proxy_reward": total,
    }


def summarize_episode(
    diagnostics: list[dict[str, Any]],
    truth: list[dict[str, Any]],
    score: ScoreStats,
    reward_cfg: dict[str, float] | None = None,
) -> dict[str, Any]:
    reward_cfg = reward_cfg or {}
    diagnostics = sorted(diagnostics, key=lambda record: finite(record.get("sim_time_s"), 0.0))
    truth = sorted(truth, key=lambda record: finite(record.get("sim_time_s"), 0.0))
    truth_times = [finite(record.get("sim_time_s"), 0.0) for record in truth]
    sim_times = [finite(record.get("sim_time_s"), 0.0) for record in diagnostics]
    duration_s = max(0.0, sim_times[-1] - sim_times[0]) if len(sim_times) >= 2 else 0.0

    tracked_frames = sum(1 for record in diagnostics if finite(record.get("tracked_count"), 0.0) > 0)
    tracking_state_frames = sum(1 for record in diagnostics if int(finite(record.get("track_state"), -1)) >= 2)
    control_valid_frames = sum(
        1 for record in diagnostics if bool(get_nested(record, ["control_target", "valid"], False))
    )
    fire_advice_frames = sum(
        1 for record in diagnostics if finite(get_nested(record, ["command", "fire_advice"], 0.0)) > 0
    )

    yaw_diff_deg = [
        finite(get_nested(record, ["command", "yaw_diff_deg"], 0.0))
        for record in diagnostics
    ]
    pitch_diff_deg = [
        finite(get_nested(record, ["command", "pitch_diff_deg"], 0.0))
        for record in diagnostics
    ]
    distances_m = [
        finite(get_nested(record, ["command", "distance_m"], math.nan), math.nan)
        for record in diagnostics
    ]
    distances_m = [value for value in distances_m if math.isfinite(value)]

    current_target_errors = []
    predicted_target_errors = []
    truth_time_errors = []
    for record in diagnostics:
        sim_time = finite(record.get("sim_time_s"), 0.0)
        truth_record = nearest_by_time(truth, truth_times, sim_time)
        if truth_record is None:
            continue
        truth_time_errors.append(abs(finite(truth_record.get("sim_time_s"), 0.0) - sim_time))
        command_target = vector3(get_nested(record, ["control_target", "control_target_position"], []))
        truth_target = truth_position_for_control(record, truth_record)
        if command_target is not None and truth_target is not None:
            current_target_errors.append(distance(command_target, truth_target))

        prediction_s = finite(
            get_nested(record, ["delay", "prediction_s"], get_nested(record, ["control_target", "prediction_time_s"], 0.0))
        )
        future_truth = nearest_by_time(truth, truth_times, sim_time + max(0.0, prediction_s))
        if future_truth is None:
            continue
        future_target = truth_position_for_control(record, future_truth)
        if command_target is not None and future_target is not None:
            predicted_target_errors.append(distance(command_target, future_target))

    resolved_shots = score.hits + score.misses
    effective_dps = score.hits * K_DAMAGE_PER_HIT / duration_s if duration_s > 0.0 else 0.0
    return {
        "frames": len(diagnostics),
        "truth_frames": len(truth),
        "duration_s": duration_s,
        "avg_fps": len(diagnostics) / duration_s if duration_s > 0.0 else None,
        "input_modes": dict(Counter(str(record.get("input_mode", "")) for record in diagnostics)),
        "bridge_paths": dict(Counter(str(record.get("bridge_path", "")) for record in diagnostics)),
        "tracking": {
            "tracked_count_positive_ratio": tracked_frames / len(diagnostics) if diagnostics else None,
            "track_state_ge_2_ratio": tracking_state_frames / len(diagnostics) if diagnostics else None,
            "control_target_valid_ratio": control_valid_frames / len(diagnostics) if diagnostics else None,
            "fire_advice_ratio": fire_advice_frames / len(diagnostics) if diagnostics else None,
        },
        "score": {
            "shots": score.shots,
            "hits": score.hits,
            "misses": score.misses,
            "hit_rate": score.hit_rate,
            "resolved_hit_rate": score.hits / resolved_shots if resolved_shots > 0 else None,
            "score_dps": score.dps,
            "effective_dps": effective_dps,
            "score_elapsed_s": score.elapsed,
        },
        "command_error_deg": {
            "yaw_diff": metric_summary(yaw_diff_deg),
            "pitch_diff": metric_summary(pitch_diff_deg),
        },
        "distance_m": metric_summary(distances_m),
        "truth_alignment_s": metric_summary(truth_time_errors),
        "control_target_error_m": {
            "current": metric_summary(current_target_errors),
            "predicted": metric_summary(predicted_target_errors),
        },
        "reward_proxy": summarize_rewards(diagnostics, score, reward_cfg),
    }


def load_reward_config(config_path: Path | None) -> dict[str, float]:
    if config_path is None or not config_path.exists():
        return {}
    try:
        import yaml
    except ImportError:
        return {}
    with config_path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    reward = config.get("reward", {}) if isinstance(config, dict) else {}
    return {str(key): float(value) for key, value in reward.items()} if isinstance(reward, dict) else {}


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze one RL simulator replay episode.")
    parser.add_argument("--bridge-dir", default=str(default_bridge_dir()))
    parser.add_argument("--diagnostics", default="")
    parser.add_argument("--truth", default="")
    parser.add_argument("--score", default="")
    parser.add_argument("--config", default="configs/rl_sim.yaml")
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    bridge_dir = expand_path(args.bridge_dir)
    diagnostics_path = Path(args.diagnostics) if args.diagnostics else bridge_dir / "tracking_diagnostics.jsonl"
    truth_path = Path(args.truth) if args.truth else bridge_dir / "target_truth.jsonl"
    score_path = Path(args.score) if args.score else bridge_dir / "score.txt"
    reward_cfg = load_reward_config(Path(args.config) if args.config else None)

    diagnostics = load_jsonl(diagnostics_path)
    truth = load_jsonl(truth_path)
    score = read_score(score_path)
    summary = summarize_episode(diagnostics, truth, score, reward_cfg)
    rendered = json.dumps(summary, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
