#!/usr/bin/env python3
"""Gymnasium-style wrapper for HFUT auto-aim simulator logs/bridge files.

The first supported target is a residual policy:
  existing command + [delta_yaw, delta_pitch] and an optional fire gate.

The environment has two modes:
  replay: read existing diagnostics/truth/score files without launching Webots.
  bridge: watch a live bridge directory and write rl_action.json.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # pragma: no cover - exercised when deps are absent.
    gym = None
    spaces = None


OBSERVATION_SIZE = 30
ACTION_SIZE = 3


@dataclass
class ScoreStats:
    shots: int = 0
    hits: int = 0
    misses: int = 0
    hit_rate: float = 0.0
    dps: float = 0.0
    elapsed: float = 0.0
    fire_advice: int = 0
    fired: int = 0
    cooldown_remaining: float = 0.0


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8-sig") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"config must be a mapping: {path}")
    return data


def expand_path(value: Any, fallback: str | Path) -> Path:
    raw = str(value or fallback)
    return Path(os.path.expandvars(raw)).expanduser()


def default_bridge_dir() -> Path:
    env_dir = os.environ.get("WEBOTS_ROS_FREE_BRIDGE_DIR", "")
    return expand_path(env_dir, Path.home() / "hfut_auto_aim_webots")


def parse_score_text(text: str) -> ScoreStats:
    values: dict[str, str] = {}
    for token in text.replace("\n", " ").split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        values[key.strip()] = value.strip()

    def as_int(key: str) -> int:
        try:
            return int(float(values.get(key, "0")))
        except ValueError:
            return 0

    def as_float(key: str) -> float:
        try:
            return float(values.get(key, "0"))
        except ValueError:
            return 0.0

    return ScoreStats(
        shots=as_int("shots"),
        hits=as_int("hits"),
        misses=as_int("misses"),
        hit_rate=as_float("hit_rate"),
        dps=as_float("dps"),
        elapsed=as_float("elapsed"),
        fire_advice=as_int("fire_advice"),
        fired=as_int("fired"),
        cooldown_remaining=as_float("cooldown_remaining"),
    )


def read_score(path: Path) -> ScoreStats:
    if not path.exists():
        return ScoreStats()
    return parse_score_text(path.read_text(encoding="utf-8-sig", errors="replace"))


def iter_jsonl(path: Path) -> Iterable[dict[str, Any]]:
    if not path.exists():
        return
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
                yield record


def latest_jsonl(path: Path) -> dict[str, Any] | None:
    latest = None
    for record in iter_jsonl(path):
        latest = record
    return latest


def get_nested(record: dict[str, Any] | None, keys: list[str], default: Any = 0.0) -> Any:
    value: Any = record or {}
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def vector3(value: Any) -> list[float]:
    if not isinstance(value, list) or len(value) < 3:
        return [0.0, 0.0, 0.0]
    out = []
    for item in value[:3]:
        try:
            out.append(float(item))
        except (TypeError, ValueError):
            out.append(0.0)
    return out


def finite(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def build_observation(
    diagnostic: dict[str, Any] | None,
    truth: dict[str, Any] | None,
    score: ScoreStats,
    previous_action: np.ndarray,
) -> np.ndarray:
    command = get_nested(diagnostic, ["command"], {})
    control = get_nested(diagnostic, ["control_target"], {})
    state = get_nested(diagnostic, ["state_estimate"], {})
    delay = get_nested(diagnostic, ["delay"], {})

    current_center = vector3(control.get("current_center"))
    predicted_center = vector3(control.get("predicted_center"))
    target_position = vector3(get_nested(truth, ["target_position"], [0.0, 0.0, 0.0]))
    state_position = vector3(state.get("position"))
    state_velocity = vector3(state.get("velocity"))

    obs = [
        1.0 if diagnostic else 0.0,
        finite(get_nested(diagnostic, ["sim_time_s"], 0.0)),
        finite(get_nested(diagnostic, ["tracked_count"], 0.0)),
        finite(get_nested(diagnostic, ["track_state"], 0.0)),
        finite(command.get("yaw_deg")),
        finite(command.get("pitch_deg")),
        finite(command.get("yaw_diff_deg")),
        finite(command.get("pitch_diff_deg")),
        finite(command.get("yaw_velocity_dps")),
        finite(command.get("pitch_velocity_dps")),
        finite(command.get("distance_m")),
        finite(command.get("fire_advice")),
        1.0 if bool(control.get("valid", False)) else 0.0,
        finite(control.get("yaw_velocity_rad_s")),
        finite(delay.get("prediction_s")),
        finite(delay.get("flight_time_s")),
        finite(delay.get("processing_delay_s")),
        finite(score.shots),
        finite(score.hits),
        finite(score.misses),
        finite(score.hit_rate),
        finite(score.dps),
        *current_center,
        *predicted_center,
        finite(previous_action[0]),
        finite(previous_action[1]),
    ]

    # Keep a stable shape even while the protocol evolves.
    if len(obs) < OBSERVATION_SIZE:
        obs.extend([0.0] * (OBSERVATION_SIZE - len(obs)))
    return np.asarray(obs[:OBSERVATION_SIZE], dtype=np.float32)


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=path.name, suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, separators=(",", ":"))
            handle.write("\n")
        os.replace(tmp_name, path)
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)


class HfutAutoAimEnv(gym.Env if gym else object):
    metadata = {"render_modes": []}

    def __init__(self, config_path: str | Path = "configs/rl_sim.yaml"):
        if gym is None or spaces is None:
            raise RuntimeError(
                "gymnasium is required. Install with: pip install -r tools/requirements-rl.txt"
            )
        self.config_path = Path(config_path)
        self.config = load_config(self.config_path)
        self.mode = str(self.config.get("mode", "replay"))
        self.bridge_dir = expand_path(self.config.get("bridge_dir", ""), default_bridge_dir())
        self.diagnostics_path = self._bridge_file("diagnostics_file")
        self.truth_path = self._bridge_file("truth_file")
        self.score_path = self._bridge_file("score_file")
        self.action_path = self._bridge_file("action_file")

        episode_cfg = self.config.get("episode", {}) or {}
        action_cfg = self.config.get("action", {}) or {}
        self.max_steps = int(episode_cfg.get("max_steps", 1800))
        self.frame_timeout_s = float(episode_cfg.get("frame_timeout_s", 2.0))
        self.max_delta_yaw = float(action_cfg.get("max_delta_yaw_rad", 0.0349066))
        self.max_delta_pitch = float(action_cfg.get("max_delta_pitch_rad", 0.0349066))
        self.fire_gate_threshold = float(action_cfg.get("fire_gate_threshold", 0.0))

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(ACTION_SIZE,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(OBSERVATION_SIZE,), dtype=np.float32
        )

        self._step_index = 0
        self._previous_action = np.zeros(ACTION_SIZE, dtype=np.float32)
        self._previous_score = ScoreStats()
        self._replay_records: list[dict[str, Any]] = []
        self._truth_records: list[dict[str, Any]] = []

    def _bridge_file(self, key: str) -> Path:
        value = str(self.config.get(key, ""))
        path = Path(value)
        return path if path.is_absolute() else self.bridge_dir / path

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        self._step_index = 0
        self._previous_action = np.zeros(ACTION_SIZE, dtype=np.float32)
        self._previous_score = read_score(self.score_path)
        if self.mode == "replay":
            self._replay_records = list(iter_jsonl(self.diagnostics_path))
            self._truth_records = list(iter_jsonl(self.truth_path))
        diagnostic = self._record_at(0)
        truth = self._truth_at(0)
        observation = build_observation(
            diagnostic, truth, self._previous_score, self._previous_action
        )
        return observation, {"mode": self.mode, "bridge_dir": str(self.bridge_dir)}

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, -1.0, 1.0)
        scaled_action = np.asarray(
            [
                float(action[0]) * self.max_delta_yaw,
                float(action[1]) * self.max_delta_pitch,
                1.0 if float(action[2]) >= self.fire_gate_threshold else 0.0,
            ],
            dtype=np.float32,
        )
        self._write_action(scaled_action)

        if self.mode == "bridge":
            diagnostic = self._wait_latest(self.diagnostics_path)
            truth = latest_jsonl(self.truth_path)
        else:
            self._step_index += 1
            diagnostic = self._record_at(self._step_index)
            truth = self._truth_at(self._step_index)

        score = read_score(self.score_path)
        observation = build_observation(diagnostic, truth, score, scaled_action)
        reward = self._reward(diagnostic, score, scaled_action)
        terminated = False
        truncated = self._step_index >= self.max_steps
        if self.mode == "replay" and self._step_index >= max(0, len(self._replay_records) - 1):
            truncated = True
        self._previous_action = scaled_action
        self._previous_score = score
        return observation, reward, terminated, truncated, {"score": score.__dict__}

    def _record_at(self, index: int) -> dict[str, Any] | None:
        if not self._replay_records:
            return None
        return self._replay_records[min(index, len(self._replay_records) - 1)]

    def _truth_at(self, index: int) -> dict[str, Any] | None:
        if not self._truth_records:
            return None
        return self._truth_records[min(index, len(self._truth_records) - 1)]

    def _wait_latest(self, path: Path) -> dict[str, Any] | None:
        deadline = time.monotonic() + self.frame_timeout_s
        latest = latest_jsonl(path)
        while latest is None and time.monotonic() < deadline:
            time.sleep(0.01)
            latest = latest_jsonl(path)
        self._step_index += 1
        return latest

    def _write_action(self, action: np.ndarray) -> None:
        payload = {
            "seq": self._step_index,
            "delta_yaw_rad": float(action[0]),
            "delta_pitch_rad": float(action[1]),
            "fire_gate": int(action[2] > 0.0),
            "timestamp_s": time.time(),
        }
        atomic_write_json(self.action_path, payload)

    def _reward(
        self,
        diagnostic: dict[str, Any] | None,
        score: ScoreStats,
        action: np.ndarray,
    ) -> float:
        reward_cfg = self.config.get("reward", {}) or {}
        hit_delta = score.hits - self._previous_score.hits
        miss_delta = score.misses - self._previous_score.misses
        tracked_count = finite(get_nested(diagnostic, ["tracked_count"], 0.0))
        command = get_nested(diagnostic, ["command"], {})
        yaw_err = abs(finite(command.get("yaw_diff_deg"))) / 10.0
        pitch_err = abs(finite(command.get("pitch_diff_deg"))) / 10.0
        smoothness = float(np.linalg.norm(action[:2] - self._previous_action[:2]))

        return float(
            reward_cfg.get("hit", 10.0) * hit_delta
            + reward_cfg.get("miss", -4.0) * miss_delta
            + reward_cfg.get("aim_error", -1.0) * (yaw_err + pitch_err)
            + reward_cfg.get("command_smoothness", -0.05) * smoothness
            + (reward_cfg.get("lost_target", -0.5) if tracked_count <= 0 else 0.0)
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect the HFUT RL sim wrapper.")
    parser.add_argument("--config", default="configs/rl_sim.yaml")
    parser.add_argument("--steps", type=int, default=5)
    args = parser.parse_args()

    if gym is None:
        print("gymnasium is not installed; config and helper parsing are still available.")
        config = load_config(Path(args.config))
        print(json.dumps(config, indent=2))
        return 0

    env = HfutAutoAimEnv(args.config)
    obs, info = env.reset()
    print(f"reset: obs_shape={obs.shape} info={info}")
    total_reward = 0.0
    for _ in range(args.steps):
        obs, reward, terminated, truncated, info = env.step(np.zeros(ACTION_SIZE, dtype=np.float32))
        total_reward += reward
        print(
            f"step={env._step_index} reward={reward:.3f} "
            f"done={terminated or truncated} score={info['score']}"
        )
        if terminated or truncated:
            break
    print(f"total_reward={total_reward:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
