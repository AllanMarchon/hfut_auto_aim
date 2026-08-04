#!/usr/bin/env python3
"""Train a residual RL policy for the HFUT auto-aim simulator wrapper."""

from __future__ import annotations

import argparse
from pathlib import Path

from stable_baselines3 import PPO, SAC
from stable_baselines3.common.monitor import Monitor

from hfut_env import HfutAutoAimEnv, load_config


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/rl_sim.yaml")
    parser.add_argument("--algo", choices=["ppo", "sac"], default=None)
    parser.add_argument("--timesteps", type=int, default=None)
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    config = load_config(Path(args.config))
    training = config.get("training", {}) or {}
    algo = args.algo or str(training.get("algorithm", "ppo")).lower()
    total_timesteps = args.timesteps or int(training.get("total_timesteps", 200000))
    seed = int(training.get("seed", 42))
    log_dir = Path(args.output or training.get("log_dir", "tools/rl/runs"))
    log_dir.mkdir(parents=True, exist_ok=True)

    env = Monitor(HfutAutoAimEnv(args.config), filename=str(log_dir / "monitor.csv"))
    if algo == "ppo":
        model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=str(log_dir), seed=seed)
    else:
        model = SAC("MlpPolicy", env, verbose=1, tensorboard_log=str(log_dir), seed=seed)
    model.learn(total_timesteps=total_timesteps)
    model.save(str(log_dir / "final_model"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
