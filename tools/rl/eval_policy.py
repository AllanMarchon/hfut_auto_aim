#!/usr/bin/env python3
"""Evaluate a zero-action or Stable-Baselines3 policy on the HFUT RL wrapper."""

from __future__ import annotations

import argparse

import numpy as np

from hfut_env import ACTION_SIZE, HfutAutoAimEnv


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/rl_sim.yaml")
    parser.add_argument("--model", default="")
    parser.add_argument("--steps", type=int, default=1800)
    args = parser.parse_args()

    model = None
    if args.model:
        from stable_baselines3 import PPO

        model = PPO.load(args.model)

    env = HfutAutoAimEnv(args.config)
    obs, _ = env.reset()
    total_reward = 0.0
    for _ in range(args.steps):
        if model is None:
            action = np.zeros(ACTION_SIZE, dtype=np.float32)
        else:
            action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        if terminated or truncated:
            break
    score = info.get("score", {})
    print(f"total_reward={total_reward:.3f}")
    print(f"score={score}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
