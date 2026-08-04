# RL simulator plan

This branch keeps RL as an optional simulator experiment. It must not replace
the normal `predicted` or `mpc` auto-aim path until the policy beats the
baseline difficulty matrix.

## What can be done on Windows now

- Keep this branch current with `main`.
- Maintain `configs/rl_sim.yaml`.
- Edit and syntax-check the Python wrapper in `tools/rl/`.
- Parse existing `tracking_diagnostics.jsonl`, `target_truth.jsonl`, and
  `score.txt` files if they are copied from a Linux/Webots run.
- Run zero-action replay tests to verify observation and reward code shape.

## What still needs Linux/Webots

- Build and run `bringup_sim`.
- Run the Webots difficulty matrix.
- Train against a live simulator.
- Validate that an RL residual action does not degrade the baseline.

## First RL target

Do not train an end-to-end image policy first. The first policy should be a
small residual over the existing command:

- action[0]: yaw residual in radians
- action[1]: pitch residual in radians
- action[2]: fire gate

The baseline auto-aim remains responsible for detection, PnP, tracking,
prediction, and ballistic compensation.

## Linux bring-up sequence

1. Build and test the main project.
2. Run the `armor_pose` difficulty matrix for `predicted` and `mpc`.
3. Save those reports as the baseline.
4. Run `tools/rl/eval_policy.py` with the zero-action policy.
5. Add the C++ residual hook only after replay observation/reward code is
   stable.
6. Train PPO/SAC only after zero-action performance matches the baseline.

## Commands

Install Python dependencies:

```bash
python3 -m pip install -r tools/requirements-rl.txt
```

Inspect wrapper state without training:

```bash
python3 tools/rl/hfut_env.py --config configs/rl_sim.yaml --steps 5
```

Summarize one copied Webots/Gestalt replay:

```bash
python3 tools/rl/replay_analyzer.py \
  --bridge-dir /tmp/hfut_auto_aim_webots \
  --config configs/rl_sim.yaml
```

Evaluate a zero-action policy:

```bash
python3 tools/rl/eval_policy.py --config configs/rl_sim.yaml --steps 1800
```

Train later, after live sim support is connected:

```bash
python3 tools/rl/train.py --config configs/rl_sim.yaml --algo ppo
```
