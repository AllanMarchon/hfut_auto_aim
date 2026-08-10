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

- Build and run `bringup_sim_armor_pose` for the first detector-free loop.
- Build and run the full `bringup_sim` only after ONNX Runtime is installed.
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

A zero-action policy must preserve the baseline command: zero yaw/pitch residual
and fire gate open. This lets us prove the wrapper itself is not changing the
old simulator behavior.

## Linux bring-up sequence

1. Build the armor-pose simulator target with detector disabled.
2. Launch the ROS-free Webots side so it writes `armor_pose_frame.bin`.
3. Launch `bringup_sim_armor_pose` so it writes `gimbal_command.bin`.
4. Save those reports as the baseline.
5. Run `tools/rl/eval_policy.py` with the zero-action policy.
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

Configure and build the detector-free armor-pose simulator path:

```bash
./scripts/build_armor_pose_sim.sh
```

Equivalent manual CMake commands:

```bash
rm -rf build
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DHFUT_ENABLE_DETECTOR=OFF \
  -DHFUT_ENABLE_PIPELINE=ON
cmake --build build --parallel 2
```

Run the armor-pose simulator loop with diagnostics and the optional C++
residual hook enabled:

```bash
./scripts/run_armor_pose_sim.sh --strategy=predicted --diagnostics --rl-action
```

Launch the Webots side in another terminal first. On a headless NUC this script
automatically uses `xvfb-run` when `DISPLAY` is empty:

```bash
./scripts/run_webots_armor_pose_sim.sh stationary --batch --mode=fast --no-rendering
```

The moving target variant uses the same world and file bridge:

```bash
./scripts/run_webots_armor_pose_sim.sh moving --batch --mode=fast --no-rendering
```

Expected bridge files after both processes start:

```text
/tmp/hfut_auto_aim_webots/armor_pose_frame.bin
/tmp/hfut_auto_aim_webots/gimbal_command.bin
/tmp/hfut_auto_aim_webots/tracking_diagnostics.jsonl
/tmp/hfut_auto_aim_webots/target_truth.jsonl
```

With `--rl-action` and no explicit path, `bringup_sim_armor_pose` reads
`rl_action.json` from the active bridge directory. Missing action files are
treated as no-op.

The full vision-mode `bringup_sim` still requires ONNX Runtime. If the machine
does not have ONNX Runtime under `/opt/onnxruntime-gpu`, either install it or
configure CMake with `-DONNXRUNTIME_ROOT=/path/to/onnxruntime` before enabling
`HFUT_ENABLE_DETECTOR=ON`.

Train later, after live sim support is connected:

```bash
python3 tools/rl/train.py --config configs/rl_sim.yaml --algo ppo
```
