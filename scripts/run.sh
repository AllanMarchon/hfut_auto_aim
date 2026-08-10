#!/usr/bin/env bash
# Run the ros-free auto_aim program against a running Webots sim.
# Start the sim FIRST (in another terminal) with the ros-free bridge:
#   cd ~/hfut_auto_aim_sim && WEBOTS_CAMERA_CONTROLLER=ros_free_camera_bridge \
#       ./run_stationary_spin_target_test.sh
#
# Configs live in <repo>/configs/ (topic-split):
#   gimbal_pipeline.yaml  总控开关（模式/实现/策略选择）
#   simulation.yaml       仿真桥接、相机外参、弹速
#   detector.yaml         NN/传统检测器
#   tracker.yaml          跟踪/滤波/输出平滑 + 跟踪防抖
#   controller.yaml       解算/延时/开火/MPC
# Override the whole set with --config-dir=/path/to/configs.
# Use --strategy=predicted for a per-process comparison against the checked-in
# MPC default; Webots should normally use mpc_state command interpretation.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${PROJECT_DIR}/build/bringup_sim"

if [ ! -x "${BIN}" ]; then
  echo "bringup_sim not built; run scripts/build.sh first." >&2
  exit 1
fi

# onnxruntime-gpu is a loose binary install, not on the default loader path.
export LD_LIBRARY_PATH="${ONNXRUNTIME_ROOT:-/opt/onnxruntime-gpu}/lib:${LD_LIBRARY_PATH:-}"
DEFAULT_BRIDGE_DIR="${HOME}/hfut_auto_aim_webots"
export WEBOTS_ROS_FREE_BRIDGE_DIR="${WEBOTS_ROS_FREE_BRIDGE_DIR:-${DEFAULT_BRIDGE_DIR}}"

# Default config set = <repo>/configs; a later --config-dir in "$@" wins.
exec "${BIN}" "--config-dir=${PROJECT_DIR}/configs" "$@"
