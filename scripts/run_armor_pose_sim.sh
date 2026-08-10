#!/usr/bin/env bash
# Run the detector-free armor_pose simulator auto-aim loop.
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${PROJECT_DIR}/build/bringup_sim_armor_pose"

if [ ! -x "${BIN}" ]; then
  echo "bringup_sim_armor_pose not built; configure with HFUT_ENABLE_DETECTOR=OFF and HFUT_ENABLE_PIPELINE=ON first." >&2
  exit 1
fi

DEFAULT_BRIDGE_DIR="${HOME}/hfut_auto_aim_webots"
export WEBOTS_ROS_FREE_BRIDGE_DIR="${WEBOTS_ROS_FREE_BRIDGE_DIR:-${DEFAULT_BRIDGE_DIR}}"
mkdir -p "${WEBOTS_ROS_FREE_BRIDGE_DIR}"

exec "${BIN}" \
  "--config-dir=${PROJECT_DIR}/configs" \
  "--bridge-dir=${WEBOTS_ROS_FREE_BRIDGE_DIR}" \
  "$@"
