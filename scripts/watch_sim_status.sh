#!/usr/bin/env bash
# Watch the ROS-free Webots auto-aim bridge status in real time.
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_BRIDGE_DIR="${HOME}/hfut_auto_aim_webots"
export WEBOTS_ROS_FREE_BRIDGE_DIR="${WEBOTS_ROS_FREE_BRIDGE_DIR:-${DEFAULT_BRIDGE_DIR}}"

exec python3 "${PROJECT_DIR}/tools/sim/live_status.py" \
  --bridge-dir "${WEBOTS_ROS_FREE_BRIDGE_DIR}" \
  "$@"
