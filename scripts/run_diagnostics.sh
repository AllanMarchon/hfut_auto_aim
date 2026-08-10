#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_BRIDGE_DIR="${HOME}/hfut_auto_aim_webots"
export WEBOTS_ROS_FREE_BRIDGE_DIR="${WEBOTS_ROS_FREE_BRIDGE_DIR:-${DEFAULT_BRIDGE_DIR}}"

"${PROJECT_DIR}/scripts/run.sh" --diagnostics "$@"
