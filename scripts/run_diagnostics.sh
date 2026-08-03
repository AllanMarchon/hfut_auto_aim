#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export WEBOTS_ROS_FREE_BRIDGE_DIR="${WEBOTS_ROS_FREE_BRIDGE_DIR:-/tmp/hfut_auto_aim_webots}"

"${PROJECT_DIR}/scripts/run.sh" --diagnostics "$@"
