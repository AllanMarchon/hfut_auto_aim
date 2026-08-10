#!/usr/bin/env bash
# Launch the ROS-free Webots armor_pose simulation side.
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="${PROJECT_DIR}/sim/webots"

scenario="${1:-stationary}"
case "${scenario}" in
  stationary)
    shift || true
    exec "${SIM_DIR}/run_stationary_spin_target_test.sh" "$@"
    ;;
  moving)
    shift || true
    exec "${SIM_DIR}/run_moving_spin_target_test.sh" "$@"
    ;;
  *)
    exec "${SIM_DIR}/run_stationary_spin_target_test.sh" "$@"
    ;;
esac
