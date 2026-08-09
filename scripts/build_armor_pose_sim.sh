#!/usr/bin/env bash
# Configure + build the detector-free armor_pose simulator target.
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"

cmake -S "${PROJECT_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" \
  -DHFUT_ENABLE_DETECTOR=OFF \
  -DHFUT_ENABLE_PIPELINE=ON

cmake --build "${BUILD_DIR}" --parallel "${BUILD_JOBS:-2}" --target bringup_sim_armor_pose
