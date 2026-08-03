#!/usr/bin/env bash
# Configure + build hfut_auto_aim into build/.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"

cmake -S "${PROJECT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
# Heavy template TUs (qpOASES MPC, UKF) are memory-hungry; cap parallelism to
# avoid OOM. Override with BUILD_JOBS.
cmake --build "${BUILD_DIR}" --parallel "${BUILD_JOBS:-2}"
