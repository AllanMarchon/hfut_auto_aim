#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"

if [[ ! -x "${BUILD_DIR}/video_test" ]]; then
  echo "video_test is not built; run scripts/build.sh first" >&2
  exit 1
fi

cd "${PROJECT_DIR}"
exec "${BUILD_DIR}/video_test" \
  --video="${VIDEO_PATH:-${PROJECT_DIR}/test_video/output.avi}" \
  --camera-info="${CAMERA_INFO_PATH:-${PROJECT_DIR}/test_video/camera_info.yaml}" \
  --config-dir="${CONFIG_DIR:-${PROJECT_DIR}/configs}" \
  --calibration-mode="${CALIBRATION_MODE:-center_crop}" \
  --output="${OVERLAY_PATH:-/tmp/hfut_video_test_overlay.avi}" \
  --diagnostics="${DIAGNOSTICS_PATH:-/tmp/hfut_video_test.jsonl}" \
  --summary="${SUMMARY_PATH:-/tmp/hfut_video_test_summary.json}" \
  "$@"
