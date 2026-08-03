#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DIAGNOSTICS_PATH="${DIAGNOSTICS_PATH:-/tmp/hfut_video_test.jsonl}"

exec python3 "${PROJECT_DIR}/tools/debug_3d/server.py" \
  --diagnostics "${DIAGNOSTICS_PATH}" \
  "$@"
