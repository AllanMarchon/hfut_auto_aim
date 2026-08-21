#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${AUTO_AIM_PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
MODE="${AUTO_AIM_MODE:-live}"
START_DELAY="${AUTO_AIM_START_DELAY:-5}"
EXTRA_ARGS_TEXT="${AUTO_AIM_ARGS:---allow-fire}"

export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

if [[ "$START_DELAY" =~ ^[0-9]+$ ]] && (( START_DELAY > 0 )); then
    sleep "$START_DELAY"
fi

if [[ -f /opt/MVS/bin/set_env_path.sh ]]; then
    export MVCAM_SDK_PATH="${MVCAM_SDK_PATH:-/opt/MVS}"
    set +u
    # shellcheck disable=SC1091
    source /opt/MVS/bin/set_env_path.sh || echo "[auto_aim] warning: failed to source /opt/MVS/bin/set_env_path.sh"
    set -u
fi

if [[ -d /opt/MVS/lib/64 ]]; then
    export LD_LIBRARY_PATH="/opt/MVS/lib/64:${LD_LIBRARY_PATH:-}"
fi

if [[ -d /usr/lib/openvino-2025.3.0 ]]; then
    export LD_LIBRARY_PATH="/usr/lib/openvino-2025.3.0:${LD_LIBRARY_PATH:-}"
fi

cd "$PROJECT_DIR"

EXTRA_ARGS=()
if [[ -n "$EXTRA_ARGS_TEXT" ]]; then
    read -r -a EXTRA_ARGS <<< "$EXTRA_ARGS_TEXT"
fi

echo "[auto_aim] project=$PROJECT_DIR"
echo "[auto_aim] mode=$MODE args=${EXTRA_ARGS[*]:-}"

exec /usr/bin/python3 scripts/start.py --mode "$MODE" "${EXTRA_ARGS[@]}"
