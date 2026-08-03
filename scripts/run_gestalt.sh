#!/usr/bin/env bash
# Run hfut_auto_aim in WSL2 against tools/gestalt_bridge_windows.py on Windows.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -z "${GESTALT_BRIDGE_HOST:-}" ]; then
  NETWORKING_MODE="$(wslinfo --networking-mode 2>/dev/null || true)"
  if [ "${NETWORKING_MODE}" = "mirrored" ]; then
    export GESTALT_BRIDGE_HOST=127.0.0.1
  else
    GESTALT_BRIDGE_HOST="$(ip route show default | awk 'NR == 1 {print $3}')"
    if [ -z "${GESTALT_BRIDGE_HOST}" ]; then
      echo "Cannot determine the Windows host address; set GESTALT_BRIDGE_HOST." >&2
      exit 1
    fi
    export GESTALT_BRIDGE_HOST
  fi
fi

echo "Gestalt Windows bridge: ${GESTALT_BRIDGE_HOST}:47000" >&2
exec "${PROJECT_DIR}/scripts/run.sh" --gestalt "$@"
