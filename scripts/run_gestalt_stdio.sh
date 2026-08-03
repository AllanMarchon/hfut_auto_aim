#!/usr/bin/env bash
# Launch the Windows proxy from WSL and carry frames over WSL interop pipes.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "${PROJECT_DIR}/tools/gestalt_stdio_launcher.py" "$@" "--debug" "--allow-fire"
