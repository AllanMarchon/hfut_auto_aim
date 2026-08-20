#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${1:-auto_aim}"
SERVICE_NAME="${SERVICE_NAME%.service}"
UNIT_PATH="/etc/systemd/system/${SERVICE_NAME}.service"

if [[ "$(id -u)" -ne 0 ]]; then
    echo "This script must be run as root: sudo bash scripts/uninstall_auto_aim_service.sh" >&2
    exit 1
fi

systemctl disable --now "$SERVICE_NAME" 2>/dev/null || true
rm -f "$UNIT_PATH"
systemctl daemon-reload
systemctl reset-failed "$SERVICE_NAME" 2>/dev/null || true

echo "Removed ${SERVICE_NAME}.service"
echo "Environment file, if any, was left at /etc/default/${SERVICE_NAME}"
