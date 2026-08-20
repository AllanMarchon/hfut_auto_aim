#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${1:-auto_aim}"
SERVICE_NAME="${SERVICE_NAME%.service}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
UNIT_PATH="/etc/systemd/system/${SERVICE_NAME}.service"
ENV_PATH="/etc/default/${SERVICE_NAME}"

if [[ "$(id -u)" -ne 0 ]]; then
    echo "This script must be run as root: sudo bash scripts/install_auto_aim_service.sh" >&2
    exit 1
fi

USER_NAME="${SUDO_USER:-$(logname 2>/dev/null || id -un)}"
if [[ -z "$USER_NAME" || "$USER_NAME" == "root" ]]; then
    echo "Could not determine the non-root service user. Run with sudo from the target user." >&2
    exit 1
fi

chmod +x "$PROJECT_DIR/scripts/auto_aim_service.sh"

if [[ ! -f "$ENV_PATH" ]]; then
    cat > "$ENV_PATH" <<EOF
# HFUT auto aim service environment.
# Change these values, then run: sudo systemctl restart ${SERVICE_NAME}
AUTO_AIM_PROJECT_DIR="$PROJECT_DIR"
AUTO_AIM_MODE="live"
AUTO_AIM_ARGS="--no-web-view"
AUTO_AIM_START_DELAY="5"
PYTHONUNBUFFERED="1"
EOF
    chmod 0644 "$ENV_PATH"
else
    echo "Keeping existing $ENV_PATH"
fi

cat > "$UNIT_PATH" <<EOF
[Unit]
Description=HFUT Auto Aim
After=multi-user.target

[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$PROJECT_DIR
EnvironmentFile=-$ENV_PATH
Environment=PYTHONUNBUFFERED=1
ExecStart=/bin/bash $PROJECT_DIR/scripts/auto_aim_service.sh
Restart=always
RestartSec=3
KillSignal=SIGINT
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable "$SERVICE_NAME"
systemctl restart "$SERVICE_NAME"

echo "Installed and started ${SERVICE_NAME}.service"
echo "Status: systemctl status ${SERVICE_NAME}"
echo "Logs:   journalctl -u ${SERVICE_NAME} -f"
