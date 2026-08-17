#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="g1_vision_skill7.service"
UNIT_SRC="${SCRIPT_DIR}/${SERVICE_NAME}"
UNIT_DST="/etc/systemd/system/${SERVICE_NAME}"

if [[ ! -f "${UNIT_SRC}" ]]; then
    echo "Missing unit file: ${UNIT_SRC}" >&2
    exit 1
fi

if [[ "${EUID}" -ne 0 ]]; then
    echo "Root is required to install ${SERVICE_NAME}."
    echo "Re-running with sudo..."
    exec sudo "$0" "$@"
fi

install -m 0644 "${UNIT_SRC}" "${UNIT_DST}"
systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"

echo
echo "Installed and enabled: ${SERVICE_NAME}"
echo
echo "Start now:"
echo "  sudo systemctl start ${SERVICE_NAME}"
echo
echo "Stop:"
echo "  sudo systemctl stop ${SERVICE_NAME}"
echo
echo "Logs:"
echo "  journalctl -u ${SERVICE_NAME} -f"
