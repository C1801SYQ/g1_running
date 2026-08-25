#!/usr/bin/env bash
# Skill 6 preparation-only RealSense run.
#
# This mirrors the Skill 7 audit flow, but deliberately stops before any
# real-robot deployment step: no rl_real_g1, no lowcmd, no systemd install.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATUS_PID=""
STATUS_PORT="${G1_VISION_STATUS_PORT:-15002}"
STATUS_SECONDS="${G1_SKILL6_PREPARE_STATUS_SECONDS:-120}"

cleanup() {
  if [[ -n "${STATUS_PID}" ]] && kill -0 "${STATUS_PID}" 2>/dev/null; then
    kill "${STATUS_PID}" 2>/dev/null || true
    wait "${STATUS_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if pgrep -f '(^|/)(rl_real_g1|g1_ctrl)([[:space:]]|$)' >/dev/null; then
  echo "Refusing Skill 6 prepare: a robot motion controller is already running." >&2
  exit 2
fi

echo "* SKILL 6 PREPARE ONLY"
echo "* RealSense + vision dry-run: robot command output stays disabled."
echo "* This script does not start rl_real_g1 and does not install any service."
echo "* Inspect /g1/race/desired_cmd_vel and /g1/race/safe_cmd_vel for commands;"
echo "  /g1/race/cmd_vel remains zero because command_output_enabled=false."

(
  end_time=$((SECONDS + STATUS_SECONDS))
  while (( SECONDS < end_time )); do
    python3 -c 'import os, socket; port=int(os.environ.get("G1_VISION_STATUS_PORT","15002")); s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto(b"G1_VISION_SPRINT 1\n",("127.0.0.1",port)); s.close()'
    sleep 0.5
  done
) &
STATUS_PID=$!

G1_AUDIT_MISSION=sprint100m \
G1_AUDIT_UDP_ENABLED=false \
G1_VISION_CRUISE_SPEED_MPS="${G1_SKILL6_PREPARE_SPEED:-5.10}" \
G1_SKILL6_RAMP_TO_1="${G1_SKILL6_RAMP_TO_1:-0.60}" \
G1_SKILL6_RAMP_TO_3="${G1_SKILL6_RAMP_TO_3:-0.80}" \
G1_SKILL6_RAMP_TO_MAX="${G1_SKILL6_RAMP_TO_MAX:-1.00}" \
G1_VISION_STATUS_PORT="${STATUS_PORT}" \
  bash "${SCRIPT_DIR}/run_g1_realsense_dry_run.sh"
