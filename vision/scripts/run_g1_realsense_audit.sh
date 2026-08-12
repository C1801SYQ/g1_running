#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BRIDGE_BINARY="${G1_AUDIT_BRIDGE_BINARY:-${PROJECT_ROOT}/vision/integrations/g1_loco_bridge/build/g1_loco_audit_bridge}"
BRIDGE_PID=""
STATUS_PID=""

cleanup() {
  if [[ -n "${STATUS_PID}" ]] && kill -0 "${STATUS_PID}" 2>/dev/null; then
    kill "${STATUS_PID}" 2>/dev/null || true
    wait "${STATUS_PID}" 2>/dev/null || true
  fi
  if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    kill -INT "${BRIDGE_PID}" 2>/dev/null || true
    wait "${BRIDGE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ ! -x "${BRIDGE_BINARY}" ]]; then
  echo "Audit bridge binary not found: ${BRIDGE_BINARY}" >&2
  exit 1
fi
if pgrep -f '(^|/)(rl_real_g1|g1_ctrl)([[:space:]]|$)' >/dev/null; then
  echo "Refusing audit: a robot motion controller is already running." >&2
  exit 2
fi

"${BRIDGE_BINARY}" --mode=audit --max-vx=0.50 --max-wz=0.25 &
BRIDGE_PID=$!
sleep 1
if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
  echo "Audit bridge exited during startup." >&2
  wait "${BRIDGE_PID}"
  exit 1
fi

echo "Starting camera + depth safety with audit UDP on port 15003."
# Audit-only lifecycle stimulus: mimic the C++ Num7 enable heartbeat so the
# Python Num7 gate, line follower, distance gate, and hard-stop output are all
# exercised. The real command port remains physically disconnected because
# run_g1_realsense_dry_run.sh forces command_output_enabled=false.
(
  for _ in $(seq 1 50); do
    python3 -c 'import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto(b"G1_VISION_WALK_0P5M 1\n",("127.0.0.1",15002)); s.close()'
    sleep 0.5
  done
) &
STATUS_PID=$!
G1_AUDIT_UDP_ENABLED=true \
G1_AUDIT_MISSION=walk0p5m \
G1_NUM7_TARGET_M="${G1_NUM7_TARGET_M:-1.00}" \
G1_NUM7_DISTANCE_SCALE="${G1_NUM7_DISTANCE_SCALE:-0.80}" \
G1_VISION_CRUISE_SPEED_MPS="${G1_VISION_CRUISE_SPEED_MPS:-0.50}" \
  bash "${SCRIPT_DIR}/run_g1_realsense_dry_run.sh"
