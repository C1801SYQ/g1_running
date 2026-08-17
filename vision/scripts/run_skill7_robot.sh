#!/usr/bin/env bash
# Skill 7 real-robot active vision walk (direct launcher).
#
# This script starts everything in ONE terminal:
#   RealSense camera -> Python Num7 vision node -> rl_real_g1 controller
# The controller runs in the foreground, so the keyboard keys go to it:
#   press 0 -> 1 -> 7  (GetUp -> Locomotion -> Skill 7 line follow)
#
# SECURITY: this is live robot motion. The robot must be suspended and an
# E-stop must be within reach. The command output is only enabled when
# G1_NUM7_COMMAND_OUTPUT_ENABLED=1 is set explicitly.
set -euo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
REALSENSE_SETUP="${REALSENSE_SETUP:-/home/unitree/g1_realsense_ws/install/local_setup.bash}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
RL_SAR_ROOT="${G1_RUNNING_ROOT:-${PROJECT_ROOT}}/rl_sar"
ROBOT_INTERFACE="${G1_ROBOT_INTERFACE:-enP8p1s0}"

TARGET_M="${G1_NUM7_TARGET_M:-1.00}"
TARGET_M="$(python3 -c 'import sys; print(f"{float(sys.argv[1]):.2f}")' "${TARGET_M}")"
if [[ -n "${G1_NUM7_MAX_DURATION_S:-}" ]]; then
  MAX_DURATION_S="${G1_NUM7_MAX_DURATION_S}"
else
  # Default safety timeout: walking time + startup/stop margin.  This keeps
  # the old 1.00 m default at 7.0 s while allowing longer missions such as
  # 5.00 m without needing another manual environment variable.
  MAX_DURATION_S="$(python3 -c 'import sys; target=float(sys.argv[1]); print(f"{max(7.0, target / 0.5 + 5.0):.1f}")' "${TARGET_M}")"
fi
MAX_DURATION_S="$(python3 -c 'import sys; print(f"{float(sys.argv[1]):.1f}")' "${MAX_DURATION_S}")"
STOP_MARGIN_M="${G1_NUM7_STOP_MARGIN_M:-0.0}"
DISTANCE_SCALE="${G1_NUM7_DISTANCE_SCALE:-0.60}"
START_TIMEOUT_S="${G1_NUM7_START_TIMEOUT_S:-2.0}"
SETTLE_S="${G1_NUM7_SETTLE_S:-0.5}"
AUDIT_UDP_ENABLED="${G1_AUDIT_UDP_ENABLED:-true}"

OUTPUT_ENABLED="${G1_NUM7_COMMAND_OUTPUT_ENABLED:-0}"
if [[ "${OUTPUT_ENABLED}" != "1" ]]; then
  echo "REFUSING TO START: robot command output is disabled." >&2
  echo "Set G1_NUM7_COMMAND_OUTPUT_ENABLED=1 to explicitly allow motion." >&2
  echo "Robot must be suspended and an E-stop must be within reach." >&2
  exit 3
fi

if [[ "${ROBOT_INTERFACE}" == "lo" ]]; then
  echo "REFUSING TO START: G1_ROBOT_INTERFACE=lo is simulation-only." >&2
  exit 4
fi
if ! ip link show dev "${ROBOT_INTERFACE}" >/dev/null 2>&1; then
  echo "REFUSING TO START: interface not found: ${ROBOT_INTERFACE}" >&2
  exit 4
fi
if [[ ! -f "${RL_SAR_ROOT}/cmake_build/bin/rl_real_g1" ]]; then
  echo "REFUSING TO START: controller binary not found." >&2
  echo "Build it with: cd ${RL_SAR_ROOT} && cmake --build cmake_build -j4 --target rl_real_g1" >&2
  exit 5
fi
if pgrep -f '(^|/)(rl_real_g1|g1_ctrl)([[:space:]]|$)' >/dev/null; then
  echo "Refusing: a robot motion controller is already running." >&2
  exit 2
fi
if pgrep -f 'run_realsense_ros2.py' >/dev/null; then
  echo "Refusing: a vision node is already running." >&2
  exit 2
fi

CAMERA_PID=""
VISION_PID=""

cleanup() {
  set +e
  if command -v python3 >/dev/null 2>&1; then
    python3 - <<'PY' 2>/dev/null || true
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for _ in range(5):
    s.sendto(b"0.000000 0.000000 0.000000 1\n", ("127.0.0.1", 15001))
PY
  fi
  if [[ -n "${VISION_PID}" ]] && kill -0 "${VISION_PID}" 2>/dev/null; then
    kill -INT "${VISION_PID}" 2>/dev/null || true
    for _ in $(seq 1 10); do
      kill -0 "${VISION_PID}" 2>/dev/null || break
      sleep 0.5
    done
    kill -9 "${VISION_PID}" 2>/dev/null || true
  fi
  if [[ -n "${CAMERA_PID}" ]] && kill -0 "${CAMERA_PID}" 2>/dev/null; then
    kill -INT "${CAMERA_PID}" 2>/dev/null || true
    for _ in $(seq 1 10); do
      kill -0 "${CAMERA_PID}" 2>/dev/null || break
      sleep 0.5
    done
    kill -9 "${CAMERA_PID}" 2>/dev/null || true
  fi
  # The ros2 launch wrapper does not always forward SIGINT to the RealSense
  # node. Make sure the camera stream is stopped before the next run.
  pkill -INT -f 'realsense2_camera_node' 2>/dev/null || true
  pkill -INT -f 'ros2 launch realsense2_camera' 2>/dev/null || true
  sleep 1
  pkill -9 -f 'realsense2_camera_node' 2>/dev/null || true
  pkill -9 -f 'ros2 launch realsense2_camera' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "************************************************************"
echo "* SKILL 7 REAL-ROBOT ACTIVE VISION WALK (Num7)"
echo "* Robot MUST be suspended. E-stop MUST be within reach."
echo "* interface=${ROBOT_INTERFACE} target_m=${TARGET_M} max_duration_s=${MAX_DURATION_S}"
echo "************************************************************"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${REALSENSE_SETUP}" ]]; then
  echo "RealSense workspace setup not found: ${REALSENSE_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
source "${REALSENSE_SETUP}"
set -u

CAMERA_READY=0
for CAMERA_ATTEMPT in 1 2 3; do
  echo "Starting RealSense camera (attempt ${CAMERA_ATTEMPT}/3)..."
  ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    rgb_camera.color_profile:=640,480,15 \
    enable_depth:=true \
    depth_module.depth_profile:=640,480,15 \
    enable_sync:=true \
    align_depth.enable:=true \
    enable_gyro:=false \
    enable_accel:=false \
    diagnostics_period:=1.0 \
    > /tmp/num7_camera.log 2>&1 &
  CAMERA_PID=$!

  topics_ready=0
  for _ in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -qx '/camera/camera/color/image_raw' &&
       ros2 topic list 2>/dev/null | grep -qx '/camera/camera/aligned_depth_to_color/image_raw'; then
      topics_ready=1
      break
    fi
    if ! kill -0 "${CAMERA_PID}" 2>/dev/null; then
      echo "RealSense camera process exited during startup." >&2
      break
    fi
    sleep 1
  done

  if [[ "${topics_ready}" -eq 1 ]]; then
    if timeout 8 ros2 topic echo /camera/camera/color/image_raw --once >/dev/null 2>&1 &&
       timeout 8 ros2 topic echo /camera/camera/aligned_depth_to_color/image_raw --once >/dev/null 2>&1; then
      CAMERA_READY=1
      break
    fi
    echo "Camera topics exist but did not deliver frames; retrying camera..." >&2
  else
    echo "Timed out waiting for RealSense topics; retrying camera..." >&2
  fi

  if [[ -n "${CAMERA_PID}" ]] && kill -0 "${CAMERA_PID}" 2>/dev/null; then
    kill -INT "${CAMERA_PID}" 2>/dev/null || true
    for _ in $(seq 1 8); do
      kill -0 "${CAMERA_PID}" 2>/dev/null || break
      sleep 0.5
    done
    kill -9 "${CAMERA_PID}" 2>/dev/null || true
  fi
  pkill -INT -f 'realsense2_camera_node' 2>/dev/null || true
  pkill -INT -f 'ros2 launch realsense2_camera' 2>/dev/null || true
  sleep 2
  pkill -9 -f 'realsense2_camera_node' 2>/dev/null || true
  pkill -9 -f 'ros2 launch realsense2_camera' 2>/dev/null || true
  CAMERA_PID=""
done

if [[ "${CAMERA_READY}" -ne 1 ]]; then
  echo "RealSense camera failed after 3 attempts." >&2
  exit 1
fi

echo "Starting Num7 vision node (command output ENABLED)."
cd "${PROJECT_ROOT}"
python3 vision/scripts/run_realsense_ros2.py \
  --ros-args \
  -p mission:=walk0p5m \
  -p num7_target_m:="${TARGET_M}" \
  -p num7_max_duration_s:="${MAX_DURATION_S}" \
  -p num7_distance_scale:="${DISTANCE_SCALE}" \
  -p cruise_speed_mps:=1.00 \
  -p command_output_enabled:=true \
  -p audit_udp_enabled:="${AUDIT_UDP_ENABLED}" \
  > /tmp/num7_vision.log 2>&1 &
VISION_PID=$!

# Give the ROS node time to bind 15001/15002 and subscribe to camera topics.
sleep 2
if ! kill -0 "${VISION_PID}" 2>/dev/null; then
  echo "Vision node exited during startup." >&2
  wait "${VISION_PID}"
  exit 1
fi

echo
echo "Controller is starting in THIS terminal."
echo "KEYBOARD: 0 -> 1 -> 7"
echo "GAMEPAD:  A -> RB+DPadUp -> LB+DPadLeft"
echo "  P / LB+X = Passive stop, 9 / B = GetDown"
echo "  DO NOT press 5 / LB+DPadUp: that is Running (Skill 5)."
echo
# Skill 7 must enter state 1 (locomotion), not the running gait.  If the
# caller exported G1_AUTO_RUNNING=1, GetUp would jump to Running instead of
# waiting for key 1, which makes the robot walk/run unexpectedly.
unset G1_AUTO_RUNNING || true
cd "${RL_SAR_ROOT}"
G1_NUM7_TARGET_M="${TARGET_M}" \
G1_NUM7_MAX_DURATION_S="${MAX_DURATION_S}" \
G1_NUM7_STOP_MARGIN_M="${STOP_MARGIN_M}" \
G1_NUM7_DISTANCE_SCALE="${DISTANCE_SCALE}" \
G1_NUM7_START_TIMEOUT_S="${START_TIMEOUT_S}" \
G1_NUM7_SETTLE_S="${SETTLE_S}" \
G1_VISION_MAX_VX=1.00 \
G1_VISION_MAX_WZ=0.25 \
G1_VISION_STATUS_PORT=15002 \
  ./cmake_build/bin/rl_real_g1 "${ROBOT_INTERFACE}"
