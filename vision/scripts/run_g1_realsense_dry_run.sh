#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
REALSENSE_SETUP="${REALSENSE_SETUP:-/home/unitree/g1_realsense_ws/install/local_setup.bash}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CAMERA_PID=""
AUDIT_UDP_ENABLED="${G1_AUDIT_UDP_ENABLED:-false}"
AUDIT_MISSION="${G1_AUDIT_MISSION:-walk0p5m}"
TARGET_M="${G1_NUM7_TARGET_M:-1.00}"
DISTANCE_SCALE="${G1_NUM7_DISTANCE_SCALE:-0.80}"
CRUISE_SPEED_MPS="${G1_VISION_CRUISE_SPEED_MPS:-0.50}"

cleanup() {
  if [[ -n "${CAMERA_PID}" ]] && kill -0 "${CAMERA_PID}" 2>/dev/null; then
    kill -INT "${CAMERA_PID}" 2>/dev/null || true
    wait "${CAMERA_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${REALSENSE_SETUP}" ]]; then
  echo "RealSense workspace setup not found: ${REALSENSE_SETUP}" >&2
  exit 1
fi
if pgrep -f '(^|/)(rl_real_g1|g1_ctrl)([[:space:]]|$)' >/dev/null; then
  echo "Refusing dry-run: a robot motion controller is already running." >&2
  exit 2
fi

set +u
source "${ROS_SETUP}"
source "${REALSENSE_SETUP}"
set -u

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  rgb_camera.color_profile:=640,480,15 \
  enable_depth:=true \
  depth_module.depth_profile:=640,480,15 \
  enable_sync:=true \
  align_depth.enable:=true \
  enable_gyro:=false \
  enable_accel:=false \
  diagnostics_period:=1.0 &
CAMERA_PID=$!

for _ in $(seq 1 20); do
  if ros2 topic list 2>/dev/null | grep -qx '/camera/camera/color/image_raw'; then
    break
  fi
  if ! kill -0 "${CAMERA_PID}" 2>/dev/null; then
    echo "RealSense camera process exited during startup." >&2
    wait "${CAMERA_PID}"
    exit 1
  fi
  sleep 1
done

if ! ros2 topic list 2>/dev/null | grep -qx '/camera/camera/color/image_raw' ||
   ! ros2 topic list 2>/dev/null | grep -qx '/camera/camera/aligned_depth_to_color/image_raw'; then
  echo "Timed out waiting for RealSense color/aligned-depth topics." >&2
  exit 1
fi

if ! timeout 8 ros2 topic echo \
     /camera/camera/color/image_raw --once >/dev/null 2>&1 ||
   ! timeout 8 ros2 topic echo \
     /camera/camera/aligned_depth_to_color/image_raw --once >/dev/null 2>&1; then
  echo "RealSense topics exist but did not deliver color/depth frames." >&2
  exit 1
fi

echo "Starting vision in enforced DRY-RUN mode (robot command output disabled)."
cd "${PROJECT_ROOT}"
python3 vision/scripts/run_realsense_ros2.py \
  --ros-args \
  -p mission:="${AUDIT_MISSION}" \
  -p num7_target_m:="${TARGET_M}" \
  -p num7_distance_scale:="${DISTANCE_SCALE}" \
  -p cruise_speed_mps:="${CRUISE_SPEED_MPS}" \
  -p command_output_enabled:=false \
  -p audit_udp_enabled:="${AUDIT_UDP_ENABLED}"
