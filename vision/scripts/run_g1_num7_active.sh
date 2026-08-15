#!/usr/bin/env bash
# Skill 7 (Num7) real-robot active vision walk.
#
# SECURITY: this script refuses to start unless you explicitly opt in by
# setting G1_NUM7_COMMAND_OUTPUT_ENABLED=1. It never presses 0/1/7 and never
# starts rl_real_g1; you must start the controller yourself after the vision
# node is up, and the robot must be suspended with a physical E-stop nearby.
set -euo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
REALSENSE_SETUP="${REALSENSE_SETUP:-/home/unitree/g1_realsense_ws/install/local_setup.bash}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
RL_SAR_ROOT="${G1_RUNNING_ROOT:-${PROJECT_ROOT}}/rl_sar"
RELEASE_MARKER="${G1_NUM7_RELEASE_MARKER:-${PROJECT_ROOT}/vision/output/num7_headless_smoke/HARDWARE_RELEASE_READY}"
CAMERA_PID=""

# Opt-in gate: never enable robot command output by default.
OUTPUT_ENABLED="${G1_NUM7_COMMAND_OUTPUT_ENABLED:-0}"
if [[ "${OUTPUT_ENABLED}" != "1" ]]; then
  echo "REFUSING TO START: robot command output is disabled." >&2
  echo "Set G1_NUM7_COMMAND_OUTPUT_ENABLED=1 to explicitly allow motion." >&2
  echo "This is a live robot: it must be suspended and an E-stop must be" >&2
  echo "within reach before you run the controller." >&2
  exit 3
fi

if [[ ! -f "${RELEASE_MARKER}" ]] ||
   ! grep -qx 'NUM7_HARDWARE_RELEASE_READY' "${RELEASE_MARKER}"; then
  echo "REFUSING TO START: Num7 measured-qpos simulation gate has not passed." >&2
  echo "Run: bash vision/scripts/num7_headless_smoke.sh" >&2
  echo "Expected release marker: ${RELEASE_MARKER}" >&2
  exit 5
fi

marker_value() {
  awk -F= -v key="$1" '$1 == key {print $2}' "${RELEASE_MARKER}"
}
verify_release_hash() {
  local key="$1"
  local file="$2"
  local expected actual
  expected="$(marker_value "${key}")"
  if [[ -z "${expected}" ]] || [[ ! -f "${file}" ]]; then
    echo "REFUSING TO START: release marker lacks ${key} or file is missing." >&2
    exit 5
  fi
  actual="$(sha256sum "${file}" | awk '{print $1}')"
  if [[ "${actual}" != "${expected}" ]]; then
    echo "REFUSING TO START: ${key} differs from the simulated release." >&2
    exit 5
  fi
}
verify_release_hash \
  policy_sha256 \
  "${RL_SAR_ROOT}/policy/g1/robomimic/locomotion/policy_29dof.pt"
verify_release_hash \
  fsm_sha256 \
  "${RL_SAR_ROOT}/src/rl_sar/fsm_robot/fsm_g1.hpp"
verify_release_hash \
  receiver_sha256 \
  "${RL_SAR_ROOT}/src/rl_sar/include/vision_udp_command.hpp"

ROBOT_INTERFACE="${G1_ROBOT_INTERFACE:-}"
if [[ -z "${ROBOT_INTERFACE}" ]]; then
  echo "REFUSING TO START: set G1_ROBOT_INTERFACE to the verified" >&2
  echo "physical DDS interface on the Jetson." >&2
  exit 4
fi
if [[ "${ROBOT_INTERFACE}" == "lo" ]]; then
  echo "REFUSING TO START: G1_ROBOT_INTERFACE=lo is simulation-only." >&2
  exit 4
fi
if ! ip link show dev "${ROBOT_INTERFACE}" >/dev/null 2>&1; then
  echo "REFUSING TO START: interface not found: ${ROBOT_INTERFACE}" >&2
  exit 4
fi

TARGET_M="${G1_NUM7_TARGET_M:-1.00}"
MAX_DURATION_S="${G1_NUM7_MAX_DURATION_S:-7.0}"
STOP_MARGIN_M="${G1_NUM7_STOP_MARGIN_M:-0.0}"
DISTANCE_SCALE="${G1_NUM7_DISTANCE_SCALE:-0.60}"
START_TIMEOUT_S="${G1_NUM7_START_TIMEOUT_S:-2.0}"
SETTLE_S="${G1_NUM7_SETTLE_S:-0.5}"
AUDIT_UDP_ENABLED="${G1_AUDIT_UDP_ENABLED:-true}"

cleanup() {
  # Always send repeated zero commands on exit so a leftover UDP receiver
  # never keeps a stale velocity.
  if command -v python3 >/dev/null 2>&1; then
    python3 - <<'PY' 2>/dev/null || true
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for _ in range(5):
    s.sendto(b"0.000000 0.000000 0.000000 1\n", ("127.0.0.1", 15001))
PY
  fi
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
  echo "Refusing: a robot motion controller is already running." >&2
  exit 2
fi

echo "************************************************************"
echo "* SKILL 7 REAL-ROBOT ACTIVE VISION WALK (Num7)" >&2
echo "* Robot MUST be suspended. E-stop MUST be within reach." >&2
echo "* target_m=${TARGET_M}" >&2
echo "************************************************************" >&2

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


# Topic discovery is not enough: require actual color and aligned-depth frames
# before enabling the Python command node. This catches USB/MIPI streamer
# failures while the robot controller is still stopped.
if ! timeout 8 ros2 topic echo \
     /camera/camera/color/image_raw --once >/dev/null 2>&1 ||
   ! timeout 8 ros2 topic echo \
     /camera/camera/aligned_depth_to_color/image_raw --once >/dev/null 2>&1; then
  echo "RealSense topics exist but did not deliver color/depth frames." >&2
  exit 1
fi

echo "Starting Num7 vision node (command output ENABLED for this mission)."
echo
echo "IMPORTANT: the environment variables below must be EXACTLY the same in"
echo "the controller terminal. This terminal's environment does NOT propagate"
echo "to the other terminal - copy these values into the controller command."
echo
echo "In a SECOND terminal, start rl_real_g1 with the SAME values:"
echo "--------------------------------------------------------------------"
cat <<CMD
cd "${RL_SAR_ROOT}"
G1_NUM7_TARGET_M="${TARGET_M}" \\
G1_NUM7_MAX_DURATION_S="${MAX_DURATION_S}" \\
G1_NUM7_STOP_MARGIN_M="${STOP_MARGIN_M}" \\
G1_NUM7_DISTANCE_SCALE="${DISTANCE_SCALE}" \\
G1_NUM7_START_TIMEOUT_S="${START_TIMEOUT_S}" \\
G1_NUM7_SETTLE_S="${SETTLE_S}" \\
G1_VISION_MAX_VX=0.50 \\
G1_VISION_MAX_WZ=0.25 \\
G1_VISION_STATUS_PORT=15002 \\
    ./cmake_build/bin/rl_real_g1 "${ROBOT_INTERFACE}"
CMD
echo "--------------------------------------------------------------------"
echo "Then press 0 -> 1 -> 7 in the controller terminal."
echo "Note: G1_NUM7_STOP_MARGIN_M is used only by the C++ FSM. Python has no"
echo "equivalent margin; the Python gate stops at the same G1_NUM7_TARGET_M,"
echo "so both ends must share G1_NUM7_TARGET_M."
echo
cd "${PROJECT_ROOT}"
python3 vision/scripts/run_realsense_ros2.py \
  --ros-args \
  -p mission:=walk0p5m \
  -p num7_target_m:="${TARGET_M}" \
  -p num7_distance_scale:="${DISTANCE_SCALE}" \
  -p cruise_speed_mps:=0.50 \
  -p command_output_enabled:=true \
  -p audit_udp_enabled:="${AUDIT_UDP_ENABLED}"
