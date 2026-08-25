#!/usr/bin/env bash
# Running-backed Num7 visual mission for the real robot.
#
# This entry keeps the validated Skill7 camera/ROS lifecycle, but selects the
# additive RLFSMStateRLRunningVision110m adapter.  The operator sequence is
# 0 -> 1 -> 7.  Keys 5 and 6 are intentionally not used by this entry.
# Command output is opt-in and the controller binary must have been built with
# the running-vision adapter; this script never copies or builds a binary.
set -euo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
REALSENSE_SETUP="${REALSENSE_SETUP:-/home/unitree/g1_realsense_ws/install/local_setup.bash}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PROJECT_ROOT="${G1_RUNNING_VISION_REPO:-${DEFAULT_PROJECT_ROOT}}"
RL_SAR_ROOT="${PROJECT_ROOT}/rl_sar"
ROS_PYTHON="${ROS_PYTHON:-${PROJECT_ROOT}/vision/.venv_robot/bin/python3}"
ROBOT_INTERFACE="${G1_ROBOT_INTERFACE:-enP8p1s0}"
TARGET_M="${G1_RUNNING_VISION_TARGET_M:-110.0}"
CRUISE_SPEED="${G1_RUNNING_VISION_CRUISE_SPEED_MPS:-3.0}"
MAX_DURATION_S="${G1_RUNNING_VISION_MAX_DURATION_S:-260.0}"
MAX_VX="${G1_VISION_MAX_VX:-${CRUISE_SPEED}}"
MAX_WZ="${G1_VISION_MAX_WZ:-0.35}"
COMPETITION_NO_OBSTACLE_STOP="${G1_RUNNING_VISION_NO_OBSTACLE_STOP:-1}"
COMPETITION_IMU_FORWARD_ONLY="${G1_RUNNING_VISION_IMU_FORWARD_ONLY:-1}"
AUDIT_UDP_ENABLED="${G1_AUDIT_UDP_ENABLED:-true}"
# USB2-compatible D435i profiles. 640x480@6 is supported by both RGB and
# depth on this camera and keeps the aligned streams at the same resolution.
CAMERA_COLOR_PROFILE="${G1_CAMERA_COLOR_PROFILE:-640,480,6}"
CAMERA_DEPTH_PROFILE="${G1_CAMERA_DEPTH_PROFILE:-640,480,6}"

if [[ ! -x "${ROS_PYTHON}" ]]; then
  echo "REFUSING TO START: isolated ROS Python is missing: ${ROS_PYTHON}" >&2
  exit 1
fi
TARGET_M="$("${ROS_PYTHON}" -c 'import sys; print(f"{min(max(float(sys.argv[1]), 1.0), 200.0):.2f}")' "${TARGET_M}")"
CRUISE_SPEED="$("${ROS_PYTHON}" -c 'import sys; print(f"{min(max(float(sys.argv[1]), 0.2), 3.0):.3f}")' "${CRUISE_SPEED}")"
MAX_VX="$("${ROS_PYTHON}" -c 'import sys; print(f"{min(max(float(sys.argv[1]), 0.2), 3.0):.3f}")' "${MAX_VX}")"
MAX_DURATION_S="$("${ROS_PYTHON}" -c 'import sys; print(f"{min(max(float(sys.argv[1]), 1.0), 1200.0):.1f}")' "${MAX_DURATION_S}")"

case "${COMPETITION_NO_OBSTACLE_STOP,,}" in
  1|true|yes|on)
    COMPETITION_NO_OBSTACLE_STOP=true
    ;;
  0|false|no|off)
    COMPETITION_NO_OBSTACLE_STOP=false
    ;;
  *)
    echo "REFUSING TO START: G1_RUNNING_VISION_NO_OBSTACLE_STOP must be 0/1 or true/false." >&2
    exit 4
    ;;
esac

case "${COMPETITION_IMU_FORWARD_ONLY,,}" in
  1|true|yes|on)
    COMPETITION_IMU_FORWARD_ONLY=true
    ;;
  0|false|no|off)
    COMPETITION_IMU_FORWARD_ONLY=false
    ;;
  *)
    echo "REFUSING TO START: G1_RUNNING_VISION_IMU_FORWARD_ONLY must be 0/1 or true/false." >&2
    exit 4
    ;;
esac

if [[ "${G1_RUNNING_VISION_COMMAND_OUTPUT_ENABLED:-0}" != "1" ]]; then
  echo "REFUSING TO START: running-vision command output is disabled." >&2
  echo "Set G1_RUNNING_VISION_COMMAND_OUTPUT_ENABLED=1 only for a separately" >&2
  echo "approved, suspended-robot test with an E-stop within reach." >&2
  exit 3
fi
if [[ "${ROBOT_INTERFACE}" == "lo" ]]; then
  echo "REFUSING TO START: lo is simulation-only." >&2
  exit 4
fi
if ! ip link show dev "${ROBOT_INTERFACE}" >/dev/null 2>&1; then
  echo "REFUSING TO START: interface not found: ${ROBOT_INTERFACE}" >&2
  exit 4
fi
if [[ ! -x "${RL_SAR_ROOT}/cmake_build_running/bin/rl_real_g1_running" ]]; then
  echo "REFUSING TO START: running-vision controller binary not found." >&2
  echo "Build the ARM64 target with the additive adapter before deployment." >&2
  exit 5
fi
if [[ ! -f "${RL_SAR_ROOT}/policy/g1/running/config.yaml" ||
      ! -f "${RL_SAR_ROOT}/policy/g1/running/policy.pt" ]]; then
  echo "REFUSING TO START: running policy files are incomplete." >&2
  exit 5
fi
if ! grep -Eq '^[[:space:]]*num_observations:[[:space:]]*96([[:space:]]|$)' \
    "${RL_SAR_ROOT}/policy/g1/running/config.yaml"; then
  echo "REFUSING TO START: running config is not the tested 96D interface." >&2
  exit 5
fi
if pgrep -f '(^|/)(rl_real_g1|rl_real_g1_running|g1_ctrl)([[:space:]]|$)' >/dev/null; then
  echo "Refusing: a robot motion controller is already running." >&2
  exit 2
fi
if pgrep -f 'run_realsense_ros2.py' >/dev/null; then
  echo "Refusing: a vision node is already running." >&2
  exit 2
fi
if pgrep -f '/[r]ealsense2_camera/[r]ealsense2_camera_node([[:space:]]|$)' >/dev/null ||
   pgrep -f '[r]ealsense-viewer([[:space:]]|$)' >/dev/null; then
  echo "Refusing: RealSense is already in use by a camera node or viewer." >&2
  exit 2
fi

CAMERA_PID=""
VISION_PID=""

stop_background_process() {
  local pid="$1"
  local signal

  [[ -n "${pid}" ]] || return 0
  for signal in INT TERM KILL; do
    kill -0 "${pid}" 2>/dev/null || break
    kill -"${signal}" "${pid}" 2>/dev/null || true
    for _ in $(seq 1 10); do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.1
    done
  done
  wait "${pid}" 2>/dev/null || true
}

camera_topics_ready() {
  local topics

  # The ROS daemon can briefly lag a newly started RealSense node on G1-NX.
  # Check both the normal graph cache and direct discovery before declaring a
  # startup failure.
  topics="$(ros2 topic list 2>/dev/null || true)"
  if grep -Fqx '/camera/camera/color/image_raw' <<< "${topics}" &&
     grep -Fqx '/camera/camera/aligned_depth_to_color/image_raw' <<< "${topics}"; then
    return 0
  fi

  topics="$(ros2 topic list --no-daemon --spin-time 1 2>/dev/null || true)"
  grep -Fqx '/camera/camera/color/image_raw' <<< "${topics}" &&
    grep -Fqx '/camera/camera/aligned_depth_to_color/image_raw' <<< "${topics}"
}

cleanup() {
  set +e
  "${ROS_PYTHON}" - <<'PY' 2>/dev/null || true
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for _ in range(5):
    s.sendto(b"0.000000 0.000000 0.000000 1\n", ("127.0.0.1", 15001))
PY
  stop_background_process "${VISION_PID}"
  # ros2 launch does not always forward SIGINT promptly to the camera node.
  # Stop the child explicitly before reaping the launch process.
  pkill -INT -f '/[r]ealsense2_camera/[r]ealsense2_camera_node([[:space:]]|$)' \
    2>/dev/null || true
  stop_background_process "${CAMERA_PID}"
}
trap cleanup EXIT INT TERM

if [[ ! -f "${ROS_SETUP}" || ! -f "${REALSENSE_SETUP}" ]]; then
  echo "ROS/RealSense setup file is missing." >&2
  exit 1
fi
set +u
source "${ROS_SETUP}"
source "${REALSENSE_SETUP}"
set -u

echo "Starting RealSense (USB2 profile color=${CAMERA_COLOR_PROFILE}, depth=${CAMERA_DEPTH_PROFILE}, aligned depth)..."
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  rgb_camera.color_profile:="${CAMERA_COLOR_PROFILE}" \
  enable_depth:=true \
  depth_module.depth_profile:="${CAMERA_DEPTH_PROFILE}" \
  enable_sync:=true \
  align_depth.enable:=true \
  enable_gyro:=false \
  enable_accel:=false \
  diagnostics_period:=1.0 > /tmp/running_vision_camera.log 2>&1 &
CAMERA_PID=$!
for _ in $(seq 1 15); do
  if camera_topics_ready; then
    break
  fi
  if ! kill -0 "${CAMERA_PID}" 2>/dev/null; then
    echo "RealSense camera exited during startup." >&2
    exit 1
  fi
  sleep 1
done
if ! camera_topics_ready; then
  echo "Timed out waiting for RealSense topics." >&2
  exit 1
fi

cd "${PROJECT_ROOT}"
"${ROS_PYTHON}" vision/scripts/run_realsense_ros2.py \
  --ros-args \
  -p mission:=runningvision110m \
  -p running_vision_target_m:="${TARGET_M}" \
  -p running_vision_max_duration_s:="${MAX_DURATION_S}" \
  -p cruise_speed_mps:="${CRUISE_SPEED}" \
  -p competition_no_obstacle_stop:="${COMPETITION_NO_OBSTACLE_STOP}" \
  -p competition_imu_forward_only:="${COMPETITION_IMU_FORWARD_ONLY}" \
  -p command_output_enabled:=true \
  -p audit_udp_enabled:="${AUDIT_UDP_ENABLED}" \
  > /tmp/running_vision_ros2.log 2>&1 &
VISION_PID=$!
sleep 2
if ! kill -0 "${VISION_PID}" 2>/dev/null; then
  echo "Running-vision ROS node exited during startup." >&2
  wait "${VISION_PID}"
  exit 1
fi

echo "Controller keyboard sequence: 0 -> 1 -> 7"
echo "Num7 uses the 96D running policy and a 110 m Python/FSM gate."
echo "NoMachine viewer (read-only): source ROS + RealSense setup, then"
echo "${ROS_PYTHON} vision/scripts/view_num7_camera.py"
echo "Do not press 5 or 6 in this entry."
unset G1_AUTO_RUNNING || true
cd "${RL_SAR_ROOT}"
G1_RUNNING_VISION_TARGET_M="${TARGET_M}" \
G1_RUNNING_VISION_MAX_DURATION_S="${MAX_DURATION_S}" \
G1_VISION_MAX_VX="${MAX_VX}" \
G1_VISION_MAX_WZ="${MAX_WZ}" \
G1_VISION_TIMEOUT_MS="${G1_RUNNING_VISION_TIMEOUT_MS:-300}" \
G1_VISION_STATUS_PORT=15002 \
  ./cmake_build_running/bin/rl_real_g1_running "${ROBOT_INTERFACE}"
