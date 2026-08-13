#!/usr/bin/env bash
set -euo pipefail

ROS_APT_SOURCE_VERSION="1.2.0"
ROS_APT_SOURCE_SHA256="767884cf4ed03116b9d64438930a832ed854147ae435279a7924dfdf60f94433"
ROS_APT_SOURCE_DEB="/tmp/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb"
INSTALL_LOG="/home/unitree/ros_humble_install.log"

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run this script with sudo." >&2
    exit 1
fi

exec > >(tee -a "${INSTALL_LOG}") 2>&1

source /etc/os-release
if [[ "${VERSION_CODENAME:-}" != "jammy" ]]; then
    echo "Expected Ubuntu Jammy, found ${VERSION_CODENAME:-unknown}." >&2
    exit 2
fi
if [[ "$(dpkg --print-architecture)" != "arm64" ]]; then
    echo "Expected arm64, found $(dpkg --print-architecture)." >&2
    exit 2
fi

echo "Installing the official ROS 2 apt source package..."
curl --fail --location --retry 3 \
    --output "${ROS_APT_SOURCE_DEB}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb"
echo "${ROS_APT_SOURCE_SHA256}  ${ROS_APT_SOURCE_DEB}" | sha256sum --check --strict
dpkg --install "${ROS_APT_SOURCE_DEB}"
rm -f "${ROS_APT_SOURCE_DEB}"

echo "Refreshing package indexes without upgrading JetPack or Ubuntu..."
apt-get update

echo "Installing the minimal ROS 2 Humble perception toolchain..."
DEBIAN_FRONTEND=noninteractive apt-get install --yes --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-message-filters \
    ros-humble-tf2-ros \
    ros-humble-diagnostic-updater \
    ros-humble-rosbag2 \
    ros-humble-rmw-fastrtps-cpp \
    python3-colcon-common-extensions \
    python3-rosdep

source /opt/ros/humble/setup.bash
ros2 --help >/dev/null
echo "ROS_DISTRO=${ROS_DISTRO}"
echo "ROS 2 Humble minimal installation completed."

chown unitree:unitree "${INSTALL_LOG}"
