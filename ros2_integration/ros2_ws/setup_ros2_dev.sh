#!/usr/bin/env bash

set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
REPO_ROOT="${REPO_ROOT:-/workspaces/px4-python-sitl}"
ROS2_WS_ROOT="${ROS2_WS_ROOT:-${REPO_ROOT}/ros2_integration/ros2_ws}"
ROS2_SRC_ROOT="${ROS2_WS_ROOT}/src"

echo "[setup] Using ROS distro: ${ROS_DISTRO_NAME}"
echo "[setup] Using repo root: ${REPO_ROOT}"

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
  ffmpeg \
  git \
  libavcodec-dev \
  libavdevice-dev \
  libavfilter-dev \
  libavformat-dev \
  libavutil-dev \
  libboost-system-dev \
  libswresample-dev \
  libswscale-dev \
  libwebsocketpp-dev \
  pkg-config \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-$ROS_DISTRO-ament-cmake

python3 -m pip install --upgrade pip setuptools wheel build

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init
fi
rosdep update

source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

python3 -m pip install -e "${REPO_ROOT}" || python3 -m pip install "${REPO_ROOT}"
python3 -m pip install --upgrade aiohttp aiortc numpy pymavlink

rosdep install \
  --from-paths "${ROS2_SRC_ROOT}" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO_NAME}" \
  -r \
  -y

if [ "${1:-}" = "--build" ]; then
  colcon build --base-paths "${ROS2_SRC_ROOT}"
fi

echo "[setup] Done"
