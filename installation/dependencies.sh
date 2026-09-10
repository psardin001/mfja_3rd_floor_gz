#!/usr/bin/env bash
# Ubuntu 24.04 build tools and ROS 2 Jazzy runtime.
set -euo pipefail
source /etc/os-release
[[ "$ID:$VERSION_ID" == ubuntu:24.04 ]] || { echo 'Ubuntu 24.04 required.' >&2; exit 1; }
apt-get update
apt-get install -y ca-certificates curl software-properties-common
add-apt-repository -y universe
if [[ ! -f /etc/apt/sources.list.d/ros2.sources ]]; then
  ros_apt_version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')
  ros_apt_deb=$(mktemp --suffix=.deb)
  trap 'rm -f "$ros_apt_deb"' EXIT
  curl -fsSL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.noble_all.deb" -o "$ros_apt_deb"
  dpkg -i "$ros_apt_deb"
fi
apt-get update
apt-get install -y --no-install-recommends \
  build-essential cmake git pkg-config \
  python3-dev python3-venv python3-numpy python3-scipy python3-yaml \
  libboost-all-dev libeigen3-dev libassimp-dev libccd-dev liboctomap-dev \
  libqhull-dev libtinyxml2-dev liburdfdom-dev libgraphviz-dev \
  ros-dev-tools ros-jazzy-ros-base ros-jazzy-jrl-cmakemodules \
  ros-jazzy-eigenpy \
  ros-jazzy-control-msgs ros-jazzy-xacro ros-jazzy-robot-state-publisher
