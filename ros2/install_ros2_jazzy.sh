#!/usr/bin/env bash
# install_ros2_jazzy.sh -- ROS 2 Jazzy on Ubuntu 24.04 (WSL2). Run as root:
#   wsl -d Ubuntu-24.04 -u root -- bash /mnt/c/dev/duckling_sim/ros2/install_ros2_jazzy.sh
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y software-properties-common curl locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe

# Official ROS 2 apt source package (sets up the key + sources list)
VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.$(. /etc/os-release && echo ${UBUNTU_CODENAME})_all.deb"
dpkg -i /tmp/ros2-apt-source.deb
apt-get update

apt-get install -y ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
  python3-pip python3-venv
echo "ROS2_INSTALL_DONE"
