#!/usr/bin/env bash
# setup_ws.sh -- build the duck ROS 2 workspace. Run as your normal WSL user (not root):
#   wsl -d Ubuntu-24.04 -- bash /mnt/c/dev/duckling_sim/ros2/setup_ws.sh
set -eo pipefail
source /opt/ros/jazzy/setup.bash
python3 -m pip install --user --break-system-packages -q mujoco     # MuJoCo for the sim node
mkdir -p ~/duck_ws/src
rm -rf ~/duck_ws/src/duck_description
cp -r /mnt/c/dev/duckling_sim/ros2/duck_description ~/duck_ws/src/
cd ~/duck_ws
colcon build --symlink-install
grep -q "ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
grep -q "duck_ws/install/setup.bash" ~/.bashrc || echo "source ~/duck_ws/install/setup.bash" >> ~/.bashrc
echo "DUCK_WS_BUILD_DONE"
