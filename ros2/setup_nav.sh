#!/usr/bin/env bash
# setup_nav.sh -- build the duck_nav package (SLAM + YOLO + navigation). Run as your normal WSL user:
#   wsl -d Ubuntu-24.04 -- bash /mnt/c/dev/duckling_sim/ros2/setup_nav.sh
# One-time, as root, first:  wsl -d Ubuntu-24.04 -u root -- apt-get install -y ros-jazzy-vision-msgs python3-scipy python3-pil
set -eo pipefail
source /opt/ros/jazzy/setup.bash
python3 -m pip install --user --break-system-packages -q onnxruntime "numpy<2"
P=/mnt/c/dev/duckling_sim
PKG=$P/ros2/duck_nav
mkdir -p $PKG/models $PKG/mjcf/assets
cp $P/policies/velstand.onnx $P/policies/alpha_sitstand.onnx $P/policies/roulade.onnx $P/policies/alpha_ground_pick.onnx $P/yolo/duck_yolo.onnx $PKG/models/
cp $P/model/duck_v3_gc.xml $P/model/duck_v3_gc_world.xml $P/model/world.json $PKG/mjcf/
cp $P/model/assets/*.stl $PKG/mjcf/assets/
rm -rf ~/duck_ws/src/duck_nav ~/duck_ws/src/duck_description
cp -r $PKG $P/ros2/duck_description ~/duck_ws/src/
cd ~/duck_ws && colcon build --symlink-install 2>&1 | tail -3
echo "DUCK_NAV_BUILD_DONE"
