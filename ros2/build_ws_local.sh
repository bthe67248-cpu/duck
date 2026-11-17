#!/usr/bin/env bash
# build_ws_local.sh -- copy this checkout's ROS packages (as they are, no model regeneration) into ~/duck_ws,
# build, and run the duck_nav unit tests. setup_nav.sh is the full pipeline that also refreshes models/policies.
set -eo pipefail
P=${DUCK_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
source /opt/ros/jazzy/setup.bash
rm -rf ~/duck_ws/src/duck_nav ~/duck_ws/src/duck_description
mkdir -p ~/duck_ws/src && cp -r "$P/ros2/duck_nav" "$P/ros2/duck_description" ~/duck_ws/src/
cd ~/duck_ws && colcon build --symlink-install 2>&1 | tail -3
cd ~/duck_ws/src/duck_nav && python3 -m pytest -q test 2>&1 | tail -2
