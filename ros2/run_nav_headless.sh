#!/usr/bin/env bash
# run_nav_headless.sh [extra launch args] -- run the whole ROS 2 navigation demo without RViz and wait
# for the referee's summary.   wsl -d Ubuntu-24.04 -- bash /mnt/c/dev/duckling_sim/ros2/run_nav_headless.sh
source /opt/ros/jazzy/setup.bash; source ~/duck_ws/install/setup.bash
OUT=${OUT:-$HOME/duck_ws/run}; rm -rf "$OUT"
pkill -f "duck_nav/lib" 2>/dev/null; pkill -f robot_state_publisher 2>/dev/null; sleep 1
ros2 launch duck_nav duck_nav.launch.py rviz:=false out_dir:="$OUT" "$@" > /tmp/duck_nav.log 2>&1 &
for i in $(seq 1 ${WAIT:-420}); do
  [ -f "$OUT/summary.txt" ] && break
  grep -q "process has died" /tmp/duck_nav.log && { echo "A NODE DIED"; break; }
  sleep 5
done
sleep 2
pkill -INT -f "ros2 launch duck_nav"; sleep 2; pkill -f "duck_nav/lib"; pkill -f robot_state_publisher
grep -E "world_sim.*(sim [0-9]+ s|up:)|duck_nav\]|duck_slam\]|duck_yolo\]|died|Traceback|Error" /tmp/duck_nav.log | sed 's/\[INFO\] \[[0-9.]*\] //' | cut -c1-200 | tail -40
echo "=== summary"; cat "$OUT/summary.txt" 2>/dev/null || echo "no summary written"
mkdir -p /mnt/c/dev/duckling_sim/results/ros2_nav && cp "$OUT"/* /mnt/c/dev/duckling_sim/results/ros2_nav/ 2>/dev/null
