#!/usr/bin/env bash
# run_nav_headless.sh [sensors:=ideal|realistic] [other launch args] -- run the whole ROS 2 navigation demo
# without RViz and wait for the referee's summary.
#   wsl -d Ubuntu-24.04 -- bash /mnt/c/dev/duckling_sim/ros2/run_nav_headless.sh sensors:=realistic
# Results are copied to <repo>/results/post_fix/ros2_nav_<sensors>[_<TAG>]/. Only the processes this script
# launched are stopped (its own process group); other ROS processes on the machine are left alone.
P=${DUCK_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
source /opt/ros/jazzy/setup.bash; source ~/duck_ws/install/setup.bash
SENSORS=ideal
for a in "$@"; do case $a in sensors:=*) SENSORS=${a#sensors:=};; esac; done
OUT=${OUT:-$HOME/duck_ws/run_$SENSORS}; rm -rf "$OUT"
LOG=$(mktemp /tmp/duck_nav.XXXX.log)
setsid ros2 launch duck_nav duck_nav.launch.py rviz:=false out_dir:="$OUT" "$@" > "$LOG" 2>&1 &
PGID=$!
for i in $(seq 1 ${WAIT:-420}); do
  [ -f "$OUT/summary.txt" ] && break
  grep -q "process has died" "$LOG" && { echo "A NODE DIED"; break; }
  sleep 5
done
sleep 2
kill -INT -- -$PGID 2>/dev/null; sleep 3; kill -TERM -- -$PGID 2>/dev/null; sleep 1; kill -KILL -- -$PGID 2>/dev/null
grep -E "world_sim.*(sim [0-9]+ s|up )|expired|duck_nav\]|duck_slam\]|duck_yolo\]|died|Traceback|Error" "$LOG" | sed 's/\[INFO\] \[[0-9.]*\] //' | cut -c1-220 | tail -45
echo "=== summary"; cat "$OUT/summary.txt" 2>/dev/null || echo "no summary written"
DEST=$P/results/post_fix/ros2_nav_${SENSORS}${TAG:+_$TAG}
mkdir -p "$DEST" && cp "$OUT"/* "$DEST"/ 2>/dev/null; cp "$LOG" "$DEST/launch.log"
