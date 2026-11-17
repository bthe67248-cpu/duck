#!/usr/bin/env bash
# run_nav_rviz_check.sh -- run the demo WITH RViz until the referee finishes, then report RViz complaints.
source /opt/ros/jazzy/setup.bash; source ~/duck_ws/install/setup.bash
OUT=$HOME/duck_ws/run2; rm -rf "$OUT"
setsid ros2 launch duck_nav duck_nav.launch.py out_dir:="$OUT" "$@" > /tmp/duck_nav_rviz.log 2>&1 &
LAUNCH=$!
for i in $(seq 1 40); do [ -f "$OUT/summary.txt" ] && break; sleep 5; done
sleep 2
echo "rviz running: $(pgrep -c rviz2)"
echo "--- rviz complaints (count, text)"
grep -i -E "rviz2.*(error|warn|fail|drop|extrapolat|transform)" /tmp/duck_nav_rviz.log | sed 's/\[[0-9.]*\]//' | cut -c1-170 | sort | uniq -c | sort -rn | head -8
kill -INT -- -$LAUNCH 2>/dev/null; sleep 3; kill -TERM -- -$LAUNCH 2>/dev/null   # only what this script started
echo "--- summary"; cat "$OUT/summary.txt"
P=${DUCK_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
mkdir -p $P/results/post_fix/ros2_nav_rviz; cp "$OUT"/* $P/results/post_fix/ros2_nav_rviz/
