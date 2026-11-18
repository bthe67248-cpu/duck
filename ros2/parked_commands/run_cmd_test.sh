#!/usr/bin/env bash
# run_cmd_test.sh -- start the demo in manual mode and test every basic command against the true pose.
#   wsl -d Ubuntu-24.04 -- bash /mnt/c/dev/duckling_sim/ros2/run_cmd_test.sh
source /opt/ros/jazzy/setup.bash; source ~/duck_ws/install/setup.bash
pkill -f "duck_nav/lib" 2>/dev/null; pkill -f robot_state_publisher 2>/dev/null; sleep 1
ros2 launch duck_nav duck_nav.launch.py rviz:=false autonomous:=false out_dir:=/tmp/duck_cmd_run > /tmp/duck_cmd.log 2>&1 &
sleep 8
mkdir -p /mnt/c/dev/duckling_sim/results/ros2_nav
python3 /mnt/c/dev/duckling_sim/ros2/cmd_test.py /mnt/c/dev/duckling_sim/results/ros2_nav/command_test.txt
echo "exit code $?"
pkill -INT -f "ros2 launch duck_nav"; sleep 2; pkill -f "duck_nav/lib"; pkill -f robot_state_publisher
grep -E "died|Traceback|Error" /tmp/duck_cmd.log | head -5
