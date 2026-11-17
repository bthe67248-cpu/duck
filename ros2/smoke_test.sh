#!/usr/bin/env bash
# smoke_test.sh -- headless check of the ROS 2 duck demo (no RViz window needed).
#   wsl -d Ubuntu-24.04 -- bash /mnt/c/dev/duckling_sim/ros2/smoke_test.sh
source /opt/ros/jazzy/setup.bash
source ~/duck_ws/install/setup.bash
ros2 launch duck_description duck_demo.launch.py rviz:=false > /tmp/duck_launch.log 2>&1 &
LAUNCH=$!
sleep 6
echo "--- nodes";  ros2 node list
echo "--- topics"; ros2 topic list
echo "--- /joint_states rate"; timeout 4 ros2 topic hz /joint_states 2>&1 | tail -2
echo "--- pitch standing";      ros2 topic echo /duck/pitch_deg --once 2>&1 | head -1
echo "--- TF odom->ankle_left"; timeout 3 ros2 run tf2_ros tf2_echo odom ankle_left 2>&1 | grep -m1 Translation
echo "--- push 2 N (0.10 N.s), expect it to stay up"
ros2 topic pub --once /duck/push std_msgs/msg/Float64 "{data: 2.0}" > /dev/null
sleep 2
ros2 topic echo /duck/pitch_deg --once 2>&1 | head -1
timeout 3 ros2 run tf2_ros tf2_echo odom trunk_base 2>&1 | grep -m1 Translation
echo "--- push 8 N (0.40 N.s), expect it to fall"
ros2 topic pub --once /duck/push std_msgs/msg/Float64 "{data: 8.0}" > /dev/null
sleep 2
timeout 3 ros2 run tf2_ros tf2_echo odom trunk_base 2>&1 | grep -m1 Translation
echo "--- reset"
ros2 topic pub --once /duck/reset std_msgs/msg/Empty "{}" > /dev/null
sleep 1
timeout 3 ros2 run tf2_ros tf2_echo odom trunk_base 2>&1 | grep -m1 Translation
kill -INT $LAUNCH; sleep 1; pkill -f duck_mujoco_sim; pkill -f robot_state_publisher
echo "--- launch log tail"; tail -5 /tmp/duck_launch.log
