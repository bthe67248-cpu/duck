"""Perception acquisition only: requires configured ToF adapter and calibrated TF.

No navigator, policy, Bridge or command publisher is started. This intentionally
does not supply simulated transforms or enable navigation on uncalibrated odometry.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sensor_config", description="Absolute path to measured ROS parameter YAML"),
        Node(package="duck_nav", executable="usb_camera_node", output="screen",
             parameters=[LaunchConfiguration("sensor_config"), {"use_sim_time": False}]),
        Node(package="duck_nav", executable="duck_tof", output="screen",
             parameters=[LaunchConfiguration("sensor_config"), {"use_sim_time": False}]),
        Node(package="duck_nav", executable="sensor_pose_node", output="screen",
             parameters=[LaunchConfiguration("sensor_config"), {"use_sim_time": False}]),
    ])
