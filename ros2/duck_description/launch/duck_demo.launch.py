"""ros2 launch duck_description duck_demo.launch.py [rviz:=false]"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("duck_description")
    with open(os.path.join(share, "urdf", "duck_v3.urdf")) as f:
        urdf = f.read()
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        Node(package="duck_description", executable="duck_mujoco_sim", output="screen",
             parameters=[{"mjcf": os.path.join(share, "mjcf", "duck_v3_scene.xml")}]),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", os.path.join(share, "rviz", "duck.rviz")]),
    ])
