"""ros2 launch duck_nav duck_nav.launch.py [rviz:=false] [autonomous:=false] [scan_matching:=false] [timeout:=600.0] [out_dir:=...]
autonomous:=false -> the navigator stays idle until something publishes true on /nav/enable."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("duck_nav")
    with open(os.path.join(get_package_share_directory("duck_description"), "urdf", "duck_v3.urdf")) as f:
        urdf = f.read()
    sim_time = {"use_sim_time": True}
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("scan_matching", default_value="true"),
        DeclareLaunchArgument("autonomous", default_value="true"),
        DeclareLaunchArgument("timeout", default_value="600.0"),
        DeclareLaunchArgument("out_dir", default_value=os.path.expanduser("~/duck_ws/run")),
        Node(package="duck_nav", executable="world_sim", output="screen",
             parameters=[{"mjcf": os.path.join(share, "mjcf", "duck_v3_gc_world.xml"),
                          "policy": os.path.join(share, "models", "velstand.onnx"),
                          "world_json": os.path.join(share, "mjcf", "world.json")}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}, sim_time]),
        Node(package="duck_nav", executable="yolo_node", output="screen",
             parameters=[{"model": os.path.join(share, "models", "duck_yolo.onnx")}, sim_time]),
        Node(package="duck_nav", executable="slam_node", output="screen",
             parameters=[{"scan_matching": ParameterValue(LaunchConfiguration("scan_matching"), value_type=bool)}, sim_time]),
        Node(package="duck_nav", executable="nav_node", output="screen",
             parameters=[{"autonomous": ParameterValue(LaunchConfiguration("autonomous"), value_type=bool)}, sim_time]),
        Node(package="duck_nav", executable="score_node", output="screen",
             parameters=[{"timeout": ParameterValue(LaunchConfiguration("timeout"), value_type=float),
                          "out_dir": LaunchConfiguration("out_dir")}, sim_time]),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", os.path.join(share, "rviz", "duck_nav.rviz")], parameters=[sim_time]),
    ])
