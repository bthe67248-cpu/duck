"""ros2 launch duck_nav duck_nav.launch.py [sensors:=ideal|realistic] [rviz:=false] [autonomous:=false]
                                          [scan_matching:=false] [timeout:=600.0] [out_dir:=...] [seed:=0]
sensors:=ideal      perfect 320x240 RGB-D camera (the original simulation; an upper bound, not the real robot)
sensors:=realistic  noisy RGB camera + simulated VL53L5CX 8x8 ToF (45 x 45 deg, 10 Hz, 2 m usable, dropouts,
                    60 ms latency). SLAM maps from the 64 ToF zones and ranges YOLO boxes with them.
autonomous:=false -> the navigator stays idle until something publishes true on /nav/enable."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nodes(context):
    share = get_package_share_directory("duck_nav")
    with open(os.path.join(get_package_share_directory("duck_description"), "urdf", "duck_v3.urdf")) as f:
        urdf = f.read()
    sensors = LaunchConfiguration("sensors").perform(context)
    if sensors not in ("ideal", "realistic"):
        raise RuntimeError(f"sensors:={sensors} -- use ideal or realistic")
    sim_time = {"use_sim_time": True}
    seed = int(LaunchConfiguration("seed").perform(context))
    return [
        Node(package="duck_nav", executable="world_sim", output="screen",
             parameters=[{"mjcf": os.path.join(share, "mjcf", "duck_v3_gc_world.xml"),
                          "policy": os.path.join(share, "models", "velstand.onnx"),
                          "world_json": os.path.join(share, "mjcf", "world.json"),
                          "sensor_profile": sensors, "seed": seed}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}, sim_time]),
        Node(package="duck_nav", executable="yolo_node", output="screen",
             parameters=[{"model": os.path.join(share, "models", "duck_yolo.onnx")}, sim_time]),
        Node(package="duck_nav", executable="slam_node", output="screen",
             parameters=[{"scan_matching": ParameterValue(LaunchConfiguration("scan_matching"), value_type=bool),
                          "sensor_mode": "depth" if sensors == "ideal" else "tof",
                          "target_radius": 0.04,                    # the sim beacon is a known 4 cm-radius,
                          "target_height": 0.28},                  # 28 cm tall cylinder
                         sim_time]),
        Node(package="duck_nav", executable="nav_node", output="screen",
             parameters=[{"autonomous": ParameterValue(LaunchConfiguration("autonomous"), value_type=bool)}, sim_time]),
        Node(package="duck_nav", executable="score_node", output="screen",
             parameters=[{"timeout": ParameterValue(LaunchConfiguration("timeout"), value_type=float),
                          "out_dir": LaunchConfiguration("out_dir"), "profile": sensors}, sim_time]),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", os.path.join(share, "rviz", "duck_nav.rviz")], parameters=[sim_time]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sensors", default_value="ideal"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("scan_matching", default_value="true"),
        DeclareLaunchArgument("autonomous", default_value="true"),
        DeclareLaunchArgument("timeout", default_value="600.0"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("out_dir", default_value=os.path.expanduser("~/duck_ws/run")),
        OpaqueFunction(function=nodes),
    ])
