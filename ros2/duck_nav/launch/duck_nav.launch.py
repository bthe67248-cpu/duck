"""ros2 launch duck_nav duck_nav.launch.py [sensors:=ideal|realistic] [rviz:=false] [autonomous:=false]
                                          [scan_matching:=false] [matcher:=correlative|grid] [depth_sweep:=true]
                                          [real_time:=false] [rt_factor:=1.0] [timeout:=600.0] [out_dir:=...] [seed:=0]
sensors:=ideal      perfect 320x240 RGB-D camera (the original simulation; an upper bound, not the real robot)
sensors:=realistic  noisy RGB camera + simulated VL53L5CX 8x8 ToF (45 x 45 deg, 10 Hz, 2 m usable, dropouts,
                    60 ms latency). SLAM maps from the 64 ToF zones and ranges YOLO boxes with them.
autonomous:=false -> the navigator stays idle until something publishes true on /nav/enable."""
import os
import json

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
             parameters=[{"mjcf": LaunchConfiguration("mjcf").perform(context) or os.path.join(share, "mjcf", "duck_v3_gc_world.xml"),
                          "policy": os.path.join(share, "models", "velstand.onnx"),
                          "world_json": LaunchConfiguration("world_json").perform(context) or os.path.join(share, "mjcf", "world.json"),
                          "sensor_profile": sensors, "seed": seed,
                          "real_time": ParameterValue(LaunchConfiguration("real_time"), value_type=bool),
                          "rt_factor": ParameterValue(LaunchConfiguration("rt_factor"), value_type=float)}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}, sim_time]),
        Node(package="duck_nav", executable="yolo_node", output="screen",
             parameters=[{"model": os.path.join(share, "models", "duck_yolo.onnx")}, sim_time]),
        Node(package="duck_nav", executable="slam_node", output="screen",
             parameters=[{"scan_matching": ParameterValue(LaunchConfiguration("scan_matching"), value_type=bool),
                          "sensor_mode": "depth" if sensors == "ideal" else "tof",
                          "loop_closure": ParameterValue(LaunchConfiguration("loop_closure"), value_type=bool),
                          "attitude_filter": LaunchConfiguration("attitude_filter"),  # quaternion | tilt
                          "translation": LaunchConfiguration("translation"),  # command | flow | flow_or_command
                          "flow_method": LaunchConfiguration("flow_method"),
                          "gyro_calib_s": ParameterValue(LaunchConfiguration("gyro_calib_s"), value_type=float),
                          "graph_max_nodes": ParameterValue(LaunchConfiguration("graph_max_nodes"), value_type=int),
                          "matcher": LaunchConfiguration("matcher"),  # correlative | grid
                          "depth_sweep": ParameterValue(LaunchConfiguration("depth_sweep"), value_type=bool),
                          "target_radius": 0.04,                    # the sim beacon is a known 4 cm-radius,
                          "target_height": 0.28},                  # 28 cm tall cylinder
                         sim_time]),
        Node(package="duck_nav", executable="nav_node", output="screen",
             parameters=[{"autonomous": ParameterValue(LaunchConfiguration("autonomous"), value_type=bool)}, sim_time]),
        Node(package="duck_nav", executable="score_node", output="screen",
             parameters=[{"timeout": ParameterValue(LaunchConfiguration("timeout"), value_type=float),
                          "out_dir": LaunchConfiguration("out_dir"), "profile": sensors,
                          "beacon_x": ParameterValue(LaunchConfiguration("beacon_x"), value_type=float),
                          "beacon_y": ParameterValue(LaunchConfiguration("beacon_y"), value_type=float),
                          "room": [float(v) for v in json.loads(LaunchConfiguration("room").perform(context))]}, sim_time]),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", os.path.join(share, "rviz", "duck_nav.rviz")], parameters=[sim_time]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sensors", default_value="ideal"),
        DeclareLaunchArgument("mjcf", default_value=""),
        DeclareLaunchArgument("world_json", default_value=""),
        DeclareLaunchArgument("beacon_x", default_value="4.0"),
        DeclareLaunchArgument("beacon_y", default_value="1.3"),
        DeclareLaunchArgument("room", default_value="[-0.67, 4.47, -1.97, 1.97]"),
        DeclareLaunchArgument("loop_closure", default_value="false"),
        DeclareLaunchArgument("attitude_filter", default_value="quaternion"),
        DeclareLaunchArgument("translation", default_value="command"),
        DeclareLaunchArgument("flow_method", default_value="rect_or_image"),
        DeclareLaunchArgument("gyro_calib_s", default_value="0.0"),
        DeclareLaunchArgument("graph_max_nodes", default_value="160"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("scan_matching", default_value="true"),
        DeclareLaunchArgument("matcher", default_value="correlative"),
        DeclareLaunchArgument("real_time", default_value="true"),   # false: as fast as the simulator can step
        DeclareLaunchArgument("rt_factor", default_value="1.0"),
        DeclareLaunchArgument("depth_sweep", default_value="true"),
        DeclareLaunchArgument("autonomous", default_value="true"),
        DeclareLaunchArgument("timeout", default_value="600.0"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("out_dir", default_value=os.path.expanduser("~/duck_ws/run")),
        OpaqueFunction(function=nodes),
    ])
