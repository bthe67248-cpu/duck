"""ros2 launch duck_nav duck_nav.launch.py [sensors:=ideal|realistic] [rviz:=false] [autonomous:=false]
                                          [scan_matching:=false] [matcher:=correlative|grid] [depth_sweep:=true]
                                          [real_time:=false] [rt_factor:=1.0] [timeout:=600.0] [out_dir:=...] [seed:=0]
sensors:=ideal      perfect 320x240 RGB-D camera (the original simulation; an upper bound, not the real robot)
sensors:=realistic  the V4 robot (model/duck_v4_gc_world.xml): noisy RGB camera on the head + a simulated Sipeed A010
                    ToF in the chest (25 x 25 zones binned from 100 x 100, 70 x 60 deg, 0.2-2.5 m, 15 Hz, dropouts,
                    60 ms latency). SLAM maps from the ToF zones and ranges YOLO boxes with them; the duck turns in place
                    at start-up to map all round (the body sensor has no head sweep).
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
    with open(os.path.join(get_package_share_directory("duck_description"), "urdf", "duck_v4.urdf")) as f:
        urdf = f.read()
    sensors = LaunchConfiguration("sensors").perform(context)
    if sensors not in ("ideal", "realistic"):
        raise RuntimeError(f"sensors:={sensors} -- use ideal or realistic")
    sim_time = {"use_sim_time": True}
    seed = int(LaunchConfiguration("seed").perform(context))
    # body-fixed depth sensor mapping policy (design/v4/README.md): keyframe mapping, never-observed cells scored above
    # known free space, no unconfirmed partial relocalisation, small relocalisation corrections applied directly
    body = sensors == "realistic" and LaunchConfiguration("body_slam").perform(context).lower() in ("true", "1")
    body_params = dict(map_move=0.1, map_turn=0.17, unknown_ll=-1.125, reloc_partial=False,
                       confirm_free_xy=0.08, confirm_free_th=0.07,
                       # second-peak match test (FrameLocalizer): off, closed loop showed no net gain (design/v4/README.md,
                       # "Late drift and the yaw sweep"); reloc_peak_margin:=1.0 is the variant worth trying
                       peak_margin=0.0, map_min_gap=0.0, reloc_peak_margin=0.0, partial_tracks=False) if body else {}
    as_bool = lambda v: v.lower() in ("true", "1")  # noqa: E731
    for k, cast in (("peak_margin", float), ("map_min_gap", float), ("reloc_peak_margin", float), ("partial_tracks", as_bool)):
        v = LaunchConfiguration(k).perform(context)
        if v != "":                                      # explicit launch argument overrides the profile
            body_params[k] = cast(v)
    return [
        Node(package="duck_nav", executable="world_sim", output="screen",
             parameters=[{"mjcf": LaunchConfiguration("mjcf").perform(context) or os.path.join(share, "mjcf", "duck_v4_gc_world.xml"),
                          "policy": os.path.join(share, "models", "velstand.onnx"),
                          "world_json": LaunchConfiguration("world_json").perform(context) or os.path.join(share, "mjcf", "world.json"),
                          "sensor_profile": sensors, "seed": seed,
                          "real_time": ParameterValue(LaunchConfiguration("real_time"), value_type=bool),
                          "tof_zones": ParameterValue(LaunchConfiguration("tof_zones"), value_type=int),
                          "tof_fov_deg": ParameterValue(LaunchConfiguration("tof_fov_deg"), value_type=float),
                          "tof_fov_v_deg": ParameterValue(LaunchConfiguration("tof_fov_v_deg"), value_type=float),
                          "tof_max_range": ParameterValue(LaunchConfiguration("tof_max_range"), value_type=float),
                          "tof_min_range": ParameterValue(LaunchConfiguration("tof_min_range"), value_type=float),
                          "tof_noise_rel": ParameterValue(LaunchConfiguration("tof_noise_rel"), value_type=float),
                          "tof_rate_hz": ParameterValue(LaunchConfiguration("tof_rate_hz"), value_type=float),
                          "tof_mount": LaunchConfiguration("tof_mount"),   # head | body (A010 bracket on the torso)
                          "sensor_errors": ParameterValue(LaunchConfiguration("sensor_errors"), value_type=bool),
                          "servo_delay": ParameterValue(LaunchConfiguration("servo_delay"), value_type=float),
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
                          "reloc_window": ParameterValue(LaunchConfiguration("reloc_window"), value_type=float),  # m
                          "reloc_ang": ParameterValue(LaunchConfiguration("reloc_ang"), value_type=float),  # rad
                          "reloc_confirm": ParameterValue(LaunchConfiguration("reloc_confirm"), value_type=bool),
                          "bootstrap_turn": ParameterValue(LaunchConfiguration("survey_turn"), value_type=float),
                          "tof_zones": ParameterValue(LaunchConfiguration("tof_zones"), value_type=int),
                          "tof_fov_deg": ParameterValue(LaunchConfiguration("tof_fov_deg"), value_type=float),
                          "tof_fov_v_deg": ParameterValue(LaunchConfiguration("tof_fov_v_deg"), value_type=float),
                          "tof_max_range": ParameterValue(LaunchConfiguration("tof_max_range"), value_type=float),
                          **body_params,
                          "target_radius": 0.04,                    # the sim beacon is a known 4 cm-radius,
                          "target_height": 0.28},                  # 28 cm tall cylinder
                         sim_time]),
        Node(package="duck_nav", executable="nav_node", output="screen",
             parameters=[{"autonomous": ParameterValue(LaunchConfiguration("autonomous"), value_type=bool),
                          "survey_turn": ParameterValue(LaunchConfiguration("survey_turn"), value_type=float),
                          "weak_look_after": ParameterValue(LaunchConfiguration("weak_look_after"), value_type=float),
                          "yaw_sweep_rate": ParameterValue(LaunchConfiguration("yaw_sweep_rate"), value_type=float),
                          "corner_handling": ParameterValue(LaunchConfiguration("corner_handling"), value_type=bool)},
                         sim_time]),
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
        # realistic profile = V4 hardware: A010 on the body (25 x 25 binned; tof_zones:=100 for the full array), SLAM with
        # confirmed small-window relocalisation, start-up survey turn. Old VL53L5CX on the head: tof_mount:=head
        # tof_zones:=8 tof_fov_deg:=45 tof_fov_v_deg:=0 tof_max_range:=2.0 tof_min_range:=0.02 tof_noise_rel:=0.015
        # tof_rate_hz:=10 survey_turn:=0 weak_look_after:=0
        DeclareLaunchArgument("tof_zones", default_value="25"),
        DeclareLaunchArgument("tof_fov_deg", default_value="70.0"),
        DeclareLaunchArgument("tof_fov_v_deg", default_value="60.0"),
        DeclareLaunchArgument("tof_max_range", default_value="2.5"),
        DeclareLaunchArgument("tof_min_range", default_value="0.2"),
        DeclareLaunchArgument("tof_noise_rel", default_value="0.01"),
        DeclareLaunchArgument("tof_rate_hz", default_value="15.0"),
        DeclareLaunchArgument("tof_mount", default_value="body"),
        # realistic profile: IMU bias/noise, encoder error, extrinsic error and timing jitter (world_sim
        # SENSOR_ERROR_DEFAULTS); off by default until paired trials have measured what they cost SLAM
        DeclareLaunchArgument("sensor_errors", default_value="false"),
        DeclareLaunchArgument("servo_delay", default_value="-1.0"),         # s; -1 = the model's servo_delay_s
        DeclareLaunchArgument("reloc_window", default_value="0.8"),          # m, relocalisation search
        DeclareLaunchArgument("reloc_ang", default_value="0.4363323129985824"),   # rad (25 deg)
        DeclareLaunchArgument("reloc_confirm", default_value="true"),
        DeclareLaunchArgument("survey_turn", default_value="6.6"),          # rad turned in place at start; SLAM maps every frame meanwhile
        DeclareLaunchArgument("weak_look_after", default_value="3.0"),      # s weak before looking around (body depth)
        DeclareLaunchArgument("body_slam", default_value="true"),           # body-sensor mapping policy (realistic)
        DeclareLaunchArgument("yaw_sweep_rate", default_value="0.0"),       # rad/s body yaw oscillation while walking
        DeclareLaunchArgument("peak_margin", default_value=""),             # "" = body_slam profile value
        DeclareLaunchArgument("map_min_gap", default_value=""),
        DeclareLaunchArgument("partial_tracks", default_value=""),
        DeclareLaunchArgument("reloc_peak_margin", default_value=""),
        DeclareLaunchArgument("corner_handling", default_value="true"),     # false: plain pure pursuit + clearance guard
        DeclareLaunchArgument("autonomous", default_value="true"),
        DeclareLaunchArgument("timeout", default_value="600.0"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("out_dir", default_value=os.path.expanduser("~/duck_ws/run")),
        OpaqueFunction(function=nodes),
    ])
