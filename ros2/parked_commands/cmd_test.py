"""cmd_test.py -- send each basic command and check, against MuJoCo's true pose, that the duck did it.
Run by ros2/run_cmd_test.sh while the demo is up with autonomous:=false.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

rclpy.init()
node = Node("cmd_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
gt = {}; mode = {"m": "?"}; status = {"s": ""}


def on_gt(m):
    o = m.pose.orientation
    gt.update(x=m.pose.position.x, y=m.pose.position.y, z=m.pose.position.z, yaw=math.degrees(2 * math.atan2(o.z, o.w)))


node.create_subscription(PoseStamped, "ground_truth/pose", on_gt, 10)
node.create_subscription(String, "duck/mode", lambda m: mode.update(m=m.data), 10)
node.create_subscription(String, "nav/status", lambda m: status.update(s=m.data), 10)
pub = node.create_publisher(String, "duck/command", 10)


def sim_now():
    return node.get_clock().now().nanoseconds * 1e-9


def wait(sim_seconds):
    t0 = sim_now(); w0 = time.time()
    while sim_now() - t0 < sim_seconds and time.time() - w0 < sim_seconds * 4 + 20:
        rclpy.spin_once(node, timeout_sec=0.05)


def dyaw(a, b):
    return (b - a + 180) % 360 - 180


wait(4.0)
rows, ok_all = [], True
tests = [  # command, sim seconds to wait, check(before, after, lowest z, modes seen) -> (ok, text)
    ("forward 4", 5.5, lambda a, b, zmin, ms: (math.hypot(b["x"] - a["x"], b["y"] - a["y"]) > 0.4, f"moved {math.hypot(b['x'] - a['x'], b['y'] - a['y']):.2f} m")),
    ("back 3", 4.5, lambda a, b, zmin, ms: (math.hypot(b["x"] - a["x"], b["y"] - a["y"]) > 0.3, f"moved {math.hypot(b['x'] - a['x'], b['y'] - a['y']):.2f} m")),
    ("turn left 2", 3.5, lambda a, b, zmin, ms: (dyaw(a["yaw"], b["yaw"]) > 40, f"turned {dyaw(a['yaw'], b['yaw']):+.0f} deg")),
    ("right 2", 3.5, lambda a, b, zmin, ms: (dyaw(a["yaw"], b["yaw"]) < -40, f"turned {dyaw(a['yaw'], b['yaw']):+.0f} deg")),
    ("step left 3", 4.5, lambda a, b, zmin, ms: (math.hypot(b["x"] - a["x"], b["y"] - a["y"]) > 0.12, f"moved {math.hypot(b['x'] - a['x'], b['y'] - a['y']):.2f} m")),
    ("sit", 5.0, lambda a, b, zmin, ms: (b["z"] < 0.085, f"torso height {a['z']:.3f} -> {b['z']:.3f} m")),
    ("stand", 6.0, lambda a, b, zmin, ms: (b["z"] > 0.105, f"torso height {a['z']:.3f} -> {b['z']:.3f} m")),
    ("roll", 6.0, lambda a, b, zmin, ms: ("roll" in ms and b["z"] > 0.105, f"roll policy ran, moved {math.hypot(b['x'] - a['x'], b['y'] - a['y']):.2f} m, ends at height {b['z']:.3f} m")),
    ("pick", 5.0, lambda a, b, zmin, ms: ("pick" in ms and b["z"] > 0.105, f"pick policy ran, torso dipped to {zmin:.3f} m, ends at {b['z']:.3f} m")),
    ("dance", 22.5, lambda a, b, zmin, ms: (b["z"] > 0.105, f"20 s routine, ends upright at {b['z']:.3f} m, turned {dyaw(a['yaw'], b['yaw']):+.0f} deg net")),
    ("jump over the moon", 1.5, lambda a, b, zmin, ms: (math.hypot(b["x"] - a["x"], b["y"] - a["y"]) < 0.05, "not a command: ignored, duck stayed put")),
    ("explore", 12.0, lambda a, b, zmin, ms: (status["s"] in ("exploring", "heading for the beacon") or status["s"].startswith("beacon"), f"navigator took over: '{status['s']}'")),
    ("stop", 3.0, lambda a, b, zmin, ms: (status["s"] == "manual control", f"navigator released: '{status['s']}'")),
]
for cmd, secs, check in tests:
    a = dict(gt); zmin = a["z"]; seen = set()
    pub.publish(String(data=cmd))
    t0 = sim_now(); w0 = time.time()
    while sim_now() - t0 < secs and time.time() - w0 < secs * 4 + 20:
        rclpy.spin_once(node, timeout_sec=0.05); zmin = min(zmin, gt["z"]); seen.add(mode["m"])
    ok, text = check(a, dict(gt), zmin, seen)
    ok_all &= ok
    rows.append(f"{cmd:20s} {'PASS' if ok else 'FAIL'}  {text}")
    print(rows[-1], flush=True)
open(sys.argv[1], "w").write("\n".join(rows) + "\n")
rclpy.try_shutdown()
sys.exit(0 if ok_all else 1)
