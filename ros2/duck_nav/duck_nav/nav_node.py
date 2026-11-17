"""nav_node -- explore the room, visit the beacon when YOLO+SLAM have located it, keep exploring until the map is done.

Subscribes  /map (OccupancyGrid), /slam/pose (PoseStamped), /goal (PointStamped)
Publishes   /cmd_vel (Twist: linear.x = walk command, angular.z = turn), /head_yaw_cmd (Float64, sweeping),
            /plan (nav_msgs/Path), /nav/status (std_msgs/String)
Planner: wavefront over free cells kept 12.5 cm from obstacles, to the beacon or else the nearest frontier (1 Hz).
Driver: pure pursuit of the path (20 Hz). If the duck makes no progress for 8 s it backs off by turning.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

from . import duckslam as ds


class NavNode(Node):
    def __init__(self):
        super().__init__("duck_nav")
        self.declare_parameter("walk_cmd", 0.5)
        self.declare_parameter("autonomous", True)       # False: wait for "explore" on /nav/enable
        self.enabled = bool(self.get_parameter("autonomous").value)
        self.grid = None; self.pose = None; self.goal = None
        self.path = None; self.mode = "explore"; self.visited_goal = False; self.finished = False
        self.progress_ref = None; self.unstick_until = 0.0; self.status = ""; self.done_count = 0
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_head = self.create_publisher(Float64, "head_yaw_cmd", 10)
        self.pub_plan = self.create_publisher(Path, "plan", 2)
        self.pub_status = self.create_publisher(String, "nav/status", 10)
        self.create_subscription(OccupancyGrid, "map", self.on_map, 1)
        self.create_subscription(PoseStamped, "slam/pose", self.on_pose, 10)
        self.create_subscription(PointStamped, "goal", lambda m: setattr(self, "goal", (m.point.x, m.point.y)), 10)
        self.create_subscription(Bool, "nav/enable", self.on_enable, 10)
        self.create_timer(1.0, self.replan)
        self.create_timer(0.05, self.drive)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_map(self, msg):
        self.grid = np.array(msg.data, np.int8).reshape(msg.info.height, msg.info.width).T

    def on_pose(self, msg):
        o = msg.pose.orientation
        self.pose = (msg.pose.position.x, msg.pose.position.y, 2 * math.atan2(o.z, o.w))

    def on_enable(self, msg):
        if msg.data != self.enabled:
            self.enabled = msg.data; self.path = None; self.progress_ref = None
            if msg.data:
                self.finished = False; self.done_count = 0
            else:
                self.say("manual control")

    def say(self, text):
        if text != self.status:
            self.status = text
            self.get_logger().info(f"[t={self.now():.0f} s] {text}")
        self.pub_status.publish(String(data=text))

    def replan(self):
        if self.grid is None or self.pose is None or self.finished or not self.enabled:
            return
        goal = None if self.visited_goal else self.goal
        if goal is not None and math.hypot(goal[0] - self.pose[0], goal[1] - self.pose[1]) < 0.36:
            self.visited_goal = True
            self.say("beacon reached; exploring the rest of the room")
            goal = None
        self.path, self.mode = ds.plan(self.grid, self.pose, goal)
        self.done_count = self.done_count + 1 if self.mode == "done" else 0
        if self.mode == "done":
            if self.now() > 20.0 and self.done_count >= 5:               # nothing left to explore, five plans in a row
                self.finished = True
                self.say("exploration complete" + ("" if self.visited_goal else " (beacon never reached)"))
            return
        self.say("heading for the beacon" if self.mode == "goal" else
                 ("exploring" if not self.visited_goal else "beacon reached; exploring the rest of the room"))
        p = Path(); p.header.frame_id = "map"; p.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path[::3]:
            ps = PoseStamped(); ps.header = p.header; ps.pose.position.x, ps.pose.position.y = x, y; ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.pub_plan.publish(p)

    def drive(self):
        if not self.enabled:
            return                                       # someone else is driving (/duck/command)
        t = self.now()
        cmd = Twist()
        self.pub_head.publish(Float64(data=0.9 * math.sin(2 * math.pi * t / 4.0)))      # head sweep
        if self.pose is None or self.finished:
            self.pub_cmd.publish(cmd); return
        if t < self.unstick_until:
            cmd.angular.z = 1.2; self.pub_cmd.publish(cmd); return
        if self.progress_ref is None or math.hypot(self.pose[0] - self.progress_ref[0], self.pose[1] - self.progress_ref[1]) > 0.10:
            self.progress_ref = (self.pose[0], self.pose[1], t)
        elif t - self.progress_ref[2] > 8.0 and self.path:
            self.unstick_until = t + 2.5; self.progress_ref = None
            self.get_logger().info("no progress for 8 s: turning away and replanning")
        if self.path:
            ahead = [p for p in self.path if math.hypot(p[0] - self.pose[0], p[1] - self.pose[1]) > 0.22]
            tgt = ahead[0] if ahead else self.path[-1]
            e = (math.atan2(tgt[1] - self.pose[1], tgt[0] - self.pose[0]) - self.pose[2] + math.pi) % (2 * math.pi) - math.pi
            if abs(e) > 0.6:
                cmd.angular.z = 1.2 * float(np.sign(e))
            else:
                cmd.linear.x = float(self.get_parameter("walk_cmd").value); cmd.angular.z = float(np.clip(2.0 * e, -1.0, 1.0))
        elif t > 3.0:
            cmd.angular.z = 1.0                                                         # nothing mapped yet: look around
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()
