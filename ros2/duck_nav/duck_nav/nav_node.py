"""nav_node -- explore the room, visit the beacon when YOLO+SLAM have located it, keep exploring until the map is done.

Subscribes  /map (OccupancyGrid; its resolution/origin/size are used and validated), /slam/pose (PoseStamped),
            /slam/state (String: "ok" | "recovering" ...), /goal (PointStamped, frame "map"), /nav/enable (Bool)
Publishes   /cmd_vel (Twist: linear.x = walk command, angular.z = turn), /head_yaw_cmd (Float64, sweeping),
            /plan (nav_msgs/Path), /nav/status (String), /nav/result (String, latched: final outcome)
Planner: wavefront over free cells kept `clearance` m from obstacles (Euclidean), to the beacon or else the nearest
         frontier (1 Hz). Driver: pure pursuit of the path (20 Hz).

Command ownership: while enabled this node is the only /cmd_vel publisher. When it is disabled, finishes, stops for
a fault, or shuts down it publishes a zero Twist; world_sim also expires any command older than its `cmd_timeout`,
so a stopped publisher cannot leave a walk command latched.

Outcome states (/nav/status): exploring | heading for the beacon | escape ... | waiting for observations |
stale pose | stale map | slam recovering | robot is outside the map | goal rejected | exploration stalled |
exploration complete. Reaching the beacon is reported separately from exploration completion (/nav/result).
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64, String

from . import duckslam as ds


class NavNode(Node):
    def __init__(self):
        super().__init__("duck_nav")
        self.declare_parameter("walk_cmd", 0.5)
        self.declare_parameter("autonomous", True)       # False: wait for true on /nav/enable
        self.declare_parameter("clearance", ds.ROBOT_RADIUS)
        self.declare_parameter("pose_timeout", 0.5)      # s (sim time): older pose -> stop
        self.declare_parameter("map_timeout", 3.0)       # s (sim time): older map -> stop
        self.declare_parameter("min_explored_m2", 2.0)   # reachable known-free area needed before "complete"
        self.declare_parameter("no_frontier_plans", 5)   # consecutive no-frontier plans before judging
        self.declare_parameter("goal_reached_m", 0.36)
        self.enabled = bool(self.get_parameter("autonomous").value)
        self.grid = None; self.info = None; self.map_t = None
        self.pose = None; self.pose_t = None; self.slam_state = "ok"; self.goal = None
        self.path = None; self.mode = "explore"; self.visited_goal = False; self.finished = False
        self.progress_ref = None; self.unstick_until = 0.0; self.status = ""; self.no_frontier = 0
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_head = self.create_publisher(Float64, "head_yaw_cmd", 10)
        self.pub_plan = self.create_publisher(Path, "plan", 2)
        self.pub_status = self.create_publisher(String, "nav/status", 10)
        self.pub_result = self.create_publisher(String, "nav/result",
                                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(OccupancyGrid, "map", self.on_map, 1)
        self.create_subscription(PoseStamped, "slam/pose", self.on_pose, 10)
        self.create_subscription(String, "slam/state", lambda m: setattr(self, "slam_state", m.data), 10)
        self.create_subscription(PointStamped, "goal", self.on_goal, 10)
        self.create_subscription(Bool, "nav/enable", self.on_enable, 10)
        self.create_timer(1.0, self.replan)
        self.create_timer(0.05, self.drive)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs ------------------------------------------------------------------------------
    def on_map(self, msg):
        i = msg.info
        if msg.header.frame_id != "map" or i.resolution <= 0 or i.width == 0 or i.height == 0 \
                or len(msg.data) != i.width * i.height:
            self.get_logger().warn(f"map rejected: frame '{msg.header.frame_id}', res {i.resolution}, "
                                   f"{i.width}x{i.height}, {len(msg.data)} cells")
            return
        q = i.origin.orientation
        if abs(q.x) + abs(q.y) + abs(q.z) > 1e-6:
            self.get_logger().warn("map rejected: rotated map origins are not supported")
            return
        self.info = ds.MapInfo(res=float(i.resolution), x0=float(i.origin.position.x), y0=float(i.origin.position.y),
                               nx=int(i.width), ny=int(i.height))
        self.grid = ds.grid_from_msg_data(msg.data, i.width, i.height)
        self.map_t = self.now()

    def on_pose(self, msg):
        if msg.header.frame_id != "map":
            return
        o = msg.pose.orientation
        self.pose = (msg.pose.position.x, msg.pose.position.y, 2 * math.atan2(o.z, o.w))
        self.pose_t = self.now()

    def on_goal(self, msg):
        g = (msg.point.x, msg.point.y)
        if msg.header.frame_id != "map" or not all(math.isfinite(v) for v in g):
            self.say(f"goal rejected: frame '{msg.header.frame_id}'"); return
        if self.info is not None and not self.info.inside(*self.info.cell(*g)):
            self.say(f"goal rejected: ({g[0]:.2f}, {g[1]:.2f}) is outside the map"); return
        self.goal = g

    def on_enable(self, msg):
        if msg.data == self.enabled:
            return
        self.enabled = msg.data; self.path = None; self.progress_ref = None
        if msg.data:
            self.finished = False; self.no_frontier = 0
            self.say("autonomy enabled")
        else:
            self.stop("manual control")                 # hand over with the robot commanded still

    # ---- outputs ------------------------------------------------------------------------------
    def say(self, text):
        if text != self.status:
            self.status = text
            self.get_logger().info(f"[t={self.now():.1f} s] {text}")
        self.pub_status.publish(String(data=text))

    def stop(self, why=None):
        self.pub_cmd.publish(Twist()); self.path = None
        if why:
            self.say(why)

    def finish(self, result):
        self.finished = True
        self.stop(result)
        self.pub_result.publish(String(data=result + ("; beacon reached" if self.visited_goal else "; beacon NOT reached")))

    def fault(self):
        """Reason the robot must not move right now, or None."""
        t = self.now()
        if self.pose_t is None or self.grid is None:
            return "waiting for observations"
        if t - self.pose_t > self.get_parameter("pose_timeout").value:
            return "stale pose"
        if t - self.map_t > self.get_parameter("map_timeout").value:
            return "stale map"
        if self.slam_state != "ok":
            return f"slam {self.slam_state}"
        return None

    # ---- planning -----------------------------------------------------------------------------
    def replan(self):
        if self.finished or not self.enabled:
            return
        f = self.fault()
        if f:
            self.path = None; self.say(f); return
        goal = None if self.visited_goal else self.goal
        if goal is not None and math.hypot(goal[0] - self.pose[0], goal[1] - self.pose[1]) < self.get_parameter("goal_reached_m").value:
            self.visited_goal = True; goal = None
            self.say("beacon reached; exploring the rest of the room")
        self.path, self.mode = ds.plan(self.grid, self.pose, goal, info=self.info,
                                       clearance=float(self.get_parameter("clearance").value))
        if self.mode != "no_frontier":
            self.no_frontier = 0
        if self.path is None:
            if self.mode == "no_frontier":
                self.no_frontier += 1
                state, area, open_f = ds.exploration_state(self.grid, self.pose, self.info,
                                                           min_area_m2=float(self.get_parameter("min_explored_m2").value))
                if self.no_frontier >= int(self.get_parameter("no_frontier_plans").value):
                    if state == "complete":
                        self.finish(f"exploration complete ({area:.1f} m2 reachable, closed)")
                    elif state == "stalled":
                        self.finish(f"exploration stalled: no usable frontier but the room is not closed "
                                    f"({area:.1f} m2 reachable, open frontier {open_f} cells)")
                    else:
                        self.say(f"waiting for observations ({area:.2f} m2 known free)")
                else:
                    self.say(ds.STATUS_TEXT["no_frontier"])
            else:
                self.say(ds.STATUS_TEXT[self.mode])     # out_of_map / start_blocked: drive() stops
            return
        self.say(ds.STATUS_TEXT[self.mode] if self.mode != "explore" or not self.visited_goal
                 else "beacon reached; exploring the rest of the room")
        p = Path(); p.header.frame_id = "map"; p.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path[::3] + [self.path[-1]]:
            ps = PoseStamped(); ps.header = p.header; ps.pose.position.x, ps.pose.position.y = x, y; ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.pub_plan.publish(p)

    # ---- driving ------------------------------------------------------------------------------
    def drive(self):
        if not self.enabled or self.finished:
            return                      # someone else owns /cmd_vel, or we are done; a zero was sent on the transition
        t = self.now()
        self.pub_head.publish(Float64(data=0.9 * math.sin(2 * math.pi * t / 4.0)))      # head sweep
        if self.fault() or self.mode in ("out_of_map", "start_blocked"):
            self.stop(); return
        cmd = Twist()
        if t < self.unstick_until:
            cmd.angular.z = 1.2; self.pub_cmd.publish(cmd); return
        if self.progress_ref is None or math.hypot(self.pose[0] - self.progress_ref[0], self.pose[1] - self.progress_ref[1]) > 0.10:
            self.progress_ref = (self.pose[0], self.pose[1], t)
        elif t - self.progress_ref[2] > 8.0 and self.path:
            self.unstick_until = t + 2.5; self.progress_ref = None
            self.get_logger().info("no progress for 8 s: turning in place and replanning")
        if self.path:
            ahead = [p for p in self.path if math.hypot(p[0] - self.pose[0], p[1] - self.pose[1]) > 0.22]
            tgt = ahead[0] if ahead else self.path[-1]
            e = (math.atan2(tgt[1] - self.pose[1], tgt[0] - self.pose[0]) - self.pose[2] + math.pi) % (2 * math.pi) - math.pi
            if abs(e) > 0.6:
                cmd.angular.z = 1.2 * float(np.sign(e))
            else:
                cmd.linear.x = float(self.get_parameter("walk_cmd").value); cmd.angular.z = float(np.clip(2.0 * e, -1.0, 1.0))
        elif self.mode == "no_frontier":
            cmd.angular.z = 1.0                                                         # look around in place
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_cmd.publish(Twist())                # leave the robot commanded still
        except Exception:                                # noqa: BLE001
            pass
        node.destroy_node(); rclpy.try_shutdown()
