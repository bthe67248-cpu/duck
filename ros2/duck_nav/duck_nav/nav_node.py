"""nav_node -- explore the room, visit the beacon when YOLO+SLAM have located it, keep exploring until the map is done.

Subscribes  /map (OccupancyGrid; its resolution/origin/size are used and validated), /slam/pose (PoseStamped),
            /slam/state (String: "ok" | "recovering" ...), /goal (PointStamped, frame "map"), /nav/enable (Bool)
Publishes   /cmd_vel (Twist: linear.x = walk command, angular.z = turn), /head_yaw_cmd (Float64, sweeping),
            /plan (nav_msgs/Path), /nav/status (String), /nav/result (String, latched: final outcome)
Planner: weighted wavefront over free cells kept `clearance` m from obstacles (Euclidean), preferring cells with
         `plan_comfort` m to spare, to the beacon or else the nearest frontier (1 Hz).
Driver (20 Hz): pure pursuit with a lookahead that shrinks at corners and next to obstacles and a walk command that
         drops with path curvature. When the pure-pursuit arc fails the footprint check (`motion_clear`) a fan of turn
         rates is searched for a clear arc that still makes progress along the path; only if none exists does the
         clearance guard stop forward motion and turn in place. After `guard_recover_s` of guard without progress the
         robot gives up that frontier for `frontier_avoid_s` (exploring) or makes a local manoeuvre: turn to the
         clearest heading and take a short guarded step (beacon / repeated). `corner_handling:=false` restores plain
         pure pursuit + guard.

Command ownership: while enabled this node is the only /cmd_vel publisher. When it is disabled, finishes, stops for
a fault, or shuts down it publishes a zero Twist; world_sim also expires any command older than its `cmd_timeout`,
so a stopped publisher cannot leave a walk command latched.

Outcome states (/nav/status): exploring | heading for the beacon | escape ... | waiting for observations |
stale pose | stale map | slam recovering | robot is outside the map | goal rejected | exploration stalled |
exploration complete. Reaching the beacon is reported separately from exploration completion (/nav/result).
"""
import math
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
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
        self.declare_parameter("slam_state_timeout", 0.75)
        self.declare_parameter("speed_bound_mps", 0.35)  # calibrate on hardware; conservative simulated forward speed
        self.declare_parameter("min_explored_m2", 2.0)   # reachable known-free area needed before "complete"
        self.declare_parameter("no_frontier_plans", 5)   # consecutive no-frontier plans before judging
        self.declare_parameter("goal_reached_m", 0.36)
        # corners: weighted planning, adaptive lookahead, arc search and guard recovery (False: plain pursuit + guard)
        self.declare_parameter("corner_handling", True)
        self.declare_parameter("plan_comfort", 0.10)      # m beyond `clearance` the planner prefers to keep (0 = off)
        self.declare_parameter("guard_recover_s", 2.0)    # s of clearance guard without progress before recovering
        self.declare_parameter("frontier_avoid_s", 60.0)  # s a frontier the robot could not reach is skipped
        # body-fixed depth sensor (no head sweep for mapping): turn in place to map all round at start-up, and turn
        # slowly to find mapped walls again when localisation stays weak. 0 = off (head-mounted sensor).
        self.declare_parameter("survey_turn", 0.0)        # rad of in-place turning before exploring (2*pi = full turn)
        self.declare_parameter("survey_rate", 0.5)        # turn command during the survey / look-around
        self.declare_parameter("weak_look_after", 0.0)    # s of "slam weak" before turning in place to look around
        # body yaw oscillation while walking (head-sweep substitute for a body-fixed sensor): +- yaw_sweep_rate rad/s
        # added to the steering command, period yaw_sweep_period s. 0 = off.
        self.declare_parameter("yaw_sweep_rate", 0.0)
        self.declare_parameter("yaw_sweep_period", 3.0)
        self.survey_done = False; self.survey_last = None; self.survey_acc = 0.0; self.weak_since = None
        for name in ("pose_timeout", "map_timeout", "slam_state_timeout", "speed_bound_mps"):
            ds.positive_finite(self.get_parameter(name).value, name)
        self.enabled = bool(self.get_parameter("autonomous").value)
        self.lock = threading.RLock()                    # replan / drive / callbacks run on different threads
        self.grid = None; self.info = None; self.map_t = None
        self.pose = None; self.pose_t = None; self.slam_state = "initializing"; self.slam_state_t = None; self.goal = None
        self.revision_stamp = -math.inf
        self.map_revision = 0
        self.distances = None
        self.path = None; self.mode = "explore"; self.visited_goal = False; self.finished = False
        self.progress_ref = None; self.unstick_until = 0.0; self.status = ""; self.no_frontier = 0
        self.guard_time = 0.0; self.episode_recoveries = 0; self.avoid = []; self.manoeuvre = None
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_head = self.create_publisher(Float64, "head_yaw_cmd", 10)
        self.pub_plan = self.create_publisher(Path, "plan", 2)
        self.pub_status = self.create_publisher(String, "nav/status", 10)
        self.pub_result = self.create_publisher(String, "nav/result",
                                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(OccupancyGrid, "map", self.on_map, 1)
        self.create_subscription(PoseStamped, "slam/pose", self.on_pose, 10)
        self.create_subscription(String, "slam/state", self.on_slam_state, 10)
        self.create_subscription(String, "slam/map_revision", self.on_map_revision, 10)
        self.create_subscription(PointStamped, "goal", self.on_goal, 10)
        self.create_subscription(Bool, "nav/enable", self.on_enable, 10)
        # planning runs in its own callback group so a slow plan never blocks the 20 Hz command stream
        self.create_timer(1.0, self.replan, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.05, self.drive, callback_group=MutuallyExclusiveCallbackGroup())

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
        info = ds.MapInfo(res=float(i.resolution), x0=float(i.origin.position.x), y0=float(i.origin.position.y),
                          nx=int(i.width), ny=int(i.height))
        grid = ds.grid_from_msg_data(msg.data, i.width, i.height)
        _, distances = ds._clearance_m(grid, info)
        with self.lock:
            self.info, self.grid, self.map_t = info, grid, self.stamp(msg.header)
            self.distances = distances

    @staticmethod
    def stamp(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def on_slam_state(self, msg):
        with self.lock:
            self.slam_state = msg.data; self.slam_state_t = self.now()

    def on_pose(self, msg):
        if msg.header.frame_id != "map":
            return
        o = msg.pose.orientation
        pose = (msg.pose.position.x, msg.pose.position.y, 2 * math.atan2(o.z, o.w))
        if not all(math.isfinite(v) for v in pose):
            return
        with self.lock:
            self.pose = pose; self.pose_t = self.stamp(msg.header)

    def on_map_revision(self, msg):
        try:
            revision, ns = (int(x) for x in msg.data.split(":"))
        except (ValueError, TypeError):
            return
        with self.lock:
            if revision <= getattr(self, "map_revision", 0):
                return
            self.map_revision = revision; self.revision_stamp = ns * 1e-9
            self.path = None; self.goal = None; self.visited_goal = False
            self.progress_ref = None; self.unstick_until = 0.0; self.no_frontier = 0; self.avoid = []
            self.stop("map corrected; waiting for fresh observations")

    def on_goal(self, msg):
        if self.stamp(msg.header) <= getattr(self, "revision_stamp", -math.inf):
            return
        g = (msg.point.x, msg.point.y)
        if msg.header.frame_id != "map" or not all(math.isfinite(v) for v in g):
            self.say(f"goal rejected: frame '{msg.header.frame_id}'"); return
        if self.info is not None and not self.info.inside(*self.info.cell(*g)):
            self.say(f"goal rejected: ({g[0]:.2f}, {g[1]:.2f}) is outside the map"); return
        self.goal = g

    def on_enable(self, msg):
        with self.lock:
            self.set_enabled(msg)

    def set_enabled(self, msg):
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

    def stop(self, why=None, center_head=True):
        self.pub_cmd.publish(Twist()); self.path = None
        if center_head:
            self.pub_head.publish(Float64(data=0.0))
        self.unstick_until = 0.0; self.progress_ref = None; self.manoeuvre = None
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
        if min(self.pose_t, self.map_t) < getattr(self, "revision_stamp", -math.inf):
            return "waiting for corrected map and pose"
        if not 0 <= t - self.pose_t <= self.get_parameter("pose_timeout").value:
            return "stale pose"
        if not 0 <= t - self.map_t <= self.get_parameter("map_timeout").value:
            return "stale map"
        if self.slam_state_t is None or not 0 <= t - self.slam_state_t <= self.get_parameter("slam_state_timeout").value:
            return "stale slam state"
        if self.slam_state != "ok":
            return f"slam {self.slam_state}"
        return None

    # ---- planning -----------------------------------------------------------------------------
    def replan(self):
        with self.lock:
            if self.finished or not self.enabled:
                return
            f = self.fault()
            if f:
                self.path = None; self.say(f); return
            revision = getattr(self, "map_revision", 0)
            grid, info, pose = self.grid, self.info, self.pose
            goal = None if self.visited_goal else self.goal
            if goal is not None and math.hypot(goal[0] - pose[0], goal[1] - pose[1]) < self.get_parameter("goal_reached_m").value:
                self.visited_goal = True; goal = None
                self.say("beacon reached; exploring the rest of the room")
        # the expensive part runs without the lock, on a consistent snapshot of grid/info/pose
        clearance = float(self.get_parameter("clearance").value)
        corner = bool(self.get_parameter("corner_handling").value)
        comfort = float(self.get_parameter("plan_comfort").value) if corner else 0.0
        with self.lock:
            now = self.now()
            self.avoid = [a for a in self.avoid if a[3] > now]
            avoid = [a[:3] for a in self.avoid]
        path, mode = ds.plan(grid, pose, goal, info=info, clearance=clearance, comfort=comfort, avoid=avoid)
        if mode == "no_frontier" and avoid:              # skipped frontiers never end exploration
            path, mode = ds.plan(grid, pose, goal, info=info, clearance=clearance, comfort=comfort)
        state = None
        if mode == "no_frontier":
            state = ds.exploration_state(grid, pose, info, min_area_m2=float(self.get_parameter("min_explored_m2").value))
        with self.lock:
            if self.finished or not self.enabled or revision != getattr(self, "map_revision", 0):
                return                                   # inputs changed while planning
            if self.fault():
                self.path = None; return
            self.apply_plan(path, mode, state)

    def apply_plan(self, path, mode, state):
        self.path, self.mode = path, mode
        if mode != "no_frontier":
            self.no_frontier = 0
        if path is None:
            if mode == "no_frontier":
                self.no_frontier += 1
                st, area, open_f = state
                if self.no_frontier >= int(self.get_parameter("no_frontier_plans").value):
                    if st == "complete":
                        self.finish(f"exploration complete ({area:.1f} m2 reachable, closed)")
                    elif st == "stalled":
                        self.finish(f"exploration stalled: no usable frontier but the room is not closed "
                                    f"({area:.1f} m2 reachable, open frontier {open_f} cells)")
                    else:
                        self.say(f"waiting for observations ({area:.2f} m2 known free)")
                else:
                    self.say(ds.STATUS_TEXT["no_frontier"])
            else:
                self.say(ds.STATUS_TEXT[mode])          # out_of_map / start_blocked: drive() stops
            return
        self.say(ds.STATUS_TEXT[mode] if mode != "explore" or not self.visited_goal
                 else "beacon reached; exploring the rest of the room")
        msg = Path(); msg.header.frame_id = "map"; msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path[::3] + [path[-1]]:
            ps = PoseStamped(); ps.header = msg.header; ps.pose.position.x, ps.pose.position.y = x, y; ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_plan.publish(msg)

    # ---- body-fixed depth: survey and look-around ----------------------------------------------
    def look_around(self, t, fault):
        """Start-up survey turn and look-around while weak (body-fixed depth sensor). True: a turn was sent.
        Only soft faults (slam weak / initializing) are overridden; stale pose/map/state still stop the robot."""
        cmd = Twist(); cmd.angular.z = float(self.get_parameter("survey_rate").value)
        soft = fault is None or fault.startswith(("slam weak", "slam initializing", "slam blind"))
        survey = float(self.get_parameter("survey_turn").value)
        if survey > 0 and not self.survey_done:
            if self.pose is None or self.grid is None or not soft:
                return False
            if self.survey_last is None:
                self.survey_last, self.survey_acc = self.pose[2], 0.0
                self.say("survey: turning in place to map all round")
            self.survey_acc += abs((self.pose[2] - self.survey_last + math.pi) % (2 * math.pi) - math.pi)
            self.survey_last = self.pose[2]
            if self.survey_acc < survey:
                self.pub_cmd.publish(cmd); return True
            self.survey_done = True; self.say("survey done")
            return False
        after = float(self.get_parameter("weak_look_after").value)
        if after > 0 and fault is not None and fault.startswith(("slam weak", "slam blind")):
            self.weak_since = t if self.weak_since is None else self.weak_since
            if t - self.weak_since > after:
                # turn 1.5 s, hold still 2 s: a confirmed relocalisation needs two agreeing matches
                if (t - self.weak_since - after) % 3.5 < 1.5:
                    if self.status != "slam weak: looking around":
                        self.say("slam weak: looking around")
                    self.pub_cmd.publish(cmd); return True
            return False
        self.weak_since = None
        return False

    # ---- driving ------------------------------------------------------------------------------
    def drive(self):
        with self.lock:                                  # enable/finish cannot slip in between check and publish
            self.drive_step()

    def drive_step(self):
        if not self.enabled or self.finished:
            return                      # someone else owns /cmd_vel, or we are done; a zero was sent on the transition
        t = self.now()
        self.pub_head.publish(Float64(data=0.9 * math.sin(2 * math.pi * t / 4.0)))      # head sweep
        fault = self.fault()
        if self.look_around(t, fault):
            return
        if fault or self.mode in ("out_of_map", "start_blocked"):
            self.stop(fault, center_head=False); return      # keep gathering head-sweep constraints while still
        cmd = Twist()
        if t < self.unstick_until:
            cmd.angular.z = 1.2; self.pub_cmd.publish(cmd); return
        path = self.path
        if self.progress_ref is None or math.hypot(self.pose[0] - self.progress_ref[0], self.pose[1] - self.progress_ref[1]) > 0.10:
            if self.progress_ref is not None:
                self.guard_time = 0.0; self.episode_recoveries = 0      # moved: a new stuck episode starts from zero
            self.progress_ref = (self.pose[0], self.pose[1], t)
        elif t - self.progress_ref[2] > 8.0 and path:
            self.unstick_until = t + 2.5; self.progress_ref = None
            self.get_logger().info("no progress for 8 s: turning in place and replanning")
        corner = bool(self.get_parameter("corner_handling").value)
        if corner and self.manoeuvre is not None and self.run_manoeuvre(t):
            return
        if path:
            escape = self.mode == "escape"
            clearance = float(self.get_parameter("clearance").value)
            bound = float(self.get_parameter("speed_bound_mps").value)
            walk = float(self.get_parameter("walk_cmd").value)
            if escape or not corner:
                lookahead, turn = (0.06 if escape else 0.22), 0.0
            else:
                lookahead, turn = ds.adaptive_lookahead(path, self.pose, info=self.info, distances=self.distances,
                                                        clearance=clearance)
            tgt = ds.path_target(path, self.pose, lookahead=lookahead)
            e = (math.atan2(tgt[1] - self.pose[1], tgt[0] - self.pose[0]) - self.pose[2] + math.pi) % (2 * math.pi) - math.pi
            if abs(e) > 0.6:
                cmd.angular.z = 1.2 * float(np.sign(e))
            else:
                # slow with curvature: walk_cmd at a straight path, SLOW_WALK of it at a right-angle corner
                cmd.linear.x = walk * (1.0 - (1.0 - SLOW_WALK) * min(1.0, turn / (math.pi / 2)))
                cmd.angular.z = float(np.clip(2.0 * e, -1.0, 1.0))
                sweep = float(self.get_parameter("yaw_sweep_rate").value)
                if sweep > 0 and self.mode != "escape":
                    period = max(float(self.get_parameter("yaw_sweep_period").value), 0.5)
                    cmd.angular.z = float(np.clip(cmd.angular.z + sweep * math.sin(2 * math.pi * t / period), -1.2, 1.2))
                kw = dict(info=self.info, clearance=clearance, escape=escape, distances=self.distances)
                if not ds.motion_clear(self.grid, self.pose, bound, cmd.angular.z, **kw):
                    alt = None
                    if corner:   # same conservative speed bound: any arc chosen here also passes the guard below
                        alt = ds.choose_arc(self.grid, self.pose, path, bound, ARC_RATES, preferred=cmd.angular.z, **kw)
                    if alt is not None:
                        cmd.angular.z = alt; cmd.linear.x = min(cmd.linear.x, walk * SLOW_WALK)
                        self.say("near an obstacle: steering on a clear arc")
                    else:
                        cmd.linear.x = 0.0
                        w = float(np.clip(2.0 * e, -1.0, 1.0))
                        cmd.angular.z = w if not corner else float(np.sign(e) or 1.0) * max(0.5, abs(w))
                        self.say("path clearance guard: turning in place and replanning")
                        if corner:
                            self.guard_time += 0.05
                            if self.guard_time >= float(self.get_parameter("guard_recover_s").value):
                                self.recover(t)
                # the stop guard stays the safety net: never walk on an arc that fails the footprint check
                if cmd.linear.x > 0 and not ds.motion_clear(self.grid, self.pose, bound, cmd.angular.z, **kw):
                    cmd.linear.x = 0.0
        elif self.mode == "no_frontier":
            cmd.angular.z = 1.0                                                         # look around in place
        self.pub_cmd.publish(cmd)

    # ---- guard recovery -----------------------------------------------------------------------
    def recover(self, t):
        """The guard kept the robot from following its path. Exploring: skip that frontier for a while (first try in
        this spot). Otherwise: turn to the clearest heading and take a short guarded step, then replan."""
        self.guard_time = 0.0; self.episode_recoveries += 1
        if self.mode == "explore" and self.path and self.episode_recoveries == 1:
            x, y = self.path[-1]
            self.avoid.append((x, y, 0.4, t + float(self.get_parameter("frontier_avoid_s").value)))
            self.path = None
            self.say("path clearance guard: trying another frontier"); return
        heading = ds.best_clear_heading(self.grid, self.pose, float(self.get_parameter("speed_bound_mps").value),
                                        info=self.info, clearance=float(self.get_parameter("clearance").value),
                                        distances=self.distances)
        if heading is None:
            self.unstick_until = t + 2.5                     # nothing clear all round: the old turn-in-place unstick
            self.say("path clearance guard: no clear heading, turning in place"); return
        self.manoeuvre = [heading, t + 6.0, None]
        self.say("path clearance guard: local manoeuvre")

    def run_manoeuvre(self, t):
        """Turn in place to the chosen heading, then walk slowly up to MANOEUVRE_STEP_S while the arc stays clear.
        True while it is still driving; on the way out the path is dropped so the next plan starts from here."""
        yaw, until, walk_until = self.manoeuvre
        err = (yaw - self.pose[2] + math.pi) % (2 * math.pi) - math.pi
        cmd = Twist()
        if t <= until:
            if walk_until is None and abs(err) > 0.15:
                cmd.angular.z = float(np.sign(err)) * max(0.5, min(1.2, 2.0 * abs(err)))
                self.pub_cmd.publish(cmd); return True
            if walk_until is None:
                walk_until = self.manoeuvre[2] = t + MANOEUVRE_STEP_S
            w = float(np.clip(2.0 * err, -1.0, 1.0))
            if t < walk_until and ds.motion_clear(self.grid, self.pose, float(self.get_parameter("speed_bound_mps").value),
                                                  w, info=self.info, clearance=float(self.get_parameter("clearance").value),
                                                  distances=self.distances):
                cmd.linear.x = float(self.get_parameter("walk_cmd").value) * SLOW_WALK; cmd.angular.z = w
                self.pub_cmd.publish(cmd); return True
        self.manoeuvre = None; self.path = None
        return False


SLOW_WALK = 0.7                                  # fraction of walk_cmd near obstacles (0.35 of 0.5; < 0.25 = stand)
ARC_RATES = tuple(np.linspace(-1.2, 1.2, 13))    # turn rates (rad/s) tried when the pure-pursuit arc is not clear
MANOEUVRE_STEP_S = 1.2


def main():
    rclpy.init()
    node = NavNode()
    ex = MultiThreadedExecutor(num_threads=3); ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:                                    # noqa: BLE001  executor torn down by a signal mid-wait
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.pub_cmd.publish(Twist())                # leave the robot commanded still
        node.destroy_node(); rclpy.try_shutdown()
