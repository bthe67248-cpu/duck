"""score_node -- referee. Compares the SLAM estimate with MuJoCo's ground truth and writes a run summary.

Subscribes  /ground_truth/pose, /slam/pose, /goal, /map, /nav/status, /nav/result, /yolo/detections
Writes      <out_dir>/summary.txt, <out_dir>/result.json and <out_dir>/map.png when the navigator publishes a
            final /nav/result or when `timeout` simulated seconds have passed (reported as TIMEOUT, never as success).
            It never feeds anything back to the robot.

Scoring notes / limits
- SLAM error pairs each estimate with the ground-truth pose of the SAME stamp (bounded history), not the latest one.
- A fall = trunk tilt > 45 deg or trunk height < `fall_height` (from the true pose); counted on the transition,
  except while /duck/mode is a deliberate posture (sit, rise, roll, pick).
- Exploration outcome and beacon outcome are reported separately; "beacon reached" uses the TRUE distance.
- Coverage is the classified fraction of the true room interior (from world.json walls if given), not of the grid.
"""
import json
import math
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray


def stamp_ns(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


class ScoreNode(Node):
    def __init__(self):
        super().__init__("duck_score")
        self.declare_parameter("out_dir", os.path.expanduser("~/duck_ws/run"))
        self.declare_parameter("timeout", 600.0)
        self.declare_parameter("beacon_x", 4.0); self.declare_parameter("beacon_y", 1.3)
        self.declare_parameter("beacon_reach_m", 0.45)        # true centre distance that counts as "reached"
        self.declare_parameter("fall_height", 0.085)
        self.declare_parameter("room", [-0.67, 4.47, -1.97, 1.97])   # x0 x1 y0 y1 of the room interior (world.json walls)
        self.declare_parameter("profile", "")
        self.revision_ns = -1; self.map_revision = 0
        self.gt_hist = {}; self.gt = None; self.goal = None; self.grid = None; self.info = None
        self.true_xy, self.est_xy, self.errs = [], [], []
        self.falls = 0; self.was_up = True; self.events = []; self.status = ""; self.result = None
        self.t_beacon = None; self.d_min = math.inf
        self.det = {}; self.frames = 0; self.written = False
        self.create_subscription(PoseStamped, "ground_truth/pose", self.on_gt, 50)
        self.create_subscription(PoseStamped, "slam/pose", self.on_est, 50)
        self.create_subscription(PointStamped, "goal", self.on_goal, 10)
        self.create_subscription(String, "slam/map_revision", self.on_map_revision, 10)
        self.create_subscription(OccupancyGrid, "map", self.on_map, 1)
        self.create_subscription(String, "nav/status", self.on_status, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "nav/result", self.on_result, latched)
        self.create_subscription(Detection2DArray, "yolo/detections", self.on_det, 5)
        self.mode = "walk"
        self.create_subscription(String, "duck/mode", lambda m: setattr(self, "mode", m.data), 10)
        self.create_timer(0.5, self.check_done)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_goal(self, m):
        if stamp_ns(m.header) > self.revision_ns:
            self.goal = (m.point.x, m.point.y)

    def on_map_revision(self, m):
        try:
            revision, ns = (int(x) for x in m.data.split(":"))
        except (ValueError, TypeError):
            return
        if revision > self.map_revision:
            self.map_revision = revision; self.revision_ns = ns; self.goal = None

    def on_gt(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.gt = (p.x, p.y, p.z)
        self.gt_hist[stamp_ns(m.header)] = (p.x, p.y)
        if len(self.gt_hist) > 500:
            for k in sorted(self.gt_hist)[:100]:
                del self.gt_hist[k]
        up_z = 1 - 2 * (q.x * q.x + q.y * q.y)                 # body z-axis . world z
        up = p.z > self.get_parameter("fall_height").value and up_z > math.cos(math.radians(45))
        if self.mode in ("sit", "rise", "roll", "pick"):
            up = True                                           # deliberate low postures are not falls
        if self.was_up and not up:
            self.falls += 1; self.events.append(f"{self.now():6.1f} s  FALL (true pose)")
        self.was_up = up
        d = math.hypot(p.x - self.get_parameter("beacon_x").value, p.y - self.get_parameter("beacon_y").value)
        self.d_min = min(self.d_min, d)
        if self.t_beacon is None and d < self.get_parameter("beacon_reach_m").value:
            self.t_beacon = self.now(); self.events.append(f"{self.t_beacon:6.1f} s  beacon reached (true distance {d:.2f} m)")

    def on_est(self, m):
        gt = self.gt_hist.get(stamp_ns(m.header))
        if gt is None:
            return                                              # no ground truth with this stamp: not scored
        self.true_xy.append(gt); self.est_xy.append((m.pose.position.x, m.pose.position.y))
        self.errs.append(math.hypot(gt[0] - m.pose.position.x, gt[1] - m.pose.position.y))

    def on_map(self, m):
        self.grid = np.array(m.data, np.int8).reshape(m.info.height, m.info.width); self.info = m.info

    def on_det(self, m):
        self.frames += 1
        for d in m.detections:
            if d.results:
                c = d.results[0].hypothesis.class_id
                self.det[c] = self.det.get(c, 0) + 1

    def on_status(self, m):
        if m.data != self.status:
            self.status = m.data; self.events.append(f"{self.now():6.1f} s  {m.data}")

    def on_result(self, m):
        self.result = m.data

    def check_done(self):
        if self.written or self.gt is None:
            return
        if self.result is not None or self.now() > self.get_parameter("timeout").value:
            self.write()

    def coverage(self):
        if self.grid is None:
            return 0.0
        x0, x1, y0, y1 = self.get_parameter("room").value
        res, ox, oy = self.info.resolution, self.info.origin.position.x, self.info.origin.position.y
        r0, r1 = int((y0 - oy) / res), int((y1 - oy) / res); c0, c1 = int((x0 - ox) / res), int((x1 - ox) / res)
        sub = self.grid[max(r0, 0):max(r1, 0), max(c0, 0):max(c1, 0)]
        return float((sub != -1).mean()) * 100 if sub.size else 0.0

    def write(self):
        self.written = True
        out = self.get_parameter("out_dir").value; os.makedirs(out, exist_ok=True)
        bx, by = self.get_parameter("beacon_x").value, self.get_parameter("beacon_y").value
        walked = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.true_xy, self.true_xy[1:]))
        exploration = self.result.split(";")[0] if self.result else f"TIMEOUT at {self.now():.0f} s (not complete)"
        goal_err = None if self.goal is None else math.hypot(self.goal[0] - bx, self.goal[1] - by)
        errs = np.array(self.errs) if self.errs else np.array([np.nan])
        res = {
            "profile": self.get_parameter("profile").value,
            "exploration": exploration,
            "exploration_success": bool(self.result and self.result.startswith("exploration complete")),
            "beacon_reached": self.t_beacon is not None, "beacon_reached_t": self.t_beacon,
            "beacon_min_true_distance_m": None if math.isinf(self.d_min) else round(self.d_min, 3),
            "beacon_estimate": None if self.goal is None else [round(v, 3) for v in self.goal],
            "beacon_estimate_error_m": None if goal_err is None else round(goal_err, 3),
            "slam_error_m": {"mean": float(np.nanmean(errs)), "max": float(np.nanmax(errs)), "final": float(errs[-1]),
                             "samples": len(self.errs)},
            "walked_m": round(walked, 2), "falls": self.falls,
            "yolo": {"frames": self.frames, **self.det},
            "room_coverage_pct": round(self.coverage(), 1),
            "sim_time_s": round(self.now(), 1),
        }
        lines = [
            f"exploration         : {exploration}",
            f"beacon              : " + (f"reached at {self.t_beacon:.0f} s (true distance < {self.get_parameter('beacon_reach_m').value} m)"
                                         if self.t_beacon else f"NOT reached (closest {res['beacon_min_true_distance_m']} m)"),
            f"beacon estimate     : {res['beacon_estimate']}, true ({bx}, {by}), error {res['beacon_estimate_error_m']} m",
            f"SLAM position error : mean {res['slam_error_m']['mean']:.3f} m, max {res['slam_error_m']['max']:.3f} m, "
            f"final {res['slam_error_m']['final']:.3f} m ({len(self.errs)} stamp-matched samples)",
            f"path walked         : {walked:.1f} m, falls {self.falls}",
            f"YOLO                : {self.frames} frames, detections {self.det}",
            f"room coverage       : {res['room_coverage_pct']:.0f}% of the room interior classified",
            "events:"] + ["  " + e for e in self.events]
        with open(os.path.join(out, "summary.txt"), "w") as f:
            f.write("\n".join(lines) + "\n")
        with open(os.path.join(out, "result.json"), "w") as f:
            json.dump(res, f, indent=2)
        try:
            from PIL import Image, ImageDraw
            g = self.grid; img = np.full(g.shape + (3,), 128, np.uint8); img[g == 0] = 255; img[g == 100] = 0
            s = 4; im = Image.fromarray(np.flipud(img)).resize((g.shape[1] * s, g.shape[0] * s), Image.NEAREST)
            dr = ImageDraw.Draw(im); r_, ox, oy = self.info.resolution, self.info.origin.position.x, self.info.origin.position.y
            px = lambda x, y: ((x - ox) / r_ * s, (g.shape[0] - (y - oy) / r_) * s)
            if len(self.true_xy) > 1:
                dr.line([px(*p) for p in self.true_xy], fill=(0, 170, 0), width=2)
                dr.line([px(*p) for p in self.est_xy], fill=(220, 0, 0), width=2)
            cx, cy = px(bx, by); dr.ellipse([cx - 7, cy - 7, cx + 7, cy + 7], outline=(255, 0, 255), width=3)
            if self.goal:
                ex, ey = px(*self.goal); dr.line([ex - 8, ey, ex + 8, ey], fill=(255, 0, 255), width=2); dr.line([ex, ey - 8, ex, ey + 8], fill=(255, 0, 255), width=2)
            im.save(os.path.join(out, "map.png"))
        except Exception as e:                                             # noqa: BLE001
            self.get_logger().warn(f"map image not written: {e}")
        self.get_logger().info("RUN FINISHED\n" + "\n".join(lines))
        # The runner must not kill this process between result.json and map rendering.
        # Rename within the output directory gives it an atomic completion marker.
        marker = os.path.join(out, "run_complete.json")
        with open(marker + ".tmp", "w") as f:
            json.dump({"schema_version": 1, "sim_time_s": res["sim_time_s"],
                       "artifacts": [name for name in ("result.json", "summary.txt", "map.png")
                                     if os.path.isfile(os.path.join(out, name))]}, f)
        os.replace(marker + ".tmp", marker)


def main():
    rclpy.init()
    node = ScoreNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:                                    # noqa: BLE001  torn down by a signal mid-wait
        if rclpy.ok():
            raise
    finally:
        node.destroy_node(); rclpy.try_shutdown()
