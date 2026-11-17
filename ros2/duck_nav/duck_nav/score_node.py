"""score_node -- referee. Compares the SLAM estimate with MuJoCo's ground truth and writes a run summary.

Subscribes  /ground_truth/pose, /slam/pose, /goal, /map, /nav/status, /yolo/detections
Writes      <out_dir>/summary.txt and <out_dir>/map.png when the navigator reports "exploration complete"
            or when `timeout` simulated seconds have passed. It never feeds anything back to the robot.
"""
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray


class ScoreNode(Node):
    def __init__(self):
        super().__init__("duck_score")
        self.declare_parameter("out_dir", os.path.expanduser("~/duck_ws/run"))
        self.declare_parameter("timeout", 600.0)
        self.declare_parameter("beacon_x", 4.0); self.declare_parameter("beacon_y", 1.3)
        self.gt = None; self.est = None; self.goal = None; self.grid = None; self.info = None
        self.true_xy, self.est_xy, self.errs = [], [], []
        self.falls = 0; self.was_up = True; self.events = []; self.status = ""; self.t_beacon = None; self.d_beacon = None
        self.det = {"beacon": 0, "box": 0}; self.frames = 0; self.written = False
        self.create_subscription(PoseStamped, "ground_truth/pose", self.on_gt, 10)
        self.create_subscription(PoseStamped, "slam/pose", lambda m: setattr(self, "est", (m.pose.position.x, m.pose.position.y)), 10)
        self.create_subscription(PointStamped, "goal", lambda m: setattr(self, "goal", (m.point.x, m.point.y)), 10)
        self.create_subscription(OccupancyGrid, "map", self.on_map, 1)
        self.create_subscription(String, "nav/status", self.on_status, 10)
        self.create_subscription(Detection2DArray, "yolo/detections", self.on_det, 5)
        self.create_timer(0.2, self.sample)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, m):
        self.gt = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        up = m.pose.position.z > 0.085
        if self.was_up and not up:
            self.falls += 1
        self.was_up = up

    def on_map(self, m):
        self.grid = np.array(m.data, np.int8).reshape(m.info.height, m.info.width); self.info = m.info

    def on_det(self, m):
        self.frames += 1
        for d in m.detections:
            if d.results:
                self.det[d.results[0].hypothesis.class_id] = self.det.get(d.results[0].hypothesis.class_id, 0) + 1

    def on_status(self, m):
        if m.data != self.status:
            self.status = m.data; self.events.append(f"{self.now():6.0f} s  {m.data}")
            if m.data.startswith("beacon reached") and self.t_beacon is None and self.gt:
                self.t_beacon = self.now()
                self.d_beacon = math.hypot(self.gt[0] - self.get_parameter("beacon_x").value, self.gt[1] - self.get_parameter("beacon_y").value)

    def sample(self):
        if self.written or self.gt is None or self.est is None:
            return
        self.true_xy.append(self.gt[:2]); self.est_xy.append(self.est)
        self.errs.append(math.hypot(self.gt[0] - self.est[0], self.gt[1] - self.est[1]))
        if self.status.startswith("exploration complete") or self.now() > self.get_parameter("timeout").value:
            self.write()

    def write(self):
        self.written = True
        out = self.get_parameter("out_dir").value; os.makedirs(out, exist_ok=True)
        bx, by = self.get_parameter("beacon_x").value, self.get_parameter("beacon_y").value
        walked = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.true_xy, self.true_xy[1:]))
        known = float((self.grid != -1).mean()) * 100 if self.grid is not None else 0.0
        lines = [
            f"finished            : {self.status or 'no status'} at {self.now():.0f} s simulated",
            f"beacon              : " + (f"reached at {self.t_beacon:.0f} s, true distance {self.d_beacon:.2f} m" if self.t_beacon else "NOT reached"),
            f"beacon position     : estimated {None if self.goal is None else tuple(round(v, 2) for v in self.goal)}, true ({bx}, {by})",
            f"SLAM position error : mean {np.mean(self.errs):.3f} m, max {np.max(self.errs):.3f} m, final {self.errs[-1]:.3f} m",
            f"path walked         : {walked:.1f} m, falls {self.falls}",
            f"YOLO                : {self.frames} frames, {self.det.get('beacon', 0)} beacon and {self.det.get('box', 0)} box detections",
            f"map                 : {known:.0f}% of the grid classified",
            "events:"] + ["  " + e for e in self.events]
        open(os.path.join(out, "summary.txt"), "w").write("\n".join(lines) + "\n")
        try:
            from PIL import Image, ImageDraw
            g = self.grid; img = np.full(g.shape + (3,), 128, np.uint8); img[g == 0] = 255; img[g == 100] = 0
            s = 4; im = Image.fromarray(np.flipud(img)).resize((g.shape[1] * s, g.shape[0] * s), Image.NEAREST)
            dr = ImageDraw.Draw(im); res, ox, oy = self.info.resolution, self.info.origin.position.x, self.info.origin.position.y
            px = lambda x, y: ((x - ox) / res * s, (g.shape[0] - (y - oy) / res) * s)
            dr.line([px(*p) for p in self.true_xy], fill=(0, 170, 0), width=2)
            dr.line([px(*p) for p in self.est_xy], fill=(220, 0, 0), width=2)
            cx, cy = px(bx, by); dr.ellipse([cx - 7, cy - 7, cx + 7, cy + 7], outline=(255, 0, 255), width=3)
            if self.goal:
                ex, ey = px(*self.goal); dr.line([ex - 8, ey, ex + 8, ey], fill=(255, 0, 255), width=2); dr.line([ex, ey - 8, ex, ey + 8], fill=(255, 0, 255), width=2)
            im.save(os.path.join(out, "map.png"))
        except Exception as e:                                             # noqa: BLE001
            self.get_logger().warn(f"map image not written: {e}")
        self.get_logger().info("RUN FINISHED\n" + "\n".join(lines))


def main():
    rclpy.init()
    node = ScoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()
