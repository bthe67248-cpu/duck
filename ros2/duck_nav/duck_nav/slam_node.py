"""slam_node -- DuckSLAM as a ROS 2 node.

Subscribes  /imu (gyro + accel -> complementary attitude filter), /cmd_vel (gait command, expires after 0.5 s),
            sensor_mode:=depth (ideal profile): /camera/depth + /camera/pose_level (same stamp) + /camera/camera_info
            sensor_mode:=tof   (realistic):     /tof/zones (PointCloud2) + /tof/pose_level (same stamp),
                                                /camera/pose_level + /camera/camera_info for detection association
            /yolo/detections (vision_msgs/Detection2DArray)
Publishes   /map (OccupancyGrid, 1 Hz), TF map -> base_level, /slam/pose (PoseStamped),
            /slam/state (String: "ok" | "initializing" | "weak" | "stale" | "recovering"),
            /slam/match (String: local constraint acceptance reason and map overlap; not an accuracy bound),
            /goal (PointStamped): the beacon's map position, only when a detection has a trustworthy range,
            /slam/beacon (String): "ranged ..." / "unranged: ..." for every beacon detection (diagnostics)

Timing: every sensor frame is processed at the SLAM pose of its acquisition stamp (from a bounded pose history),
and any scan-match correction is carried forward to the current pose. Detections wait (up to `det_wait` s) for
the sensor frame with their stamp; unmatched ones expire.
"""
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import Detection2DArray

from . import duckslam as ds


def stamp_ns(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


def quat_to_mat(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class StampBuffer:
    """Bounded {stamp_ns: value} with exact and nearest lookups."""

    def __init__(self, n):
        self.n = n; self.d = {}

    def put(self, k, v):
        self.d[k] = v
        if len(self.d) > self.n:
            for old in sorted(self.d)[:len(self.d) - self.n]:
                del self.d[old]

    def get(self, k):
        return self.d.get(k)

    def nearest(self, k, tol_ns):
        if not self.d:
            return None, None
        best = min(self.d, key=lambda s: abs(s - k))
        return (best, self.d[best]) if abs(best - k) <= tol_ns else (None, None)


class SlamNode(Node):
    def __init__(self):
        super().__init__("duck_slam")
        self.declare_parameter("sensor_mode", "depth")        # "depth" (ideal RGB-D) | "tof" (realistic USB+ToF)
        self.declare_parameter("v_gain", 0.445)               # m/s per unit forward command (MuJoCo calibration)
        self.declare_parameter("v_gain_rev", 0.59)            # MuJoCo calibration; moves only beyond -0.4
        self.declare_parameter("v_gain_lat", 0.0)             # asymmetric in MuJoCo -> not predicted
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("scan_matching", True)
        self.declare_parameter("scan_timeout", 0.75)          # acquisition age, not publication/receipt age
        self.declare_parameter("match_timeout", 1.5)
        self.declare_parameter("sweep_window", 1.0)           # short local constraints; NOT loop closure
        # depth mode matches over the sweep window too: single narrow camera frames let the +-30 cm correlative
        # search lock onto wrong alignments (ideal trial: 0.68 m mean error without the window, 0.09 m with it),
        # and the grid matcher rejects most single-wall frames as degenerate geometry
        self.declare_parameter("depth_sweep", True)
        self.declare_parameter("loop_closure", False)          # experimental bounded keyframe graph
        self.declare_parameter("graph_max_nodes", 160)
        self.declare_parameter("matcher", "correlative")      # correlative (Olson 2009) | grid (old +-7.5 cm search)
        # tuned offline on a recorded realistic run (tools/slam_replay.py); see results/slam_tuning/README.md
        self.declare_parameter("tof_min_hits", 8)             # ToF hits in the sweep window needed to match
        self.declare_parameter("match_gain", 0.6)             # fraction of each correction applied (damps aliasing)
        self.declare_parameter("fresh_check", False)          # also require the newest frame alone to agree
        self.declare_parameter("slip_estimation", True)       # scale gait odometry by observed/modelled motion
        self.declare_parameter("relocalize_after", 3.0)       # s without healthy localisation -> widen the search
        self.declare_parameter("fall_tilt_deg", 35.0)         # filtered tilt above this -> "recovering"
        self.declare_parameter("upright_tilt_deg", 15.0)      # ... back to "ok" after `upright_hold` s below this
        self.declare_parameter("upright_hold", 1.0)
        self.declare_parameter("target_radius", 0.0)          # m; 0.04 only for the known sim beacon cylinder
        self.declare_parameter("tof_range_convention", "radial")
        self.declare_parameter("tof_fov_deg", 45.0)
        self.declare_parameter("tof_max_range", 2.0)
        self.declare_parameter("det_wait", 1.0)
        self.declare_parameter("target_height", 0.0)          # m; known target height -> box-size depth (0 = off)
        self.declare_parameter("size_agreement", 0.3)         # ToF depth must be within 30% of the box-size depth
        self.declare_parameter("size_max_range", 2.5)         # m; beyond this a box-size-only range is not used
        self.declare_parameter("range_spread_max", 0.15)      # m; ToF zones in a box must agree this well
        self.declare_parameter("goal_confirmations", 2)       # consistent ranged sightings before /goal is published
        self.mode = str(self.get_parameter("sensor_mode").value)
        if self.mode not in ("depth", "tof"):
            raise ValueError("sensor_mode must be 'depth' or 'tof'")
        gains = ds.OdomGains(fwd=float(self.get_parameter("v_gain").value), rev=float(self.get_parameter("v_gain_rev").value),
                             lat=float(self.get_parameter("v_gain_lat").value))
        if self.mode == "tof":
            self.slam = ds.DuckSLAM(gains, max_r=float(self.get_parameter("tof_max_range").value), min_hits=int(self.get_parameter("tof_min_hits").value))
            fov = float(self.get_parameter("tof_fov_deg").value)
            self.tof_rays = ds.tof_zone_rays(8, fov, fov)
        else:
            self.slam = ds.DuckSLAM(gains)
        self.tilt = ds.TiltFilter()
        # matching / mapping / relocalisation / slip policy, shared with tools/slam_replay.py
        self.loc = ds.FrameLocalizer(
            self.slam, matcher=self.matcher_name(),
            loop_closure=bool(self.get_parameter("loop_closure").value),
            graph_max_nodes=int(self.get_parameter("graph_max_nodes").value),
            sweep=float(self.get_parameter("sweep_window").value),
            use_sweep=self.mode == "tof" or bool(self.get_parameter("depth_sweep").value),
            gain=float(self.get_parameter("match_gain").value), fresh=bool(self.get_parameter("fresh_check").value),
            slip=bool(self.get_parameter("slip_estimation").value),
            relocalize_after=float(self.get_parameter("relocalize_after").value), map_move=0.0, map_turn=0.0,
            scan_timeout=float(self.get_parameter("scan_timeout").value),
            match_timeout=float(self.get_parameter("match_timeout").value))
        self.health = self.loc.health
        self.last_frame_ns = -1
        self.cmd = (0.0, 0.0); self.cmd_t = -1e9; self.last_imu_t = None
        self.state = "ok"; self.upright_since = None
        self.recovery_started = None; self.recovery_warned = False
        self.map_revision_ns = -1
        self.K = None
        self.pose_hist = StampBuffer(200)                     # 4 s of 50 Hz poses
        self.cam_poses = StampBuffer(20); self.sensor_poses = StampBuffer(30)
        self.pending_frames = deque(maxlen=6)                 # sensor frames waiting for their pose
        self.frames = StampBuffer(15)                         # stamp -> (kind, data, slam pose at acquisition)
        self.pending_dets = deque(maxlen=20)
        self.goal = None; self.goal_n = 0; self.candidates = []
        self.pub_map = self.create_publisher(OccupancyGrid, "map", 1)
        self.pub_pose = self.create_publisher(PoseStamped, "slam/pose", 10)
        self.pub_state = self.create_publisher(String, "slam/state", 10)
        self.pub_match = self.create_publisher(String, "slam/match", 10)
        self.pub_revision = self.create_publisher(String, "slam/map_revision", 10)
        self.pub_goal = self.create_publisher(PointStamped, "goal", 10)
        self.pub_beacon = self.create_publisher(String, "slam/beacon", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, "imu", self.on_imu, 50)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_subscription(CameraInfo, "camera/camera_info", self.on_info, 2)
        self.create_subscription(PoseStamped, "camera/pose_level", self.on_cam_pose, 10)
        if self.mode == "depth":
            self.create_subscription(Image, "camera/depth", self.on_frame, 2)
        else:
            self.create_subscription(PoseStamped, "tof/pose_level", self.on_sensor_pose, 10)
            self.create_subscription(PointCloud2, "tof/zones", self.on_frame, 5)
        self.create_subscription(Detection2DArray, "yolo/detections", self.on_det, 5)
        self.create_timer(1.0, self.publish_map)
        self.create_timer(0.2, self.flush_detections)
        self.create_timer(0.2, self.publish_state)

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def publish_state(self):
        state = self.state if self.state != "ok" else self.health.state(self.now_ns() * 1e-9)
        self.pub_state.publish(String(data=state))

    # 1. predict (50 Hz) ------------------------------------------------------------------
    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y); self.cmd_t = self.now_ns() * 1e-9

    def on_imu(self, msg):
        t = stamp_ns(msg.header) * 1e-9
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        dt = t - self.last_imu_t if self.last_imu_t is not None else 0.0
        self.last_imu_t = t
        up = self.tilt.update(gyro, acc, dt if 0 < dt < 0.2 else 0.0)
        tilt = math.degrees(self.tilt.tilt())
        if self.state == "ok" and tilt > self.get_parameter("fall_tilt_deg").value:
            self.state = "recovering"; self.upright_since = None
            self.recovery_started = t; self.recovery_warned = False
            self.loc.reset_after_fall()
            self.get_logger().warn(f"tilt {tilt:.0f} deg: fall/recovery, map and translation updates suspended")
        elif self.state == "recovering":
            if not self.recovery_warned and self.recovery_started is not None and t - self.recovery_started >= 10.0:
                self.recovery_warned = True
                self.get_logger().warn(f"recovery persists for {t - self.recovery_started:.1f} s, tilt {tilt:.0f} deg; "
                                       "translation and mapping remain suspended until upright")
            if tilt < self.get_parameter("upright_tilt_deg").value:
                self.upright_since = self.upright_since or t
                if t - self.upright_since > self.get_parameter("upright_hold").value:
                    self.state = "ok"
                    self.get_logger().info("upright again: motion remains gated on accepted local scan constraints")
            else:
                self.upright_since = None
        self.publish_state()
        if 0 < dt < 0.2:
            # heading keeps integrating through a stumble (the tilt-corrected yaw rate stays valid); translation from
            # the gait command and map updates are suspended while recovering
            fresh = t - self.cmd_t < self.get_parameter("cmd_timeout").value and self.state == "ok"
            vx, vy = self.cmd if fresh else (0.0, 0.0)
            self.slam.predict(vx, ds.world_yaw_rate(gyro, up), dt, cmd_vy=vy)
        self.pose_hist.put(stamp_ns(msg.header), self.slam.pose.copy())
        x, y, th = self.slam.pose
        tf = TransformStamped(); tf.header.stamp = msg.header.stamp; tf.header.frame_id = "map"; tf.child_frame_id = "base_level"
        tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
        self.tf.sendTransform(tf)
        p = PoseStamped(); p.header.stamp = msg.header.stamp; p.header.frame_id = "map"
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        self.pub_pose.publish(p)

    # 2. sensor frames -------------------------------------------------------------------------
    def on_info(self, m):
        if m.k[0] > 0:
            self.K = ds.Intrinsics(m.width, m.height, m.k[0], m.k[4], m.k[2], m.k[5])

    @staticmethod
    def _pose(msg):
        o, p = msg.pose.orientation, msg.pose.position
        return quat_to_mat(o.w, o.x, o.y, o.z), np.array([p.x, p.y, p.z])

    def on_cam_pose(self, msg):
        self.cam_poses.put(stamp_ns(msg.header), self._pose(msg))
        if self.mode == "depth":
            self.retry_pending()

    def on_sensor_pose(self, msg):
        self.sensor_poses.put(stamp_ns(msg.header), self._pose(msg))
        self.retry_pending()

    def retry_pending(self):
        for _ in range(len(self.pending_frames)):
            self.on_frame(self.pending_frames.popleft())

    def on_frame(self, msg):
        ns = stamp_ns(msg.header)
        if ns <= self.last_frame_ns or not 0 <= (self.now_ns() - ns) * 1e-9 <= self.health.scan_timeout:
            return                                          # replay/late packets cannot refresh localization
        poses = self.cam_poses if self.mode == "depth" else self.sensor_poses
        sp = poses.get(ns)
        if sp is None:
            self.pending_frames.append(msg); return            # wait for the sensor pose with the same stamp
        if self.state != "ok":
            return
        hist_k, pose_acq = self.pose_hist.nearest(ns, 30_000_000)
        if pose_acq is None:
            return
        self.last_frame_ns = ns
        R, t = sp
        if self.mode == "depth":
            if self.K is None:
                return
            depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
            ang, r, hit = self.slam.scan_from_depth(depth, R, t, self.K)
            data = depth
        else:
            ranges, valid = self.read_tof(msg)
            ang, r, hit = self.slam.scan_from_tof(ranges, valid, self.tof_rays, R, t,
                                                  self.get_parameter("tof_range_convention").value,
                                                  fov_h_deg=float(self.get_parameter("tof_fov_deg").value))
            pts = ds.tof_points(ranges, valid, self.tof_rays, self.get_parameter("tof_range_convention").value)
            data = (pts @ R.T + t, ranges[valid] if len(pts) else np.zeros(0))   # level-frame points + ranges
        accepted = False
        if len(ang):
            current = self.slam.pose.copy()
            revision = self.loc.graph.revision if self.loc.graph is not None else 0
            accepted, corr = self.loc.step(ns * 1e-9, pose_acq, ang, r, hit,      # match + map at the acquisition pose
                                           match=bool(self.get_parameter("scan_matching").value))
            after = pose_acq + corr
            self.slam.pose = ds.carry_pose_correction(current, pose_acq, after)
            if np.any(corr):                                   # later history entries get it too, so a frame acquired
                for k in [k for k in self.pose_hist.d if k > ns]:     # before this one was processed is not
                    self.pose_hist.d[k] = ds.carry_pose_correction(self.pose_hist.d[k], pose_acq, after)
            pose_acq = after
            if self.loc.graph is not None and self.loc.graph.revision != revision:
                self.invalidate_map_caches()
        # an empty frame (no return within range) says nothing about localisation quality: not counted as a
        # failed match (counting it reset the health streak, stopped map updates and diverged in replay)
        reason = self.slam.match_reason if len(ang) else "empty_scan"
        if not self.get_parameter("scan_matching").value:
            reason = "matching_disabled; navigation gated"
        self.pub_match.publish(String(data=f"{reason}; overlap={self.slam.match_overlap:.2f}"
                                           + (f"; graph={self.loc.graph.reason}; revision={self.loc.graph.revision}"
                                              if self.loc.graph is not None else "")))
        self.publish_state()
        self.frames.put(ns, (data, pose_acq.copy()))

    def invalidate_map_caches(self):
        """Old observations/goals cannot be carried by a single nonrigid graph correction."""
        self.frames.d.clear(); self.pending_dets.clear()
        self.goal = None; self.goal_n = 0; self.candidates.clear()
        self.map_revision_ns = self.now_ns()
        event = f"{self.loc.graph.revision}:{self.map_revision_ns}"
        self.pub_revision.publish(String(data=event))
        self.publish_map()

    def matcher_name(self):
        m = self.get_parameter("matcher").value
        if m not in ("correlative", "grid"):
            raise ValueError("matcher must be 'correlative' or 'grid'")
        return m

    @staticmethod
    def read_tof(msg):
        from sensor_msgs_py import point_cloud2
        rec = point_cloud2.read_points(msg, field_names=("range", "status", "zone"), skip_nans=False)
        n = 64
        ranges = np.full(n, np.nan); valid = np.zeros(n, bool)
        for r, s, z in zip(rec["range"], rec["status"], rec["zone"]):
            z = int(z)
            if 0 <= z < n:
                ranges[z] = r; valid[z] = int(s) == 0 and np.isfinite(r)
        return ranges, valid

    # 3. beacon position from YOLO boxes + range --------------------------------------------
    def on_det(self, msg):
        if stamp_ns(msg.header) <= self.map_revision_ns:
            return
        if any(d.results and d.results[0].hypothesis.class_id == "beacon" for d in msg.detections):
            self.pending_dets.append(msg)
            self.flush_detections()

    def flush_detections(self):
        now = self.now_ns(); wait = int(self.get_parameter("det_wait").value * 1e9)
        keep = deque(maxlen=self.pending_dets.maxlen)
        while self.pending_dets:
            msg = self.pending_dets.popleft()
            ns = stamp_ns(msg.header)
            if self.process_det(msg, ns):
                continue
            if now - ns < wait:
                keep.append(msg)
            else:
                self.pub_beacon.publish(String(data="unranged: no matching sensor frame (expired)"))
        self.pending_dets = keep

    def process_det(self, msg, ns):
        """True when the detection could be evaluated (ranged or definitively unranged)."""
        if self.K is None:
            return False
        if self.mode == "depth":
            fr = self.frames.get(ns)
            if fr is None:
                return False
            cam = self.cam_poses.get(ns)
        else:
            k, fr = self.frames.nearest(ns, 150_000_000)        # ToF runs at its own rate: nearest within 150 ms
            if fr is None:
                newest = max(self.frames.d) if self.frames.d else -1
                return newest > ns + 150_000_000                # a later frame exists -> this one never will
            cam = self.cam_poses.get(ns)
        if cam is None:
            return False
        data, pose = fr
        R_bc, t_bc = cam
        if self.mode == "tof":
            _, pose_cam = self.pose_hist.nearest(ns, 30_000_000)
            if pose_cam is None:
                return False
            data = (self.shift_level(data[0], pose, pose_cam), data[1])   # ToF points into the camera-time body frame
            pose = pose_cam
        for det in msg.detections:
            if not det.results or det.results[0].hypothesis.class_id != "beacon":
                continue
            cx, cy, w, h = det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.size_x, det.bbox.size_y
            pb, why = (self.range_depth if self.mode == "depth" else self.range_tof)(data, cx, cy, w, h, R_bc, t_bc)
            if pb is None:
                self.pub_beacon.publish(String(data=f"unranged: {why}")); continue
            rad = float(self.get_parameter("target_radius").value)
            if rad > 0:                                          # surface -> centre of a known cylinder
                pb[:2] *= (np.hypot(*pb[:2]) + rad) / max(np.hypot(*pb[:2]), 1e-6)
            c, s = math.cos(pose[2]), math.sin(pose[2])
            g = np.array([pose[0] + c * pb[0] - s * pb[1], pose[1] + s * pb[0] + c * pb[1]])
            self.pub_beacon.publish(String(data=f"ranged ({g[0]:.2f}, {g[1]:.2f})"))
            self.accept(g)
        if self.goal is not None:
            p = PointStamped(); p.header.stamp = msg.header.stamp; p.header.frame_id = "map"
            p.point.x, p.point.y = float(self.goal[0]), float(self.goal[1])
            self.pub_goal.publish(p)
        return True

    def accept(self, g):
        """Temporal agreement: a goal is published only after `goal_confirmations` sightings within 0.3 m."""
        if self.goal is not None:
            if np.hypot(*(g - self.goal)) > 0.5:
                self.pub_beacon.publish(String(data="ranged sighting rejected: disagrees with the estimate by > 0.5 m"))
                return
            self.goal_n += 1
            self.goal = self.goal + (g - self.goal) / min(self.goal_n, 20)
            return
        self.candidates = [c for c in self.candidates if np.hypot(*(g - c)) < 0.3] + [g]
        if len(self.candidates) >= int(self.get_parameter("goal_confirmations").value):
            self.goal = np.mean(self.candidates, axis=0); self.goal_n = len(self.candidates)
            self.get_logger().info(f"beacon located by YOLO + {self.mode}: ({self.goal[0]:.2f}, {self.goal[1]:.2f}) in the map")

    @staticmethod
    def shift_level(pts, pose_from, pose_to):
        """Level-frame points seen at SLAM pose `pose_from` -> level frame at `pose_to` (planar motion between)."""
        if len(pts) == 0:
            return pts
        c, s = math.cos(pose_from[2]), math.sin(pose_from[2])
        wx = pose_from[0] + c * pts[:, 0] - s * pts[:, 1]; wy = pose_from[1] + s * pts[:, 0] + c * pts[:, 1]
        dx, dy = wx - pose_to[0], wy - pose_to[1]
        c, s = math.cos(pose_to[2]), math.sin(pose_to[2])
        return np.stack([c * dx + s * dy, -s * dx + c * dy, pts[:, 2]], 1)

    def range_depth(self, depth, cx, cy, w, h, R_bc, t_bc):
        K = self.K
        u0, u1 = int(max(0, cx - w / 4)), int(min(K.width, cx + w / 4 + 1))
        v0, v1 = int(max(0, cy - h / 4)), int(min(K.height, cy + h / 4 + 1))
        patch = depth[v0:v1, u0:u1]
        if patch.size == 0:
            return None, "empty box"
        z = float(np.median(patch))
        if not 0.1 < z < self.slam.max_r:
            return None, f"depth {z:.2f} m out of range"
        return ds.pixel_to_level(cx, cy, z, R_bc, t_bc, K), ""

    def size_depth(self, cy, h):
        """Camera-axis depth from the box height of a target of known height; None if unknown or cut off."""
        H = float(self.get_parameter("target_height").value)
        K = self.K
        if H <= 0 or h < 8 or cy - h / 2 <= 1 or cy + h / 2 >= K.height - 1:
            return None                                       # no known size, tiny box, or clipped by the image edge
        return K.fy * H / h

    def range_tof(self, data, cx, cy, w, h, R_bc, t_bc):
        """Depth from the ToF zones that project into the box, bearing from the camera.

        The 8x8 zones are 5.6 deg apart, so the zone centres place the target up to ~3 deg off sideways; the
        camera box centre is far more precise. So: camera ray through the box centre, at the ToF depth."""
        pts, rng = data
        zs = self.size_depth(cy, h)
        if len(pts):
            u, v, z = ds.level_to_pixel(pts, R_bc, t_bc, self.K)
            inside = (z > 0.05) & (np.abs(u - cx) <= w / 4 + 2) & (np.abs(v - cy) <= h / 4 + 2)
        else:
            inside = np.zeros(0, bool)
        if inside.any():
            r = rng[inside]
            if r.max() - r.min() > self.get_parameter("range_spread_max").value:
                return None, f"ToF zones in the box disagree ({r.min():.2f}-{r.max():.2f} m): object/background mix"
            zt = float(np.median(z[inside]))
            if zs is not None and abs(zt - zs) > float(self.get_parameter("size_agreement").value) * zs:
                return None, f"ToF depth {zt:.2f} m vs box-size depth {zs:.2f} m: ToF probably sees the background"
            return ds.pixel_to_level(cx, cy, zt, R_bc, t_bc, self.K), ""
        if zs is not None and 0.1 < zs < float(self.get_parameter("size_max_range").value):
            return ds.pixel_to_level(cx, cy, zs, R_bc, t_bc, self.K), ""   # no ToF zone on it: size only
        return None, "no valid ToF zone projects into the box" if len(pts) else "no valid ToF zones"

    def publish_map(self):
        info = self.slam.info
        g = OccupancyGrid(); g.header.stamp = self.get_clock().now().to_msg(); g.header.frame_id = "map"
        g.info.resolution = info.res; g.info.width, g.info.height = info.nx, info.ny
        g.info.origin.position.x, g.info.origin.position.y = info.x0, info.y0; g.info.origin.orientation.w = 1.0
        g.data = self.slam.occupancy().T.flatten().tolist()              # row-major, rows along y
        self.pub_map.publish(g)


def main():
    rclpy.init()
    node = SlamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:                                    # noqa: BLE001  torn down by a signal mid-wait
        if rclpy.ok():
            raise
    finally:
        node.destroy_node(); rclpy.try_shutdown()
