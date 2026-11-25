"""duckslam.py -- the DuckSLAM core (no ROS in here): predict, depth/ToF -> scan, scan matching, grid map,
the wavefront planner and the exploration-state check. Parameterised so the ROS nodes and unit tests share it.

Conventions
- Grids are indexed [ix, iy] (shape (nx, ny)); ROS OccupancyGrid data is row-major with rows along y,
  so convert with `grid_from_msg_data` / `.T.flatten()`.
- Cell values: -1 unknown, 0 free, 100 occupied (ROS convention).
- All map geometry comes from a `MapInfo`; the module-level constants are only the default map used by the
  simulated room. Nothing silently assumes them when a node receives a different map.
"""
import math
from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from .pose_graph import PoseGraph, carry_pose_correction


@dataclass(frozen=True)
class MapInfo:
    res: float = 0.025          # m per cell
    x0: float = -1.5            # world x of cell (0, 0)'s lower corner
    y0: float = -2.75
    nx: int = 260               # cells along x  (default map: x -1.5..5.0)
    ny: int = 220               # cells along y  (default map: y -2.75..2.75)

    def cell(self, x, y):
        """World (x, y) -> integer cell (ix, iy) using floor (no truncation towards zero)."""
        return int(math.floor((x - self.x0) / self.res)), int(math.floor((y - self.y0) / self.res))

    def inside(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny

    def center(self, i, j):
        return self.x0 + (i + 0.5) * self.res, self.y0 + (j + 0.5) * self.res


DEFAULT_MAP = MapInfo()
# Back-compat names for scripts that import them.
RES, X0, Y0, NX, NY = DEFAULT_MAP.res, DEFAULT_MAP.x0, DEFAULT_MAP.y0, DEFAULT_MAP.nx, DEFAULT_MAP.ny
MAX_R = 2.5                     # ideal depth camera: ranges used for mapping (m)


def cell(x, y, info=DEFAULT_MAP):
    return info.cell(x, y)


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole camera; fx/fy/cx/cy in pixels (from sensor_msgs/CameraInfo K)."""
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist: tuple = ()            # plumb_bob (k1, k2, p1, p2[, k3]) for the raw, unrectified image; () = ideal pinhole

    @staticmethod
    def from_fovy(width, height, fovy_deg):
        f = (height / 2) / math.tan(math.radians(fovy_deg) / 2)
        return Intrinsics(width, height, f, f, width / 2, height / 2)


SIM_CAMERA = Intrinsics.from_fovy(320, 240, 60.0)       # the simulated ideal camera only


def distort(xn, yn, dist):
    """Ideal normalised image coordinates (x right, y down) -> distorted, OpenCV plumb_bob model."""
    k1, k2, p1, p2, k3 = (tuple(dist) + (0.0,) * 5)[:5]
    r2 = xn * xn + yn * yn
    radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
    return (xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn),
            yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn)


def undistort(xd, yd, dist, iters=20):
    """Inverse of `distort` by fixed-point iteration (as cv2.undistortPoints); exact enough for moderate lenses."""
    xn, yn = xd, yd
    for _ in range(iters):
        dx, dy = distort(xn, yn, dist)
        xn, yn = xn + (xd - dx), yn + (yd - dy)
    return xn, yn


def pixel_to_level(u, v, z, R_bc, t_bc, K=SIM_CAMERA):
    """Pixel (u, v) of the raw image at camera-axis depth z -> point in the level body frame (MuJoCo camera: looks
    along -z, y up). Lens distortion in K.dist is removed first, so YOLO boxes from the raw image land correctly."""
    xn, yn = (u - K.cx) / K.fx, (v - K.cy) / K.fy
    if K.dist:
        xn, yn = undistort(xn, yn, K.dist)
    pc = np.array([xn * z, -yn * z, -z])
    return R_bc @ pc + t_bc


def level_to_pixel(p_level, R_bc, t_bc, K):
    """Points (N, 3) in the level body frame -> pixel (u, v) and camera-axis depth z; z <= 0 means behind."""
    pc = (np.atleast_2d(p_level) - t_bc) @ R_bc         # R_bc^T (p - t), row form
    z = -pc[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        xn, yn = pc[:, 0] / z, -pc[:, 1] / z
        if K.dist:
            xn, yn = distort(xn, yn, K.dist)
        u = K.cx + xn * K.fx
        v = K.cy + yn * K.fy
    return u, v, z


# ---------------------------------------------------------------- VL53L5CX-style multizone ToF -----------
def tof_zone_rays(n=8, fov_h_deg=45.0, fov_v_deg=45.0):
    """Unit centre-ray directions for an n x n zone sensor, in the sensor frame (+x boresight, +y left, +z up).
    Zone order is row-major starting top-left as seen from behind the sensor; this MUST be verified on the
    real part (ST's zone order depends on mounting and the driver) with left/right/up/down targets.
    Returns (n*n, 3). A centre ray per zone is a first approximation: a real zone integrates its whole cone."""
    ah = np.radians(fov_h_deg) / n
    av = np.radians(fov_v_deg) / n
    rays = []
    for r in range(n):                                  # r = 0 is the top row
        el = (n / 2 - 0.5 - r) * av
        for c in range(n):                              # c = 0 is the left column
            az = (n / 2 - 0.5 - c) * ah
            d = np.array([1.0, math.tan(az), math.tan(el)])
            rays.append(d / np.linalg.norm(d))
    return np.array(rays)


def tof_points(ranges, valid, rays, convention="radial"):
    """Zone ranges (m) -> points in the sensor frame. `convention`: 'radial' = distance along the zone ray,
    'axial' = distance along the boresight (x). Which one the driver reports must be checked on hardware."""
    r = np.asarray(ranges, float)
    if convention == "axial":
        r = r / np.maximum(rays[:, 0], 1e-6)
    pts = rays * r[:, None]
    return pts[np.asarray(valid, bool)]


# ---------------------------------------------------------------- SLAM core -----------------------------
@dataclass
class OdomGains:
    """Learned-gait command -> measured SI velocity, from ros2/duck_nav/tools/calibrate_odom.py in the open
    MuJoCo scene with the upstream velstand policy (results/post_fix/odom_calibration.txt). Simulation only:
    NOT valid for the real Feetech robot. Measured: forward 0.44-0.45 m/s per unit for 0.3-0.5; reverse only moves
    for commands beyond about -0.4 (0.59 m/s per unit at -0.5, nothing at -0.3); lateral is asymmetric
    (+0.3 moves at 0.18 m/s per unit, -0.3 does not move), so lateral is not predicted (gain 0)."""
    fwd: float = 0.445          # m/s per unit forward command
    rev: float = 0.59           # m/s per unit reverse command
    lat: float = 0.0            # m/s per unit lateral command (0 = not predicted)
    deadband: float = 0.25      # forward / lateral |command| below this: the policy stands still
    rev_deadband: float = 0.4   # reverse |command| below this: the policy stands still


def positive_finite(value, name):
    """Validate timing/speed bounds shared by the pure helpers and ROS nodes."""
    if isinstance(value, (bool, np.bool_)) or not math.isfinite(float(value)) or float(value) <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


class LocalizationHealth:
    """Acquisition-time health, independent of the rate of predicted pose publication.

    This is a conservative gate, not an accuracy estimate. Two accepted constraints
    are required after a failure; an old success cannot keep navigation enabled.
    """

    def __init__(self, scan_timeout=0.75, match_timeout=1.5):
        self.scan_timeout = positive_finite(scan_timeout, "scan_timeout")
        self.match_timeout = positive_finite(match_timeout, "match_timeout")
        self.last_scan = None; self.last_match = None; self.streak = 0

    def observe(self, stamp, accepted):
        if self.last_scan is not None and stamp <= self.last_scan:
            return
        if self.last_scan is not None and stamp - self.last_scan > self.scan_timeout:
            self.streak = 0
        self.last_scan = stamp
        self.streak = self.streak + 1 if accepted else 0
        if accepted:
            self.last_match = stamp

    def invalidate(self):
        self.last_match = None; self.streak = 0

    def state(self, now):
        if self.last_scan is None:
            return "initializing"
        if not 0 <= now - self.last_scan <= self.scan_timeout:
            return "stale"
        if self.last_match is None or not 0 <= now - self.last_match <= self.match_timeout or self.streak < 2:
            return "weak"
        return "ok"


class ScanWindow:
    """Short, odometry-compensated head sweep; never a loop closure or global relocalizer.

    Store endpoints in the prediction frame and re-express them at the newest pose.
    Bounded duration limits command-odometry error. Clear after a correction/fall.
    """

    def __init__(self, seconds=1.5, max_frames=20):
        self.seconds = positive_finite(seconds, "sweep_window")
        if isinstance(max_frames, bool) or not isinstance(max_frames, int) or max_frames <= 0:
            raise ValueError("max_frames must be a positive integer")
        self.frames = deque(maxlen=max_frames)

    def clear(self):
        self.frames.clear()

    def add(self, stamp, pose, ang, ranges, hit):
        if self.frames and stamp <= self.frames[-1][0]:
            return
        a = np.asarray(ang)[hit] + pose[2]; rr = np.asarray(ranges)[hit]
        points = np.column_stack((pose[0] + rr * np.cos(a), pose[1] + rr * np.sin(a)))
        self.frames.append((stamp, points))
        while self.frames and stamp - self.frames[0][0] > self.seconds:
            self.frames.popleft()

    def scan(self, pose):
        if not self.frames:
            return np.zeros(0), np.zeros(0), np.zeros(0, bool)
        pts = np.concatenate([p for _, p in self.frames], axis=0) - np.asarray(pose[:2])
        ang = np.arctan2(pts[:, 1], pts[:, 0]) - pose[2]
        return ang, np.linalg.norm(pts, axis=1), np.ones(len(pts), bool)

    def correct(self, before, after):
        """Carry an accepted rigid pose correction into all stored endpoints."""
        a = after[2] - before[2]; c, s = math.cos(a), math.sin(a)
        rot = np.array([[c, -s], [s, c]])
        self.frames = deque(((t, (p - before[:2]) @ rot.T + after[:2]) for t, p in self.frames),
                            maxlen=self.frames.maxlen)


class DuckSLAM:
    def __init__(self, gains=None, info=DEFAULT_MAP, max_r=MAX_R, min_hits=12):
        if not isinstance(gains, OdomGains):            # old call style: DuckSLAM(v_gain)
            gains = OdomGains(fwd=float(gains)) if gains is not None else OdomGains()
        self.gains = gains
        self.info = info
        self.max_r = max_r
        self.min_hits = min_hits        # scan hits needed before a scan match is attempted
        self.pose = np.array([0.0, 0.0, 0.0])
        self.logodds = np.zeros((info.nx, info.ny), np.float32)
        self.n_match = 0
        self.match_reason = "initializing"
        self.match_overlap = 0.0
        self.map_version = 0             # bumped by update_map; invalidates the cached likelihood tables
        self._tables = None
        self.match_cov = None            # 3x3 covariance of the last correlative match (x, y, theta)
        self.match_gap = None            # score gap to the best separate peak (when peak_margin > 0)
        self.last_offset = None          # measured (undamped) pose offset of the last applied match
        # online odometry calibration: slip = observed / commanded-model forward motion, from scan corrections
        self.slip = 1.0
        self._slip_num = self._slip_den = 0.0
        self._raw_fwd = 0.0              # model forward distance since the last slip update (unscaled, signed)
        # log-likelihood floor for returns landing in never-observed cells (None = same as known free space, the
        # clamp). A body-fixed sensor that turns keeps seeing just past the mapped end of a wall; scored like free
        # space, those returns pull the pose sideways onto the mapped end, the frame is mapped there, and the next
        # frame repeats it (replay: 5 mm per frame while turning in place).
        self.unknown_ll = None

    def body_velocity(self, cmd_vx, cmd_vy):
        g = self.gains
        vx = 0.0
        if cmd_vx >= g.deadband:
            vx = g.fwd * cmd_vx
        elif cmd_vx <= -g.rev_deadband:
            vx = g.rev * cmd_vx
        vy = g.lat * cmd_vy if abs(cmd_vy) >= g.deadband else 0.0
        return vx, vy

    def predict(self, cmd_vx, yaw_rate, dt, cmd_vy=0.0):
        """Dead-reckoning step. `yaw_rate` must be the WORLD yaw rate (see `world_yaw_rate`), not raw gyro z.
        Translation is scaled by the online slip estimate (see `observe_slip`)."""
        vx, vy = self.body_velocity(cmd_vx, cmd_vy)
        self._raw_fwd += vx * dt
        vx, vy = self.slip * vx, self.slip * vy
        self.pose[2] = (self.pose[2] + yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        self.pose[0] += (c * vx - s * vy) * dt
        self.pose[1] += (s * vx + c * vy) * dt

    def predict_measured(self, vx, vy, yaw_rate, dt):
        """Dead-reckoning step from a MEASURED body-frame velocity (m/s), e.g. camera floor flow. Same heading
        handling as `predict`, but no gait-command gain and no slip scaling (the measurement is already metric)."""
        self.pose[2] = (self.pose[2] + yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        self.pose[0] += (c * vx - s * vy) * dt
        self.pose[1] += (s * vx + c * vy) * dt

    @staticmethod
    def scan_from_points(pb, min_bin_points=1, bin_deg=2.0, bin_offset=0.0):
        """Points in the level body frame (N, 3) -> 2-D scan (bearing, range, hit) for mapping/matching.
        `min_bin_points` = 3 suits the dense depth image; sparse ToF zones use 1 (each zone is one measurement).
        Bins are centred on (k + bin_offset) * bin_deg."""
        if len(pb) == 0:
            return np.zeros(0), np.zeros(0), np.zeros(0, bool)
        rng = np.hypot(pb[:, 0], pb[:, 1]); bear = np.arctan2(pb[:, 1], pb[:, 0])
        obst = (pb[:, 2] > 0.03) & (pb[:, 2] < 0.30)
        floor = pb[:, 2] < 0.03
        bins = np.round(np.degrees(bear) / bin_deg - bin_offset).astype(int)
        ang, r, hit = [], [], []
        for b in np.unique(bins):
            sel = bins == b
            o = sel & obst
            if o.sum() >= min_bin_points:
                ang.append(b); r.append(float(np.percentile(rng[o], 10))); hit.append(True)
            else:
                fl = sel & floor
                if fl.sum() >= min_bin_points:
                    ang.append(b); r.append(float(rng[fl].max())); hit.append(False)
        return np.radians(bin_deg * (np.array(ang, float) + bin_offset)), np.array(r), np.array(hit, bool)

    def scan_from_depth(self, depth, R_bc, t_bc, K=SIM_CAMERA, stride=2):
        """Ideal dense depth image (camera-axis z, metres) -> scan."""
        d = depth[::stride, ::stride]
        u, v = np.meshgrid(np.arange(0, K.width, stride) + 0.5 - K.cx, np.arange(0, K.height, stride) + 0.5 - K.cy)
        ok = (d > 0.03) & (d < self.max_r)
        pc = np.stack([u * d / K.fx, -v * d / K.fy, -d], -1)[ok]
        return self.scan_from_points(pc @ R_bc.T + t_bc, min_bin_points=3)

    def scan_from_tof(self, ranges, valid, rays, R_bs, t_bs, convention="radial", fov_h_deg=45.0, n=8, points=None):
        """Sparse ToF zones -> scan, one bearing bin per zone column. Invalid / no-return zones are dropped:
        they are NOT free space. `points` (sensor-frame xyz of the valid zones, as the publisher computed them, e.g.
        from calibrated rays) replaces the reconstruction from `ranges` and the ideal `rays`."""
        ps = tof_points(ranges, valid, rays, convention) if points is None else np.asarray(points, float).reshape(-1, 3)
        ps = ps[np.linalg.norm(ps, axis=1) < self.max_r]
        return self.scan_from_points(ps @ R_bs.T + t_bs, min_bin_points=1, bin_deg=fov_h_deg / n,
                                     bin_offset=0.5 if n % 2 == 0 else 0.0)     # one bin per zone column

    def match(self, ang, r, hit, current_scan=None):
        info = self.info
        occ = (self.logodds > 0.8).astype(np.float32)
        self.match_overlap = 0.0
        if hit.sum() < self.min_hits:
            self.match_reason = "insufficient_hits"
            return False
        if occ.sum() < 80:
            self.match_reason = "map_not_ready"
            return False
        field = ndimage.gaussian_filter(occ, 1.6)
        px, py = r[hit] * np.cos(ang[hit]), r[hit] * np.sin(ang[hit])
        # A straight wall cannot constrain translation along itself. A large count
        # of nearly identical zones/sweep repeats must not masquerade as 2-D support.
        spread = np.linalg.eigvalsh(np.cov(np.vstack((px, py))))
        if spread[0] < 0.0025:
            self.match_reason = "degenerate_geometry"
            return False
        # All 9 x 7 x 7 candidate corrections scored in one array operation (same search, same tie-break
        # order as the original nested loop, ~30x faster: it matters at 10 Hz on the UNO Q's A53 cores).
        dths = np.radians(np.arange(-6, 6.1, 1.5)); dxy = np.arange(-0.075, 0.076, 0.025)
        th = self.pose[2] + dths
        wx = np.cos(th)[:, None] * px - np.sin(th)[:, None] * py                 # (T, N)
        wy = np.sin(th)[:, None] * px + np.cos(th)[:, None] * py
        ix = np.floor((self.pose[0] + dxy[None, :, None, None] + wx[:, None, None, :] - info.x0) / info.res).astype(int)
        iy = np.floor((self.pose[1] + dxy[None, None, :, None] + wy[:, None, None, :] - info.y0) / info.res).astype(int)
        ix, iy = np.broadcast_arrays(ix, iy)                                      # (T, X, Y, N)
        inside = (ix >= 0) & (ix < info.nx) & (iy >= 0) & (iy < info.ny)
        vals = np.where(inside, field[np.clip(ix, 0, info.nx - 1), np.clip(iy, 0, info.ny - 1)], 0.0)
        scores = vals.sum(-1) / len(px) - (0.02 * (np.abs(dxy)[None, :, None] + np.abs(dxy)[None, None, :]) / 0.075
                                           + 0.02 * np.abs(dths)[:, None, None] / math.radians(6))
        t_, x_, y_ = np.unravel_index(np.argmax(scores), scores.shape)
        best_s, best = float(scores[t_, x_, y_]), (float(dxy[x_]), float(dxy[y_]), float(dths[t_]))
        dx, dy, dth = best
        c, s = math.cos(self.pose[2] + dth), math.sin(self.pose[2] + dth)
        ix = np.floor((self.pose[0] + dx + c * px - s * py - info.x0) / info.res).astype(int)
        iy = np.floor((self.pose[1] + dy + s * px + c * py - info.y0) / info.res).astype(int)
        inside = (ix >= 0) & (ix < info.nx) & (iy >= 0) & (iy < info.ny)
        distance = ndimage.distance_transform_edt(~occ.astype(bool)) * info.res
        self.match_overlap = float((distance[ix[inside], iy[inside]] < 0.075).sum() / len(px))
        if current_scan is not None:
            # An old sweep agreeing with the map cannot conceal a new occlusion,
            # dropout, or displacement in the latest acquisition.
            aa, rr, hh = current_scan
            xx, yy = rr[hh] * np.cos(aa[hh]), rr[hh] * np.sin(aa[hh])
            ii = np.floor((self.pose[0] + dx + c * xx - s * yy - info.x0) / info.res).astype(int)
            jj = np.floor((self.pose[1] + dy + s * xx + c * yy - info.y0) / info.res).astype(int)
            ok = (ii >= 0) & (ii < info.nx) & (jj >= 0) & (jj < info.ny)
            overlap = float((distance[ii[ok], jj[ok]] < 0.075).sum() / max(len(xx), 1))
            if len(xx) < 4 or overlap < 0.7:
                self.match_reason = "fresh_scan_disagrees"
                return False
        if max(abs(dx), abs(dy)) >= 0.074 or abs(dth) >= math.radians(5.9):
            self.match_reason = "search_boundary"
            return False
        if best_s > 0.12 and self.match_overlap >= 0.7:
            self.pose += 0.6 * np.array(best)
            self.n_match += 1
            self.match_reason = "accepted"
            return True
        self.match_reason = "low_overlap"
        return False

    def observe_slip(self, decay=0.9, min_den=0.05, lo=0.0, hi=1.3, relax=0.02, max_fwd_std=0.04, leak=0.0):
        """Update the slip factor after a sensor frame. A walking robot that pushes against an obstacle keeps
        stepping, so the gait command says it moves while it does not (the gait version of wheel slip). The
        scan matcher then keeps pulling the pose back; the ratio of observed to modelled forward motion over
        the last ~1 s (exponential window, `decay` per frame) becomes the scale applied in `predict`.
        Call once per processed frame, after matching (uses last_offset when the match was applied)."""
        u = np.array([math.cos(self.pose[2]), math.sin(self.pose[2])])
        usable = self.last_offset is not None and (
            self.match_cov is None or float(u @ self.match_cov[:2, :2] @ u) <= max_fwd_std ** 2)
        if usable:                                       # only corrections that pin down the forward position
            observed = self.slip * self._raw_fwd + float(self.last_offset[:2] @ u)
            self._slip_num = decay * self._slip_num + observed * math.copysign(1.0, self._raw_fwd or 1.0)
            self._slip_den = decay * self._slip_den + abs(self._raw_fwd)
            self._raw_fwd = 0.0
            if self._slip_den > min_den:
                self.slip = float(np.clip(self._slip_num / self._slip_den, lo, hi))
        if abs(self._raw_fwd) < 1e-4:                    # standing still: drift back toward the model
            self.slip += relax * (1.0 - self.slip)
        self.slip += leak * (1.0 - self.slip)            # optional constant pull toward the model

    # ---- correlative scan matching (Olson, "Real-Time Correlative Scan Matching", ICRA 2009) ----------------
    def likelihood_tables(self, sigma=0.05, block=4):
        """Log-likelihood table of a return landing in each cell: its distance to the nearest occupied cell,
        blurred by sensor noise `sigma` and clamped at 2.5 sigma so one stray point cannot dominate. Also the
        block-max table, Lb[i, j] = max(L[i:i+block, j:j+block]), which upper-bounds the score of every fine
        translation inside a coarse block, so the coarse-to-fine search returns the brute-force optimum.
        Cached until the map changes."""
        key = (self.map_version, sigma, block, self.unknown_ll)
        if self._tables is not None and self._tables[0] == key:
            return self._tables[1:]
        occ = self.logodds > 0.8
        d = ndimage.distance_transform_edt(~occ) * self.info.res if occ.any() else np.full(occ.shape, 1e3)
        L = (-0.5 * np.minimum(d / sigma, 2.5) ** 2).astype(np.float32)
        if self.unknown_ll is not None:
            unknown = (self.logodds > -0.5) & ~occ
            L[unknown] = np.maximum(L[unknown], np.float32(self.unknown_ll))
        pad = np.pad(L, ((0, block - 1), (0, block - 1)), constant_values=L.min())
        Lb = np.lib.stride_tricks.sliding_window_view(pad, (block, block)).max(axis=(-1, -2))
        self._tables = (key, L, Lb, d)
        return L, Lb, d

    def match_correlative(self, ang, r, hit, current_scan=None, window=0.30, ang_window=math.radians(12),
                          ang_step=math.radians(1.0), sigma=0.05, prior_xy=0.20, prior_th=math.radians(8),
                          max_std_xy=0.06, max_std_th=math.radians(3.0), min_overlap=0.6, neff_cap=40, gain=1.0,
                          peak_margin=0.0, peak_radius=0.125, peak_ang=math.radians(5), peak_cap=8.0):
        """Exhaustive correlative match over +-`window` m and +-`ang_window`, with an exact two-level bound,
        a Gaussian odometry prior, and the match covariance from the posterior over the local grid (Olson eq. 3).

        True: constrained in every direction, pose moved to the posterior mean. Along a featureless wall only
        the well-constrained direction is corrected (reason "partial") and False is returned, so the motion
        gate stays conservative. Sets match_reason, match_overlap and match_cov."""
        info = self.info
        self.match_overlap = 0.0; self.match_cov = None; self.last_offset = None; self.match_gap = None
        if hit.sum() < self.min_hits:
            self.match_reason = "insufficient_hits"; return False
        if (self.logodds > 0.8).sum() < 80:
            self.match_reason = "map_not_ready"; return False
        L, Lb, dist = self.likelihood_tables(sigma)
        px, py = r[hit] * np.cos(ang[hit]), r[hit] * np.sin(ang[hit])
        n = len(px); w_pt = min(n, neff_cap) / n        # sweep repeats are correlated: cap the effective count
        res, B = info.res, 4
        W = int(round(window / res)); W += (-W) % B      # fine offsets -W..W-1 cells, in coarse blocks of B
        dth = np.arange(-ang_window, ang_window + 1e-9, ang_step)
        th = self.pose[2] + dth
        T = len(th)
        cx = (self.pose[0] + np.cos(th)[:, None] * px - np.sin(th)[:, None] * py - info.x0) / res
        cy = (self.pose[1] + np.sin(th)[:, None] * px + np.cos(th)[:, None] * py - info.y0) / res
        ix0, iy0 = np.floor(cx).astype(int), np.floor(cy).astype(int)            # (T, N)
        lo_val = float(L.min())

        def score(table, i, j):
            ok = (i >= 0) & (i < info.nx) & (j >= 0) & (j < info.ny)
            return np.where(ok, table[np.clip(i, 0, info.nx - 1), np.clip(j, 0, info.ny - 1)], lo_val).sum(-1) * w_pt

        def prior(ox, oy, d_th):
            return -0.5 * (((ox * res) ** 2 + (oy * res) ** 2) / prior_xy ** 2 + (d_th / prior_th) ** 2)

        # 1. coarse upper bound for every (theta, block x, block y)
        blocks = np.arange(-W, W, B)
        nb = len(blocks)
        ub = score(Lb, ix0[:, None, None, :] + blocks[None, :, None, None],
                   iy0[:, None, None, :] + blocks[None, None, :, None])                # (T, nb, nb)
        near = np.where(blocks < 0, np.minimum(blocks + B - 1, 0), blocks)               # offset closest to 0
        ub = ub + prior(near[None, :, None], near[None, None, :], dth[:, None, None])
        # 2. branch and bound: refine blocks in bound order until no block can beat the best fine score
        order = np.argsort(ub, axis=None)[::-1]
        best_s, best = -np.inf, None
        fine = np.arange(B)
        for flat in order:
            if ub.flat[flat] <= best_s:
                break
            t_i, bx, by = np.unravel_index(flat, ub.shape)
            ox = blocks[bx] + fine; oy = blocks[by] + fine
            v = score(L, ix0[t_i][None, None, :] + ox[:, None, None], iy0[t_i][None, None, :] + oy[None, :, None])
            v = v + prior(ox[:, None], oy[None, :], dth[t_i])
            k = np.unravel_index(np.argmax(v), v.shape)
            if v[k] > best_s:
                best_s, best = float(v[k]), (int(ox[k[0]]), int(oy[k[1]]), int(t_i))
        bx_, by_, bt = best
        if abs(bx_) >= W - 1 or by_ <= -W + 1 or by_ >= W - 1 or bt in (0, T - 1):
            self.match_reason = "search_boundary"; return False
        # 3. posterior over a local fine grid around the optimum -> mean and covariance
        lo = np.arange(-4, 5)
        ts = np.arange(max(bt - 3, 0), min(bt + 4, T))
        S = score(L, ix0[ts][:, None, None, :] + (bx_ + lo)[None, :, None, None],
                  iy0[ts][:, None, None, :] + (by_ + lo)[None, None, :, None])
        S = S + prior((bx_ + lo)[None, :, None], (by_ + lo)[None, None, :], dth[ts][:, None, None])
        p = np.exp(S - S.max()); p /= p.sum()
        X = np.stack(np.broadcast_arrays(dth[ts][:, None, None] * 0 + (bx_ + lo)[None, :, None] * res,
                                         dth[ts][:, None, None] * 0 + (by_ + lo)[None, None, :] * res,
                                         dth[ts][:, None, None] + 0 * lo[None, :, None]), -1).reshape(-1, 3)
        pf = p.reshape(-1)
        mean = pf @ X
        cov = (X - mean).T @ ((X - mean) * pf[:, None]) + np.diag([res ** 2 / 12, res ** 2 / 12, ang_step ** 2 / 12])
        self.match_cov = cov
        # fit quality at the optimum: fraction of points within 7.5 cm of a mapped wall
        c, s_ = math.cos(th[bt]), math.sin(th[bt])

        def overlap(xx, yy):
            gx = self.pose[0] + bx_ * res + c * xx - s_ * yy; gy = self.pose[1] + by_ * res + s_ * xx + c * yy
            gi = np.floor((gx - info.x0) / res).astype(int); gj = np.floor((gy - info.y0) / res).astype(int)
            ok = (gi >= 0) & (gi < info.nx) & (gj >= 0) & (gj < info.ny)
            return float((dist[gi[ok], gj[ok]] < 0.075).sum() / max(len(xx), 1))

        self.match_overlap = overlap(px, py)
        if self.match_overlap < min_overlap:
            self.match_reason = "low_overlap"; return False
        if current_scan is not None:                     # the newest frame alone must agree with the map too
            aa, rr, hh = current_scan
            xx, yy = rr[hh] * np.cos(aa[hh]), rr[hh] * np.sin(aa[hh])
            if len(xx) < 3 or overlap(xx, yy) < 0.6:
                self.match_reason = "fresh_scan_disagrees"; return False
        if math.sqrt(cov[2, 2]) > max_std_th:
            self.match_reason = "ambiguous_heading"; return False
        evals, evecs = np.linalg.eigh(cov[:2, :2])
        if peak_margin > 0 and math.sqrt(evals[0]) <= max_std_xy:
            # The covariance above only sees +-4 cells around the optimum, so a second, separate peak elsewhere in the
            # search window (a box edge or wall end seen 0.6 m along a wall) does not widen it. Find the best score
            # among candidates that differ from the optimum in a CONSTRAINED direction (both axes for a full match,
            # the across-wall axis for a partial one, whose along-wall ridge is expected) or in heading.
            v_ex = None if math.sqrt(evals[1]) <= max_std_xy else evecs[:, 0]
            self.match_gap = self._second_peak_gap(ub, blocks, B, best_s, (bx_, by_, bt), score, prior, L, ix0, iy0,
                                                   dth, v_ex, peak_radius, peak_ang, peak_cap)
            if self.match_gap < peak_margin:
                self.match_reason = "second_peak"; return False
        if math.sqrt(evals[1]) <= max_std_xy:
            self.pose += gain * mean; self.n_match += 1; self.last_offset = mean
            self.match_reason = "accepted"; return True
        if math.sqrt(evals[0]) <= max_std_xy:            # e.g. along a wall: correct across it and heading only
            v = evecs[:, 0]
            self.pose[:2] += gain * v * (v @ mean[:2]); self.pose[2] += gain * mean[2]
            self.last_offset = np.array([*(v * (v @ mean[:2])), mean[2]])
            self.match_reason = "partial"; return False
        self.match_reason = "ambiguous"; return False

    def _second_peak_gap(self, ub, blocks, B, best_s, best, score, prior, L, ix0, iy0, dth, v_ex, radius, ang, cap):
        """Score gap (log-likelihood units, capped at `cap`) between the optimum and the best candidate outside its
        neighbourhood: farther than `ang` in heading, or farther than `radius` m in position (along `v_ex` only
        when given). Exact, by the same block bounds as the main branch and bound."""
        res = self.info.res
        bx_, by_, bt = best
        second = best_s - cap
        fine = np.arange(B)
        for flat in np.argsort(ub, axis=None)[::-1]:
            if ub.flat[flat] <= second:
                break
            t_i, bx, by = np.unravel_index(flat, ub.shape)
            ox = blocks[bx] + fine; oy = blocks[by] + fine
            dx = (ox[:, None] - bx_) * res; dy = (oy[None, :] - by_) * res
            if abs(dth[t_i] - dth[bt]) > ang:
                keep = np.ones((B, B), bool)
            elif v_ex is None:
                keep = np.hypot(dx, dy) > radius
            else:
                keep = np.abs(v_ex[0] * dx + v_ex[1] * dy) > radius
            if not keep.any():
                continue
            v = score(L, ix0[t_i][None, None, :] + ox[:, None, None], iy0[t_i][None, None, :] + oy[None, :, None])
            v = v + prior(ox[:, None], oy[None, :], dth[t_i])
            second = max(second, float(v[keep].max()))
        return best_s - second

    def update_map(self, ang, r, hit, free_weight=0.35, hit_weight=1.2):
        self.map_version += 1
        info = self.info
        x, y, th = self.pose
        steps = np.arange(info.res, self.max_r, info.res)
        for a, rr, h in zip(ang, r, hit):
            ca, sa = math.cos(th + a), math.sin(th + a)
            s = steps[steps < rr - info.res]
            ix = np.floor((x + ca * s - info.x0) / info.res).astype(int)
            iy = np.floor((y + sa * s - info.y0) / info.res).astype(int)
            k = (ix >= 0) & (ix < info.nx) & (iy >= 0) & (iy < info.ny)
            self.logodds[ix[k], iy[k]] -= free_weight
            if h:
                i, j = info.cell(x + ca * rr, y + sa * rr)
                if info.inside(i, j):
                    self.logodds[i, j] += hit_weight
        np.clip(self.logodds, -4, 6, out=self.logodds)

    def occupancy(self):
        """int8 grid in ROS convention: -1 unknown, 0 free, 100 occupied. Shape (nx, ny)."""
        g = np.full((self.info.nx, self.info.ny), -1, np.int8)
        g[self.logodds < -0.5] = 0
        g[self.logodds > 0.8] = 100
        return g



class FrameLocalizer:
    """Per-frame scan-matching and mapping policy, shared by slam_node and tools/slam_replay.py.

    `step` is called with the robot pose at the frame's acquisition time; it matches, decides whether to map,
    and returns (accepted, correction). The caller carries the correction to the current pose.
      - matching: the correlative matcher (default) or the old grid search, over a short sweep window
      - relocalisation: after `relocalize_after` s without a healthy state, the search widens to
        +-`reloc_window` m / +-`reloc_ang` rad with a weak prior (one bad step must not be permanent)
      - mapping: while bootstrapping (until localisation is first healthy), while the map is too small to
        match against, and afterwards on accepted/healthy frames only once the body has moved `map_move` m
        or turned `map_turn` rad since the last map update (GMapping-style keyframes; standing still adds no
        information and lets map and pose creep together)
      - slip: online odometry scale from forward corrections (DuckSLAM.observe_slip)
    """

    def __init__(self, slam, matcher="correlative", sweep=1.0, gain=0.6, fresh=False, slip=True,
                 relocalize_after=3.0, reloc_window=0.8, reloc_ang=math.radians(25), map_move=0.03,
                 map_turn=math.radians(3), scan_timeout=0.75, match_timeout=1.5, use_sweep=True,
                 loop_closure=False, graph_max_nodes=160, reloc_confirm=False, confirm_xy=0.06,
                 confirm_th=math.radians(4), bootstrap_turn=0.0, map_max_std=0.0, map_accepted_only=False,
                 reloc_partial=True, unknown_ll=None, confirm_free_xy=0.0, confirm_free_th=0.0, match_kw=None,
                 partial_tracks=False, map_min_gap=0.0, reloc_peak_margin=0.0):
        self.slam = slam
        # map_min_gap (log-likelihood units, 0 = off): after bootstrap, map a frame only when its match was accepted
        # with at least this score gap to any separate peak (DuckSLAM.match_correlative peak test). An aliased match
        # may still correct the pose (it is damped and later matches can undo it), but it must not become map evidence:
        # once mapped, the next frames match the wrong map and the error is permanent.
        self.map_min_gap = map_min_gap
        # reloc_peak_margin (0 = off): while relocalising (wide window, weak prior), reject a match whose best separate
        # peak scores within this of the optimum. Normal tracking keeps its small damped corrections: rejecting them
        # too cost 10-20 % of the time the motion gate allowed driving in replay, and the duck stood still.
        self.reloc_peak_margin = reloc_peak_margin
        # partial_tracks: a partial match (pose fixed across a wall and in heading, free along it) restarts the
        # relocalisation timer. Facing one wall is under-determined, not lost; the wide relocalisation search with its
        # weak prior is exactly where such a view aliases (replay v4snew_s5: a confirmed 0.6 m jump along a wall after
        # 50 s of harmless partial matches).
        self.partial_tracks = partial_tracks
        self.match_kw = dict(match_kw or {})     # extra DuckSLAM.match_correlative keywords (e.g. peak_margin)
        if map_min_gap > 0 and not self.match_kw.get("peak_margin"):
            self.match_kw["peak_margin"] = 1e-9          # compute the gap; reject only exact ties
        slam.unknown_ll = unknown_ll
        # map_max_std (m, 0 = off): after bootstrap, map a frame only when its match was accepted with the larger
        # position std of the match covariance at most this. Each accepted match moves the pose by the mean of a
        # posterior that may be several cm wide; mapping the frame at that pose writes the error into the map, and the
        # next frame matches the shifted map (map and pose ratchet together: body-sensor replay, 5 mm per frame).
        # map_accepted_only: do not also map frames that were not accepted while the health state was still "ok".
        # reloc_partial=False: a partial (one-direction + heading) match while relocalising is not applied; with the
        # wide relocalisation prior it swung the heading by 10-30 deg per frame without any confirmation.
        self.map_max_std, self.map_accepted_only, self.reloc_partial = map_max_std, map_accepted_only, reloc_partial
        # bootstrap_turn (rad): stay in bootstrap mapping (every frame is mapped) until the body has also turned this
        # much. A body-fixed sensor has no head sweep, so the start-up map is built by turning in place; after
        # bootstrap only accepted frames are mapped, and a partly new view is rarely accepted.
        self.bootstrap_turn, self.turned, self.last_heading = bootstrap_turn, 0.0, None
        # reloc_confirm: a relocalisation correction is applied only when the next relocalisation match proposes
        # the same correction (within confirm_xy m / confirm_th rad); one confident wrong jump is the failure mode
        self.reloc_confirm, self.confirm_xy, self.confirm_th = reloc_confirm, confirm_xy, confirm_th
        # a relocalisation match whose correction is within confirm_free_xy m / confirm_free_th rad is applied without
        # confirmation (it is no jump: the pose was already right). Holding those too reset the accepted streak on every
        # other frame, so health never returned to "ok" and the search stayed wide (body-sensor replay: 85 % of frames
        # relocalising at 0.2 m error).
        self.confirm_free_xy, self.confirm_free_th = confirm_free_xy, confirm_free_th
        self.pending_reloc = None
        self.matcher, self.gain, self.fresh, self.slip = matcher, gain, fresh, slip
        self.relocalize_after, self.reloc_window, self.reloc_ang = relocalize_after, reloc_window, reloc_ang
        self.map_move, self.map_turn = map_move, map_turn
        self.health = LocalizationHealth(scan_timeout, match_timeout)
        self.sweep = ScanWindow(sweep) if use_sweep and sweep > 0 else None
        self.bootstrap = True
        self.last_ok = None              # last time the health state was "ok"
        self.last_map_pose = None
        self.relocalizing = False
        if loop_closure and np.any(slam.logodds):
            raise ValueError("loop_closure requires an empty initial map with retained raw scans")
        self.graph = PoseGraph(graph_max_nodes) if loop_closure else None
        if self.graph is not None:
            self.map_move = max(self.map_move, 0.15)
            self.map_turn = max(self.map_turn, math.radians(15))

    def reset_after_fall(self):
        self.health.invalidate()
        if self.graph is not None:
            self.graph.pending = None
        if self.sweep is not None:
            self.sweep.clear()

    def step(self, ts, pose_acq, ang, r, hit, match=True):
        slam = self.slam
        slam.pose = pose_acq.copy()
        if self.sweep is not None:
            self.sweep.add(ts, pose_acq, ang, r, hit)
            cons = self.sweep.scan(pose_acq)
        else:
            cons = (ang, r, hit)
        accepted = False
        slam.last_offset = None
        if match:
            fresh = (ang, r, hit) if self.fresh else None
            since_ok = ts - (self.last_ok if self.last_ok is not None else ts)
            self.relocalizing = not self.bootstrap and since_ok > self.relocalize_after
            if self.matcher == "correlative":
                kw = dict(window=self.reloc_window, ang_window=self.reloc_ang, prior_xy=1.0, prior_th=0.6) \
                    if self.relocalizing else {}
                if self.relocalizing and self.reloc_peak_margin > 0:
                    kw["peak_margin"] = self.reloc_peak_margin
                before = slam.pose.copy()
                accepted = slam.match_correlative(*cons, current_scan=fresh, gain=1.0 if self.relocalizing else self.gain,
                                                  **{**self.match_kw, **kw})
                if self.relocalizing and not self.reloc_partial and slam.match_reason == "partial":
                    slam.pose = before; slam.last_offset = None
                if not self.relocalizing:
                    self.pending_reloc = None
                elif accepted and self.reloc_confirm:
                    delta = slam.pose - before
                    delta[2] = (delta[2] + math.pi) % (2 * math.pi) - math.pi
                    p = self.pending_reloc           # (delta, time); consecutive sweeps share frames: need >= 0.5 s
                    if p is not None and not 0.5 <= ts - p[1] <= 3.0:
                        p = None if ts - p[1] > 3.0 else p
                    small = math.hypot(*delta[:2]) <= self.confirm_free_xy and abs(delta[2]) <= self.confirm_free_th
                    agree = small or p is not None and ts - p[1] >= 0.5 and math.hypot(*(delta[:2] - p[0][:2])) <= self.confirm_xy \
                        and abs((delta[2] - p[0][2] + math.pi) % (2 * math.pi) - math.pi) <= self.confirm_th
                    if not agree:                    # hold it: wait for an independent confirmation
                        if p is None or ts - p[1] >= 0.5:
                            self.pending_reloc = (delta, ts)
                        slam.pose = before; slam.last_offset = None
                        slam.match_reason = "reloc_unconfirmed"; accepted = False
                    else:
                        self.pending_reloc = None
            else:
                accepted = slam.match(*cons, current_scan=fresh)
        if accepted and self.sweep is not None:
            self.sweep.correct(pose_acq, slam.pose)
        # a frame rejected because it may be wrong (not merely uninformative) never enters the map on the strength of
        # an earlier healthy state: an ambiguous second peak or an unconfirmed relocalisation is exactly such a frame
        healthy = self.health.state(ts) == "ok" and slam.match_reason not in (
            "fresh_scan_disagrees", "search_boundary", "ambiguous", "ambiguous_heading", "second_peak",
            "reloc_unconfirmed")
        moved = self.last_map_pose is None or (
            math.hypot(*(slam.pose[:2] - self.last_map_pose[:2])) >= self.map_move
            or abs((slam.pose[2] - self.last_map_pose[2] + math.pi) % (2 * math.pi) - math.pi) >= self.map_turn)
        good = accepted or (healthy and not self.map_accepted_only)
        if good and self.map_max_std > 0 and not self.bootstrap:
            cov = slam.match_cov
            good = accepted and cov is not None and math.sqrt(float(np.linalg.eigvalsh(cov[:2, :2])[1])) <= self.map_max_std
        if good and self.map_min_gap > 0 and not self.bootstrap:
            good = accepted and slam.match_gap is not None and slam.match_gap >= self.map_min_gap
        should_map = self.bootstrap or slam.match_reason == "map_not_ready" or (good and moved)
        if self.graph is not None:
            # The graph owns every mapped scan: rebuilding must never discard untracked evidence.
            should_map = should_map and (moved or self.bootstrap or slam.match_reason == "map_not_ready")
            if self.graph.full:
                self.graph.reason = "disabled_capacity"
        if should_map:
            if self.graph is None or self.graph.full:
                # Once saturated, continue mapping but permanently stop graph additions/rebuilds.
                slam.update_map(ang, r, hit)
            else:
                before = slam.pose.copy()
                closure_scan = self.sweep.scan(before) if self.sweep is not None else (ang, r, hit)
                closed = self.graph.add(ts, before, (ang, r, hit), closure_scan, allow_closure=match)
                if closed:
                    after = self.graph.nodes[-1].pose.copy()
                    # Reintegrate every retained raw acquisition at its optimized keyframe pose.
                    slam.logodds.fill(0)
                    slam.map_version += 1; slam._tables = None
                    for node in self.graph.nodes:
                        slam.pose = node.pose.copy()
                        slam.update_map(*node.scan)
                    slam.pose = after
                    self.health.invalidate()
                    if self.sweep is not None:
                        self.sweep.clear()
                    slam.last_offset = None  # a graph correction is not evidence of gait slip
                else:
                    slam.update_map(ang, r, hit)
            self.last_map_pose = slam.pose.copy()
        self.health.observe(ts, accepted)
        if self.last_heading is not None:
            self.turned += abs((pose_acq[2] - self.last_heading + math.pi) % (2 * math.pi) - math.pi)
        self.last_heading = pose_acq[2]
        if self.health.state(ts) == "ok":
            self.last_ok = ts
            if self.turned >= self.bootstrap_turn:
                self.bootstrap = False
        elif self.last_ok is None and not self.bootstrap:
            self.last_ok = ts
        elif self.partial_tracks and not self.relocalizing and slam.match_reason == "partial":
            self.last_ok = ts            # constrained across a wall: under-determined, not lost (no wide search)
        if self.slip:
            slam.observe_slip()
        corr = slam.pose - pose_acq
        corr[2] = (corr[2] + math.pi) % (2 * math.pi) - math.pi
        return accepted, corr


def world_yaw_rate(gyro_body, up_body):
    """World yaw rate from the body gyro and the world 'up' direction expressed in the body frame
    (from a filtered attitude estimate). Equals gyro z only when the body is level."""
    u = np.asarray(up_body, float)
    n = np.linalg.norm(u)
    return float(np.dot(gyro_body, u / n)) if n > 1e-6 else 0.0


class TiltFilter:
    """Complementary attitude filter: tracks the world 'up' vector in the body frame. The accelerometer is
    trusted only when its magnitude is near 1 g (dynamic motion corrupts it as a gravity observation)."""

    def __init__(self, tau=0.5, g=9.81, accel_band=0.15):
        self.up = np.array([0.0, 0.0, 1.0]); self.tau = tau; self.g = g; self.band = accel_band
        self.initialised = False

    def update(self, gyro, acc, dt):
        acc = np.asarray(acc, float); gyro = np.asarray(gyro, float)
        an = np.linalg.norm(acc)
        if not self.initialised:
            if an > 1e-3:
                self.up = acc / an; self.initialised = True
            return self.up
        # a fixed world direction seen from the rotating body:  du/dt = -w x u
        self.up = self.up - np.cross(gyro, self.up) * dt
        self.up /= np.linalg.norm(self.up)
        if abs(an - self.g) < self.band * self.g:
            a = dt / (self.tau + dt)
            self.up = (1 - a) * self.up + a * acc / an
            self.up /= np.linalg.norm(self.up)
        return self.up

    def tilt(self):
        return math.acos(float(np.clip(self.up[2], -1.0, 1.0)))

    def heading_rate(self, gyro, dt):
        """World yaw rate for odometry: the gyro projected on the filtered 'up' (planar approximation)."""
        return world_yaw_rate(gyro, self.up)


def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class AttitudeFilter:
    """Full 3-D attitude (body -> world quaternion, Mahony-style complementary filter) with gyro bias estimation.

    Replaces TiltFilter + world_yaw_rate for heading: integrating the whole rotation keeps yaw correct while
    the body rolls and pitches through the gait, where projecting the gyro on a filtered 'up' vector drifts
    (recorded realistic runs: 14-26 deg over 200-600 s with a perfect simulated gyro). Same `up`/`tilt()`
    interface as TiltFilter; `yaw()` is the heading to feed odometry.
      - roll/pitch: corrected towards the accelerometer only when |a| is within `accel_band` of 1 g
        (gain `kp`); the roll/pitch gyro bias is learned from the same error (gain `ki`)
      - yaw bias is not observable from gravity, and it cannot be learned while the walking policy runs:
        the torso keeps rotating slowly (~1 deg/s) even when commanded to stand, which an online
        zero-velocity update mistakes for bias (it learned 0.016 rad/s on a bias-free recording). Measure
        it with `calibrate()` while the robot is held still before walking (e.g. at power-up on a stand).
    Yaw itself is never corrected here; scan matching does that."""

    def __init__(self, kp=1.0, ki=0.02, g=9.81, accel_band=0.15, max_bias=0.1):
        self.q = np.array([1.0, 0.0, 0.0, 0.0]); self.bias = np.zeros(3)
        self.kp, self.ki, self.g, self.band, self.max_bias = kp, ki, g, accel_band, max_bias
        self.initialised = False; self._dyaw = 0.0

    def calibrate(self, gyro_samples, max_std=0.01):
        """Set the gyro bias from samples taken while the body is held still. Returns False (bias unchanged)
        if the samples are too noisy to be a still period."""
        g = np.asarray(gyro_samples, float).reshape(-1, 3)
        if len(g) < 10 or float(g.std(axis=0).max()) > max_std:
            return False
        self.bias = np.clip(g.mean(axis=0), -self.max_bias, self.max_bias)
        return True

    @property
    def up(self):
        """World 'up' expressed in the body frame (rotate (0,0,1) by the inverse attitude)."""
        w, x, y, z = self.q
        return np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])

    def tilt(self):
        return math.acos(float(np.clip(self.up[2], -1.0, 1.0)))

    def yaw(self):
        return quat_yaw(self.q)

    def heading_rate(self, gyro, dt):
        """World yaw rate for odometry: the yaw change of the last `update` over its dt."""
        return self._dyaw / dt if dt > 0 else 0.0

    def update(self, gyro, acc, dt):
        gyro = np.asarray(gyro, float); acc = np.asarray(acc, float)
        an = np.linalg.norm(acc)
        if not self.initialised:
            if an > 1e-3:                                 # start level with the measured gravity, yaw 0
                u = acc / an
                axis = np.cross(u, [0.0, 0.0, 1.0]); s = np.linalg.norm(axis); c = float(u[2])
                if s > 1e-9:
                    ang = math.atan2(s, c); axis = axis / s
                    self.q = np.array([math.cos(ang / 2), *(math.sin(ang / 2) * axis)])
                self.initialised = True
            return self.up
        self._dyaw = 0.0
        if dt <= 0:
            return self.up
        yaw0 = quat_yaw(self.q)
        w = gyro - self.bias
        if abs(an - self.g) < self.band * self.g:
            e = np.cross(acc / an, self.up)               # rotation taking the estimate onto the measurement
            w = w + self.kp * e
            self.bias[:2] = np.clip(self.bias[:2] - self.ki * e[:2] * dt, -self.max_bias, self.max_bias)
        dq = np.array([1.0, *(0.5 * w * dt)])
        self.q = quat_mul(self.q, dq); self.q /= np.linalg.norm(self.q)
        self._dyaw = (quat_yaw(self.q) - yaw0 + math.pi) % (2 * math.pi) - math.pi
        return self.up


def make_attitude_filter(mode="quaternion"):
    """'quaternion' (AttitudeFilter, default) or 'tilt' (the older TiltFilter + world_yaw_rate)."""
    if mode == "quaternion":
        return AttitudeFilter()
    if mode == "tilt":
        return TiltFilter()
    raise ValueError("attitude_filter must be 'quaternion' or 'tilt'")


# ---------------------------------------------------------------- planning -----------------------------
ROBOT_RADIUS = 0.125            # m. Default clearance; NOT verified against the real envelope (sway, head, shield).

STATUS_TEXT = {
    "goal": "heading for the beacon",
    "explore": "exploring",
    "escape": "inside the clearance margin: backing out along known-free cells",
    "start_blocked": "robot cell is mapped occupied or boxed in: stopping",
    "out_of_map": "robot is outside the map: stopping",
    "no_frontier": "no reachable frontier",
}


def grid_from_msg_data(data, width, height):
    """ROS OccupancyGrid data (row-major, rows along y) -> (nx, ny) array."""
    return np.asarray(data, np.int8).reshape(height, width).T


def _clearance_m(grid, info):
    occ = grid == 100
    if occ.any():
        return occ, ndimage.distance_transform_edt(~occ) * info.res
    return occ, np.full(grid.shape, np.inf)


def motion_clear(grid, pose, speed, yaw_rate, horizon=0.6, info=DEFAULT_MAP,
                 clearance=ROBOT_RADIUS, escape=False, distances=None):
    """Check the commanded arc against known-free cells and the footprint margin.

    `speed` is a conservative measured SI speed bound, not a gait command. Escape
    may start within the margin but must never reduce its existing clearance.
    This sampled check is a local guard, not a dynamics or braking guarantee.
    """
    speed = positive_finite(speed, "speed_bound_mps")
    horizon = positive_finite(horizon, "motion horizon")
    if not math.isfinite(yaw_rate):
        raise ValueError("yaw_rate must be finite")
    if distances is None:
        _, distances = _clearance_m(grid, info)
    start = info.cell(*pose[:2])
    if not info.inside(*start) or grid[start] == 100:
        return False
    required = min(clearance, distances[start]) if escape else clearance
    steps = max(2, int(math.ceil(abs(speed) * horizon / (info.res / 3))))
    for t in np.linspace(0, horizon, steps + 1)[1:]:
        ij = info.cell(*arc_end(pose, speed, yaw_rate, t))
        if not info.inside(*ij) or grid[ij] != 0 or distances[ij] + 1e-9 < required:
            return False
        if escape:
            required = min(clearance, distances[ij])
    return True


def path_target(path, pose, lookahead=0.22):
    """Walk forward from the nearest path point, never chase the old start behind us."""
    nearest = _nearest_index(path, pose)
    for p in path[nearest:]:
        if math.hypot(p[0] - pose[0], p[1] - pose[1]) >= lookahead:
            return p
    return path[-1]


def _nearest_index(path, pose, start=0, stop=None):
    seg = path[start:stop]
    return start + min(range(len(seg)), key=lambda i: math.hypot(seg[i][0] - pose[0], seg[i][1] - pose[1]))


def path_turn_ahead(path, pose, span=0.35):
    """Heading change (rad) of the path over the `span` m ahead of the point nearest the robot (between the chords of
    its first and second halves)."""
    i0 = _nearest_index(path, pose)
    pts = [path[i0]]
    run = 0.0
    for p in path[i0 + 1:]:
        run += math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]); pts.append(p)
        if run >= span:
            break
    if len(pts) < 3:
        return 0.0
    m = len(pts) // 2                                    # two half chords: the staircase of grid steps averages out
    h1 = math.atan2(pts[m][1] - pts[0][1], pts[m][0] - pts[0][0])
    h2 = math.atan2(pts[-1][1] - pts[m][1], pts[-1][0] - pts[m][0])
    return abs((h2 - h1 + math.pi) % (2 * math.pi) - math.pi)


def adaptive_lookahead(path, pose, info=DEFAULT_MAP, distances=None, clearance=ROBOT_RADIUS,
                       base=0.22, minimum=0.10, comfort=0.08):
    """Pure-pursuit lookahead that shrinks at corners and next to obstacles, so the chord the robot aims along stays
    close to the clearance-respecting path. Returns (lookahead m, path turn ahead rad)."""
    turn = path_turn_ahead(path, pose)
    la = base - (base - minimum) * min(1.0, turn / (math.pi / 2))
    if distances is not None:
        ij = info.cell(*pose[:2])
        if info.inside(*ij):
            room = min(1.0, max(0.0, (float(distances[ij]) - clearance) / comfort))  # 0 at the margin, 1 with room
            la = min(la, minimum + (base - minimum) * room)
    return la, turn


def arc_end(pose, speed, yaw_rate, t):
    """Position after `t` s on the constant speed / turn-rate arc from `pose`."""
    if abs(yaw_rate) < 1e-6:
        return pose[0] + speed * t * math.cos(pose[2]), pose[1] + speed * t * math.sin(pose[2])
    return (pose[0] + speed / yaw_rate * (math.sin(pose[2] + yaw_rate * t) - math.sin(pose[2])),
            pose[1] - speed / yaw_rate * (math.cos(pose[2] + yaw_rate * t) - math.cos(pose[2])))


def choose_arc(grid, pose, path, speed, yaw_rates, preferred=0.0, horizon=0.6, info=DEFAULT_MAP,
               clearance=ROBOT_RADIUS, escape=False, distances=None, min_progress=0.03):
    """Pick the turn rate whose arc (at the conservative speed bound `speed`) passes `motion_clear` and makes the
    most progress along `path`: arc length up to the path point nearest the arc's end, minus the cross-track
    distance there. Returns the turn rate, or None when no clear arc gains `min_progress` m along the path."""
    if distances is None:
        _, distances = _clearance_m(grid, info)
    s = [0.0]
    for a, b in zip(path, path[1:]):
        s.append(s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    i0 = _nearest_index(path, pose)
    stop = i0 + 1
    while stop < len(path) and s[stop] - s[i0] < 2.5 * speed * horizon:
        stop += 1                                        # only the stretch of path the arc can reach
    stop = min(len(path), stop + 1)
    best = None
    for w in yaw_rates:
        if not motion_clear(grid, pose, speed, w, horizon=horizon, info=info, clearance=clearance,
                            escape=escape, distances=distances):
            continue
        end = arc_end(pose, speed, w, horizon)
        j = _nearest_index(path, end, i0, stop)
        progress = s[j] - s[i0]
        if progress < min_progress:
            continue
        score = progress - math.hypot(end[0] - path[j][0], end[1] - path[j][1]) - 0.02 * abs(w - preferred)
        if best is None or score > best[0]:
            best = (score, float(w))
    return None if best is None else best[1]


def best_clear_heading(grid, pose, speed, horizon=0.6, info=DEFAULT_MAP, clearance=ROBOT_RADIUS, distances=None,
                       n=12):
    """Local manoeuvre for a stuck robot: of `n` headings all round, the one whose straight walk passes
    `motion_clear` and ends with the most clearance (ties go to the smaller turn). Returns the heading or None."""
    if distances is None:
        _, distances = _clearance_m(grid, info)
    best = None
    for k in range(n):
        p = (pose[0], pose[1], pose[2] + 2 * math.pi * k / n)
        if not motion_clear(grid, p, speed, 0.0, horizon=horizon, info=info, clearance=clearance, distances=distances):
            continue
        ij = info.cell(*arc_end(p, speed, 0.0, horizon))
        score = min(float(distances[ij]), 1.0) - 0.01 * min(k, n - k)
        if best is None or score > best[0]:
            best = (score, (p[2] + math.pi) % (2 * math.pi) - math.pi)
    return None if best is None else best[1]


def _bfs(trav, start):
    nx, ny = trav.shape
    dist = np.full((nx, ny), -1, np.int32); parent = {}
    dist[start] = 0; q = deque([start])
    while q:
        i, j = q.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            a, b = i + di, j + dj
            if not (0 <= a < nx and 0 <= b < ny) or not trav[a, b] or dist[a, b] >= 0:
                continue
            if di and dj and not (trav[i + di, j] and trav[i, j + dj]):
                continue                                    # no diagonal corner cutting
            dist[a, b] = dist[i, j] + 1; parent[(a, b)] = (i, j); q.append((a, b))
    return dist, parent


def _trace(parent, target, start, info):
    path = [tuple(int(v) for v in target)]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return [info.center(i, j) for i, j in path]


def _argmin_where(mask, values):
    big = np.iinfo(np.int64).max
    return np.unravel_index(np.argmin(np.where(mask, values.astype(np.int64), big)), mask.shape)


def _wall_cost(clear_m, clearance, comfort, weight):
    """Extra cost per metre inside the comfort band beyond the hard clearance: 0 with `comfort` m to spare,
    `weight` right at the margin, quadratic in between."""
    if comfort <= 0 or weight <= 0:
        return np.zeros(clear_m.shape)
    short = np.clip((clearance + comfort - clear_m) / comfort, 0.0, 1.0)
    return weight * short ** 2


def _dijkstra(trav, start, cost, res):
    """Shortest weighted paths over 8-connected traversable cells (no diagonal corner cutting). An edge costs its
    length times 1 + the mean wall cost of its two cells. Returns (dist in m, inf where unreachable; flat predecessor
    indices, negative for none)."""
    nx, ny = trav.shape
    flat = np.arange(nx * ny).reshape(nx, ny)
    rows, cols, weights = [], [], []
    for di, dj in ((1, 0), (0, 1), (1, 1), (1, -1)):
        a = (slice(0, nx - di), slice(max(0, -dj), ny - max(0, dj)))
        b = (slice(di, nx), slice(max(0, dj), ny + min(0, dj)))
        ok = trav[a] & trav[b]
        if di and dj:                                    # both orthogonal neighbours must be traversable too
            ok &= trav[(b[0], a[1])] & trav[(a[0], b[1])]
        rows.append(flat[a][ok]); cols.append(flat[b][ok])
        weights.append(res * math.hypot(di, dj) * (1.0 + 0.5 * (cost[a][ok] + cost[b][ok])))
    graph = coo_matrix((np.concatenate(weights), (np.concatenate(rows), np.concatenate(cols))),
                       shape=(nx * ny, nx * ny)).tocsr()
    dist, pred = dijkstra(graph, directed=False, indices=int(flat[start]), return_predecessors=True)
    return dist.reshape(nx, ny), pred


def _trace_pred(pred, target, start, info):
    ny = info.ny
    k = int(target[0]) * ny + int(target[1]); s0 = int(start[0]) * ny + int(start[1])
    cells = [k]
    while cells[-1] != s0:
        cells.append(int(pred[cells[-1]]))
    cells.reverse()
    return [info.center(c // ny, c % ny) for c in cells]


def plan(grid, pose, goal=None, goal_radius=0.34, info=DEFAULT_MAP, clearance=ROBOT_RADIUS, min_frontier=8,
         comfort=0.10, wall_weight=3.0, avoid=()):
    """Wavefront plan on an occupancy grid (nx, ny; -1/0/100).
    Returns (path [(x, y)...] or None, mode); mode is one of STATUS_TEXT's keys.
    - Raw occupied cells are never made traversable.
    - Cells within `clearance` of an occupied cell are not traversable (Euclidean distance transform).
    - The robot's own footprint counts as free while it is still UNKNOWN (the body stands there and the camera
      cannot see under itself); a footprint cell mapped occupied stays occupied.
    - A start inside the clearance margin gets an 'escape' path over known-free cells to the nearest cell with
      full clearance, instead of clearing the margin.
    - Paths are weighted shortest paths: a cell within `comfort` m beyond the clearance costs up to `wall_weight`
      extra per metre, so paths keep off walls where there is room and only hug the margin in narrow gaps
      (comfort=0 gives plain shortest paths).
    - `avoid`: [(x, y, r)...] frontier cells within r of these points are not chosen (frontiers the robot could not
      reach); the beacon goal is never avoided."""
    if grid.shape != (info.nx, info.ny):
        raise ValueError(f"grid shape {grid.shape} does not match map info ({info.nx}, {info.ny})")
    ci, cj = info.cell(pose[0], pose[1])
    if not info.inside(ci, cj):
        return None, "out_of_map"
    occ, clear_m = _clearance_m(grid, info)
    if occ[ci, cj]:
        return None, "start_blocked"
    ii, jj = np.ogrid[:info.nx, :info.ny]
    foot = ((ii - ci) ** 2 + (jj - cj) ** 2) * info.res ** 2 <= clearance ** 2
    known_free = (grid == 0) | (foot & (grid == -1))
    trav = known_free & (clear_m >= clearance)
    start = (ci, cj)

    if not trav[start]:
        dist, parent = _bfs(known_free & ~occ, start)
        cand = (dist > 0) & trav
        if not cand.any():
            return None, "start_blocked"
        return _trace(parent, _argmin_where(cand, dist), start, info), "escape"

    dist, pred = _dijkstra(trav, start, _wall_cost(clear_m, clearance, comfort, wall_weight), info.res)
    reached = np.isfinite(dist)
    if goal is not None:
        gi, gj = info.cell(*goal)
        d2 = (ii - gi) ** 2 + (jj - gj) ** 2
        cand = reached & (d2 * info.res ** 2 <= goal_radius ** 2)
        if cand.any():
            return _trace_pred(pred, _argmin_where(cand, d2), start, info), "goal"
    frontier = frontier_cells(grid, reached) & (dist > 0.35)
    lab, n = ndimage.label(frontier, structure=np.ones((3, 3)))        # same connectivity as exploration_state
    if n:
        sizes = ndimage.sum(frontier, lab, range(1, n + 1))
        big = np.isin(lab, np.nonzero(sizes >= min_frontier)[0] + 1)
        for x, y, r in avoid:
            ai, aj = info.cell(x, y)
            big &= ((ii - ai) ** 2 + (jj - aj) ** 2) * info.res ** 2 > r ** 2
        if big.any():
            cost = np.where(big, dist, np.inf)
            return _trace_pred(pred, np.unravel_index(np.argmin(cost), cost.shape), start, info), "explore"
    return None, "no_frontier"


def frontier_cells(grid, reached):
    near_unknown = ndimage.binary_dilation(grid == -1, iterations=1)
    return reached & near_unknown


def exploration_state(grid, pose, info=DEFAULT_MAP, min_area_m2=2.0, min_open_frontier=8):
    """Decide what 'no reachable frontier' means. Returns (state, reachable_area_m2, largest_open_frontier_cells):
    - 'waiting'  : the known-free area connected to the robot is below `min_area_m2` (e.g. an all-unknown grid);
    - 'stalled'  : that region touches the map edge, or still has a connected frontier (free cells bordering
                   unknown space) of at least `min_open_frontier` cells, e.g. an unexplored doorway the planner
                   cannot use (too narrow for the clearance, or too close);
    - 'complete' : the region is closed by mapped obstacles (only isolated specks of unknown remain) and is at
                   least `min_area_m2`. Coverage is judged on the room the robot can reach, not the whole grid."""
    ci, cj = info.cell(pose[0], pose[1])
    if not info.inside(ci, cj):
        return "stalled", 0.0, 0
    passable = (grid == 0)
    passable[ci, cj] = grid[ci, cj] != 100
    lab, _ = ndimage.label(passable, structure=np.ones((3, 3)))     # connectivity only: fast, no path needed
    reached = lab == lab[ci, cj] if lab[ci, cj] else np.zeros_like(passable)
    reached[ci, cj] = passable[ci, cj]
    area = float(reached.sum()) * info.res ** 2
    if area < min_area_m2:
        return "waiting", area, 0
    lab, n = ndimage.label(frontier_cells(grid, reached), structure=np.ones((3, 3)))
    largest = int(ndimage.sum(np.ones_like(lab), lab, range(1, n + 1)).max()) if n else 0
    edge = reached[0, :].any() or reached[-1, :].any() or reached[:, 0].any() or reached[:, -1].any()
    if edge or largest >= min_open_frontier:
        return "stalled", area, largest
    return "complete", area, largest
