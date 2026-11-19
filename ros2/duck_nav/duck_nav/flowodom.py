"""Floor optical-flow odometry: body planar motion from consecutive head-camera images, and the policy that turns it
into translation for dead reckoning. Shared by slam_node (translation:=flow_or_command) and tools/slam_replay.py.

Inputs per image: a grayscale image, the camera pose (R, t) in the level body frame at that image's stamp (origin on
the floor under the torso, heading removed; /camera/pose_level, which already contains the head/neck motion) and the
3x3 intrinsics K. Camera convention: MuJoCo (x right, y up, looks along -z). No ground truth is used.

Trackers (results/floor_flow/NOTES.md):
  rect   LK in floor-rectified top-down images (8 mm per pixel). Motion scale 0.96-1.00 on the 12 recorded bags, but
         a fit for only ~45-55 % of image pairs.
  image  LK in the camera image, seeded with the head-motion prediction. More fits, but the LK window on a
         perspective-distorted patch under-estimates motion by ~4-8 %.
  rect_or_image  rect, else image.
Result convention: (dx, dy, dth, n_inliers), (dx, dy) in the EARLIER image's level frame, p_k = R(dth) p_k1 + t.

Simulation caveat: the simulated floor is a strongly textured checker; real floors may give far fewer features.
"""
import bisect
import math

import cv2
import numpy as np

PIX = 0.5                                   # pixel centre offset: index i covers [i, i + 1) in the intrinsics' frame
GRID = dict(x0=-0.4, x1=1.4, y0=-1.0, y1=1.0, res=0.008)    # top-down image: level-frame floor area, metres
METHODS = ("image", "rect", "rect_or_image")


def floor_points(px, R, t, K):
    """Pixels (N, 2) -> floor points (N, 2) in the level frame, NaN where the ray does not hit the floor ahead."""
    u = (px[:, 0] + PIX - K[0, 2]) / K[0, 0]
    v = (px[:, 1] + PIX - K[1, 2]) / K[1, 1]
    d = np.stack([u, -v, -np.ones_like(u)], 1) @ R.T          # MuJoCo camera: x right, y up, looks along -z
    s = -t[2] / d[:, 2]
    ok = (d[:, 2] < -1e-3) & (s > 0)
    p = t[None, :2] + s[:, None] * d[:, :2]
    p[~ok] = np.nan
    return p


def project(p, R, t, K):
    """Floor points (N, 2) in the level frame -> pixels (N, 2) of a camera at (R, t)."""
    P = np.column_stack([p, np.zeros(len(p))]) - t
    c = P @ R                                                  # into camera axes
    z = -c[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = K[0, 0] * c[:, 0] / z + K[0, 2] - PIX
        v = -K[1, 1] * c[:, 1] / z + K[1, 2] - PIX
    out = np.column_stack([u, v]); out[z <= 1e-3] = np.nan
    return out


def rigid_fit(a, b):
    """Least-squares R, t with a ~= R b + t (2-D)."""
    ca, cb = a.mean(0), b.mean(0)
    H = (b - cb).T @ (a - ca)
    th = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    c, s = math.cos(th), math.sin(th)
    t = ca - np.array([[c, -s], [s, c]]) @ cb
    return t, th


def ransac_rigid(a, b, rng, iters=200, tol=0.02):
    best = None
    n = len(a)
    for _ in range(iters):
        i, j = rng.choice(n, 2, replace=False)
        if np.linalg.norm(b[i] - b[j]) < 0.05:
            continue
        t, th = rigid_fit(a[[i, j]], b[[i, j]])
        c, s = math.cos(th), math.sin(th)
        res = np.linalg.norm(a - (b @ np.array([[c, s], [-s, c]]) + t), axis=1)
        inl = res < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 6:
        return None
    t, th = rigid_fit(a[best], b[best])
    return t, th, int(best.sum())


def _grid(g=GRID):
    nc = int(round((g["x1"] - g["x0"]) / g["res"])); nr = int(round((g["y1"] - g["y0"]) / g["res"]))
    gx, gy = np.meshgrid(g["x0"] + g["res"] * (np.arange(nc) + 0.5), g["y0"] + g["res"] * (np.arange(nr) + 0.5))
    return np.column_stack([gx.ravel(), gy.ravel()]), nr, nc


_GRID = _grid()


def rectify(img, R, t, K, horizon_row=None):
    """Warp a camera image onto the floor plane of its level frame: returns (top-down image, valid mask).
    Pixel (c, r) of the result is the floor point (x0 + res (c + .5), y0 + res (r + .5)). Rows above `horizon_row`
    (default H / 3, the floor is in the lower part) are not used."""
    grid, nr, nc = _GRID
    H, W = img.shape
    px = project(grid, R, t, K)
    mx = px[:, 0].reshape(nr, nc).astype(np.float32); my = px[:, 1].reshape(nr, nc).astype(np.float32)
    top = H / 3 if horizon_row is None else horizon_row
    ok = np.isfinite(mx) & np.isfinite(my) & (mx >= 0) & (mx <= W - 1) & (my >= top) & (my <= H - 1)
    mx = np.where(ok, mx, -1).astype(np.float32); my = np.where(ok, my, -1).astype(np.float32)
    return cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0), ok.astype(np.uint8)


def _to_floor(c, g=GRID):
    return np.column_stack([g["x0"] + g["res"] * (c[:, 0] + 0.5), g["y0"] + g["res"] * (c[:, 1] + 0.5)])


def track_rect(img0, p0, img1, p1, K, rng, win=11, levels=2, tol=0.01, gate=0.12, erode=1, fb=0.3, quality=0.02,
               minpts=6):
    """Body motion (t, dth, inliers) between two frames by tracking in top-down images, or None.
    p0, p1 = (R, t) camera poses in the level frames of the two images."""
    a, va = rectify(img0, p0[0], p0[1], K)
    b, vb = rectify(img1, p1[0], p1[1], K)
    e = win // 2 if erode is None else erode
    va = cv2.erode(va & vb, np.ones((e, e), np.uint8)) if e > 1 else va & vb       # only where both frames see the floor
    pts = cv2.goodFeaturesToTrack(a, 300, quality, 6, mask=va * 255)
    if pts is None or len(pts) < minpts:
        return None
    lk = dict(winSize=(win, win), maxLevel=levels, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    q, st, _ = cv2.calcOpticalFlowPyrLK(a, b, pts, None, **lk)
    back, st2, _ = cv2.calcOpticalFlowPyrLK(b, a, q, None, **lk)
    pts = pts.reshape(-1, 2); q = q.reshape(-1, 2)
    ok = (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm(back.reshape(-1, 2) - pts, axis=1) < fb)
    ok &= np.linalg.norm(q - pts, axis=1) * GRID["res"] < gate    # body motion in 0.2 s is a few cm; the checker repeats
    if ok.sum() < minpts:
        return None
    fit = ransac_rigid(_to_floor(pts[ok]), _to_floor(q[ok]), rng, tol=tol)
    return None if fit is None else (fit[0], fit[1], fit[2])


IMAGE_LK = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
                flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
MIN_POINTS = 8
MAX_FLOOR_RANGE = 1.5       # m: ignore floor points farther than this from the camera (pixel footprint grows)
FB_TOL = 1.0                # px forward-backward tracking tolerance


def track_image(img0, p0, img1, p1, K, rng, guess=None):
    """Body motion (dx, dy, dth, n_inliers) by tracking corners in the camera image, or None. LK is seeded with the
    head-motion prediction (zero body motion, or `guess` = (dx, dy, dth) if given)."""
    (R0, t0), (R1, t1) = p0, p1
    H, W = img0.shape
    mask = np.zeros((H, W), np.uint8); mask[H // 3:, :] = 255     # floor is in the lower part when looking ahead
    c0 = cv2.goodFeaturesToTrack(img0, 200, 0.01, 6, mask=mask)
    if c0 is None or len(c0) < MIN_POINTS:
        return None
    c0 = c0.reshape(-1, 2)
    f0 = floor_points(c0, R0, t0, K)
    ok = np.isfinite(f0).all(1) & (np.linalg.norm(f0 - t0[:2], axis=1) < MAX_FLOOR_RANGE)
    c0, f0 = c0[ok], f0[ok]
    if len(c0) < MIN_POINTS:
        return None
    f_seed = f0
    if guess is not None:                                          # p_k1 = R(dth)^T (p_k - t)
        c, s = math.cos(guess[2]), math.sin(guess[2])
        f_seed = (f0 - np.array(guess[:2])) @ np.array([[c, -s], [s, c]])
    seed = project(f_seed, R1, t1, K)
    good = np.isfinite(seed).all(1)
    c0, f0, seed = c0[good], f0[good], seed[good]
    if len(c0) < MIN_POINTS:
        return None
    q0 = c0.astype(np.float32).reshape(-1, 1, 2)
    c1, st, _ = cv2.calcOpticalFlowPyrLK(img0, img1, q0, seed.astype(np.float32).reshape(-1, 1, 2), **IMAGE_LK)
    back, st2, _ = cv2.calcOpticalFlowPyrLK(img1, img0, c1, q0, **IMAGE_LK)
    c1 = c1.reshape(-1, 2)
    fb = np.linalg.norm(back.reshape(-1, 2) - c0, axis=1)
    ok = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < FB_TOL)
    if ok.sum() < MIN_POINTS:
        return None
    f1 = floor_points(c1[ok], R1, t1, K)
    a, b = f0[ok], f1
    fin = np.isfinite(b).all(1)
    if fin.sum() < MIN_POINTS:
        return None
    fit = ransac_rigid(a[fin], b[fin], rng)
    if fit is None:
        return None
    return float(fit[0][0]), float(fit[0][1]), float(fit[1]), int(fit[2])


def estimate(img0, img1, pose0, pose1, K, rng, method="rect_or_image"):
    """One image pair with the chosen tracker (see module docstring): (dx, dy, dth, n_inliers) or None."""
    if method not in METHODS:
        raise ValueError(f"flow method must be one of {METHODS}")
    if method != "image":
        r = track_rect(img0, pose0, img1, pose1, K, rng)
        if r is not None:
            t, th, n = r
            return float(t[0]), float(t[1]), float(th), int(n)
        if method == "rect":
            return None
    return track_image(img0, pose0, img1, pose1, K, rng)


class _Track:
    """Cumulative world-frame displacement over time with linear interpolation (for window differences)."""

    def __init__(self, keep=20.0):
        self.t, self.x, self.y, self.keep = [], [], [], keep

    def add(self, t, dx, dy):
        self.t.append(t)
        self.x.append((self.x[-1] if self.x else 0.0) + dx)
        self.y.append((self.y[-1] if self.y else 0.0) + dy)
        if self.keep and len(self.t) > 4000 and self.t[0] < t - self.keep:      # bound memory in a live node
            k = bisect.bisect_left(self.t, t - self.keep)
            del self.t[:k], self.x[:k], self.y[:k]

    def at(self, t):
        i = bisect.bisect_left(self.t, t)
        if i <= 0:
            return np.array([self.x[0], self.y[0]]) if self.t else np.zeros(2)
        if i >= len(self.t):
            return np.array([self.x[-1], self.y[-1]])
        w = (t - self.t[i - 1]) / max(self.t[i] - self.t[i - 1], 1e-9)
        return np.array([self.x[i - 1] + w * (self.x[i] - self.x[i - 1]), self.y[i - 1] + w * (self.y[i] - self.y[i - 1])])


class FlowTranslation:
    """Translation for dead reckoning from floor-flow image pairs ("flow"), with the gait command as fallback for
    pairs without a fit ("flow_or_command"). Causal:
      - between images the pose advances at the last measured body velocity (`body_velocity`), or at the command
        model's velocity when there is none (flow_or_command) or zero (flow)
      - when the estimate for pair (s0, s1) is available (after s1), `on_pair` returns the world-frame residual
        (measured displacement minus what was extrapolated over [s0, s1]) to add to the current pose
      - a pair overlapping a non-upright interval (`mark_not_ok`) is discarded
    Call `record` after every dead-reckoning step with the world displacement actually applied and the command
    model's world displacement over that step."""

    def __init__(self, mode="flow_or_command", stale=0.5):
        if mode not in ("flow", "flow_or_command"):
            raise ValueError("FlowTranslation mode must be 'flow' or 'flow_or_command'")
        self.mode, self.stale = mode, stale
        self.odo, self.cmd_model = _Track(), _Track()
        self.v_flow, self.v_flow_t, self.bad_t = None, -1e9, -1e9
        self.n_ok = self.n_missing = 0

    def mark_not_ok(self, t):
        self.bad_t = t

    def body_velocity(self, t, cmd_body, ok=True):
        if not ok:
            return (0.0, 0.0)
        if self.v_flow is not None and t - self.v_flow_t < self.stale:
            return self.v_flow
        return tuple(cmd_body) if self.mode == "flow_or_command" else (0.0, 0.0)

    def record(self, t, d_world, cmd_world):
        self.odo.add(t, float(d_world[0]), float(d_world[1]))
        self.cmd_model.add(t, float(cmd_world[0]), float(cmd_world[1]))

    def on_pair(self, s0, s1, est, heading_s0):
        """Residual (2,) in the world frame to add to the current pose, or None (pair unusable)."""
        if not self.odo.t or self.bad_t >= s0:
            self.v_flow = None
            return None
        ext = self.odo.at(s1) - self.odo.at(s0)                  # what the extrapolation applied over [s0, s1]
        if est is not None:
            c, s = math.cos(heading_s0), math.sin(heading_s0)
            meas = np.array([c * est[0] - s * est[1], s * est[0] + c * est[1]])
            self.v_flow, self.v_flow_t = (est[0] / (s1 - s0), est[1] / (s1 - s0)), s1
            self.n_ok += 1
        else:
            self.n_missing += 1
            self.v_flow = None
            meas = (self.cmd_model.at(s1) - self.cmd_model.at(s0)) if self.mode == "flow_or_command" else np.zeros(2)
        return meas - ext
