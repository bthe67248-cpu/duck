"""Ground-truth-free floor optical-flow odometry for the offline SLAM replay (tools/slam_replay.py).

`estimate_pair` returns the body's planar motion between two consecutive camera images using only the images,
the camera poses in the level body frame at the two stamps (/camera/pose_level) and the intrinsics. Nothing here
reads ground truth, so it can run in a replay or (cost permitting) on the robot.

MIRRORS: tools/floor_flow.py `run()` per-pair logic as of base commit 34fd1b6 (corners in the lower 2/3 of the
image, LK seeded from the known head motion, forward-backward check, back-projection to z=0, RANSAC 2-D rigid
fit). The geometry helpers (floor_points, project, rigid_fit, ransac_rigid) are imported from floor_flow when
that module can be imported (it needs ROS), else the identical local copies below are used. When floor_flow's
estimator improves, port the change into `estimate_pair` (that is the only place that has to change).

Result convention: (dx, dy, dth, n_inliers) with (dx, dy) in the EARLIER frame's level frame and p_k = R(dth) p_k1
+ t, i.e. the body displacement over the pair expressed in the body axes at the earlier image.
"""
import math

import cv2
import numpy as np

try:                                                   # shared helpers, so a fix there is picked up here
    from floor_flow import floor_points, project, rigid_fit, ransac_rigid  # noqa: F401
    from floor_flow import track_rect                    # top-down (floor-rectified) tracker, no 4-8 % under-scale
    HELPERS = "floor_flow"
except ImportError:                                     # no ROS (rosbag2_py) in this environment
    HELPERS = "local copy of floor_flow @ 34fd1b6"
    track_rect = None

    def floor_points(px, R, t, K):
        """Pixels (N, 2) -> floor points (N, 2) in the level frame, NaN where the ray does not hit the floor ahead."""
        u = (px[:, 0] + 0.5 - K[0, 2]) / K[0, 0]
        v = (px[:, 1] + 0.5 - K[1, 2]) / K[1, 1]
        d = np.stack([u, -v, -np.ones_like(u)], 1) @ R.T      # MuJoCo camera: x right, y up, looks along -z
        s = -t[2] / d[:, 2]
        ok = (d[:, 2] < -1e-3) & (s > 0)
        p = t[None, :2] + s[:, None] * d[:, :2]
        p[~ok] = np.nan
        return p

    def project(p, R, t, K):
        """Floor points (N, 2) in the level frame -> pixels (N, 2) of a camera at (R, t)."""
        P = np.column_stack([p, np.zeros(len(p))]) - t
        c = P @ R
        z = -c[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = K[0, 0] * c[:, 0] / z + K[0, 2] - 0.5
            v = -K[1, 1] * c[:, 1] / z + K[1, 2] - 0.5
        out = np.column_stack([u, v]); out[z <= 1e-3] = np.nan
        return out

    def rigid_fit(a, b):
        """Least-squares R, t with a ~= R b + t (2-D)."""
        ca, cb = a.mean(0), b.mean(0)
        H = (b - cb).T @ (a - ca)
        th = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        c, s = math.cos(th), math.sin(th)
        return ca - np.array([[c, -s], [s, c]]) @ cb, th

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

LK = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
          flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
MIN_POINTS = 8
MAX_FLOOR_RANGE = 1.5       # m: ignore floor points farther than this from the camera (pixel footprint grows)
FB_TOL = 1.0                # px forward-backward tracking tolerance


def estimate_pair(img0, img1, pose0, pose1, K, rng, guess=None):
    """Body planar motion between two grayscale images. pose = (R, t) of the camera in the level frame at that
    image's stamp. guess = optional (dx, dy, dth) prior body motion used to seed the tracker (default zero motion,
    leaving only the head motion). Returns (dx, dy, dth, n_inliers) or None when there is no usable fit."""
    R0, t0 = pose0; R1, t1 = pose1
    H, W = img0.shape
    mask = np.zeros((H, W), np.uint8); mask[H // 3:, :] = 255     # floor is in the lower part when looking ahead
    p0 = cv2.goodFeaturesToTrack(img0, 200, 0.01, 6, mask=mask)
    if p0 is None or len(p0) < MIN_POINTS:
        return None
    p0 = p0.reshape(-1, 2)
    f0 = floor_points(p0, R0, t0, K)
    ok = np.isfinite(f0).all(1) & (np.linalg.norm(f0 - t0[:2], axis=1) < MAX_FLOOR_RANGE)
    p0, f0 = p0[ok], f0[ok]
    if len(p0) < MIN_POINTS:
        return None
    f_seed = f0
    if guess is not None:                                          # p_k1 = R(dth)^T (p_k - t)
        c, s = math.cos(guess[2]), math.sin(guess[2])
        f_seed = (f0 - np.array(guess[:2])) @ np.array([[c, -s], [s, c]])
    seed = project(f_seed, R1, t1, K)
    good = np.isfinite(seed).all(1)
    p0, f0, seed = p0[good], f0[good], seed[good]
    if len(p0) < MIN_POINTS:
        return None
    q0 = p0.astype(np.float32).reshape(-1, 1, 2)
    p1, st, _ = cv2.calcOpticalFlowPyrLK(img0, img1, q0, seed.astype(np.float32).reshape(-1, 1, 2), **LK)
    back, st2, _ = cv2.calcOpticalFlowPyrLK(img1, img0, p1, q0, **LK)
    p1 = p1.reshape(-1, 2)
    fb = np.linalg.norm(back.reshape(-1, 2) - p0, axis=1)
    ok = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < FB_TOL)
    if ok.sum() < MIN_POINTS:
        return None
    f1 = floor_points(p1[ok], R1, t1, K)
    a, b = f0[ok], f1
    fin = np.isfinite(b).all(1)
    if fin.sum() < MIN_POINTS:
        return None
    fit = ransac_rigid(a[fin], b[fin], rng)
    if fit is None:
        return None
    return float(fit[0][0]), float(fit[0][1]), float(fit[1]), int(fit[2])


METHODS = ("image", "rect", "rect_or_image")


def estimate(img0, img1, pose0, pose1, K, rng, method="rect_or_image"):
    """One pair with the chosen tracker:
      image          estimate_pair above (LK in the camera image; under-estimates motion by ~4-8 %)
      rect           floor_flow.track_rect (LK in floor-rectified top-down images; scale 0.96-1.00, but finds a
                     fit for only ~54 % of pairs)
      rect_or_image  rect, else image"""
    if method not in METHODS:
        raise ValueError(f"flow method must be one of {METHODS}")
    if method != "image":
        if track_rect is None:
            raise RuntimeError("flow method 'rect' needs tools/floor_flow.py importable (ROS environment)")
        r = track_rect(img0, pose0, img1, pose1, K, rng)
        if r is not None:
            t, th, n = r
            return float(t[0]), float(t[1]), float(th), int(n)
        if method == "rect":
            return None
    return estimate_pair(img0, img1, pose0, pose1, K, rng)


def compute_flow_pairs(imgs, poses, K, seed=0, max_t=None, method="image"):
    """Estimates for every consecutive stamp pair 0.1 s < dt < 0.3 s: [(s0, s1, estimate_or_None), ...].
    imgs / poses: {stamp: gray image / (R, t)}. No ground truth involved; falls are handled by the replay's own
    tilt gate (it discards pairs overlapping a non-upright interval)."""
    rng = np.random.default_rng(seed)
    stamps = sorted(set(imgs) & set(poses))
    out = []
    for s0, s1 in zip(stamps[:-1], stamps[1:]):
        if not 0.1 < s1 - s0 < 0.3 or (max_t is not None and s1 > max_t):
            continue
        out.append((s0, s1, estimate(imgs[s0], imgs[s1], poses[s0], poses[s1], K, rng, method)))
    return out
