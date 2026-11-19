"""floor_flow.py BAG [BAG ...] [--duration S] [--output JSON] [--method rect|warp|image] -- offline floor optical-flow
odometry, scored against ground truth next to the gait-command odometry slam_node uses.

Needs a bag recorded with /camera/image_raw, /camera/pose_level, /camera/camera_info, /ground_truth/pose and
/cmd_vel (tools/trial_matrix.py records them). Simulation geometry: /camera/pose_level is the camera pose in the
level body frame (origin on the floor under the torso, heading removed) at the image stamp; on the robot this
comes from neck/leg encoders + IMU tilt. Pixel centres are at index + 0.5 (checked against a MuJoCo render).

Per consecutive image pair (method "rect", the default):
  1. both images are warped onto the floor plane z = 0 of their OWN level frame (top-down, 8 mm per pixel) using
     /camera/pose_level; head motion and perspective vanish, what is left is the body motion (a few cm)
  2. corners in the first top-down image, pyramidal Lucas-Kanade into the second, forward-backward check
  3. RANSAC 2-D rigid fit p_k = R(dth) p_k1 + t on the floor coordinates: (t, dth) is the body motion over the pair,
     in the frame of image k
Why top-down: tracking in the camera image treats the window as a pure translation, but on the floor the image
motion varies by ~10 % across a 21 px window (perspective), and Lucas-Kanade then under-estimates the motion by
about 4 % (measured on noise-free MuJoCo renders: 6-10 % at 2-4 cm per pair; top-down: 0-3 %). Method "image"
is the old camera-image tracker (seeded with the head-motion prediction) kept for comparison.

Scoring. Everything is integrated with the TRUE heading, so only translation is compared. Per-pair (0.2 s)
errors are reported for orientation, but they and "distance travelled" include the gait's back-and-forth sway, so
the headline scores are NET displacements over windows:
  window  W seconds (2 and 10), non-overlapping, inside one run of consecutive pairs (no gap, no fall)
  error   |estimated net displacement - true net displacement| (m), both in the frame at the window start;
          estimators: flow, command (gait-command x OdomGains, what slam_node predicts before slip estimation /
          scan matching), flow_or_command (flow, command where flow found no fit). A missing flow pair counts
          as zero motion for "flow".
  moving  windows whose true net displacement exceeds 0.02 m/s * W: scale = sum(est . true) / sum(true . true),
          err_rel = sum(err) / sum(|true|)
  pushing windows where the command predicts more than 0.03 m/s * W of net motion but the robot achieved
          less than a quarter of it (blocked by an obstacle): median error of each estimator, n windows
Also: flow coverage (share of pairs with a fit), median inliers, runtime per pair (flow only, wall clock,
Python + OpenCV, excludes bag reading). Run in WSL after `source /opt/ros/jazzy/setup.bash`.
"""
import argparse
import bisect
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import duck_nav.duckslam as ds  # noqa: E402

TOPICS = ["/camera/image_raw", "/camera/pose_level", "/camera/camera_info", "/ground_truth/pose", "/cmd_vel",
          "/clock"]
PIX = 0.5                                   # pixel centre offset: index i covers [i, i + 1) in the intrinsics' frame
# top-down grid in the level frame: x forward, y left; res m per pixel
GRID = dict(x0=-0.4, x1=1.4, y0=-1.0, y1=1.0, res=0.008)


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def quat_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load(path, duration):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    missing = [t for t in TOPICS[:4] if t not in types]
    if missing:
        raise SystemExit(f"{path}: missing topics {missing} (record them with tools/trial_matrix.py)")
    r.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    imgs, cposes, gt, cmds, clock, K = {}, {}, [], [], [], None
    while r.has_next():
        tp, d, trx = r.read_next()
        m = deserialize_message(d, types[tp])
        if tp == "/camera/image_raw":
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
            imgs[round(stamp(m.header), 4)] = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
        elif tp == "/camera/pose_level":
            p, o = m.pose.position, m.pose.orientation
            cposes[round(stamp(m.header), 4)] = (quat_R(o.w, o.x, o.y, o.z), np.array([p.x, p.y, p.z]))
        elif tp == "/camera/camera_info" and K is None:
            K = np.array(m.k).reshape(3, 3)
        elif tp == "/ground_truth/pose":
            p, o = m.pose.position, m.pose.orientation
            gt.append((stamp(m.header), np.array([p.x, p.y, ds.quat_yaw((o.w, o.x, o.y, o.z))]),
                       1 - 2 * (o.x * o.x + o.y * o.y)))
        elif tp == "/cmd_vel":
            cmds.append((trx * 1e-9, m.linear.x, m.linear.y))
        elif tp == "/clock":
            clock.append((trx * 1e-9, m.clock.sec + m.clock.nanosec * 1e-9))
    gt.sort(key=lambda g: g[0]); clock.sort()
    ck = [c[0] for c in clock]
    cmd_sim = []                                  # /cmd_vel has no header: map receive time to sim time via /clock
    for trx, vx, vy in cmds:
        j = min(bisect.bisect_left(ck, trx), len(clock) - 1)
        cmd_sim.append((clock[j][1], vx, vy))
    cmd_sim.sort()
    stamps = sorted(set(imgs) & set(cposes))
    t0 = stamps[0]
    stamps = [s for s in stamps if s - t0 <= duration]
    return imgs, cposes, K, gt, cmd_sim, stamps




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


def gt_at(gt, gts, t):
    i = min(bisect.bisect_left(gts, t), len(gt) - 1)
    return gt[i]


def gt_interp(gt, gts, t):
    """Ground-truth (x, y, yaw) linearly interpolated at time t (stamps are 20 ms apart, images fall on them)."""
    i = min(max(bisect.bisect_left(gts, t - 1e-6), 1), len(gt) - 1)
    a, b = gt[i - 1], gt[i]
    w = min(max((t - a[0]) / (b[0] - a[0]), 0.0), 1.0)
    return np.array([a[1][0] + w * (b[1][0] - a[1][0]), a[1][1] + w * (b[1][1] - a[1][1]),
                     a[1][2] + w * wrap(b[1][2] - a[1][2])])


def command_motion(cmd_sim, cts, t0, t1, gains, step=0.02):
    """Body-frame displacement predicted from the gait command over [t0, t1) (commands expire after 0.5 s)."""
    slam = ds.DuckSLAM(gains)
    disp = np.zeros(2)
    t = t0
    while t < t1:
        j = bisect.bisect_right(cts, t) - 1
        vx, vy = (cmd_sim[j][1], cmd_sim[j][2]) if j >= 0 and t - cmd_sim[j][0] < 0.5 else (0.0, 0.0)
        bx, by = slam.body_velocity(vx, vy)
        disp += np.array([bx, by]) * min(step, t1 - t)
        t += step
    return disp


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


def floor_homography(p0, p1, K):
    """3x3 map from pixel indices of image 0 to pixel indices of image 1 for points on the floor plane z = 0,
    assuming the body did not move between the frames (only the camera pose in the level frame changed)."""
    def to_floor(R, t):                                          # pixel index -> homogeneous floor point (x, y, 1)
        A = R @ np.diag([1.0, -1.0, -1.0])                       # MuJoCo camera axes: x right, y up, looks along -z
        M = np.array([-t[2] * A[0] + t[0] * A[2], -t[2] * A[1] + t[1] * A[2], A[2]]) @ np.linalg.inv(K)
        return M @ np.array([[1.0, 0, PIX], [0, 1.0, PIX], [0, 0, 1.0]])
    return np.linalg.inv(to_floor(*p1)) @ to_floor(*p0)


def track_warp(img0, p0, img1, p1, K, rng, win=21, levels=3, tol=0.02, fb=0.5, mask=None, erode=None):
    """Body motion (t, dth, inliers) between two frames, or None. Image 1 is warped into the view of image 0 with
    the floor-plane homography from the known camera poses (zero body motion), so Lucas-Kanade only sees the body
    motion on top of a patch that is no longer perspective-distorted by the head motion; the tracked points are
    mapped back to image 1 and back-projected onto the floor. Pixel resolution stays native (no resampling of
    image 0)."""
    H, W = img0.shape
    Hm = floor_homography(p0, p1, K)
    flags = cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP
    w1 = cv2.warpPerspective(img1, Hm, (W, H), flags=flags)
    valid = cv2.warpPerspective(np.full((H, W), 255, np.uint8), Hm, (W, H), flags=flags)
    e = max(win // 2, 1) if erode is None else erode
    valid = cv2.erode((valid > 250).astype(np.uint8), np.ones((e, e), np.uint8))
    if mask is None:
        mask = np.zeros((H, W), np.uint8); mask[H // 3:, :] = 255
    c0 = cv2.goodFeaturesToTrack(img0, 200, 0.01, 6, mask=mask & (valid * 255))
    if c0 is None or len(c0) < 8:
        return None
    c0 = c0.reshape(-1, 2)
    f0 = floor_points(c0, p0[0], p0[1], K)
    ok = np.isfinite(f0).all(1) & (np.linalg.norm(f0 - p0[1][:2], axis=1) < 1.5)
    c0, f0 = c0[ok], f0[ok]
    if len(c0) < 8:
        return None
    lk = dict(winSize=(win, win), maxLevel=levels, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    P = c0.astype(np.float32).reshape(-1, 1, 2)
    q, st, _ = cv2.calcOpticalFlowPyrLK(img0, w1, P, None, **lk)
    back, st2, _ = cv2.calcOpticalFlowPyrLK(w1, img0, q, None, **lk)
    q = q.reshape(-1, 2)
    ok = (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm(back.reshape(-1, 2) - c0, axis=1) < fb)
    ok &= np.linalg.norm(q - c0, axis=1) < 40                    # body motion is a few cm = a few pixels
    if ok.sum() < 8:
        return None
    h = np.column_stack([q[ok], np.ones(ok.sum())]) @ Hm.T       # warped-image points -> image 1
    c1 = h[:, :2] / h[:, 2:3]
    f1 = floor_points(c1, p1[0], p1[1], K)
    a, b = f0[ok], f1
    fin = np.isfinite(b).all(1)
    if fin.sum() < 8:
        return None
    fit = ransac_rigid(a[fin], b[fin], rng, tol=tol)
    return None if fit is None else (fit[0], fit[1], fit[2])


def track_image(img0, p0, img1, p1, K, rng, mask=None):
    """Old tracker: corners in the camera image, LK seeded with the head-motion prediction (zero body motion),
    back-projected onto the floor. Biased ~4 % low (see module docstring)."""
    (R0, t0), (R1, t1) = p0, p1
    H, W = img0.shape
    if mask is None:
        mask = np.zeros((H, W), np.uint8); mask[H // 3:, :] = 255
    lk = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
              flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
    c0 = cv2.goodFeaturesToTrack(img0, 200, 0.01, 6, mask=mask)
    if c0 is None or len(c0) < 8:
        return None
    c0 = c0.reshape(-1, 2)
    f0 = floor_points(c0, R0, t0, K)
    ok = np.isfinite(f0).all(1) & (np.linalg.norm(f0 - t0[:2], axis=1) < 1.5)
    c0, f0 = c0[ok], f0[ok]
    if len(c0) < 8:
        return None
    guess = project(f0, R1, t1, K)
    good = np.isfinite(guess).all(1)
    c0, f0, guess = c0[good], f0[good], guess[good]
    if len(c0) < 8:
        return None
    P = c0.astype(np.float32).reshape(-1, 1, 2)
    c1, st, _ = cv2.calcOpticalFlowPyrLK(img0, img1, P, guess.astype(np.float32).reshape(-1, 1, 2), **lk)
    back, st2, _ = cv2.calcOpticalFlowPyrLK(img1, img0, c1, P, **lk)
    c1 = c1.reshape(-1, 2)
    fb = np.linalg.norm(back.reshape(-1, 2) - c0, axis=1)
    ok = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < 1.0)
    if ok.sum() < 8:
        return None
    f1 = floor_points(c1[ok], R1, t1, K)
    a, b = f0[ok], f1
    fin = np.isfinite(b).all(1)
    if fin.sum() < 8:
        return None
    fit = ransac_rigid(a[fin], b[fin], rng)
    return None if fit is None else (fit[0], fit[1], fit[2])


TRACKERS = {}


def run(path, duration, rng, method="rect"):
    """Rows per consecutive image pair: t (stamp of the first image), true = [x, y, dth] and cmd = [x, y] body
    motion in the frame of image k, flow = [x, y, dth] or None, inliers, ms (wall time of the flow estimate)."""
    imgs, cposes, K, gt, cmd_sim, stamps = load(path, duration)
    gts = [g[0] for g in gt]; cts = [c[0] for c in cmd_sim]
    gains = ds.OdomGains()
    track = TRACKERS[method]
    rows = []
    for k in range(len(stamps) - 1):
        s0, s1 = stamps[k], stamps[k + 1]
        if not 0.1 < s1 - s0 < 0.3:
            continue
        g0, g1 = gt_at(gt, gts, s0), gt_at(gt, gts, s1)
        if min(g0[2], g1[2]) < math.cos(math.radians(30)):       # fallen / falling: skip (camera is not on the floor)
            continue
        q0, q1 = gt_interp(gt, gts, s0), gt_interp(gt, gts, s1)
        c, s = math.cos(q0[2]), math.sin(q0[2])
        d = q1[:2] - q0[:2]
        true_rel = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], wrap(q1[2] - q0[2])])
        cmd_rel = command_motion(cmd_sim, cts, s0, s1, gains)
        tic = time.perf_counter()
        est = track(imgs[s0], cposes[s0], imgs[s1], cposes[s1], K, rng)
        ms = (time.perf_counter() - tic) * 1000
        rows.append(dict(t=s0, true=true_rel.tolist(), cmd=cmd_rel.tolist(),
                         flow=None if est is None else [float(est[0][0]), float(est[0][1]), float(est[1])],
                         inliers=0 if est is None else est[2], ms=ms))
    return rows


TRACKERS.update(rect=track_rect, warp=track_warp, image=track_image)


def windows(rows, key, W):
    """Non-overlapping W-second windows inside runs of consecutive pairs (no gap, no fall). Returns a list of
    (t, true_net, est_net): both displacements integrated with the TRUE heading, in the frame at the window start."""
    n = int(round(W / 0.2))
    out = []
    i = 0
    while i + n <= len(rows):
        seg = rows[i:i + n]
        if any(seg[j + 1]["t"] - seg[j]["t"] > 0.3 for j in range(n - 1)):
            i += 1                                              # gap inside: slide until a clean run starts
            continue
        th = 0.0; pt = np.zeros(2); pe = np.zeros(2)
        for r in seg:
            c, s = math.cos(th), math.sin(th)
            Rm = np.array([[c, -s], [s, c]])
            if key == "command":
                src = r["cmd"]
            elif key == "flow":
                src = r["flow"]
            else:                                               # flow, falling back to the command when missing
                src = r["flow"] if r["flow"] is not None else r["cmd"]
            pt = pt + Rm @ np.array(r["true"][:2])
            pe = pe + Rm @ np.array(src[:2] if src is not None else [0.0, 0.0])
            th += r["true"][2]
        out.append((seg[0]["t"], pt, pe))
        i += n
    return out


def window_scores(rows, W):
    """Net-displacement scores at window length W seconds (definitions in the module docstring)."""
    res = {}
    wins = {k: windows(rows, k, W) for k in ("command", "flow", "flow_or_command")}
    if not wins["command"]:
        return res
    tr = np.array([w[1] for w in wins["command"]])
    cm = np.array([w[2] for w in wins["command"]])
    moving = np.linalg.norm(tr, axis=1) > 0.02 * W
    pushing = (np.linalg.norm(cm, axis=1) > 0.03 * W) & (np.linalg.norm(tr, axis=1) < 0.25 * np.linalg.norm(cm, axis=1))
    res["windows"] = int(len(tr)); res["moving_windows"] = int(moving.sum()); res["pushing_windows"] = int(pushing.sum())
    for k, ws in wins.items():
        es = np.array([w[2] for w in ws])
        err = np.linalg.norm(es - tr, axis=1)
        d = dict(err_cm_median=float(np.median(err) * 100), err_cm_mean=float(np.mean(err) * 100))
        if moving.any():
            d["scale"] = float((es[moving] * tr[moving]).sum() / (tr[moving] ** 2).sum())
            d["err_rel_pct"] = float(100 * err[moving].sum() / np.linalg.norm(tr[moving], axis=1).sum())
        if pushing.any():
            d["pushing_err_cm_median"] = float(np.median(err[pushing]) * 100)
        res[k] = d
    return res


def summarize(name, rows):
    tr = np.array([r["true"] for r in rows])
    cm = np.array([r["cmd"] for r in rows])
    has = np.array([r["flow"] is not None for r in rows])
    fl = np.array([r["flow"] if r["flow"] is not None else [np.nan] * 3 for r in rows])
    out = dict(bag=name, pairs=len(rows), flow_coverage=float(has.mean()),
               ms_per_pair=float(np.mean([r["ms"] for r in rows])) if rows and "ms" in rows[0] else None)
    out["pair_err_cm_median"] = {
        "command": float(np.median(np.linalg.norm(cm - tr[:, :2], axis=1)) * 100),
        "flow": float(np.nanmedian(np.linalg.norm(fl[has, :2] - tr[has, :2], axis=1)) * 100) if has.any() else None}
    if has.any():
        out["flow_heading_err_deg_median"] = float(np.degrees(np.nanmedian(np.abs(fl[has, 2] - tr[has, 2]))))
        out["flow_pairs_err_gt_3cm_pct"] = float(100 * (np.linalg.norm(fl[has, :2] - tr[has, :2], axis=1) > 0.03).mean())
        mv = has & (np.linalg.norm(tr[:, :2], axis=1) > 0.01)     # regression of flow on truth over pairs with a fit
        out["flow_scale_pairs"] = float((fl[mv, :2] * tr[mv, :2]).sum() / (tr[mv, :2] ** 2).sum()) if mv.any() else None
        out["flow_inliers_median"] = float(np.median([r["inliers"] for r in rows if r["flow"] is not None]))
    for W in (2, 10):
        out[f"win_{W}s"] = window_scores(rows, W)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--duration", type=float, default=600.0)
    ap.add_argument("--method", choices=["rect", "warp", "image"], default="rect")
    ap.add_argument("--output")
    a = ap.parse_args()
    results = []
    for b in a.bags:
        rng = np.random.default_rng(0)
        rows = run(os.path.expanduser(b), a.duration, rng, a.method)
        s = summarize(os.path.basename(b.rstrip("/")), rows)
        results.append(s)
        print(json.dumps(s))
    if a.output:
        with open(a.output, "w") as f:
            json.dump(results, f, indent=1)


if __name__ == "__main__":
    main()
