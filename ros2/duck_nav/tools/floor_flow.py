"""floor_flow.py BAG [BAG ...] [--duration S] [--output JSON] -- offline floor optical-flow odometry, scored
against ground truth next to the gait-command odometry slam_node uses.

Needs a bag recorded with /camera/image_raw, /camera/pose_level, /camera/camera_info, /ground_truth/pose and
/cmd_vel (tools/trial_matrix.py records them). Simulation geometry: /camera/pose_level is the camera pose in the
level body frame (origin on the floor under the torso, heading removed) at the image stamp; on the robot this
comes from neck/leg encoders + IMU tilt.

Per consecutive image pair:
  1. corners in frame k (lower part of the image, where the floor is)
  2. predicted position in frame k+1 from the known head motion with zero body motion (the floor is a repeating
     0.2 m checker: seeding the tracker this way leaves only the body motion, a few cm, to search)
  3. pyramidal Lucas-Kanade tracking, forward-backward check
  4. both pixel sets back-projected onto the floor plane z = 0 of their own level frame
  5. RANSAC 2-D rigid fit p_k = R(dth) p_k1 + t: (t, dth) is the body motion over the pair, in frame k
Scored per pair and integrated over the run with the TRUE heading (so only translation is compared):
  flow        the estimate above
  command     gait-command x gain (OdomGains), what slam_node predicts before slip estimation / scan matching
  flow_or_command  the flow estimate, or the command for pairs where flow found no fit
Prints per-bag drift and per-pair error statistics. Run in WSL after `source /opt/ros/jazzy/setup.bash`.
"""
import argparse
import bisect
import json
import math
import os
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import duck_nav.duckslam as ds  # noqa: E402

TOPICS = ["/camera/image_raw", "/camera/pose_level", "/camera/camera_info", "/ground_truth/pose", "/cmd_vel",
          "/clock"]


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
    u = (px[:, 0] + 0.5 - K[0, 2]) / K[0, 0]
    v = (px[:, 1] + 0.5 - K[1, 2]) / K[1, 1]
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


def run(path, duration, rng):
    imgs, cposes, K, gt, cmd_sim, stamps = load(path, duration)
    gts = [g[0] for g in gt]; cts = [c[0] for c in cmd_sim]
    gains = ds.OdomGains()
    lk = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
              flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
    H, W = next(iter(imgs.values())).shape
    mask = np.zeros((H, W), np.uint8); mask[H // 3:, :] = 255     # floor is in the lower part when looking ahead
    rows = []
    pos = {"truth": None, "flow": None, "command": None}
    for k in range(len(stamps) - 1):
        s0, s1 = stamps[k], stamps[k + 1]
        if not 0.1 < s1 - s0 < 0.3:
            continue
        g0, g1 = gt_at(gt, gts, s0), gt_at(gt, gts, s1)
        if min(g0[2], g1[2]) < math.cos(math.radians(30)):       # fallen / falling: skip (camera is not on the floor)
            pos = {key: None for key in pos}
            continue
        R0, t0 = cposes[s0]; R1, t1 = cposes[s1]
        c, s = math.cos(g0[1][2]), math.sin(g0[1][2])
        d = g1[1][:2] - g0[1][:2]
        true_rel = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], wrap(g1[1][2] - g0[1][2])])
        cmd_rel = command_motion(cmd_sim, cts, s0, s1, gains)
        est = None
        p0 = cv2.goodFeaturesToTrack(imgs[s0], 200, 0.01, 6, mask=mask)
        if p0 is not None and len(p0) >= 8:
            p0 = p0.reshape(-1, 2)
            f0 = floor_points(p0, R0, t0, K)
            ok = np.isfinite(f0).all(1) & (np.linalg.norm(f0 - t0[:2], axis=1) < 1.5)
            p0, f0 = p0[ok], f0[ok]
            if len(p0) >= 8:
                guess = project(f0, R1, t1, K)                       # zero body motion, known head motion
                good = np.isfinite(guess).all(1)
                p0, f0, guess = p0[good], f0[good], guess[good]
                if len(p0) >= 8:
                    p1, st, _ = cv2.calcOpticalFlowPyrLK(imgs[s0], imgs[s1], p0.astype(np.float32).reshape(-1, 1, 2),
                                                         guess.astype(np.float32).reshape(-1, 1, 2), **lk)
                    back, st2, _ = cv2.calcOpticalFlowPyrLK(imgs[s1], imgs[s0], p1, p0.astype(np.float32).reshape(-1, 1, 2),
                                                            **lk)
                    p1 = p1.reshape(-1, 2)
                    fb = np.linalg.norm(back.reshape(-1, 2) - p0, axis=1)
                    ok = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < 1.0)
                    if ok.sum() >= 8:
                        f1 = floor_points(p1[ok], R1, t1, K)
                        a, b = f0[ok], f1
                        fin = np.isfinite(b).all(1)
                        if fin.sum() >= 8:
                            fit = ransac_rigid(a[fin], b[fin], rng)
                            if fit is not None:
                                est = (fit[0], fit[1], fit[2])
        row = dict(t=s0, true=true_rel.tolist(), cmd=cmd_rel.tolist(),
                   flow=None if est is None else [float(est[0][0]), float(est[0][1]), float(est[1])],
                   inliers=0 if est is None else est[2])
        rows.append(row)
    return rows


def integrate(rows, key, seg=60.0):
    """Integrate body-frame translations with the TRUE heading; restart every `seg` s and after any gap.
    Returns per-segment end errors (m) and travelled distance (m). Missing flow estimates count as zero motion."""
    errs, dist = [], []
    t_start = None; pe = pt = None; th = 0.0; last_t = None; travelled = 0.0
    for r in rows:
        if t_start is None or r["t"] - t_start >= seg or (last_t is not None and r["t"] - last_t > 0.5):
            if pe is not None and travelled > 0.2:
                errs.append(float(np.linalg.norm(pe - pt))); dist.append(travelled)
            t_start, pe, pt, th, travelled = r["t"], np.zeros(2), np.zeros(2), 0.0, 0.0
        last_t = r["t"]
        c, s = math.cos(th), math.sin(th)
        Rm = np.array([[c, -s], [s, c]])
        tr = np.array(r["true"][:2])
        if key == "command":
            src = r["cmd"]
        elif key == "flow":
            src = r["flow"]
        else:                                                     # flow, falling back to the command when missing
            src = r["flow"] if r["flow"] is not None else r["cmd"]
        m = src[:2] if src is not None else [0.0, 0.0]
        pt = pt + Rm @ tr; pe = pe + Rm @ np.array(m)
        travelled += float(np.linalg.norm(tr))
        th += r["true"][2]
    if pe is not None and travelled > 0.2:
        errs.append(float(np.linalg.norm(pe - pt))); dist.append(travelled)
    return errs, dist


def summarize(name, rows):
    tr = np.array([r["true"] for r in rows])
    cm = np.array([r["cmd"] for r in rows])
    has = np.array([r["flow"] is not None for r in rows])
    fl = np.array([r["flow"] if r["flow"] is not None else [np.nan] * 3 for r in rows])
    out = dict(bag=name, pairs=len(rows), flow_coverage=float(has.mean()))
    out["pair_err_cm"] = {
        "command": float(np.median(np.linalg.norm(cm - tr[:, :2], axis=1)) * 100),
        "flow": float(np.nanmedian(np.linalg.norm(fl[has, :2] - tr[has, :2], axis=1)) * 100),
    }
    out["flow_heading_err_deg_median"] = float(np.degrees(np.nanmedian(np.abs(fl[has, 2] - tr[has, 2]))))
    ferr = np.linalg.norm(fl[has, :2] - tr[has, :2], axis=1)
    mv = has & (np.linalg.norm(tr[:, :2], axis=1) > 0.01)        # scale: estimated / true motion along the true direction
    if mv.any():
        u = tr[mv, :2] / np.linalg.norm(tr[mv, :2], axis=1, keepdims=True)
        out["flow_scale"] = float(np.sum(fl[mv, :2] * u) / np.sum(np.linalg.norm(tr[mv, :2], axis=1)))
        out["command_scale"] = float(np.sum(cm[mv] * u) / np.sum(np.linalg.norm(tr[mv, :2], axis=1)))
        out["flow_lateral_bias_cm"] = float(np.mean(fl[mv, 1] - tr[mv, 1]) * 100)
    out["flow_pairs_err_gt_3cm_pct"] = float(100 * (ferr > 0.03).mean()) if has.any() else None
    out["flow_inliers_median"] = float(np.median([r["inliers"] for r in rows if r["flow"] is not None])) if has.any() else None
    for key in ("command", "flow", "flow_or_command"):
        e, d = integrate(rows, key)
        out[f"{key}_drift_pct"] = float(100 * np.sum(e) / max(np.sum(d), 1e-9))
        out[f"{key}_seg_err_m_mean"] = float(np.mean(e)) if e else None
    stuck = (np.linalg.norm(tr[:, :2], axis=1) < 0.005) & (np.linalg.norm(cm, axis=1) > 0.02)
    out["pushing_pairs"] = int(stuck.sum())
    if stuck.any():
        out["pushing_err_cm"] = {"command": float(np.median(np.linalg.norm(cm[stuck] - tr[stuck, :2], axis=1)) * 100),
                                 "flow": float(np.nanmedian(np.linalg.norm(fl[stuck & has, :2] - tr[stuck & has, :2], axis=1)) * 100)
                                 if (stuck & has).any() else None}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--duration", type=float, default=600.0)
    ap.add_argument("--output")
    a = ap.parse_args()
    rng = np.random.default_rng(0)
    results = []
    for b in a.bags:
        rows = run(os.path.expanduser(b), a.duration, rng)
        s = summarize(os.path.basename(b.rstrip("/")), rows)
        results.append(s)
        print(json.dumps(s))
    if a.output:
        with open(a.output, "w") as f:
            json.dump(results, f, indent=1)


if __name__ == "__main__":
    main()
