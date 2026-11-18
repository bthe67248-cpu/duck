"""Offline SLAM replay against ground truth (ToF / realistic profile). Runs in the ROS 2 Python environment.

    python3 tools/slam_replay.py <bag_dir> [--variant patch|opt|olson] [--src <dir containing duck_nav/>]

Reads /imu, /cmd_vel, /tof/zones, /tof/pose_level, /ground_truth/pose and /clock from a bag recorded with
the realistic profile, runs the slam_node pipeline (tilt filter, gait-command prediction, acquisition-time
ToF processing, sweep window, matching, mapping) without ROS, and reports pose error against ground truth.
Open loop: the robot's path is the recorded one, so this measures localisation, not navigation. It mirrors
slam_node.on_imu / on_frame; keep the two in step when either changes.
"""
import argparse
import bisect
import math
import sys
import time
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("--variant", default="opt", choices=["patch", "opt", "olson"])
ap.add_argument("--src", default=str(Path(__file__).resolve().parents[1]))
ap.add_argument("--quiet", action="store_true")
args = ap.parse_args()
sys.path.insert(0, args.src)
import duck_nav.duckslam as ds  # noqa: E402

import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402

TOPICS = ["/imu", "/cmd_vel", "/tof/zones", "/tof/pose_level", "/ground_truth/pose", "/clock"]


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def quat_to_mat(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    out = []
    while r.has_next():
        topic, data, t_rx = r.read_next()
        out.append((topic, deserialize_message(data, types[topic]), t_rx * 1e-9))
    return out


msgs = read_bag(args.bag)
clock = [(t_rx, m.clock.sec + m.clock.nanosec * 1e-9) for tp, m, t_rx in msgs if tp == "/clock"]
crx = [c[0] for c in clock]


def sim_time(t_rx):                                   # receive (wall) time -> sim time, for header-less /cmd_vel
    i = min(max(bisect.bisect_left(crx, t_rx), 1), len(clock) - 1)
    return clock[i - 1][1] if t_rx - clock[i - 1][0] < clock[i][0] - t_rx else clock[i][1]


events = []
tof_pose = {}
gt = []
for tp, m, t_rx in msgs:
    if tp == "/imu":
        events.append((stamp(m.header), 0, m))
    elif tp == "/cmd_vel":
        events.append((sim_time(t_rx), 1, m))
    elif tp == "/tof/zones":
        events.append((stamp(m.header) + 1e-6, 2, m))  # after the IMU sample of the same stamp, like live delivery
    elif tp == "/tof/pose_level":
        o, p = m.pose.orientation, m.pose.position
        tof_pose[round(stamp(m.header), 6)] = (quat_to_mat(o.w, o.x, o.y, o.z), np.array([p.x, p.y, p.z]))
    elif tp == "/ground_truth/pose":
        o = m.pose.orientation
        gt.append((stamp(m.header), m.pose.position.x, m.pose.position.y, 2 * math.atan2(o.z, o.w)))
events.sort(key=lambda e: (e[0], e[1]))
gt.sort(); gts = [g[0] for g in gt]

gains = ds.OdomGains()
slam = ds.DuckSLAM(gains, max_r=2.0, min_hits=4 if args.variant == "main" else 12)
rays = ds.tof_zone_rays(8, 45.0, 45.0)
tilt = ds.TiltFilter()
health = ds.LocalizationHealth(0.75, 1.5) if hasattr(ds, "LocalizationHealth") else None
sweep = ds.ScanWindow(1.5) if hasattr(ds, "ScanWindow") else None
cmd, cmd_t, last_imu = (0.0, 0.0), -1e9, None
hist = {}                     # stamp -> pose
hist_keys = []
bootstrap = True
errs, ok_frac, n_frames, n_acc, match_s = [], [], 0, 0, []
reasons = {}


def nearest_hist(t, tol=0.03):
    i = bisect.bisect_left(hist_keys, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(hist_keys) and abs(hist_keys[j] - t) <= tol and (best is None or abs(hist_keys[j] - t) < abs(best - t)):
            best = hist_keys[j]
    return best


def read_tof(msg):
    ranges = np.full(64, np.nan); valid = np.zeros(64, bool)
    for rng, st, z in point_cloud2.read_points(msg, field_names=("range", "status", "zone"), skip_nans=False):
        if st == 0 and np.isfinite(rng):
            ranges[int(z)] = rng; valid[int(z)] = True
    return ranges, valid


next_err_t = 0.0
for t, kind, m in events:
    if kind == 0:
        gyro = np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        acc = np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        dt = t - last_imu if last_imu is not None else 0.0
        last_imu = t
        up = tilt.update(gyro, acc, dt if 0 < dt < 0.2 else 0.0)
        if 0 < dt < 0.2:
            vx, vy = cmd if t - cmd_t < 0.5 else (0.0, 0.0)
            slam.predict(vx, ds.world_yaw_rate(gyro, up), dt, cmd_vy=vy)
        hist[t] = slam.pose.copy(); hist_keys.append(t)
        if t >= next_err_t:                                   # error sampled at 10 Hz against same-stamp truth
            next_err_t = t + 0.1
            i = bisect.bisect_left(gts, t)
            if i < len(gt) and abs(gts[i] - t) < 0.011:
                errs.append(math.hypot(gt[i][1] - slam.pose[0], gt[i][2] - slam.pose[1]))
                if health is not None:
                    ok_frac.append(health.state(t) == "ok")
    elif kind == 1:
        cmd, cmd_t = (m.linear.x, m.linear.y), t
    else:
        ts = t - 1e-6
        sp = tof_pose.get(round(ts, 6))
        k = nearest_hist(ts)
        if sp is None or k is None:
            continue
        pose_acq = hist[k].copy()
        R, tt = sp
        ranges, valid = read_tof(m)
        ang, r, hit = slam.scan_from_tof(ranges, valid, rays, R, tt, "radial", fov_h_deg=45.0)
        n_frames += 1
        if not len(ang):
            continue
        current = slam.pose.copy(); slam.pose = pose_acq.copy()
        t0 = time.perf_counter()
        if args.variant == "olson":
            sweep.add(ts, pose_acq, ang, r, hit)
            accepted = slam.match_correlative(*sweep.scan(pose_acq), current_scan=(ang, r, hit))
        else:
            sweep.add(ts, pose_acq, ang, r, hit)
            accepted = slam.match(*sweep.scan(pose_acq), current_scan=(ang, r, hit))
        match_s.append(time.perf_counter() - t0)
        reasons[slam.match_reason] = reasons.get(slam.match_reason, 0) + 1
        corr = slam.pose - pose_acq
        if accepted:
            bootstrap = False; n_acc += 1
            sweep.correct(pose_acq, slam.pose)
        stationary = np.linalg.norm(pose_acq[:2]) < 0.05 and abs(pose_acq[2]) < math.radians(3)
        healthy = (health.state(ts) == "ok" and slam.match_reason not in ("fresh_scan_disagrees", "search_boundary"))
        if args.variant == "patch":
            do_map = accepted or (bootstrap and stationary)
        else:
            do_map = accepted or healthy or (bootstrap and stationary)
        if do_map:
            slam.update_map(ang, r, hit)
        slam.pose = current + corr
        if np.any(corr):
            for kk in hist_keys[bisect.bisect_right(hist_keys, ts):]:
                hist[kk] = hist[kk] + corr
        health.observe(ts, accepted)

e = np.array(errs)
dur = events[-1][0] - events[0][0]
print(f"variant={args.variant:6s} {dur:5.0f} s  error mean {e.mean():.3f} m  median {np.median(e):.3f}  "
      f"max {e.max():.3f}  final {e[-1]:.3f}  | ok {100 * np.mean(ok_frac):.0f}% of time  "
      f"accepted {n_acc}/{n_frames}  match {1e3 * np.mean(match_s):.1f} ms")
if not args.quiet:
    print("  reasons:", dict(sorted(reasons.items(), key=lambda kv: -kv[1])))
