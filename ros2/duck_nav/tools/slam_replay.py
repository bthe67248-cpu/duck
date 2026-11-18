"""Offline SLAM replay against ground truth (ToF / realistic profile). Runs in the ROS 2 Python environment.

    python3 tools/slam_replay.py <bag_dir> [--config JSON | --grid JSON_LIST] [--src <dir containing duck_nav/>]

Reads /imu, /cmd_vel, /tof/zones, /tof/pose_level, /ground_truth/pose and /clock (plus the recorded /slam/pose
for reference) from a bag recorded with the realistic profile, runs the slam_node pipeline (tilt filter,
gait-command prediction, acquisition-time ToF processing, optional sweep window, matching, mapping) without
ROS, and reports pose error against same-stamp ground truth. Open loop: the robot's path is the recorded one,
so this measures localisation, not navigation (a variant that would have stopped the robot is still scored on
the recorded motion; "ok" is how often its motion gate would have allowed driving). It mirrors
slam_node.on_imu / on_frame; keep the two in step when either changes.

Config keys (defaults = current slam_node behaviour):
  matcher   correlative | grid | none          sweep    seconds of accumulated frames (0 = single frame)
  min_hits  hits needed to match               map      always | gated (accepted, healthy or bootstrap)
  fresh     current-frame agreement check      kw       extra keyword arguments for the matcher
"""
import argparse
import bisect
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("--config", default="{}")
ap.add_argument("--grid", help="JSON list of configs, evaluated on one read of the bag")
ap.add_argument("--src", default=str(Path(__file__).resolve().parents[1]))
args = ap.parse_args()
sys.path.insert(0, args.src)
import duck_nav.duckslam as ds  # noqa: E402

import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402

TOPICS = ["/imu", "/cmd_vel", "/tof/zones", "/tof/pose_level", "/ground_truth/pose", "/clock", "/slam/pose"]
DEFAULT = dict(matcher="correlative", sweep=1.5, min_hits=12, map="gated", fresh=True, kw={})


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
clock = sorted((t_rx, m.clock.sec + m.clock.nanosec * 1e-9) for tp, m, t_rx in msgs if tp == "/clock")
crx = [c[0] for c in clock]


def sim_time(t_rx):                                   # receive (wall) time -> sim time, for header-less /cmd_vel
    i = min(max(bisect.bisect_left(crx, t_rx), 1), len(clock) - 1)
    return clock[i - 1][1] if t_rx - clock[i - 1][0] < clock[i][0] - t_rx else clock[i][1]


def read_tof(msg):
    ranges = np.full(64, np.nan); valid = np.zeros(64, bool)
    for rng, st, z in point_cloud2.read_points(msg, field_names=("range", "status", "zone"), skip_nans=False):
        if st == 0 and np.isfinite(rng):
            ranges[int(z)] = rng; valid[int(z)] = True
    return ranges, valid


events, tof_pose, gt, live = [], {}, [], []
for tp, m, t_rx in msgs:
    if tp == "/imu":
        g = m.angular_velocity; a = m.linear_acceleration
        events.append((stamp(m.header), 0, (np.array([g.x, g.y, g.z]), np.array([a.x, a.y, a.z]))))
    elif tp == "/cmd_vel":
        events.append((sim_time(t_rx), 1, (m.linear.x, m.linear.y)))
    elif tp == "/tof/zones":
        events.append((stamp(m.header) + 1e-6, 2, (round(stamp(m.header), 6), read_tof(m))))
    elif tp == "/tof/pose_level":
        o, p = m.pose.orientation, m.pose.position
        tof_pose[round(stamp(m.header), 6)] = (quat_to_mat(o.w, o.x, o.y, o.z), np.array([p.x, p.y, p.z]))
    elif tp == "/slam/pose":
        live.append((stamp(m.header), m.pose.position.x, m.pose.position.y))
    elif tp == "/ground_truth/pose":
        gt.append((stamp(m.header), m.pose.position.x, m.pose.position.y))
events.sort(key=lambda e: (e[0], e[1]))
gt.sort(); gts = [g[0] for g in gt]
dur = events[-1][0] - events[0][0]


def truth_err(t, x, y):
    i = bisect.bisect_left(gts, t)
    if i < len(gt) and abs(gts[i] - t) < 0.011:
        return math.hypot(gt[i][1] - x, gt[i][2] - y)
    return None


def run(cfg):
    c = {**DEFAULT, **cfg}
    slam = ds.DuckSLAM(ds.OdomGains(), max_r=2.0, min_hits=c["min_hits"])
    rays = ds.tof_zone_rays(8, 45.0, 45.0)
    tilt = ds.TiltFilter()
    health = ds.LocalizationHealth(0.75, 1.5)
    sweep = ds.ScanWindow(c["sweep"]) if c["sweep"] > 0 else None
    cmd, cmd_t, last_imu = (0.0, 0.0), -1e9, None
    hist, hist_keys = {}, []
    bootstrap = True
    errs, oks, n_frames, n_acc, match_s, reasons = [], [], 0, 0, [], {}
    slips = []
    next_err_t = 0.0
    for t, kind, data in events:
        if kind == 0:
            gyro, acc = data
            dt = t - last_imu if last_imu is not None else 0.0
            last_imu = t
            up = tilt.update(gyro, acc, dt if 0 < dt < 0.2 else 0.0)
            if 0 < dt < 0.2:
                vx, vy = cmd if t - cmd_t < 0.5 else (0.0, 0.0)
                slam.predict(vx, ds.world_yaw_rate(gyro, up), dt, cmd_vy=vy)
            hist[t] = slam.pose.copy(); hist_keys.append(t)
            if t >= next_err_t:
                next_err_t = t + 0.1
                e = truth_err(t, *slam.pose[:2])
                if e is not None:
                    errs.append(e); oks.append(health.state(t) == "ok")
            continue
        if kind == 1:
            cmd, cmd_t = data, t
            continue
        key, (ranges, valid) = data
        ts = t - 1e-6
        sp = tof_pose.get(key)
        i = bisect.bisect_left(hist_keys, ts)
        cand = [hist_keys[j] for j in (i - 1, i) if 0 <= j < len(hist_keys) and abs(hist_keys[j] - ts) <= 0.03]
        if sp is None or not cand or c["matcher"] == "none":
            continue
        k = min(cand, key=lambda q: abs(q - ts))
        pose_acq = hist[k].copy()
        R, tt = sp
        ang, r, hit = slam.scan_from_tof(ranges, valid, rays, R, tt, "radial", fov_h_deg=45.0)
        n_frames += 1
        if not len(ang):
            continue
        current = slam.pose.copy(); slam.pose = pose_acq.copy()
        if sweep is not None:
            sweep.add(ts, pose_acq, ang, r, hit)
            cons = sweep.scan(pose_acq)
        else:
            cons = (ang, r, hit)
        fresh = (ang, r, hit) if c["fresh"] else None
        t0 = time.perf_counter()
        if c["matcher"] == "correlative":
            accepted = slam.match_correlative(*cons, current_scan=fresh, **c["kw"])
        else:
            accepted = slam.match(*cons, current_scan=fresh)
        match_s.append(time.perf_counter() - t0)
        reasons[slam.match_reason] = reasons.get(slam.match_reason, 0) + 1
        corr = slam.pose - pose_acq
        if accepted:
            n_acc += 1
            if sweep is not None:
                sweep.correct(pose_acq, slam.pose)
        healthy = health.state(ts) == "ok" and slam.match_reason not in (
            "fresh_scan_disagrees", "search_boundary", "ambiguous", "ambiguous_heading")
        if c.get("debug") and n_frames % 300 == 0:
            print(f"   t={ts:6.1f} occ={(slam.logodds > 0.8).sum():5d} hits={int(hit.sum())} cons={int(cons[2].sum())} "
                  f"bootstrap={bootstrap} reason={slam.match_reason} pose={np.round(slam.pose, 2)}")
        if c["map"] == "always" or accepted or healthy or bootstrap or slam.match_reason == "map_not_ready":
            slam.update_map(ang, r, hit)
        slam.pose = current + corr
        if np.any(corr):
            for kk in hist_keys[bisect.bisect_right(hist_keys, ts):]:
                hist[kk] = hist[kk] + corr
        health.observe(ts, accepted)
        if bootstrap and health.state(ts) == "ok":
            bootstrap = False
        if c.get("slip"):
            slam.observe_slip(**c.get("slip_kw", {}))
            slips.append(slam.slip)
    e = np.array(errs)
    if c.get("trace"):
        w = int(20 / 0.1)
        if slips:
            k = max(1, len(slips) // 15)
            print("   slip per window:", " ".join(f"{np.mean(slips[i:i + k]):.2f}" for i in range(0, len(slips), k)))
        print("   error per 20 s:", " ".join(f"{e[i:i + w].mean():.2f}" for i in range(0, len(e), w)))
    top = dict(sorted(reasons.items(), key=lambda kv: -kv[1])[:4])
    print(f"{json.dumps(cfg):70s} err mean {e.mean():.3f} med {np.median(e):.3f} max {e.max():.3f} | "
          f"ok {100 * np.mean(oks):3.0f}% acc {n_acc:4d}/{n_frames} {1e3 * np.mean(match_s) if match_s else 0:4.1f} ms | {top}",
          flush=True)
    return e.mean()


if live:
    le = np.array([v for v in (truth_err(t_, x_, y_) for t_, x_, y_ in live[::5]) if v is not None])
    print(f"{dur:.0f} s bag; recorded live /slam/pose: err mean {le.mean():.3f} med {np.median(le):.3f} max {le.max():.3f}")
for cfg in (json.loads(args.grid) if args.grid else [json.loads(args.config)]):
    run(cfg)
