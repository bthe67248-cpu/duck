"""Closed-loop trial metrics from a recorded bag (runs in the ROS 2 Python environment).

    python3 tools/trial_metrics.py <bag_dir> [<bag_dir> ...] --world mjcf/world.json [--near 0.35] [--json out.json]

Per bag, from /ground_truth/pose, /slam/pose and /slam/state:
  - SLAM position error (live estimate vs same-stamp truth): mean / max, and the mean split by the TRUE distance from
    the body centre to the nearest obstacle in world.json (walls, partitions, boxes; planks ignored): "near" below
    --near m, "open" otherwise. A body-fixed sensor close to a wall sees little but that wall.
  - time with error above 0.5 m (s and % of the run)
  - localisation health: % of the time /slam/state was "ok" after it was first "ok", the number of not-ok
    episodes and their mean / max length (the recovery time: how long it takes to be healthy again)
"""
import argparse
import bisect
import json
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ap = argparse.ArgumentParser()
ap.add_argument("bags", nargs="+")
ap.add_argument("--world", required=True, help="world.json used by the run (obstacles)")
ap.add_argument("--near", type=float, default=0.25, help="m; true clearance below this counts as near a wall")
ap.add_argument("--json", help="write per-bag metrics here")
args = ap.parse_args()

boxes = [it for it in json.load(open(args.world))["items"] if it["name"] != "beacon" and it["sz"] > 0.01]


def clearance(x, y):
    """Distance from (x, y) to the nearest obstacle rectangle (oriented boxes, half sizes sx, sy)."""
    best = math.inf
    for b in boxes:
        c, s = math.cos(b["yaw"]), math.sin(b["yaw"])
        dx, dy = x - b["x"], y - b["y"]
        lx, ly = abs(c * dx + s * dy) - b["sx"], abs(-s * dx + c * dy) - b["sy"]
        best = min(best, math.hypot(max(lx, 0.0), max(ly, 0.0)) + min(max(lx, ly), 0.0))
    return best


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def read(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=["/ground_truth/pose", "/slam/pose", "/slam/state", "/clock"]))
    gt, est, states, clock = [], [], [], None
    while r.has_next():
        topic, data, _ = r.read_next()
        m = deserialize_message(data, types[topic])
        if topic == "/clock":
            clock = m.clock.sec + m.clock.nanosec * 1e-9
        elif topic == "/ground_truth/pose":
            gt.append((stamp(m.header), m.pose.position.x, m.pose.position.y))
        elif topic == "/slam/pose":
            est.append((stamp(m.header), m.pose.position.x, m.pose.position.y))
        elif clock is not None:                          # String has no header: stamp with the last /clock
            states.append((clock, m.data))
    return sorted(gt), sorted(est), states


def metrics(path):
    gt, est, states = read(path)
    gts = [g[0] for g in gt]
    e, near = [], []
    for t, x, y in est:
        i = bisect.bisect_left(gts, t)
        if i < len(gt) and abs(gts[i] - t) < 0.011:
            e.append(math.hypot(gt[i][1] - x, gt[i][2] - y)); near.append(clearance(gt[i][1], gt[i][2]) < args.near)
    e, near = np.array(e), np.array(near, bool)
    dt = (est[-1][0] - est[0][0]) / max(len(est) - 1, 1) if len(est) > 1 else 0.0
    out = dict(bag=path, samples=int(len(e)), err_mean=float(e.mean()), err_max=float(e.max()),
               err_near=float(e[near].mean()) if near.any() else None, near_pct=float(100 * near.mean()),
               err_open=float(e[~near].mean()) if (~near).any() else None,
               err_over_05_s=float((e > 0.5).sum() * dt), err_over_05_pct=float(100 * (e > 0.5).mean()))
    first = next((k for k, (_, s) in enumerate(states) if s == "ok"), None)
    if first is not None:
        st = states[first:]
        eps, start, ok_t, tot = [], None, 0.0, 0.0
        for (t0, s0), (t1, _) in zip(st, st[1:]):
            tot += t1 - t0
            ok_t += (t1 - t0) if s0 == "ok" else 0.0
            if s0 != "ok" and start is None:
                start = t0
            elif s0 == "ok" and start is not None:
                eps.append(t0 - start); start = None
        if start is not None:
            eps.append(st[-1][0] - start)
        out.update(ok_pct=float(100 * ok_t / tot) if tot else None, not_ok_episodes=len(eps),
                   recovery_mean_s=float(np.mean(eps)) if eps else 0.0, recovery_max_s=float(max(eps)) if eps else 0.0)
    return out


rows = [metrics(b) for b in args.bags]
for r in rows:
    print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()}), flush=True)
if args.json:
    with open(args.json, "w") as f:
        json.dump(rows, f, indent=1)
