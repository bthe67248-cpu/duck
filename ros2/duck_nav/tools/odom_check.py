"""Where does dead reckoning go wrong? Compare gait-command + gyro odometry with ground truth, per segment.

    python3 tools/odom_check.py <bag_dir> [--seg 10]

For each segment of --seg seconds: true and predicted distance travelled (body frame forward/sideways),
and heading change error. Runs in the ROS 2 Python environment.
"""
import argparse
import bisect
import math
import sys
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("--seg", type=float, default=10.0)
args = ap.parse_args()
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import duck_nav.duckslam as ds  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/imu", "/cmd_vel", "/ground_truth/pose", "/clock"]))
msgs = []
while r.has_next():
    tp, d, trx = r.read_next()
    msgs.append((tp, deserialize_message(d, types[tp]), trx * 1e-9))
clock = sorted((trx, m.clock.sec + m.clock.nanosec * 1e-9) for tp, m, trx in msgs if tp == "/clock")
crx = [c[0] for c in clock]


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def sim_time(t):
    i = min(max(bisect.bisect_left(crx, t), 1), len(clock) - 1)
    return clock[i][1]


ev = []
gt = []
for tp, m, trx in msgs:
    if tp == "/imu":
        ev.append((st(m.header), 0, m))
    elif tp == "/cmd_vel":
        ev.append((sim_time(trx), 1, m))
    else:
        if tp == "/ground_truth/pose":
            o = m.pose.orientation
            gt.append((st(m.header), m.pose.position.x, m.pose.position.y, 2 * math.atan2(o.z, o.w)))
ev.sort(key=lambda e: (e[0], e[1])); gt.sort(); gts = [g[0] for g in gt]
s = ds.DuckSLAM(ds.OdomGains()); tilt = ds.TiltFilter()
cmd, cmd_t, last = (0.0, 0.0), -1e9, None
rows = []
seg_start = None


def truth(t):
    i = min(bisect.bisect_left(gts, t), len(gt) - 1)
    return np.array(gt[i][1:])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


cmd_sum = 0.0
for t, k, m in ev:
    if k == 1:
        cmd, cmd_t = (m.linear.x, m.linear.y), t
        continue
    g = np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    a = np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    dt = t - last if last is not None else 0.0
    last = t
    up = tilt.update(g, a, dt if 0 < dt < 0.2 else 0.0)
    if 0 < dt < 0.2:
        vx = cmd[0] if t - cmd_t < 0.5 else 0.0
        s.predict(vx, ds.world_yaw_rate(g, up), dt)
        cmd_sum += vx * dt
    if seg_start is None:
        seg_start = (t, truth(t), s.pose.copy()); cmd_sum = 0.0
    elif t - seg_start[0] >= args.seg:
        t0, g0, p0 = seg_start; g1, p1 = truth(t), s.pose.copy()
        c, sn = math.cos(g0[2]), math.sin(g0[2])
        dg = g1[:2] - g0[:2]; dgb = np.array([c * dg[0] + sn * dg[1], -sn * dg[0] + c * dg[1]])
        c2, s2 = math.cos(p0[2]), math.sin(p0[2])
        dp = p1[:2] - p0[:2]; dpb = np.array([c2 * dp[0] + s2 * dp[1], -s2 * dp[0] + c2 * dp[1]])
        rows.append((t0, dgb[0], dpb[0], dgb[1], dpb[1], math.degrees(wrap((g1[2] - g0[2]) - (p1[2] - p0[2]))), cmd_sum))
        seg_start = (t, g1, p1); cmd_sum = 0.0
R = np.array(rows)
print(" t0    true fwd  pred fwd   true side pred side  heading err  cmd*dt")
for row in R:
    print(f"{row[0]:5.0f}  {row[1]:+7.3f}  {row[2]:+7.3f}    {row[3]:+7.3f}  {row[4]:+7.3f}   {row[5]:+6.1f} deg   {row[6]:5.2f}")
mov = R[:, 6] > 0.3
print(f"\nforward: true {R[:, 1].sum():.2f} m, predicted {R[:, 2].sum():.2f} m; moving segments ratio true/pred "
      f"{R[mov, 1].sum() / R[mov, 2].sum():.3f}; true m per cmd unit {R[mov, 1].sum() / R[mov, 6].sum():.3f}")
print(f"sideways: true {R[:, 3].sum():+.2f} m (unmodelled), |sum| {np.abs(R[:, 3]).sum():.2f} m")
print(f"heading: sum of segment errors {R[:, 5].sum():+.1f} deg, mean |err| {np.abs(R[:, 5]).mean():.1f} deg per {args.seg:.0f} s")
