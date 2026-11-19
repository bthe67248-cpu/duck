"""heading_sweep.py BAG [BAG ...] -- open-loop heading error of the attitude filters against ground truth.

Integrates the recorded /imu with TiltFilter (the older gyro-projected-on-'up' heading) and AttitudeFilter for
a few gain settings, and prints the mean absolute heading error per bag, up to the first fall (true tilt > 60 deg).
Run in WSL after `source /opt/ros/jazzy/setup.bash`, from the repository root:
    python3 results/heading/tools/heading_sweep.py ~/duck_bags/new_s1 ~/duck_bags/new_s2
"""
import bisect
import itertools
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "ros2", "duck_nav"))
import duck_nav.duckslam as ds  # noqa: E402


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=["/imu", "/ground_truth/pose"]))
    gt, imu = [], []
    while r.has_next():
        tp, d, _ = r.read_next()
        m = deserialize_message(d, types[tp])
        if tp == "/imu":
            imu.append((stamp(m.header), np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]),
                        np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])))
        else:
            o = m.pose.orientation
            gt.append((stamp(m.header), np.array([o.w, o.x, o.y, o.z])))
    return sorted(gt, key=lambda g: g[0]), sorted(imu, key=lambda i: i[0])


def score(gt, imu, make):
    gts = [g[0] for g in gt]
    f, last, errs = make(), None, []
    psi = ds.quat_yaw(gt[min(bisect.bisect_left(gts, imu[0][0]), len(gt) - 1)][1])
    for t, g, a in imu:
        dt = t - last if last is not None else 0.0
        last = t
        f.update(g, a, dt if 0 < dt < 0.2 else 0.0)
        if 0 < dt < 0.2:
            psi += f.heading_rate(g, dt) * dt
        q = gt[min(bisect.bisect_left(gts, t), len(gt) - 1)][1]
        if 1 - 2 * (q[1] ** 2 + q[2] ** 2) < math.cos(math.radians(60)):
            break
        errs.append(abs(wrap(psi - ds.quat_yaw(q))))
    return math.degrees(float(np.mean(errs)))


def main():
    data = {os.path.basename(p.rstrip("/")): load(os.path.expanduser(p)) for p in sys.argv[1:]}
    rows = [("tilt (old)", ds.TiltFilter)]
    rows += [(f"quaternion kp {kp} band {band}", (lambda kp=kp, band=band: ds.AttitudeFilter(kp=kp, accel_band=band)))
             for kp, band in itertools.product([0.0, 0.3, 1.0, 3.0], [0.05, 0.15])]
    print("mean |heading error| deg per bag:", ", ".join(data))
    for name, make in rows:
        r = [score(gt, imu, make) for gt, imu in data.values()]
        print(f"  {name:30s}", " ".join(f"{v:5.1f}" for v in r), f"  avg {np.mean(r):.1f}")


if __name__ == "__main__":
    main()
