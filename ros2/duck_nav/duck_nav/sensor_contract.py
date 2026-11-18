"""ROS-independent acquisition and frame conversions for hardware sensor adapters.

No device bus protocol is implemented here. Backends must provide metres, canonical
row-major zone order, normalized statuses and acquisition times in the ROS clock.
"""
from dataclasses import dataclass

import numpy as np

from .duckslam import tof_zone_rays


@dataclass(frozen=True)
class TofFrame:
    stamp_ns: int
    ranges: object
    status: object  # 0 valid, 1 no target/out of range, 2 dropped/invalid


def tof_records(frame, convention, fov_deg=45.0, min_range=0.02, max_range=2.0):
    """Return 64 records matching world_sim's x,y,z,range,status,zone schema.

    Invalid zones always have NaN geometry and range: they must never clear space.
    Limits apply to the backend's declared range convention, in metres.
    """
    if not isinstance(frame.stamp_ns, int) or isinstance(frame.stamp_ns, bool) or frame.stamp_ns <= 0:
        raise ValueError("stamp_ns must be a positive integer acquisition time in the ROS clock")
    if convention not in ("radial", "axial"):
        raise ValueError("range convention must be explicitly radial or axial")
    if not (np.isfinite([fov_deg, min_range, max_range]).all()
            and 0 < fov_deg < 180 and 0 < min_range < max_range):
        raise ValueError("invalid field of view or range limits")
    ranges, status = np.asarray(frame.ranges, float), np.asarray(frame.status)
    if ranges.shape != (64,) or status.shape != (64,):
        raise ValueError("backend must return 64 canonical row-major ranges and statuses")
    if not np.isin(status, [0, 1, 2]).all():
        raise ValueError("backend must normalize sensor status codes to 0, 1, 2")
    status = status.astype(np.uint8).copy()
    status[(status == 0) & (~np.isfinite(ranges) | (ranges < min_range) | (ranges > max_range))] = 2
    rays = tof_zone_rays(8, fov_deg, fov_deg)
    records = []
    for zone, (r, s, ray) in enumerate(zip(ranges, status, rays)):
        if s == 0:
            xyz = ray * (r / ray[0] if convention == "axial" else r)
            records.append((*xyz.tolist(), float(r), int(s), zone))
        else:
            records.append((float("nan"),) * 4 + (int(s), zone))
    return records


def optical_to_mujoco_quaternion(xyzw):
    """Compose level<-optical with optical<-MuJoCo (180 degrees about x).

    Translation is unchanged. Optical is x right, y down, z forward; the SLAM
    camera frame is x right, y up, -z forward. Quaternions use ROS x,y,z,w order.
    """
    q = np.asarray(xyzw, float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError("expected finite nonzero xyzw quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([w, z, -y, -x])


def positive_seconds(value, name):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def acquisition_is_fresh(stamp_ns, now_ns, max_age, last_stamp_ns=None):
    """Clock-domain and monotonic-acquisition gate; receipt time cannot refresh data."""
    positive_seconds(max_age, "max_age")
    if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns <= 0:
        return False
    return (last_stamp_ns is None or stamp_ns > last_stamp_ns) and 0 <= now_ns - stamp_ns <= max_age * 1e9
