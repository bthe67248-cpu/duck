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


def zone_rays(zones=8, fov_deg=45.0, fov_v_deg=None, rays=None):
    """Unit centre rays for an n x n zone grid (sensor frame, +x boresight), row-major top-left first.
    `fov_v_deg` None or 0 = square field of view. `rays` (n*n, 3) replaces the pinhole model with calibrated
    per-zone directions (normalised here)."""
    if isinstance(zones, bool) or not isinstance(zones, int) or zones <= 0:
        raise ValueError("zones must be a positive integer (zones per side)")
    fov_v = fov_deg if not fov_v_deg else fov_v_deg
    if not (np.isfinite([fov_deg, fov_v]).all() and 0 < fov_deg < 180 and 0 < fov_v < 180):
        raise ValueError("invalid field of view")
    if rays is None:
        return tof_zone_rays(zones, fov_deg, fov_v)
    r = np.asarray(rays, float)
    if r.shape != (zones * zones, 3) or not np.isfinite(r).all():
        raise ValueError(f"calibrated rays must be a finite ({zones * zones}, 3) array")
    n = np.linalg.norm(r, axis=1)
    if (n < 1e-9).any() or (r[:, 0] <= 0).any():
        raise ValueError("calibrated rays must be nonzero and point forward (+x)")
    return r / n[:, None]


def tof_records(frame, convention, fov_deg=45.0, min_range=0.02, max_range=2.0, zones=8, fov_v_deg=None, rays=None):
    """Return zones x zones records matching world_sim's x,y,z,range,status,zone schema
    (8 x 8 VL53L5CX; 25 x 25 / 50 x 50 / 100 x 100 Sipeed A010).

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
    dirs = zone_rays(zones, fov_deg, fov_v_deg, rays)
    n = zones * zones
    ranges, status = np.asarray(frame.ranges, float).ravel(), np.asarray(frame.status).ravel()
    if ranges.shape != (n,) or status.shape != (n,):
        raise ValueError(f"backend must return {n} canonical row-major ranges and statuses ({zones} x {zones})")
    if not np.isin(status, [0, 1, 2]).all():
        raise ValueError("backend must normalize sensor status codes to 0, 1, 2")
    status = status.astype(np.uint8).copy()
    status[(status == 0) & (~np.isfinite(ranges) | (ranges < min_range) | (ranges > max_range))] = 2
    records = []
    for zone, (r, s, ray) in enumerate(zip(ranges, status, dirs)):
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
