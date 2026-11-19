"""AttitudeFilter: heading through roll/pitch motion, tilt, gyro calibration."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from duck_nav import duckslam as ds  # noqa: E402


def rotvec_quat(v):
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return np.array([math.cos(a / 2), *(math.sin(a / 2) * np.asarray(v) / a)])


def simulate(seconds=60.0, dt=0.02, yaw_rate=0.3, wobble=0.15, bias=(0.0, 0.0, 0.0)):
    """True attitude: constant world yaw rate plus gait-like roll/pitch wobble. Yields (gyro, acc, true_q)."""
    q_prev = None
    for k in range(int(seconds / dt) + 1):
        t = k * dt
        roll, pitch = wobble * math.sin(2 * math.pi * 1.7 * t), 0.6 * wobble * math.sin(2 * math.pi * 1.7 * t + 1.0)
        yaw = yaw_rate * t
        q = ds.quat_mul(rotvec_quat([0, 0, yaw]), ds.quat_mul(rotvec_quat([0, pitch, 0]), rotvec_quat([roll, 0, 0])))
        if q_prev is not None:
            dq = ds.quat_mul(np.array([q_prev[0], *(-q_prev[1:])]), q)        # body-frame increment
            dq = dq if dq[0] >= 0 else -dq
            gyro = 2 * dq[1:] / dt + np.asarray(bias)
            w, x, y, z = q                                                    # gravity reaction in the body frame
            acc = 9.81 * np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])
            yield gyro, acc, q
        q_prev = q


def run(f, **kw):
    heading, errs = 0.0, []
    first = True
    for gyro, acc, q in simulate(**kw):
        if first:
            f.update(gyro, acc, 0.0); first = False
            heading = ds.quat_yaw(q)
            continue
        f.update(gyro, acc, 0.02)
        heading += f.heading_rate(gyro, 0.02) * 0.02
        errs.append(abs((heading - ds.quat_yaw(q) + math.pi) % (2 * math.pi) - math.pi))
    return math.degrees(max(errs)), f


def test_quaternion_heading_beats_tilt_projection_under_wobble():
    new, _ = run(ds.AttitudeFilter())
    old, _ = run(ds.TiltFilter())
    assert new < 1.0
    assert new < old


def test_tilt_follows_true_roll_pitch():
    f = ds.AttitudeFilter()
    for gyro, acc, q in simulate(seconds=20.0):
        f.update(gyro, acc, 0.02)
    w, x, y, z = q
    true_tilt = math.acos(1 - 2 * (x * x + y * y))
    assert abs(f.tilt() - true_tilt) < math.radians(1.0)


def test_calibrated_bias_removes_yaw_drift():
    b = np.array([0.01, -0.01, 0.005])
    f = ds.AttitudeFilter()
    assert f.calibrate(b + np.random.default_rng(0).normal(0, 0.001, (100, 3)))
    err, _ = run(f, bias=b)
    unc, _ = run(ds.AttitudeFilter(), bias=b)
    assert err < 1.0 and unc > 10.0


def test_calibration_rejects_motion():
    f = ds.AttitudeFilter()
    moving = np.column_stack([np.zeros(100), np.zeros(100), np.linspace(-0.3, 0.3, 100)])
    assert not f.calibrate(moving)
    assert np.all(f.bias == 0)


def test_factory():
    assert isinstance(ds.make_attitude_filter(), ds.AttitudeFilter)
    assert isinstance(ds.make_attitude_filter("tilt"), ds.TiltFilter)
