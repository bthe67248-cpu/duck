"""FlowTranslation: dead reckoning follows measured floor flow, falls back to the command, drops fallen windows."""
import math
import os
import sys

import numpy as np
import pytest

pytest.importorskip("cv2")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from duck_nav import duckslam as ds  # noqa: E402
from duck_nav.flowodom import FlowTranslation  # noqa: E402


def drive(mode, true_speed=0.10, cmd_speed=0.20, seconds=10.0, missing=(), fallen=(), dt=0.02, frame=0.2):
    """Straight walk along +x at `true_speed`; the command model claims `cmd_speed`. Flow measures the truth for every
    image pair (every `frame` s) except the pair indices in `missing`; `fallen` = (t0, t1) not-upright interval."""
    slam = ds.DuckSLAM(ds.OdomGains()); f = FlowTranslation(mode)
    cmd_body = (cmd_speed, 0.0)
    steps = int(round(seconds / dt)); per = int(round(frame / dt)); pair = 0
    for k in range(1, steps + 1):
        t = k * dt
        ok = not (fallen and fallen[0] <= t <= fallen[1])
        if not ok:
            f.mark_not_ok(t)
        vb = f.body_velocity(t, cmd_body, ok)
        before = slam.pose[:2].copy()
        slam.predict_measured(vb[0], vb[1], 0.0, dt)
        f.record(t, slam.pose[:2] - before, (cmd_body[0] * dt if ok else 0.0, 0.0))
        if k % per == 0 and t > frame:                       # estimate for pair (t - frame, t) arrives now
            s0, s1 = t - frame, t
            est = None if pair in missing else (true_speed * frame, 0.0, 0.0, 30)
            r = f.on_pair(s0, s1, est, 0.0)
            if r is not None:
                slam.pose[:2] += r
            pair += 1
    return slam.pose[0], f


def test_follows_flow_not_command():
    x, f = drive("flow_or_command")
    assert x == pytest.approx(0.10 * 10.0, abs=0.05)         # truth 1.0 m; the command alone would say 2.0 m
    assert f.n_ok > 40 and f.n_missing == 0


def test_missing_pairs_use_the_command_or_zero():
    miss = set(range(10, 20))                                # 2 s without a fit
    x_cmd, _ = drive("flow_or_command", missing=miss)
    x_zero, _ = drive("flow", missing=miss)
    assert x_cmd == pytest.approx(1.0 + 2.0 * (0.20 - 0.10), abs=0.06)   # the command over-predicts in the gap
    assert x_zero == pytest.approx(1.0 - 2.0 * 0.10, abs=0.06)           # counted as standing still


def test_pairs_overlapping_a_fall_are_dropped():
    x, f = drive("flow_or_command", fallen=(4.0, 5.0))
    assert x < 1.0                                           # no motion credited while not upright
    assert f.n_ok < 49


def test_bad_mode():
    with pytest.raises(ValueError):
        FlowTranslation("command")
