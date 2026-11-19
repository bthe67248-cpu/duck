"""flow_odom: geometry round trip and a synthetic floor image pair with known camera poses and body motion."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import flow_odom as fo  # noqa: E402

K = np.array([[250.0, 0, 160.0], [0, 250.0, 120.0], [0, 0, 1.0]])


def camera(pitch_deg=30.0, height=0.12, x=0.05):
    """Camera pose in the level frame: looks forward and down (MuJoCo camera: x right, y up, view = -z)."""
    p = math.radians(pitch_deg)
    view = np.array([math.cos(p), 0.0, -math.sin(p)])
    up = np.array([math.sin(p), 0.0, math.cos(p)])
    z = -view
    xa = np.cross(up, z)
    return np.column_stack([xa, up, z]), np.array([x, 0.0, height])


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def test_project_floor_points_round_trip():
    R, t = camera()
    px = np.array([[100.0, 150.0], [160.0, 200.0], [250.0, 230.0], [20.0, 90.0]])
    back = fo.project(fo.floor_points(px, R, t, K), R, t, K)
    assert np.allclose(back, px, atol=1e-6)


def test_floor_points_above_horizon_are_nan():
    R, t = camera(pitch_deg=5.0)
    assert np.isnan(fo.floor_points(np.array([[160.0, 0.0]]), R, t, K)).all()


def render(texture, res, origin, R, t, body_t, body_th, shape=(240, 320)):
    """Image of the textured floor from a camera at (R, t) in a level frame that is moved by body_t / rotated by
    body_th relative to the texture frame (identity for the first image)."""
    H, W = shape
    u, v = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
    q = fo.floor_points(np.column_stack([u.ravel(), v.ravel()]), R, t, K)
    p = q @ rot(body_th).T + body_t
    mx = ((p[:, 0] - origin[0]) / res).reshape(H, W).astype(np.float32)
    my = ((p[:, 1] - origin[1]) / res).reshape(H, W).astype(np.float32)
    bad = ~np.isfinite(mx) | ~np.isfinite(my)
    mx[bad] = -1; my[bad] = -1
    return cv2.remap(texture, mx, my, cv2.INTER_LINEAR, borderValue=0)


@pytest.fixture(scope="module")
def floor():
    rng = np.random.default_rng(3)
    res, origin = 0.003, (-0.5, -1.5)
    tex = cv2.GaussianBlur(rng.random((1000, 1000)).astype(np.float32), (0, 0), 2.0)
    tex = cv2.normalize(tex, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    return tex, res, origin


@pytest.mark.parametrize("motion", [(0.03, 0.0, 0.0), (0.04, 0.01, 0.03), (-0.02, 0.015, -0.04)])
def test_estimate_pair_recovers_known_body_motion(floor, motion):
    tex, res, origin = floor
    dx, dy, dth = motion
    R0, t0 = camera()
    R1, t1 = camera(pitch_deg=31.5, height=0.125, x=0.055)           # head moved a little too
    img0 = render(tex, res, origin, R0, t0, np.zeros(2), 0.0)
    img1 = render(tex, res, origin, R1, t1, np.array([dx, dy]), dth)
    est = fo.estimate_pair(img0, img1, (R0, t0), (R1, t1), K, np.random.default_rng(0))
    assert est is not None and est[3] >= 20
    assert abs(est[0] - dx) < 0.003 and abs(est[1] - dy) < 0.003
    assert abs(est[2] - dth) < math.radians(0.5)


def test_estimate_pair_uses_guess_and_rejects_blank_images(floor):
    tex, res, origin = floor
    R0, t0 = camera()
    img0 = render(tex, res, origin, R0, t0, np.zeros(2), 0.0)
    img1 = render(tex, res, origin, R0, t0, np.array([0.05, 0.0]), 0.0)
    est = fo.estimate_pair(img0, img1, (R0, t0), (R0, t0), K, np.random.default_rng(0), guess=(0.05, 0.0, 0.0))
    assert est is not None and abs(est[0] - 0.05) < 0.003
    blank = np.zeros_like(img0)
    assert fo.estimate_pair(blank, blank, (R0, t0), (R0, t0), K, np.random.default_rng(0)) is None


def test_compute_flow_pairs_skips_bad_gaps(floor):
    tex, res, origin = floor
    R0, t0 = camera()
    img = render(tex, res, origin, R0, t0, np.zeros(2), 0.0)
    imgs = {0.0: img, 0.2: img, 0.9: img, 1.1: img}
    poses = {k: (R0, t0) for k in imgs}
    pairs = fo.compute_flow_pairs(imgs, poses, K)
    assert [(a, b) for a, b, _ in pairs] == [(0.0, 0.2), (0.9, 1.1)]
    assert all(abs(e[0]) < 1e-3 for _, _, e in pairs if e is not None)
