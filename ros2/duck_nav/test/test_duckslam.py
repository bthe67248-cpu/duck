"""Regression tests for the handoff's planner / map / estimator acceptance cases (no ROS needed).
Run:  python -m pytest ros2/duck_nav/test -q"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from duck_nav import duckslam as ds  # noqa: E402

INFO = ds.DEFAULT_MAP


def empty(fill=0):
    return np.full((INFO.nx, INFO.ny), fill, np.int8)


def walled_room(x_min=-1.0, x_max=3.0, y_min=-1.5, y_max=1.5):
    g = empty(-1)
    i0, j0 = INFO.cell(x_min, y_min); i1, j1 = INFO.cell(x_max, y_max)
    g[i0:i1 + 1, j0:j1 + 1] = 0
    g[i0, j0:j1 + 1] = g[i1, j0:j1 + 1] = 100
    g[i0:i1 + 1, j0] = g[i0:i1 + 1, j1] = 100
    return g


# ---- S2 planner -------------------------------------------------------------------------------
def test_all_occupied_returns_no_path():
    path, mode = ds.plan(empty(100), (0.0, 0.0, 0.0))
    assert path is None and mode == "start_blocked"


def test_start_disk_does_not_clear_walls():
    g = walled_room()
    i, j = INFO.cell(0.05, 0.0)
    g[i, :] = 100                                     # a wall 5 cm in front of the robot, full width
    path, mode = ds.plan(g, (0.0, 0.0, 0.0), goal=(2.0, 0.0))
    assert mode != "goal", "planner must not walk through a real wall next to the start"
    if path:
        assert all(INFO.cell(x, y)[0] < i for x, y in path)


def test_no_diagonal_corner_cutting():
    # two free corridors that touch only diagonally; clearance 0 isolates the corner rule
    g = empty(100)
    g[10:20, 10] = 0                                  # corridor A along x at j = 10, ends at (19, 10)
    g[20, 11:21] = 0                                  # corridor B along y at i = 20, starts at (20, 11)
    start = INFO.center(10, 10); goal = INFO.center(20, 20)
    path, mode = ds.plan(g, (*start, 0.0), goal=goal, goal_radius=0.01, clearance=0.0)
    assert mode != "goal", "diagonal step between (19,10) and (20,11) must be refused"


def test_clearance_is_euclidean_and_narrow_gap_respects_footprint():
    g = walled_room()
    ia, _ = INFO.cell(1.0, 0.0)
    _, j0 = INFO.cell(0.0, -1.5); _, j1 = INFO.cell(0.0, 1.5)
    g[ia, j0:j1 + 1] = 100
    jc = INFO.cell(0.0, 0.0)[1]
    for gap_cells, passable in ((8, False), (14, True)):   # 0.20 m gap < 2 x 0.125 m;  0.35 m gap > 0.25 m
        gg = g.copy()
        gg[ia, jc - gap_cells // 2: jc + gap_cells // 2] = 0
        _, mode = ds.plan(gg, (0.0, 0.0, 0.0), goal=(2.0, 0.0))
        assert (mode == "goal") == passable, f"gap {gap_cells * INFO.res:.2f} m"


def test_start_in_margin_escapes_without_clearing():
    g = walled_room()
    path, mode = ds.plan(g, (-0.95, 0.0, 0.0))          # 5 cm from the west wall
    assert mode == "escape" and path
    occ = g == 100
    assert not any(occ[INFO.cell(x, y)] for x, y in path)


# ---- S3 bounds ---------------------------------------------------------------------------------
@pytest.mark.parametrize("pose", [(5.1, 0.0, 0.0), (-1.6, 0.0, 0.0), (0.0, 2.8, 0.0), (0.0, -2.9, 0.0)])
def test_out_of_map_pose_is_explicit(pose):
    path, mode = ds.plan(walled_room(), pose)
    assert path is None and mode == "out_of_map"


def test_floor_conversion_near_origin():
    assert INFO.cell(INFO.x0 - 0.001, 0.0)[0] == -1   # int() would give 0 and silently index cell 0
    assert not INFO.inside(*INFO.cell(-1.6, 0.0))


def test_grid_shape_mismatch_raises():
    with pytest.raises(ValueError):
        ds.plan(np.zeros((10, 10), np.int8), (0, 0, 0))


# ---- S5 exploration state --------------------------------------------------------------------
def test_all_unknown_is_not_complete():
    g = empty(-1)
    path, mode = ds.plan(g, (0.0, 0.0, 0.0))
    assert mode == "no_frontier"
    state, area, _ = ds.exploration_state(g, (0.0, 0.0, 0.0))
    assert state == "waiting" and area < 0.1


def test_closed_room_is_complete_and_open_room_is_not():
    g = walled_room()
    assert ds.exploration_state(g, (0.0, 0.0, 0.0))[0] == "complete"
    i0, j0 = INFO.cell(3.0, -0.3); _, j1 = INFO.cell(3.0, 0.3)
    g[i0, j0:j1] = -1                                   # a 0.6 m hole in the east wall to unknown space
    assert ds.exploration_state(g, (0.0, 0.0, 0.0))[0] == "stalled"


# ---- S7 estimator ------------------------------------------------------------------------------
def test_reverse_and_lateral_prediction():
    s = ds.DuckSLAM(ds.OdomGains(fwd=0.4, rev=0.3, lat=0.2, rev_deadband=0.25))
    s.predict(-0.5, 0.0, 1.0)
    assert s.pose[0] == pytest.approx(-0.15)
    s.pose[:] = 0; s.predict(0.0, 0.0, 1.0, cmd_vy=0.5)
    assert s.pose[1] == pytest.approx(0.10)
    s.pose[:] = 0; s.predict(0.1, 0.0, 1.0)            # inside the deadband
    assert np.allclose(s.pose, 0)


def test_reverse_uses_its_own_gain_and_deadband():
    s = ds.DuckSLAM(0.445)                              # default gains from the MuJoCo calibration
    s.predict(-0.3, 0.0, 1.0)
    assert s.pose[0] == 0.0                             # the policy stands still at -0.3
    s.predict(-0.5, 0.0, 1.0)
    assert s.pose[0] == pytest.approx(-0.295)


def test_world_yaw_rate_under_tilt():
    # body pitched 90 deg nose-up: world up is body -x; spinning about world z shows on gyro x, not z
    up = np.array([-1.0, 0.0, 0.0])
    assert ds.world_yaw_rate([-0.5, 0.0, 0.0], up) == pytest.approx(0.5)
    assert ds.world_yaw_rate([0.0, 0.0, 0.5], up) == pytest.approx(0.0)


def test_tilt_filter_ignores_dynamic_accel():
    f = ds.TiltFilter()
    f.update([0, 0, 0], [0, 0, 9.81], 0.02)
    for _ in range(50):
        f.update([0, 0, 0], [15.0, 0, 9.81], 0.02)        # big lateral accel: |a| far from 1 g
    assert f.tilt() < math.radians(1)


# ---- H4 ToF ------------------------------------------------------------------------------------
def test_tof_rays_fov_and_order():
    rays = ds.tof_zone_rays()
    assert rays.shape == (64, 3)
    az = np.degrees(np.arctan2(rays[:, 1], rays[:, 0]))
    el = np.degrees(np.arctan2(rays[:, 2], rays[:, 0]))
    assert az[0] > 0 and az[7] < 0                       # column 0 = left
    assert el[0] > 0 and el[56] < 0                      # row 0 = top
    assert az.max() - az.min() == pytest.approx(45.0 * 7 / 8, abs=0.5)


def test_tof_invalid_zones_are_not_free_space():
    s = ds.DuckSLAM(info=INFO, min_hits=4)
    rays = ds.tof_zone_rays()
    ranges = np.full(64, 1.0); valid = np.zeros(64, bool)
    ang, r, hit = s.scan_from_tof(ranges, valid, rays, np.eye(3), np.array([0, 0, 0.15]))
    assert len(ang) == 0
    s.update_map(ang, r, hit)
    assert (s.occupancy() == -1).all()


def test_tof_wall_produces_one_hit_per_column():
    s = ds.DuckSLAM(info=INFO, min_hits=4)
    rays = ds.tof_zone_rays()
    ranges = 1.0 / rays[:, 0]                             # flat wall 1 m ahead, radial convention
    ang, r, hit = s.scan_from_tof(ranges, np.ones(64, bool), rays, np.eye(3), np.array([0, 0, 0.15]))
    assert hit.sum() == 8
    assert np.allclose(r[hit] * np.cos(ang[hit]), 1.0, atol=0.03)


def test_level_to_pixel_roundtrip():
    K = ds.SIM_CAMERA
    R = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)   # camera looking along body +x
    t = np.array([0.08, 0.0, 0.25])
    p = ds.pixel_to_level(200.0, 90.0, 1.3, R, t, K)
    u, v, z = ds.level_to_pixel(p, R, t, K)
    assert (u[0], v[0], z[0]) == pytest.approx((200.0, 90.0, 1.3))
