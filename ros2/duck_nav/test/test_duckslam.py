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


# ---- Localization confidence and sparse head-sweep constraints -------------------------------
def corner_reference():
    s = ds.DuckSLAM(min_hits=12)
    for y in np.arange(-1.0, 1.01, INFO.res):
        s.logodds[INFO.cell(1.0, y)] = 3
    for x in np.arange(-1.0, 1.01, INFO.res):
        s.logodds[INFO.cell(x, 1.0)] = 3
    return s


def corner_scan(degrees):
    ang = np.radians(degrees)
    with np.errstate(divide="ignore"):
        rx = np.where(np.cos(ang) > 0, 1 / np.cos(ang), 100)
        ry = np.where(np.sin(ang) > 0, 1 / np.sin(ang), 100)
    return ang, np.minimum(rx, ry), np.ones(len(ang), bool)


def test_localization_health_requires_constraints_and_expires_independently_of_pose():
    h = ds.LocalizationHealth()
    assert h.state(0.0) == "initializing"
    h.observe(0.0, True)
    assert h.state(0.0) == "weak"  # a single lucky match cannot release the drive
    h.observe(0.1, True)
    assert h.state(0.2) == "ok"
    assert h.state(1.0) == "stale"  # even if an IMU keeps publishing predicted poses
    h.observe(1.0, False)
    assert h.state(1.0) == "weak"
    h.observe(0.5, True)  # late packet cannot undo a more recent failure
    assert h.state(1.0) == "weak"
    h.observe(1.1, True); h.observe(1.2, True)
    assert h.state(1.2) == "ok"
    h.invalidate()  # fall: previous good localization is no longer a reason to walk
    assert h.state(1.2) == "weak"
    assert h.state(-1) == "stale"  # clock reset fails closed


def test_straight_wall_repeats_are_not_full_localization_constraints():
    s = corner_reference()
    scan = corner_scan(np.linspace(-20, 20, 80))
    before = s.pose.copy()
    assert not s.match(*scan)
    assert s.match_reason == "degenerate_geometry"
    assert np.array_equal(before, s.pose)


def test_sparse_sweep_combines_eight_zone_frames_and_matches_corner():
    s = corner_reference(); window = ds.ScanWindow()
    for i, bearing in enumerate((-15, 15, 45, 75)):
        scan = corner_scan(np.linspace(bearing - 15, bearing + 15, 8))
        assert not s.match(*scan)  # an individual 8-column ToF frame is insufficient
        window.add(i * 0.1, s.pose, *scan)
    assert s.match(*window.scan(s.pose), current_scan=scan)
    assert s.match_reason == "accepted"
    assert s.match_overlap > 0.95
    assert np.linalg.norm(s.pose[:2]) < 0.02


def test_old_sweep_cannot_conceal_current_scan_disagreement():
    s = corner_reference()
    scan = corner_scan(np.linspace(-30, 100, 40))
    a, r, hit = corner_scan(np.linspace(55, 85, 8))
    assert not s.match(*scan, current_scan=(a, r * 0.5, hit))
    assert s.match_reason == "fresh_scan_disagrees"
    assert np.array_equal(s.pose, np.zeros(3))


def test_sweep_compensates_motion_and_drops_expired_constraints():
    w = ds.ScanWindow(seconds=0.5)
    w.add(0.0, np.zeros(3), np.array([0.0]), np.array([1.0]), np.array([True]))
    a, r, _ = w.scan(np.array([0.2, 0.0, math.pi / 2]))
    assert a[0] == pytest.approx(-math.pi / 2)
    assert r[0] == pytest.approx(0.8)
    w.correct(np.zeros(3), np.array([0.1, 0.2, math.pi / 2]))
    a, r, _ = w.scan(np.array([0.1, 0.2, math.pi / 2]))
    assert a[0] == pytest.approx(0.0)
    assert r[0] == pytest.approx(1.0)
    w.add(1.0, np.zeros(3), np.array([0.0]), np.array([2.0]), np.array([True]))
    assert len(w.scan(np.zeros(3))[0]) == 1


# ---- Local driving guard ---------------------------------------------------------------------
def test_path_target_does_not_turn_back_toward_old_path_start():
    path = [(x, 0.0) for x in np.linspace(0, 2, 81)]
    assert ds.path_target(path, (1.0, 0, 0))[0] >= 1.2


def test_command_arc_checks_clearance_before_cutting_corner():
    g = empty(0)
    # Forward is clear, but a left arc intrudes into a wall's footprint margin.
    i0, j0 = INFO.cell(0.18, 0.14); i1, j1 = INFO.cell(0.50, 0.18)
    g[i0:i1 + 1, j0:j1 + 1] = 100
    assert ds.motion_clear(g, (0, 0, 0), 0.35, 0.0)
    assert not ds.motion_clear(g, (0, 0, 0), 0.35, 1.0)


def test_escape_command_never_reduces_clearance_or_drives_into_unknown():
    g = walled_room()
    pose = (-0.95, 0.0, 0.0)
    assert ds.motion_clear(g, pose, 0.2, 0.0, escape=True)
    assert not ds.motion_clear(g, (*pose[:2], math.pi), 0.2, 0.0, escape=True)
    assert not ds.motion_clear(g, pose, 0.2, 0.0, escape=False)
    assert not ds.motion_clear(empty(-1), (0, 0, 0), 0.2, 0.0)


def test_sensor_outage_requires_two_new_accepted_constraints():
    h = ds.LocalizationHealth()
    h.observe(0.0, True); h.observe(0.1, True)
    assert h.state(0.1) == "ok"
    h.observe(4.0, True)
    assert h.state(4.0) == "weak"
    h.observe(4.1, True)
    assert h.state(4.1) == "ok"


def test_local_corner_constraints_reduce_small_prediction_error():
    s = corner_reference()
    s.pose[:] = (0.03, -0.02, math.radians(1.5))
    before = np.linalg.norm(s.pose)
    assert s.match(*corner_scan(np.linspace(-30, 100, 40)))
    assert np.linalg.norm(s.pose) < before


def test_match_at_search_edge_is_rejected_instead_of_claiming_relocalization():
    s = corner_reference()
    s.pose[:] = (0.2, 0.0, 0.0)
    before = s.pose.copy()
    assert not s.match(*corner_scan(np.linspace(-30, 100, 40)))
    assert s.match_reason in ("search_boundary", "low_overlap")
    assert np.array_equal(s.pose, before)


@pytest.mark.parametrize("seed", [0, 7, 42])
def test_noisy_stationary_sparse_sweep_bootstraps_without_motion(seed):
    """Numerical fixture only: point rays, static corner, not a hardware noise/accuracy model.

    Unlike the reference-map tests this starts with an empty map and exercises
    bootstrap-to-constraint acceptance. Run with -s to see accepted/gated counts.
    """
    rng = np.random.default_rng(seed)
    s = ds.DuckSLAM(max_r=2.0, min_hits=12)
    w = ds.ScanWindow(); h = ds.LocalizationHealth()
    bootstrap = True; accepted = 0; allowed = 0
    for i in range(100):
        t = i * 0.1
        head_yaw = 0.9 * math.sin(2 * math.pi * t / 4)
        ang, ranges, _ = corner_scan(np.linspace(-19.6875, 19.6875, 8) + math.degrees(head_yaw))
        valid = (ranges < s.max_r) & (rng.random(8) > 0.1)
        ang = ang[valid]
        ranges = ranges[valid] + rng.normal(0.0, 0.01, valid.sum())
        hit = np.ones(len(ranges), bool)
        before = s.pose.copy()
        w.add(t, before, ang, ranges, hit)
        good = s.match(*w.scan(before), current_scan=(ang, ranges, hit))
        if good:
            w.correct(before, s.pose)
            bootstrap = False; accepted += 1
        if bootstrap or good:
            s.update_map(ang, ranges, hit)
        h.observe(t, good)
        allowed += h.state(t) == "ok"
    print(f"static corner seed={seed}: accepted={accepted}/100, motion_gate_ok={allowed}/100")
    assert accepted >= 10 and 0 < allowed < 100
    assert not bootstrap
    assert np.linalg.norm(s.pose[:2]) < 0.03
    # Loss of observations closes the gate regardless of the last accepted count.
    assert h.state(11.0) == "stale"


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1.0, 0.0, True])
def test_localization_timing_and_motion_bounds_reject_invalid_configuration(value):
    for make in (lambda: ds.LocalizationHealth(scan_timeout=value),
                 lambda: ds.LocalizationHealth(match_timeout=value),
                 lambda: ds.ScanWindow(seconds=value),
                 lambda: ds.motion_clear(empty(0), (0, 0, 0), value, 0.0)):
        with pytest.raises(ValueError, match="finite and positive"):
            make()


@pytest.mark.parametrize("value", [0, -1, 1.5, True])
def test_sweep_frame_limit_is_a_positive_integer(value):
    with pytest.raises(ValueError, match="positive integer"):
        ds.ScanWindow(max_frames=value)


def test_vectorised_match_equals_reference_loop():
    """The array search must pick exactly what the original nested loop picked."""
    s = corner_reference(); rng = np.random.default_rng(3)
    field = __import__("scipy.ndimage", fromlist=["x"]).gaussian_filter((s.logodds > 0.8).astype(np.float32), 1.6)
    for _ in range(40):
        a, r, h = corner_scan(np.sort(rng.uniform(-40, 110, 40)))
        start = rng.normal(0, [0.04, 0.04, 0.05]); s.pose[:] = start
        px, py = r * np.cos(a), r * np.sin(a)
        best, best_s = None, -1.0
        for dth in np.radians(np.arange(-6, 6.1, 1.5)):
            c, sn = math.cos(start[2] + dth), math.sin(start[2] + dth); wx, wy = c * px - sn * py, sn * px + c * py
            for dx in np.arange(-0.075, 0.076, 0.025):
                for dy in np.arange(-0.075, 0.076, 0.025):
                    ix = np.floor((start[0] + dx + wx - INFO.x0) / INFO.res).astype(int)
                    iy = np.floor((start[1] + dy + wy - INFO.y0) / INFO.res).astype(int)
                    ok = (ix >= 0) & (ix < INFO.nx) & (iy >= 0) & (iy < INFO.ny)
                    sc = field[ix[ok], iy[ok]].sum() / len(px) - 0.02 * (abs(dx) + abs(dy)) / 0.075 \
                        - 0.02 * abs(dth) / math.radians(6)
                    if sc > best_s:
                        best_s, best = sc, (dx, dy, dth)
        accepted = s.match(a, r, h)
        if accepted:
            np.testing.assert_allclose(s.pose, start + 0.6 * np.array(best), atol=1e-12)
        else:
            assert np.array_equal(s.pose, start)


# ---- Correlative matcher (Olson 2009) -------------------------------------------------------------------
def test_correlative_recovers_errors_far_outside_the_old_search_window():
    rng = np.random.default_rng(5)
    for _ in range(25):
        s = corner_reference()
        e0 = np.array([rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2), math.radians(rng.uniform(-8, 8))])
        s.pose[:] = e0
        a, r, h = corner_scan(np.sort(rng.uniform(-40, 110, 60)))
        assert s.match_correlative(a, r, h, current_scan=(a, r, h)), s.match_reason
        assert np.hypot(*s.pose[:2]) < 0.035 and abs(s.pose[2]) < math.radians(1.5)


def test_correlative_branch_and_bound_returns_the_brute_force_optimum():
    s = corner_reference(); rng = np.random.default_rng(9)
    for _ in range(10):
        start = np.array([rng.uniform(-0.15, 0.15), rng.uniform(-0.15, 0.15), math.radians(rng.uniform(-6, 6))])
        a, r, h = corner_scan(np.sort(rng.uniform(-40, 110, 30)))
        r = r + rng.normal(0, 0.02, len(r))
        s.pose[:] = start
        s.match_correlative(a, r, h)
        # brute force over the identical fine grid, prior included
        L, _, _ = s.likelihood_tables(0.05)
        res = INFO.res; W = 12; dth = np.arange(-math.radians(12), math.radians(12) + 1e-9, math.radians(1))
        px, py = r * np.cos(a), r * np.sin(a)
        best, arg = -np.inf, None
        for k, d in enumerate(dth):
            c, sn = math.cos(start[2] + d), math.sin(start[2] + d)
            ix0 = np.floor((start[0] + c * px - sn * py - INFO.x0) / res).astype(int)
            iy0 = np.floor((start[1] + sn * px + c * py - INFO.y0) / res).astype(int)
            for ox in range(-W, W):
                for oy in range(-W, W):
                    v = L[ix0 + ox, iy0 + oy].sum() - 0.5 * (((ox * res) ** 2 + (oy * res) ** 2) / 0.2 ** 2
                                                              + (d / math.radians(8)) ** 2)
                    if v > best:
                        best, arg = v, (ox, oy, k)
        assert s.match_reason not in ("search_boundary", "insufficient_hits", "map_not_ready")
        # the posterior mean sits within a cell of the brute-force optimum
        moved = s.pose - start if s.match_reason == "accepted" else None
        if moved is not None:
            assert abs(moved[0] - arg[0] * res) <= 1.5 * res and abs(moved[1] - arg[1] * res) <= 1.5 * res


def test_correlative_wall_corrects_across_only_and_does_not_release_motion():
    s = ds.DuckSLAM(min_hits=12)
    for y in np.arange(-2, 2.01, INFO.res):
        s.logodds[INFO.cell(1.0, y)] = 3; s.logodds[INFO.cell(1.025, y)] = 3
    s.pose[:] = (0.1, 0.1, 0.0)
    a = np.radians(np.linspace(-25, 25, 40)); r = 1 / np.cos(a)
    assert not s.match_correlative(a, r, np.ones(40, bool))
    assert s.match_reason == "partial"
    assert abs(s.pose[0]) < 0.03 and s.pose[1] == pytest.approx(0.1, abs=0.01)   # x fixed, y untouched


def test_likelihood_table_cache_follows_map_updates():
    s = corner_reference()
    t1 = s.likelihood_tables()[0]
    assert s.likelihood_tables()[0] is t1
    s.update_map(np.array([0.0]), np.array([0.5]), np.array([True]))
    assert s.likelihood_tables()[0] is not t1


def test_slip_estimate_follows_a_stall_and_recovers():
    s = ds.DuckSLAM()
    for _ in range(40):                       # commanded forward 0.4 (model 0.178 m/s), duck does not move
        for _ in range(5):
            s.predict(0.4, 0.0, 0.02)
        s.last_offset = np.array([-s.slip * s._raw_fwd, 0.0, 0.0]); s.match_cov = np.eye(3) * 1e-4
        s.pose[0] += s.last_offset[0]
        s.observe_slip()
    assert s.slip < 0.1
    for _ in range(60):                       # walking normally again: matches agree with the model
        for _ in range(5):
            s.predict(0.4, 0.0, 0.02)
        s.last_offset = np.array([(1 - s.slip) * s._raw_fwd, 0.0, 0.0]); s.match_cov = np.eye(3) * 1e-4
        s.observe_slip()
    assert s.slip > 0.9


def test_slip_ignores_corrections_that_do_not_constrain_forward_motion():
    s = ds.DuckSLAM()
    for _ in range(40):
        for _ in range(5):
            s.predict(0.4, 0.0, 0.02)
        s.last_offset = np.array([-s._raw_fwd, 0.0, 0.0]); s.match_cov = np.diag([0.1 ** 2, 1e-4, 1e-4])
        s.observe_slip()
    assert s.slip == pytest.approx(1.0)


def test_frame_localizer_keeps_mapping_until_localisation_is_healthy():
    s = ds.DuckSLAM(min_hits=8); loc = ds.FrameLocalizer(s)
    a, r, h = corner_scan(np.linspace(-30, 100, 160))    # enough rays to mark the matcher's 80 wall cells
    for i in range(30):                                  # standing in a corner: bootstrap, then healthy
        loc.step(i * 0.1, np.zeros(3), a, r, h)
    assert not loc.bootstrap and loc.health.state(29 * 0.1) == "ok"
    assert (s.logodds > 0.8).sum() >= 80


def test_frame_localizer_widens_the_search_after_a_long_weak_spell():
    s = corner_reference(); loc = ds.FrameLocalizer(s, relocalize_after=1.0, use_sweep=False)
    a, r, h = corner_scan(np.linspace(-30, 100, 40))
    for i in range(5):
        loc.step(i * 0.1, np.zeros(3), a, r, h)
    assert loc.health.state(0.4) == "ok"
    off = np.array([0.5, -0.4, math.radians(15)])        # far outside the normal +-0.3 m / 12 deg search
    pose = off.copy(); t = 0.5; ok = False
    for _ in range(60):
        acc, corr = loc.step(t, pose, a, r, h); pose = pose + corr; t += 0.1
        ok = ok or loc.relocalizing
    assert ok and np.hypot(*pose[:2]) < 0.05


def test_confirmed_relocalisation_holds_a_jump_until_a_later_match_agrees():
    s = corner_reference(); loc = ds.FrameLocalizer(s, relocalize_after=1.0, use_sweep=False, reloc_confirm=True)
    a, r, h = corner_scan(np.linspace(-30, 100, 40))
    for i in range(5):
        loc.step(i * 0.1, np.zeros(3), a, r, h)
    off = np.array([0.5, -0.4, math.radians(15)])
    pose = off.copy(); t = 0.5; held = applied_at = None
    for _ in range(80):
        acc, corr = loc.step(t, pose, a, r, h)
        if loc.relocalizing and s.match_reason == "reloc_unconfirmed" and held is None:
            held = t; assert not np.any(corr)            # the first relocalisation result is not applied
        if np.hypot(*corr[:2]) > 0.2 and applied_at is None:
            applied_at = t
        pose = pose + corr; t += 0.1
    assert held is not None and applied_at is not None and applied_at - held >= 0.5 - 1e-9
    assert np.hypot(*pose[:2]) < 0.05


def test_bootstrap_mapping_lasts_until_the_body_has_turned():
    s = corner_reference(); loc = ds.FrameLocalizer(s, use_sweep=False, bootstrap_turn=math.radians(90))
    a, r, h = corner_scan(np.linspace(-30, 100, 40))
    for i in range(5):                                   # healthy, but has not turned yet: still bootstrapping
        loc.step(i * 0.1, np.zeros(3), a, r, h)
    assert loc.health.state(0.4) == "ok" and loc.bootstrap
    for i in range(5, 25):                               # turn in place 100 deg (matching corrects the heading)
        pose = np.array([0.0, 0.0, math.radians(5 * (i - 4))])
        loc.step(i * 0.1, pose, *corner_scan(np.linspace(-30, 100, 40)))
    assert loc.turned >= math.radians(90)


def test_unknown_cells_score_above_known_free_space_when_enabled():
    s = corner_reference(); s.logodds[INFO.cell(0.0, 0.0)] = -2          # one known-free cell
    L, _, _ = s.likelihood_tables()
    assert L[INFO.cell(0.0, 0.0)] == L[INFO.cell(-0.5, -0.5)]           # off: unknown scores like free space
    s.unknown_ll = -1.0
    L, _, _ = s.likelihood_tables()
    assert L[INFO.cell(-0.5, -0.5)] == pytest.approx(-1.0)               # never observed: the floor
    assert L[INFO.cell(0.0, 0.0)] < -1.0                                 # known free space: still penalised
    assert L[INFO.cell(1.0, 0.0)] == pytest.approx(0.0)                  # on a wall: unchanged


def test_map_max_std_maps_only_tightly_accepted_frames_after_bootstrap():
    a, r, h = corner_scan(np.linspace(-30, 100, 40))
    wall = corner_scan(np.linspace(-20, 20, 40))                         # one straight wall: a partial match
    mapped = {}
    for std in (0.0, 0.03):
        s = corner_reference(); loc = ds.FrameLocalizer(s, use_sweep=False, map_move=0.0, map_turn=0.0, map_max_std=std)
        for i in range(5):
            loc.step(i * 0.1, np.zeros(3), a, r, h)
        assert not loc.bootstrap
        v = s.map_version
        loc.step(0.5, np.zeros(3), a, r, h)                              # corner: accepted, tight -> mapped
        assert s.match_reason == "accepted" and s.map_version == v + 1
        v = s.map_version
        loc.step(0.6, np.zeros(3), *wall)                                # health was still "ok" before this frame
        assert s.match_reason == "partial"
        mapped[std] = s.map_version - v
    assert mapped == {0.0: 1, 0.03: 0}


def test_partial_match_is_not_applied_while_relocalising_when_disabled():
    a, r, h = corner_scan(np.linspace(-30, 100, 40))
    wall = corner_scan(np.linspace(-20, 20, 40))
    for allow in (True, False):
        s = corner_reference(); loc = ds.FrameLocalizer(s, relocalize_after=1.0, use_sweep=False, reloc_partial=allow)
        for i in range(5):
            loc.step(i * 0.1, np.zeros(3), a, r, h)
        pose = np.array([0.04, 0.0, math.radians(4)]); t = 0.5; applied = []
        for _ in range(40):
            acc, corr = loc.step(t, pose, *wall); pose = pose + corr; t += 0.1
            if loc.relocalizing and s.match_reason == "partial":
                applied.append(bool(np.any(corr)))
        assert applied and all(applied) == allow and any(applied) == allow


# ---- ambiguous (second-peak) matches ----------------------------------------------------------
def aliased_reference():
    """A wall at x = 1 with two identical 0.4 m stubs at y = 0 and y = 0.5: a view of the wall and one stub fits
    either stub equally well."""
    s = ds.DuckSLAM(min_hits=12)
    for y in np.arange(-2, 2.01, INFO.res):
        s.logodds[INFO.cell(1.0, y)] = 3
    for x in np.arange(0.6, 1.0, INFO.res):
        s.logodds[INFO.cell(x, 0.0)] = 3; s.logodds[INFO.cell(x, 0.5)] = 3
    return s


def wall_and_stub_scan():
    pts = [(1.0, y) for y in np.linspace(-0.2, 0.2, 17)] + [(x, 0.0) for x in np.linspace(0.65, 0.95, 13)]
    p = np.array(pts)
    return np.arctan2(p[:, 1], p[:, 0]), np.hypot(p[:, 0], p[:, 1]), np.ones(len(p), bool)


def test_second_peak_rejects_an_aliased_wide_search_that_the_local_covariance_accepts():
    a, r, h = wall_and_stub_scan()
    wide = dict(window=0.8, ang_window=math.radians(25), prior_xy=1.0, prior_th=0.6)
    s = aliased_reference(); s.pose[:] = (0.0, 0.26, 0.0)    # halfway between the stubs
    assert s.match_correlative(a, r, h, **wide), s.match_reason              # old behaviour: a confident jump
    s = aliased_reference(); s.pose[:] = (0.0, 0.26, 0.0)
    assert not s.match_correlative(a, r, h, peak_margin=1.0, **wide)
    assert s.match_reason == "second_peak" and s.match_gap < 1.0
    assert tuple(s.pose) == (0.0, 0.26, 0.0)


def test_second_peak_keeps_unique_matches_and_wall_partials():
    s = corner_reference(); s.pose[:] = (0.05, -0.04, math.radians(2))
    a, r, h = corner_scan(np.linspace(-40, 110, 60))
    assert s.match_correlative(a, r, h, peak_margin=1.0), s.match_reason
    assert s.match_gap > 2.0
    s = ds.DuckSLAM(min_hits=12)                         # along a wall the ridge is expected, not a second peak
    for y in np.arange(-2, 2.01, INFO.res):
        s.logodds[INFO.cell(1.0, y)] = 3; s.logodds[INFO.cell(1.025, y)] = 3
    s.pose[:] = (0.1, 0.1, 0.0)
    a = np.radians(np.linspace(-25, 25, 40)); r = 1 / np.cos(a)
    assert not s.match_correlative(a, r, np.ones(40, bool), peak_margin=1.0)
    assert s.match_reason == "partial" and abs(s.pose[0]) < 0.03


def test_map_min_gap_keeps_ambiguous_matches_out_of_the_map():
    a, r, h = wall_and_stub_scan()
    for gap, mapped in ((0.0, True), (2.0, False)):
        s = aliased_reference()
        loc = ds.FrameLocalizer(s, use_sweep=False, map_min_gap=gap, relocalize_after=0.0, reloc_confirm=False)
        loc.bootstrap = False; loc.last_ok = -10.0       # past bootstrap, relocalising (wide search)
        v0 = s.map_version
        loc.step(0.0, np.array([0.0, 0.26, 0.0]), a, r, h)
        assert (s.map_version != v0) == mapped, (gap, s.match_reason, s.match_gap)


def test_partial_matches_along_a_wall_do_not_start_relocalisation_when_tracking():
    for tracks in (False, True):
        s = ds.DuckSLAM(min_hits=12)
        for y in np.arange(-2, 2.01, INFO.res):
            s.logodds[INFO.cell(1.0, y)] = 3; s.logodds[INFO.cell(1.025, y)] = 3
        loc = ds.FrameLocalizer(s, use_sweep=False, relocalize_after=1.0, partial_tracks=tracks)
        loc.bootstrap = False; loc.last_ok = 0.0
        a = np.radians(np.linspace(-25, 25, 40)); r = 1 / np.cos(a); h = np.ones(40, bool)
        reloc = False
        for i in range(40):
            loc.step(0.1 * i, np.array([0.0, 0.1, 0.0]), a, r, h)
            reloc = reloc or loc.relocalizing
        assert reloc != tracks
