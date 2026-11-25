"""Realistic-profile sensor error models (duck_nav/sensor_errors.py) and the level-frame pose world_sim publishes."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from duck_nav.sensor_errors import G, EncoderErrors, ImuErrors, Jitter, level_pose, perturb, robot_kinematics, rotvec_to_mat


def rng(seed=0):
    return np.random.default_rng(seed)


def test_zero_parameters_give_exact_readings():
    imu = ImuErrors(rng())
    g, a = imu.measure([0.1, -0.2, 0.3], [0.0, 0.0, 9.81], 0.02)
    np.testing.assert_array_equal(g, [0.1, -0.2, 0.3]); np.testing.assert_array_equal(a, [0.0, 0.0, 9.81])
    np.testing.assert_array_equal(imu.tilt_error(), np.eye(3))
    enc = EncoderErrors(rng(), 3)
    q, v = enc.measure([0.1, 0.2, 0.3], [1.0, 2.0, 3.0])
    np.testing.assert_array_equal(q, [0.1, 0.2, 0.3]); np.testing.assert_array_equal(v, [1.0, 2.0, 3.0])
    assert Jitter(rng()).stamp(1.25) == 1.25
    R, t = perturb(rng(), 0.0, 0.0)
    np.testing.assert_array_equal(R, np.eye(3)); np.testing.assert_array_equal(t, 0.0)


def test_imu_bias_and_noise_statistics():
    imu = ImuErrors(rng(1), gyro_noise=0.002, gyro_bias=0.005, accel_noise=0.02, accel_bias=0.1)
    G_ = np.array([imu.measure([0, 0, 0], [0, 0, G], 0.02)[0] for _ in range(4000)])
    np.testing.assert_allclose(G_.mean(0), imu.bg, atol=3 * 0.002 / math.sqrt(4000) + 1e-6)
    np.testing.assert_allclose(G_.std(0), 0.002, rtol=0.1)
    assert np.any(np.abs(imu.bg) > 1e-4)                  # a turn-on bias was drawn


def test_gyro_bias_random_walk_grows_with_time():
    spread = []
    for seed in range(200):
        imu = ImuErrors(rng(seed), gyro_bias_walk=1e-3)
        for _ in range(500):                              # 10 s at 50 Hz
            imu.measure([0, 0, 0], [0, 0, G], 0.02)
        spread.append(imu.bg[2])
    assert np.std(spread) == pytest.approx(1e-3 * math.sqrt(10), rel=0.15)


def test_accelerometer_bias_tilts_the_attitude_estimate_by_bias_over_g():
    imu = ImuErrors(rng(), accel_bias=1e-9)              # draw nothing significant, then set a known bias
    imu.exact = False
    imu.ba = np.array([0.098, 0.0, 0.0])
    E = imu.tilt_error()
    up_est = E @ np.array([0, 0, 1.0])
    f = np.array([0, 0, G]) + imu.ba                      # what the accelerometer reads when level
    np.testing.assert_allclose(up_est, f / np.linalg.norm(f), atol=1e-4)
    assert math.degrees(math.acos(up_est[2])) == pytest.approx(math.degrees(0.098 / G), rel=1e-3)


def test_encoder_offset_noise_and_quantisation():
    enc = EncoderErrors(rng(3), 14, offset=math.radians(0.5), resolution=2 * math.pi / 4096)
    q = np.linspace(-1, 1, 14)
    m1, _ = enc.measure(q, np.zeros(14))
    np.testing.assert_allclose(m1 / enc.res, np.round(m1 / enc.res), atol=1e-9)
    assert np.max(np.abs(m1 - q - enc.offset)) <= enc.res / 2 + 1e-12
    assert np.std(enc.offset) == pytest.approx(math.radians(0.5), rel=0.6)


def test_jitter_keeps_stamps_increasing():
    j = Jitter(rng(4), sigma=0.01)                        # half the sample period: reorderings are likely
    s = [j.stamp(k * 0.02) for k in range(2000)]
    assert np.all(np.diff(s) > 0)
    err = np.array(s) - np.arange(2000) * 0.02
    assert np.std(err) == pytest.approx(0.0058, rel=0.2)   # normal clipped at 0: std 0.58 sigma


def test_jitter_stamp_never_after_publication():
    """Stamps later than the publish time look like future poses to nav_node ("stale pose"): clamp them."""
    j = Jitter(rng(5), sigma=0.0005)                       # world_sim imu/joints: 0.5 ms at a 2 ms step
    t = np.arange(5000) * 0.002
    s = np.array([j.stamp(x) for x in t])
    assert np.all(s <= t) and np.all(np.diff(s) > 0)
    assert np.mean(s < t) > 0.4                            # still jittered
    j = Jitter(rng(6), sigma=0.003)                        # explicit later publication (e.g. a delayed frame)
    s = np.array([j.stamp(x, publish=x + 0.01) for x in t])
    assert np.all(s <= t + 0.01) and np.any(s > t) and np.all(np.diff(s) > 0)
    j = Jitter(rng(7), sigma=0.01)                         # same publication time twice: the cap wins over strictness
    s = [j.stamp(1.0) for _ in range(5)]
    assert max(s) <= 1.0 and s == sorted(s)


def test_perturb_sizes():
    angles, shifts = [], []
    for seed in range(300):
        R, t = perturb(rng(seed), math.radians(1.0), 0.002)
        angles.append(math.degrees(math.acos(np.clip((np.trace(R) - 1) / 2, -1, 1)))); shifts.append(np.linalg.norm(t))
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.median(angles) == pytest.approx(1.54, rel=0.15)       # |N(0, 1 deg) in 3 axes|: median ~1.54 deg
    assert np.median(shifts) == pytest.approx(0.00308, rel=0.15)


def test_level_pose_matches_the_world_frame_formula():
    """level_pose(torso orientation, height, sensor pose in the torso) must equal what world_sim published before:
    Rz(-yaw) @ R_sensor_world and Rz(-yaw) @ (p_sensor_world - (x, y, 0))."""
    r = rng(5)
    for _ in range(20):
        R_t = rotvec_to_mat(r.normal(0, 0.3, 3)) @ rotvec_to_mat([0, 0, r.uniform(-3, 3)])
        p_t = np.array([r.uniform(-2, 2), r.uniform(-2, 2), 0.12])
        R_rel, p_rel = rotvec_to_mat(r.normal(0, 1, 3)), r.normal(0, 0.05, 3)
        R_w, p_w = R_t @ R_rel, R_t @ p_rel + p_t
        yaw = math.atan2(R_t[1, 0], R_t[0, 0])
        Rz = np.array([[math.cos(yaw), math.sin(yaw), 0], [-math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        R_l, p_l = level_pose(R_t, p_t[2], R_rel, p_rel)
        np.testing.assert_allclose(R_l, Rz @ R_w, atol=1e-12)
        np.testing.assert_allclose(p_l, Rz @ (p_w - np.array([p_t[0], p_t[1], 0])), atol=1e-12)


def test_tilt_error_moves_the_published_sensor_pose_by_the_tilt_angle():
    imu = ImuErrors(rng(), accel_bias=1e-9); imu.exact = False
    imu.ba = np.array([0.0, 0.17, 0.0])                   # ~1 deg
    E = imu.tilt_error()
    R_rel, p_rel = np.eye(3), np.array([0.05, 0.0, 0.0])
    R_true, _ = level_pose(np.eye(3), 0.12, R_rel, p_rel)
    R_est, _ = level_pose(np.eye(3) @ E.T, 0.12, R_rel, p_rel)
    ang = math.degrees(math.acos(np.clip((np.trace(R_true.T @ R_est) - 1) / 2, -1, 1)))
    assert ang == pytest.approx(math.degrees(0.17 / G), rel=1e-3)


def test_robot_kinematics_includes_cameras():
    """world_sim's own-kinematics copy (d_meas) must give the same camera and site poses as mj_forward: with
    mj_kinematics alone the published /camera/pose_level sat at the origin (beacon never reached on PR #11)."""
    mujoco = pytest.importorskip("mujoco")
    m = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body pos="0.1 0.2 0.3"><joint type="hinge" axis="0 0 1"/><geom size="0.05"/>'
        '<camera name="c" pos="0.05 0 0.02"/><site name="s" pos="0 0.04 0"/></body></worldbody></mujoco>')
    d = mujoco.MjData(m); d.qpos[0] = 0.7; mujoco.mj_forward(m, d)
    k = mujoco.MjData(m); k.qpos[:] = d.qpos; robot_kinematics(m, k)
    np.testing.assert_allclose(k.cam_xpos, d.cam_xpos); np.testing.assert_allclose(k.cam_xmat, d.cam_xmat)
    np.testing.assert_allclose(k.site_xpos, d.site_xpos)
    assert np.linalg.norm(k.cam_xpos[0]) > 0.3
