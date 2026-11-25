"""sensor_errors -- error models for the realistic sensor profile of world_sim (numpy only, unit-tested without ROS).

What the robot's software sees differs from the simulator's truth in four ways, each a world_sim parameter:

IMU (`ImuErrors`)        gyro / accelerometer white noise, a turn-on bias per axis (constant for the run) plus a slow
                         random walk, and a fixed mounting misalignment of the IMU against the torso. The attitude the
                         robot derives from it (`tilt_error`) is off by the accelerometer bias (bias / g rad), the
                         misalignment (roll/pitch part) and a small white attitude noise.
Encoders (`EncoderErrors`) per-joint zero offset (assembly calibration), white noise, quantisation to the encoder
                         resolution (XL330: 4096 counts per turn); velocity noise.
Extrinsics (`perturb`)   the sensor sits slightly off its CAD pose: the simulated sensor looks from the TRUE mount
                         (nominal pose composed with a small random rigid transform drawn once per run), while the pose
                         the robot publishes uses the nominal mount.
Timing (`Jitter`)        stamps carry host timestamping error (normal, sigma), kept strictly increasing per stream;
                         ToF publication latency gets a uniform extra delay.

Defaults (world_sim, realistic profile) are typical of a BMI088/ICM-42688-class IMU on a hobby robot and of XL330
servos; they are ESTIMATES to be replaced by bench measurements. sensor_profile:=ideal uses none of this.
"""
import math

import numpy as np

G = 9.81


def rotvec_to_mat(v):
    """Rotation matrix of a rotation vector (axis * angle, rad)."""
    v = np.asarray(v, float)
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def perturb(rng, rot_sigma_rad, trans_sigma_m):
    """A small random rigid transform (R, t): per-axis rotation and translation errors, normal with these sigmas."""
    return rotvec_to_mat(rng.normal(0.0, rot_sigma_rad, 3)), rng.normal(0.0, trans_sigma_m, 3)


class ImuErrors:
    def __init__(self, rng, gyro_noise=0.0, gyro_bias=0.0, gyro_bias_walk=0.0, accel_noise=0.0, accel_bias=0.0,
                 accel_bias_walk=0.0, misalign_rad=0.0, attitude_noise=0.0):
        self.rng = rng
        self.gyro_noise, self.gyro_bias_walk = gyro_noise, gyro_bias_walk
        self.accel_noise, self.accel_bias_walk = accel_noise, accel_bias_walk
        self.attitude_noise = attitude_noise
        self.bg = rng.normal(0.0, gyro_bias, 3) if gyro_bias > 0 else np.zeros(3)
        self.ba = rng.normal(0.0, accel_bias, 3) if accel_bias > 0 else np.zeros(3)
        self.mis_vec = rng.normal(0.0, misalign_rad, 3) if misalign_rad > 0 else np.zeros(3)
        self.M = rotvec_to_mat(self.mis_vec)              # torso axes -> IMU axes
        self.exact = not any((gyro_noise, gyro_bias, gyro_bias_walk, accel_noise, accel_bias, accel_bias_walk,
                              misalign_rad, attitude_noise))

    def measure(self, gyro, accel, dt):
        """IMU reading (gyro rad/s, accel m/s2) for the true torso-frame values; advances the bias random walk."""
        if self.exact:
            return np.array(gyro, float), np.array(accel, float)
        if dt > 0:
            self.bg = self.bg + self.rng.normal(0.0, self.gyro_bias_walk * math.sqrt(dt), 3)
            self.ba = self.ba + self.rng.normal(0.0, self.accel_bias_walk * math.sqrt(dt), 3)
        g = self.M @ np.asarray(gyro, float) + self.bg + self.rng.normal(0.0, self.gyro_noise, 3)
        a = self.M @ np.asarray(accel, float) + self.ba + self.rng.normal(0.0, self.accel_noise, 3)
        return g, a

    def tilt_error(self):
        """Rotation E (torso frame) by which the robot's attitude estimate is off: estimated up = E @ true up.
        The estimated torso orientation is then R_est = R_true @ E.T. Heading (yaw) is not part of it."""
        if self.exact:
            return np.eye(3)
        b = self.ba / G
        v = np.array([-b[1], b[0], 0.0]) + np.array([self.mis_vec[0], self.mis_vec[1], 0.0])
        v[:2] += self.rng.normal(0.0, self.attitude_noise, 2)
        return rotvec_to_mat(v)


class EncoderErrors:
    def __init__(self, rng, n, offset=0.0, noise=0.0, resolution=0.0, vel_noise=0.0):
        self.rng, self.noise, self.res, self.vel_noise = rng, noise, resolution, vel_noise
        self.offset = rng.normal(0.0, offset, n) if offset > 0 else np.zeros(n)

    def measure(self, q, v):
        q = np.asarray(q, float) + self.offset
        if self.noise > 0:
            q = q + self.rng.normal(0.0, self.noise, len(q))
        if self.res > 0:
            q = np.round(q / self.res) * self.res
        v = np.asarray(v, float)
        if self.vel_noise > 0:
            v = v + self.rng.normal(0.0, self.vel_noise, len(v))
        return q, v


class Jitter:
    """Stamp error for one stream: t + N(0, sigma), never later than the publication time (a message cannot carry a
    stamp from the future: nav_node rejects poses with now - stamp < 0 as stale) and strictly increasing (at least
    1 us after the previous stamp). Clamping makes the error one-sided: the stamp is up to a few sigma early, never
    late (mean -0.4 sigma, std 0.58 sigma)."""
    def __init__(self, rng, sigma=0.0):
        self.rng, self.sigma, self.last = rng, sigma, -math.inf

    def stamp(self, t, publish=None):
        """Stamp for a sample taken at t and published at `publish` (default t, i.e. in the same sim step)."""
        publish = t if publish is None else publish
        s = t + (self.rng.normal(0.0, self.sigma) if self.sigma > 0 else 0.0)
        s = max(min(s, publish), self.last + 1e-6)
        if s > publish:              # publication times closer than 1 us: keep the order, never exceed publish
            s = publish if publish > self.last else self.last
        self.last = s
        return s


def level_pose(R_trunk, z_trunk, R_rel, p_rel):
    """Sensor pose in the level body frame (heading removed, origin on the floor below the torso) from the torso
    orientation (world), torso height and the sensor pose relative to the torso. Same convention as world_sim's
    /camera/pose_level and /tof/pose_level."""
    yaw = math.atan2(R_trunk[1, 0], R_trunk[0, 0])
    Rz = np.array([[math.cos(yaw), math.sin(yaw), 0], [-math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return Rz @ R_trunk @ R_rel, Rz @ R_trunk @ p_rel + np.array([0.0, 0.0, z_trunk])
