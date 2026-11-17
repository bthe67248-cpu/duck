"""world_sim -- MuJoCo world + Duckling V3 + walking policy as a ROS 2 node (the "robot and its sensors").

Subscribes  /cmd_vel (geometry_msgs/Twist)  linear.x = walk command (0.5 = walk, below 0.25 = stand; negative = back),
                                            linear.y = side-step command, angular.z = turn.
                                            A command older than `cmd_timeout` sim seconds (or `cmd_wall_timeout`
                                            steady-clock seconds) is replaced by zero: nothing stays latched.
            /head_yaw_cmd (std_msgs/Float64) head pan, rad
            /duck/pose_cmd (std_msgs/Float32MultiArray, 10 values) head [neck pitch, head pitch, yaw, roll] +
                                            body [x, y, z, roll, pitch, yaw] commands for the walking policy
            /duck/behavior (std_msgs/String) "sit", "stand", "roll", "pick": switches to the matching policy
Publishes   /clock                      simulation time (all other nodes run with use_sim_time)
            /joint_states, /imu         50 Hz
            /camera/image_raw           rgb8, `cam_width` x `cam_height`, 5 Hz (head camera)
            /camera/camera_info         intrinsics of that image (same stamp)
            /camera/pose_level          camera pose in the level body frame: what leg/neck encoders + IMU tilt give
            sensor_profile:=ideal      -> /camera/depth  32FC1 camera-axis depth, same stamp as the image (perfect RGB-D)
            sensor_profile:=realistic  -> /tof/zones  sensor_msgs/PointCloud2, VL53L5CX-like 8x8 multizone ranges
                                          (fields x y z range status zone; status 0 valid, 1 no target / out of range,
                                          2 dropout), stamped at acquisition and published `tof_latency` later;
                                          /tof/pose_level  ToF pose in the level body frame at the same stamp;
                                          the RGB image gets sensor noise. No dense depth is published.
            TF base_level -> trunk_base body height and tilt
            /ground_truth/pose          true pose, for scoring and display only
            /world/markers              the real walls and boxes, for comparison in RViz
            /duck/mode                  which policy is driving: walk, sit, rise, roll, pick
Sampling convention: after each 4 physics steps mj_forward() is called, so qpos, body/camera/site transforms, IMU
sensordata, rendered images and the published stamp all describe the same state at sim time k * 0.02 s.
The walking policy (velstand.onnx) turns the commands into 14 servo targets at 50 Hz.
"""
import json
import math
import os
import time
from collections import deque

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import numpy as np
import onnxruntime as ort
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Float64, Header, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .duckslam import tof_zone_rays

POLICY_DT = 0.02
TOF_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(("x", "y", "z", "range"))] + \
             [PointField(name="status", offset=16, datatype=PointField.UINT8, count=1),
              PointField(name="zone", offset=18, datatype=PointField.UINT16, count=1)]


def quat_mul(a, b):
    return np.array([a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3], a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
                     a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1], a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]])


def pose_msg(stamp, frame, R, t):
    q = np.zeros(4); mujoco.mju_mat2Quat(q, np.ascontiguousarray(R).flatten())
    p = PoseStamped(); p.header.stamp = stamp; p.header.frame_id = frame
    p.pose.position.x, p.pose.position.y, p.pose.position.z = (float(v) for v in t)
    p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z = (float(v) for v in q)
    return p


class WorldSim(Node):
    def __init__(self):
        super().__init__("world_sim")
        for name in ("mjcf", "policy", "world_json"):
            self.declare_parameter(name, "")
        self.declare_parameter("real_time", True)
        self.declare_parameter("sensor_profile", "ideal")    # "ideal" (perfect RGB-D) | "realistic" (USB RGB + ToF)
        self.declare_parameter("cmd_timeout", 0.5)           # s sim time
        self.declare_parameter("cmd_wall_timeout", 1.0)      # s steady clock (host-stall guard)
        self.declare_parameter("cam_width", 320); self.declare_parameter("cam_height", 240)
        self.declare_parameter("cam_noise", 4.0)             # realistic profile: RGB noise sigma, 0..255 units
        # VL53L5CX-like ToF (realistic profile). Pololu #3417 / ST datasheet values; usable range is scene-dependent.
        self.declare_parameter("tof_zones", 8)
        self.declare_parameter("tof_fov_deg", 45.0)          # per axis (65 deg is the diagonal)
        self.declare_parameter("tof_rate_hz", 10.0)          # 8x8 supports up to 15 Hz
        self.declare_parameter("tof_max_range", 2.0)         # m; 4 m is the best-case datasheet maximum
        self.declare_parameter("tof_min_range", 0.02)
        self.declare_parameter("tof_noise_rel", 0.015); self.declare_parameter("tof_noise_abs", 0.005)
        self.declare_parameter("tof_zone_dropout", 0.03); self.declare_parameter("tof_frame_dropout", 0.02)
        self.declare_parameter("tof_latency", 0.06)          # s from acquisition to publication
        self.declare_parameter("tof_range_convention", "radial")   # what the published `range` means
        self.declare_parameter("seed", 0)
        self.profile = str(self.get_parameter("sensor_profile").value)
        if self.profile not in ("ideal", "realistic"):
            raise ValueError(f"sensor_profile must be 'ideal' or 'realistic', not '{self.profile}'")
        self.rng = np.random.default_rng(int(self.get_parameter("seed").value))

        self.m = mujoco.MjModel.from_xml_path(self.get_parameter("mjcf").value)
        self.m.opt.timestep = 0.005
        self.n_sub = round(POLICY_DT / self.m.opt.timestep)
        self.d = mujoco.MjData(self.m)
        key = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_KEY, "sim_start")
        mujoco.mj_resetDataKeyframe(self.m, self.d, key)
        mujoco.mj_forward(self.m, self.d)                    # derived quantities valid before the first read
        self.default = self.m.key_ctrl[key][:14].copy().astype(np.float32)
        jid = [self.m.actuator_trnid[i, 0] for i in range(14)]
        self.jnames = [self.m.joint(j).name for j in jid]
        self.qadr = np.array([self.m.jnt_qposadr[j] for j in jid]); self.vadr = np.array([self.m.jnt_dofadr[j] for j in jid])
        self.gyro = self.m.sensor_adr[self.m.sensor("gyro").id]; self.acc = self.m.sensor_adr[self.m.sensor("accel").id]
        self.trunk = self.m.body("trunk_base").id; self.cam = self.m.camera("head_cam").id
        self.tof_site = self.m.site("tof").id
        pdir = os.path.dirname(self.get_parameter("policy").value)
        self.sessions = {}
        for mode, fname in (("walk", os.path.basename(self.get_parameter("policy").value)), ("sit", "alpha_sitstand.onnx"),
                            ("roll", "roulade.onnx"), ("pick", "alpha_ground_pick.onnx")):
            if os.path.isfile(os.path.join(pdir, fname)):
                self.sessions[mode] = ort.InferenceSession(os.path.join(pdir, fname), providers=["CPUExecutionProvider"])
        self.inp = self.sessions["walk"].get_inputs()[0].name
        self.mode = "walk"; self.mode_left = 0.0; self.phase = 0.0
        self.twist = np.zeros(3, np.float32); self.head_yaw = 0.0; self.pose_cmd = np.zeros(10, np.float32)
        self.cmd_sim_t = -1e9; self.cmd_wall_t = -1e9; self.cmd_expired = True
        self.last = np.zeros(14, np.float32); self.cmd = np.zeros(13, np.float32)

        self.W, self.H = int(self.get_parameter("cam_width").value), int(self.get_parameter("cam_height").value)
        self.fovy = float(self.m.cam_fovy[self.cam])
        self.rgb_r = mujoco.Renderer(self.m, self.H, self.W)
        if self.profile == "ideal":
            self.dep_r = mujoco.Renderer(self.m, self.H, self.W); self.dep_r.enable_depth_rendering()
        self.opt = mujoco.MjvOption(); self.opt.geomgroup[2] = 0          # camera does not render the duck's visuals
        n = int(self.get_parameter("tof_zones").value)
        self.tof_rays = tof_zone_rays(n, float(self.get_parameter("tof_fov_deg").value), float(self.get_parameter("tof_fov_deg").value))
        self.tof_every = max(1, round(1.0 / (float(self.get_parameter("tof_rate_hz").value) * POLICY_DT)))
        self.tof_queue = deque()                              # (publish_at_sim_t, PointCloud2)
        # rays hit collision geometry (groups 0, 1, 3) of the world AND the robot, so a bill or bracket in the field
        # of view shows up as a near return instead of being hidden
        self.ray_groups = np.array([1, 1, 0, 1, 0, 0], np.uint8)
        self.k = 0; self.t0 = time.monotonic(); self.render_ms = 0.0

        self.pub_clock = self.create_publisher(ClockMsg, "clock", 10)
        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_imu = self.create_publisher(Imu, "imu", 50)
        self.pub_rgb = self.create_publisher(Image, "camera/image_raw", 2)
        self.pub_info = self.create_publisher(CameraInfo, "camera/camera_info", 2)
        self.pub_dep = self.create_publisher(Image, "camera/depth", 2) if self.profile == "ideal" else None
        self.pub_cpose = self.create_publisher(PoseStamped, "camera/pose_level", 5)
        self.pub_tof = self.create_publisher(PointCloud2, "tof/zones", 5) if self.profile == "realistic" else None
        self.pub_tpose = self.create_publisher(PoseStamped, "tof/pose_level", 10) if self.profile == "realistic" else None
        self.pub_gt = self.create_publisher(PoseStamped, "ground_truth/pose", 10)
        self.tf = TransformBroadcaster(self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_mk = self.create_publisher(MarkerArray, "world/markers", latched)
        self.publish_markers()
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_subscription(Float64, "head_yaw_cmd", self.on_head, 10)
        self.create_subscription(Float32MultiArray, "duck/pose_cmd", self.on_pose_cmd, 10)
        self.create_subscription(String, "duck/behavior", self.on_behavior, 10)
        self.pub_mode = self.create_publisher(String, "duck/mode", 10)
        self.create_timer(0.002, self.step, clock=Clock(clock_type=ClockType.STEADY_TIME))
        extra = (f"ToF {n}x{n} at {1 / (self.tof_every * POLICY_DT):.0f} Hz, latency {self.get_parameter('tof_latency').value:.2f} s"
                 if self.profile == "realistic" else "ideal depth")
        self.get_logger().info(f"world_sim up ({self.profile} sensors): 50 Hz policy, 200 Hz physics, "
                               f"camera {self.W}x{self.H} at 5 Hz, {extra}")

    def sim_t(self):
        return self.k * POLICY_DT

    def on_cmd(self, msg):
        self.twist[:] = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.cmd_sim_t = self.sim_t(); self.cmd_wall_t = time.monotonic()
        if self.cmd_expired:
            self.cmd_expired = False

    def on_head(self, msg):
        self.head_yaw = msg.data

    def on_pose_cmd(self, msg):
        if len(msg.data) == 10:
            self.pose_cmd[:] = msg.data

    def on_behavior(self, msg):
        want = msg.data.strip().lower()
        if want == "sit" and self.mode == "walk" and "sit" in self.sessions:
            self.mode = "sit"
        elif want == "stand" and self.mode == "sit":
            self.mode, self.mode_left = "rise", 4.0          # the sit/stand policy stands up itself, then walking takes over
        elif want == "roll" and self.mode == "walk" and "roll" in self.sessions:
            self.mode, self.mode_left = "roll", 2.0
        elif want == "pick" and self.mode == "walk" and "pick" in self.sessions:
            self.mode, self.phase = "pick", 0.0
        else:
            self.get_logger().info(f"behavior '{want}' ignored in mode '{self.mode}'")
            return
        self.get_logger().info(f"behavior: {self.mode}")

    def expire_command(self):
        stale = (self.sim_t() - self.cmd_sim_t > self.get_parameter("cmd_timeout").value or
                 time.monotonic() - self.cmd_wall_t > self.get_parameter("cmd_wall_timeout").value)
        if stale and not self.cmd_expired:
            if np.any(self.twist):
                self.get_logger().warn(f"cmd_vel expired at t={self.sim_t():.2f} s: standing still")
            self.twist[:] = 0.0; self.cmd_expired = True
        elif stale:
            self.twist[:] = 0.0

    def command_vector(self):
        """13-D policy command for the current mode; also advances timed behaviours (called at 50 Hz)."""
        c = np.zeros(13, np.float32)
        if self.mode == "walk":
            c[0:3] = self.twist; c[3:13] = self.pose_cmd; c[5] += self.head_yaw
        elif self.mode == "sit":
            c[0] = 1.0; c[3:7] = self.pose_cmd[:4]             # posture flag 1 = sit; head stays steerable
        elif self.mode == "pick":
            c[0], c[1] = math.cos(2 * math.pi * self.phase), math.sin(2 * math.pi * self.phase)
            self.phase += POLICY_DT / 4.0
            if self.phase >= 0.7:
                self.mode = "walk"
        if self.mode in ("rise", "roll"):                      # zero command: "stand" for sitstand, "go" for the roll
            self.mode_left -= POLICY_DT
            if self.mode_left <= 0:
                self.mode = "walk"
        return c

    def publish_markers(self):
        path = self.get_parameter("world_json").value
        if not path:
            return
        arr = MarkerArray()
        with open(path) as f:
            items = json.load(f)["items"]
        for i, it in enumerate(items):
            mk = Marker(); mk.header.frame_id = "map"; mk.ns = "world"; mk.id = i
            mk.type = Marker.CYLINDER if it["name"] == "beacon" else Marker.CUBE
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(it["x"]), float(it["y"]), float(it["sz"])
            mk.pose.orientation.z, mk.pose.orientation.w = math.sin(it["yaw"] / 2), math.cos(it["yaw"] / 2)
            mk.scale.x, mk.scale.y, mk.scale.z = 2.0 * it["sx"], 2.0 * it["sy"], 2.0 * it["sz"]
            mk.color.r, mk.color.g, mk.color.b = (float(v) for v in it["rgba"][:3]); mk.color.a = 0.45
            arr.markers.append(mk)
        self.pub_mk.publish(arr)

    # ---- sensors ---------------------------------------------------------------------------------
    def level_frame(self):
        """Rotation world->level (heading removed) and the level-frame origin (robot x, y at floor height)."""
        q = self.d.xquat[self.trunk]
        yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
        Rz = np.array([[math.cos(yaw), math.sin(yaw), 0], [-math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        return Rz, np.array([self.d.qpos[0], self.d.qpos[1], 0.0]), yaw

    def camera_info(self, stamp):
        f = (self.H / 2) / math.tan(math.radians(self.fovy) / 2)
        ci = CameraInfo(); ci.header.stamp = stamp; ci.header.frame_id = "head_cam"
        ci.width, ci.height = self.W, self.H
        ci.distortion_model = "plumb_bob"; ci.d = [0.0] * 5
        ci.k = [f, 0.0, self.W / 2, 0.0, f, self.H / 2, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        ci.p = [f, 0.0, self.W / 2, 0.0, 0.0, f, self.H / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    def tof_frame(self, stamp):
        """Simulate one VL53L5CX acquisition at the current state: centre ray per zone, noise, dropouts, max range."""
        d, m = self.d, self.m
        p0 = d.site_xpos[self.tof_site].copy(); R = d.site_xmat[self.tof_site].reshape(3, 3)
        n = len(self.tof_rays)
        rng = np.full(n, np.nan, np.float32); status = np.ones(n, np.uint8)
        gid = np.zeros(1, np.int32)
        rmax, rmin = float(self.get_parameter("tof_max_range").value), float(self.get_parameter("tof_min_range").value)
        frame_drop = self.rng.random() < float(self.get_parameter("tof_frame_dropout").value)
        for z, ray in enumerate(self.tof_rays):
            vec = R @ ray
            dist = mujoco.mj_ray(m, d, p0, vec, self.ray_groups, 1, -1, gid)
            if frame_drop or self.rng.random() < float(self.get_parameter("tof_zone_dropout").value):
                status[z] = 2; continue
            if dist < 0 or not rmin <= dist <= rmax:
                status[z] = 1; continue                      # no target / out of range: NOT free space
            sigma = max(float(self.get_parameter("tof_noise_abs").value), float(self.get_parameter("tof_noise_rel").value) * dist)
            meas = dist + self.rng.normal(0.0, sigma)
            if self.get_parameter("tof_range_convention").value == "axial":
                meas *= ray[0]
            rng[z] = meas; status[z] = 0
        pts = []
        for z in range(n):
            r = float(rng[z])
            if status[z] == 0:
                rad = r / self.tof_rays[z][0] if self.get_parameter("tof_range_convention").value == "axial" else r
                x, y, zz = (self.tof_rays[z] * rad).tolist()
            else:
                x = y = zz = float("nan")
            pts.append((x, y, zz, r, int(status[z]), z))
        hdr = Header(); hdr.stamp = stamp; hdr.frame_id = "tof"
        cloud = point_cloud2.create_cloud(hdr, TOF_FIELDS, pts)
        cloud.height, cloud.width = int(round(math.sqrt(n))), int(round(math.sqrt(n)))   # row-major zone grid
        cloud.row_step = cloud.point_step * cloud.width
        return cloud

    # ---- main loop -------------------------------------------------------------------------------
    def step(self):
        if self.get_parameter("real_time").value and self.sim_t() > time.monotonic() - self.t0:
            return                                                       # do not run faster than real time
        d, m = self.d, self.m
        self.expire_command()
        q = d.xquat[self.trunk]
        g = np.array([0.0, 0.0, -1.0]); t = np.cross(q[1:], g) * 2
        grav = g - q[0] * t + np.cross(q[1:], t)
        session = self.sessions[{"rise": "sit"}.get(self.mode, self.mode)]
        self.cmd = self.command_vector()
        obs = np.concatenate([d.sensordata[self.gyro:self.gyro + 3], grav, d.qpos[self.qadr] - self.default,
                              d.qvel[self.vadr], self.last, self.cmd]).astype(np.float32)[None]
        self.last = session.run(None, {self.inp: obs})[0][0].astype(np.float32)
        d.ctrl[:14] = self.default + self.last
        for _ in range(self.n_sub):
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)              # refresh transforms/sensors for the state just integrated
        self.k += 1
        sim_t = self.sim_t()
        stamp = rclpy.time.Time(nanoseconds=int(round(sim_t * 1e9))).to_msg()
        self.pub_clock.publish(ClockMsg(clock=stamp))

        js = JointState(); js.header.stamp = stamp; js.name = self.jnames
        js.position = d.qpos[self.qadr].tolist(); js.velocity = d.qvel[self.vadr].tolist()
        self.pub_js.publish(js)
        imu = Imu(); imu.header.stamp = stamp; imu.header.frame_id = "trunk_base"
        gy = d.sensordata[self.gyro:self.gyro + 3]; ac = d.sensordata[self.acc:self.acc + 3]
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (float(v) for v in gy)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (float(v) for v in ac)
        imu.orientation_covariance[0] = -1.0
        self.pub_imu.publish(imu)

        Rz, origin, yaw = self.level_frame()
        q = d.xquat[self.trunk]
        q_level = quat_mul(np.array([math.cos(yaw / 2), 0, 0, -math.sin(yaw / 2)]), q)   # tilt only, heading removed
        tf = TransformStamped(); tf.header.stamp = stamp; tf.header.frame_id = "base_level"; tf.child_frame_id = "trunk_base"
        tf.transform.translation.z = float(d.qpos[2])
        tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z = (float(v) for v in q_level)
        self.tf.sendTransform(tf)
        gt = PoseStamped(); gt.header.stamp = stamp; gt.header.frame_id = "map"
        gt.pose.position.x, gt.pose.position.y, gt.pose.position.z = (float(v) for v in d.qpos[:3])
        gt.pose.orientation.w, gt.pose.orientation.x, gt.pose.orientation.y, gt.pose.orientation.z = (float(v) for v in q)
        self.pub_gt.publish(gt)

        if self.k % 10 == 0:
            self.pub_mode.publish(String(data=self.mode))
        if self.k % 10 == 0:                                             # camera (+ ideal depth), 5 Hz
            t0 = time.monotonic()
            self.rgb_r.update_scene(d, camera=self.cam, scene_option=self.opt); rgb = self.rgb_r.render()
            if self.profile == "realistic":
                noise = self.rng.normal(0.0, float(self.get_parameter("cam_noise").value), rgb.shape)
                rgb = np.clip(rgb.astype(np.float32) + noise, 0, 255).astype(np.uint8)
            im = Image(); im.header.stamp = stamp; im.header.frame_id = "head_cam"
            im.height, im.width, im.encoding, im.step = self.H, self.W, "rgb8", 3 * self.W
            im.data = rgb.tobytes(); self.pub_rgb.publish(im)
            self.pub_info.publish(self.camera_info(stamp))
            if self.profile == "ideal":
                self.dep_r.update_scene(d, camera=self.cam, scene_option=self.opt); dep = self.dep_r.render()
                im = Image(); im.header.stamp = stamp; im.header.frame_id = "head_cam"
                im.height, im.width, im.encoding, im.step = self.H, self.W, "32FC1", 4 * self.W
                im.data = dep.astype(np.float32).tobytes(); self.pub_dep.publish(im)
            self.render_ms = 0.9 * self.render_ms + 0.1 * (time.monotonic() - t0) * 1000
            self.pub_cpose.publish(pose_msg(stamp, "base_level", Rz @ d.cam_xmat[self.cam].reshape(3, 3),
                                            Rz @ (d.cam_xpos[self.cam] - origin)))
        if self.profile == "realistic":
            if self.k % self.tof_every == 0:
                self.pub_tpose.publish(pose_msg(stamp, "base_level", Rz @ d.site_xmat[self.tof_site].reshape(3, 3),
                                                Rz @ (d.site_xpos[self.tof_site] - origin)))
                self.tof_queue.append((sim_t + float(self.get_parameter("tof_latency").value), self.tof_frame(stamp)))
            while self.tof_queue and self.tof_queue[0][0] <= sim_t + 1e-9:
                self.pub_tof.publish(self.tof_queue.popleft()[1])
        if self.k % 1500 == 0:
            self.get_logger().info(f"sim {sim_t:.0f} s, wall {time.monotonic() - self.t0:.0f} s, render {self.render_ms:.0f} ms per camera frame")


def main():
    rclpy.init()
    node = WorldSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()
