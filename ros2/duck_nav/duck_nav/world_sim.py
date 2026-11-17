"""world_sim -- MuJoCo world + Duckling V3 + walking policy as a ROS 2 node (the "robot and its sensors").

Subscribes  /cmd_vel (geometry_msgs/Twist)  linear.x = walk command (0.5 = walk, below 0.25 = stand; negative = back),
                                            linear.y = side-step command, angular.z = turn
            /head_yaw_cmd (std_msgs/Float64) head pan, rad
            /duck/pose_cmd (std_msgs/Float32MultiArray, 10 values) head [neck pitch, head pitch, yaw, roll] +
                                            body [x, y, z, roll, pitch, yaw] commands for the walking policy
            /duck/behavior (std_msgs/String) "sit", "stand", "roll", "pick": switches to the matching policy
Publishes   /clock                      simulation time (all other nodes run with use_sim_time)
            /joint_states, /imu         50 Hz
            /camera/image_raw           rgb8 320x240, 5 Hz        (head camera)
            /camera/depth               32FC1 metres, same stamp  (depth sensor)
            /camera/pose_level          camera pose in the level body frame: what leg/neck encoders + IMU tilt give
            TF base_level -> trunk_base body height and tilt
            /ground_truth/pose          true pose, for scoring and display only
            /world/markers              the real walls and boxes, for comparison in RViz
            /duck/mode                  which policy is driving: walk, sit, rise, roll, pick
The walking policy (velstand.onnx) turns the commands into 14 servo targets at 50 Hz.
"""
import json
import math
import os
import time

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
from sensor_msgs.msg import Image, Imu, JointState
from std_msgs.msg import Float32MultiArray, Float64, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

W, H = 320, 240


def quat_mul(a, b):
    return np.array([a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3], a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
                     a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1], a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]])


class WorldSim(Node):
    def __init__(self):
        super().__init__("world_sim")
        for name in ("mjcf", "policy", "world_json"):
            self.declare_parameter(name, "")
        self.declare_parameter("real_time", True)
        self.m = mujoco.MjModel.from_xml_path(self.get_parameter("mjcf").value)
        self.m.opt.timestep = 0.005
        self.d = mujoco.MjData(self.m)
        key = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_KEY, "sim_start")
        mujoco.mj_resetDataKeyframe(self.m, self.d, key)
        self.default = self.m.key_ctrl[key][:14].copy().astype(np.float32)
        jid = [self.m.actuator_trnid[i, 0] for i in range(14)]
        self.jnames = [self.m.joint(j).name for j in jid]
        self.qadr = np.array([self.m.jnt_qposadr[j] for j in jid]); self.vadr = np.array([self.m.jnt_dofadr[j] for j in jid])
        self.gyro = self.m.sensor_adr[self.m.sensor("gyro").id]; self.acc = self.m.sensor_adr[self.m.sensor("accel").id]
        self.trunk = self.m.body("trunk_base").id; self.cam = self.m.camera("head_cam").id
        pdir = os.path.dirname(self.get_parameter("policy").value)
        self.sessions = {}
        for mode, fname in (("walk", os.path.basename(self.get_parameter("policy").value)), ("sit", "alpha_sitstand.onnx"),
                            ("roll", "roulade.onnx"), ("pick", "alpha_ground_pick.onnx")):
            if os.path.isfile(os.path.join(pdir, fname)):
                self.sessions[mode] = ort.InferenceSession(os.path.join(pdir, fname), providers=["CPUExecutionProvider"])
        self.inp = self.sessions["walk"].get_inputs()[0].name
        self.mode = "walk"; self.mode_left = 0.0; self.phase = 0.0
        self.twist = np.zeros(3, np.float32); self.head_yaw = 0.0; self.pose_cmd = np.zeros(10, np.float32)
        self.last = np.zeros(14, np.float32); self.cmd = np.zeros(13, np.float32)
        self.rgb_r = mujoco.Renderer(self.m, H, W)
        self.dep_r = mujoco.Renderer(self.m, H, W); self.dep_r.enable_depth_rendering()
        self.opt = mujoco.MjvOption(); self.opt.geomgroup[2] = 0          # camera does not render the duck itself
        self.k = 0; self.t0 = time.monotonic(); self.render_ms = 0.0

        self.pub_clock = self.create_publisher(ClockMsg, "clock", 10)
        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_imu = self.create_publisher(Imu, "imu", 50)
        self.pub_rgb = self.create_publisher(Image, "camera/image_raw", 2)
        self.pub_dep = self.create_publisher(Image, "camera/depth", 2)
        self.pub_cpose = self.create_publisher(PoseStamped, "camera/pose_level", 2)
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
        self.get_logger().info("world_sim up: 50 Hz policy, 200 Hz physics, camera 320x240 at 5 Hz")

    def on_cmd(self, msg):
        self.twist[:] = (msg.linear.x, msg.linear.y, msg.angular.z)

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

    def command_vector(self):
        """13-D policy command for the current mode; also advances timed behaviours (called at 50 Hz)."""
        c = np.zeros(13, np.float32)
        if self.mode == "walk":
            c[0:3] = self.twist; c[3:13] = self.pose_cmd; c[5] += self.head_yaw
        elif self.mode == "sit":
            c[0] = 1.0; c[3:7] = self.pose_cmd[:4]             # posture flag 1 = sit; head stays steerable
        elif self.mode == "pick":
            c[0], c[1] = math.cos(2 * math.pi * self.phase), math.sin(2 * math.pi * self.phase)
            self.phase += 0.02 / 4.0
            if self.phase >= 0.7:
                self.mode = "walk"
        if self.mode in ("rise", "roll"):                      # zero command: "stand" for sitstand, "go" for the roll
            self.mode_left -= 0.02
            if self.mode_left <= 0:
                self.mode = "walk"
        return c

    def publish_markers(self):
        path = self.get_parameter("world_json").value
        if not path:
            return
        arr = MarkerArray()
        for i, it in enumerate(json.load(open(path))["items"]):
            mk = Marker(); mk.header.frame_id = "map"; mk.ns = "world"; mk.id = i
            mk.type = Marker.CYLINDER if it["name"] == "beacon" else Marker.CUBE
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(it["x"]), float(it["y"]), float(it["sz"])
            mk.pose.orientation.z, mk.pose.orientation.w = math.sin(it["yaw"] / 2), math.cos(it["yaw"] / 2)
            mk.scale.x, mk.scale.y, mk.scale.z = 2.0 * it["sx"], 2.0 * it["sy"], 2.0 * it["sz"]
            mk.color.r, mk.color.g, mk.color.b = (float(v) for v in it["rgba"][:3]); mk.color.a = 0.45
            arr.markers.append(mk)
        self.pub_mk.publish(arr)

    def step(self):
        sim_t = self.k * 0.02
        if self.get_parameter("real_time").value and sim_t > time.monotonic() - self.t0:
            return                                                       # do not run faster than real time
        d, m = self.d, self.m
        q = d.xquat[self.trunk]
        g = np.array([0.0, 0.0, -1.0]); t = np.cross(q[1:], g) * 2
        grav = g - q[0] * t + np.cross(q[1:], t)
        session = self.sessions[{"rise": "sit"}.get(self.mode, self.mode)]
        self.cmd = self.command_vector()
        obs = np.concatenate([d.sensordata[self.gyro:self.gyro + 3], grav, d.qpos[self.qadr] - self.default,
                              d.qvel[self.vadr], self.last, self.cmd]).astype(np.float32)[None]
        self.last = session.run(None, {self.inp: obs})[0][0].astype(np.float32)
        d.ctrl[:14] = self.default + self.last
        for _ in range(4):
            mujoco.mj_step(m, d)
        self.k += 1
        sim_t = self.k * 0.02
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

        q = d.xquat[self.trunk]
        yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
        q_level = quat_mul(np.array([math.cos(yaw / 2), 0, 0, -math.sin(yaw / 2)]), q)   # tilt only, heading removed
        tf = TransformStamped(); tf.header.stamp = stamp; tf.header.frame_id = "base_level"; tf.child_frame_id = "trunk_base"
        tf.transform.translation.z = float(d.qpos[2])
        tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z = (float(v) for v in q_level)
        self.tf.sendTransform(tf)
        gt = PoseStamped(); gt.header.stamp = stamp; gt.header.frame_id = "map"
        gt.pose.position.x, gt.pose.position.y, gt.pose.position.z = (float(v) for v in d.qpos[:3])
        gt.pose.orientation.w, gt.pose.orientation.z = math.cos(yaw / 2), math.sin(yaw / 2)
        self.pub_gt.publish(gt)

        if self.k % 10 == 0:
            self.pub_mode.publish(String(data=self.mode))
        if self.k % 10 == 0:                                             # camera + depth, 5 Hz
            t0 = time.monotonic()
            self.rgb_r.update_scene(d, camera=self.cam, scene_option=self.opt); rgb = self.rgb_r.render()
            self.dep_r.update_scene(d, camera=self.cam, scene_option=self.opt); dep = self.dep_r.render()
            self.render_ms = 0.9 * self.render_ms + 0.1 * (time.monotonic() - t0) * 1000
            for pub, enc, data, step in ((self.pub_rgb, "rgb8", rgb, 3 * W), (self.pub_dep, "32FC1", dep.astype(np.float32), 4 * W)):
                im = Image(); im.header.stamp = stamp; im.header.frame_id = "head_cam"
                im.height, im.width, im.encoding, im.step = H, W, enc, step
                im.data = data.tobytes(); pub.publish(im)
            Rz = np.array([[math.cos(yaw), math.sin(yaw), 0], [-math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
            R_bc = Rz @ d.cam_xmat[self.cam].reshape(3, 3)
            t_bc = Rz @ (d.cam_xpos[self.cam] - np.array([d.qpos[0], d.qpos[1], 0.0]))
            qc = np.zeros(4); mujoco.mju_mat2Quat(qc, R_bc.flatten())
            cp = PoseStamped(); cp.header.stamp = stamp; cp.header.frame_id = "base_level"
            cp.pose.position.x, cp.pose.position.y, cp.pose.position.z = (float(v) for v in t_bc)
            cp.pose.orientation.w, cp.pose.orientation.x, cp.pose.orientation.y, cp.pose.orientation.z = (float(v) for v in qc)
            self.pub_cpose.publish(cp)
        if self.k % 1500 == 0:
            self.get_logger().info(f"sim {sim_t:.0f} s, wall {time.monotonic() - self.t0:.0f} s, render {self.render_ms:.0f} ms per camera frame pair")


def main():
    rclpy.init()
    node = WorldSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()
