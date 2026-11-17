"""slam_node -- DuckSLAM as a ROS 2 node.

Subscribes  /imu (gyro z, tilt check), /cmd_vel (walk command), /camera/depth + /camera/pose_level (same stamp),
            /yolo/detections (vision_msgs/Detection2DArray)
Publishes   /map (nav_msgs/OccupancyGrid, 1 Hz), TF map -> base_level, /slam/pose (PoseStamped),
            /goal (PointStamped): the beacon's map position, from YOLO boxes + depth
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import Detection2DArray

from . import duckslam as ds


def stamp_ns(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


def quat_to_mat(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class SlamNode(Node):
    def __init__(self):
        super().__init__("duck_slam")
        self.declare_parameter("v_gain", 0.445)        # m/s per unit walk command (calibrated in MuJoCo)
        self.declare_parameter("scan_matching", True)
        self.slam = ds.DuckSLAM(self.get_parameter("v_gain").value)
        self.cmd_vx = 0.0; self.last_imu_t = None; self.upright = True
        self.cam_poses = {}; self.frames = {}          # stamp -> camera pose / (depth, R, t, slam pose)
        self.goal = None; self.goal_n = 0; self.pending = None
        self.pub_map = self.create_publisher(OccupancyGrid, "map", 1)
        self.pub_pose = self.create_publisher(PoseStamped, "slam/pose", 10)
        self.pub_goal = self.create_publisher(PointStamped, "goal", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, "imu", self.on_imu, 50)
        self.create_subscription(Twist, "cmd_vel", lambda m: setattr(self, "cmd_vx", m.linear.x), 10)
        self.create_subscription(PoseStamped, "camera/pose_level", self.on_cam_pose, 5)
        self.create_subscription(Image, "camera/depth", self.on_depth, 2)
        self.create_subscription(Detection2DArray, "yolo/detections", self.on_det, 5)
        self.create_timer(1.0, self.publish_map)

    # 1. predict (50 Hz) ------------------------------------------------------------------
    def on_imu(self, msg):
        t = stamp_ns(msg.header) * 1e-9
        a = msg.linear_acceleration
        self.upright = a.z > 0.85 * math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2 + 1e-9)
        if self.last_imu_t is not None and 0 < t - self.last_imu_t < 0.2:
            self.slam.predict(self.cmd_vx if self.upright else 0.0, msg.angular_velocity.z, t - self.last_imu_t)
        self.last_imu_t = t
        x, y, th = self.slam.pose
        tf = TransformStamped(); tf.header.stamp = msg.header.stamp; tf.header.frame_id = "map"; tf.child_frame_id = "base_level"
        tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
        self.tf.sendTransform(tf)
        p = PoseStamped(); p.header.stamp = msg.header.stamp; p.header.frame_id = "map"
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        self.pub_pose.publish(p)

    def on_cam_pose(self, msg):
        o, p = msg.pose.orientation, msg.pose.position
        self.cam_poses[stamp_ns(msg.header)] = (quat_to_mat(o.w, o.x, o.y, o.z), np.array([p.x, p.y, p.z]))
        for k in sorted(self.cam_poses)[:-8]:
            del self.cam_poses[k]
        if self.pending is not None and stamp_ns(self.pending.header) in self.cam_poses:   # depth arrived first
            msg_d, self.pending = self.pending, None
            self.on_depth(msg_d)

    # 2-4. scan, match, map (5 Hz) ------------------------------------------------------------
    def on_depth(self, msg):
        ns = stamp_ns(msg.header)
        if ns not in self.cam_poses:
            self.pending = msg                      # wait for the camera pose with the same stamp
            return
        if not self.upright:
            return
        R_bc, t_bc = self.cam_poses[ns]
        depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        ang, r, hit = self.slam.scan_from_depth(depth, R_bc, t_bc)
        if len(ang):
            if self.get_parameter("scan_matching").value:
                self.slam.match(ang, r, hit)
            self.slam.update_map(ang, r, hit)
        self.frames[ns] = (depth, R_bc, t_bc, self.slam.pose.copy())
        for k in sorted(self.frames)[:-10]:
            del self.frames[k]

    # 5. beacon position from YOLO boxes + depth ---------------------------------------------
    def on_det(self, msg):
        ns = stamp_ns(msg.header)
        if ns not in self.frames:
            return
        depth, R_bc, t_bc, pose = self.frames[ns]
        for det in msg.detections:
            if not det.results or det.results[0].hypothesis.class_id != "beacon":
                continue
            cx, cy, w, h = det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.size_x, det.bbox.size_y
            u0, u1 = int(max(0, cx - w / 4)), int(min(ds.W, cx + w / 4 + 1))
            v0, v1 = int(max(0, cy - h / 4)), int(min(ds.H, cy + h / 4 + 1))
            patch = depth[v0:v1, u0:u1]
            if patch.size == 0:
                continue
            z = float(np.median(patch))
            if not 0.1 < z < ds.MAX_R:
                continue
            pb = ds.pixel_to_level(cx, cy, z, R_bc, t_bc)
            pb[:2] *= (np.hypot(*pb[:2]) + 0.04) / max(np.hypot(*pb[:2]), 1e-6)      # surface -> centre of the cylinder
            c, s = math.cos(pose[2]), math.sin(pose[2])
            g = np.array([pose[0] + c * pb[0] - s * pb[1], pose[1] + s * pb[0] + c * pb[1]])
            self.goal_n += 1
            self.goal = g if self.goal is None else self.goal + (g - self.goal) / min(self.goal_n, 20)
            if self.goal_n == 1:
                self.get_logger().info(f"beacon first seen by YOLO, at ({g[0]:.2f}, {g[1]:.2f}) in the map")
        if self.goal is not None:
            p = PointStamped(); p.header.stamp = msg.header.stamp; p.header.frame_id = "map"
            p.point.x, p.point.y = float(self.goal[0]), float(self.goal[1])
            self.pub_goal.publish(p)

    def publish_map(self):
        g = OccupancyGrid(); g.header.stamp = self.get_clock().now().to_msg(); g.header.frame_id = "map"
        g.info.resolution = ds.RES; g.info.width, g.info.height = ds.NX, ds.NY
        g.info.origin.position.x, g.info.origin.position.y = ds.X0, ds.Y0; g.info.origin.orientation.w = 1.0
        g.data = self.slam.occupancy().T.flatten().tolist()              # row-major, rows along y
        self.pub_map.publish(g)


def main():
    rclpy.init()
    node = SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()
