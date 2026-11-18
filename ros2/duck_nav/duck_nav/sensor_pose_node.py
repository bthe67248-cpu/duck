"""Publish SLAM sensor poses using calibrated TF at acquisition times only.

Requires hardware joint-state/IMU TF and measured sensor extrinsics; never uses
latest TF as a fallback. Camera input frame is REP-103 optical; ToF is +x forward,
+y left, +z up. Both outputs are relative to base_level.
"""
from collections import deque
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformException, TransformListener

from .sensor_contract import optical_to_mujoco_quaternion, positive_seconds


class SensorPoseNode(Node):
    def __init__(self):
        super().__init__("sensor_pose_node")
        self.declare_parameter("level_frame", "base_level")
        self.declare_parameter("wait_timeout", 0.5)
        self.wait_timeout = positive_seconds(self.get_parameter("wait_timeout").value, "wait_timeout")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = deque(maxlen=128)
        self.camera_pub = self.create_publisher(PoseStamped, "camera/pose_level", 10)
        self.tof_pub = self.create_publisher(PoseStamped, "tof/pose_level", 10)
        self.create_subscription(Image, "camera/image_raw", lambda msg: self.enqueue(msg, True),
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "tof/zones", lambda msg: self.enqueue(msg, False),
                                 qos_profile_sensor_data)
        self.create_timer(0.01, self.flush)

    def enqueue(self, msg, camera):
        h = msg.header
        if not h.frame_id or h.stamp.sec < 0 or h.stamp.sec == h.stamp.nanosec == 0:
            self.get_logger().error("sensor frame needs a frame_id and nonzero acquisition stamp",
                                    throttle_duration_sec=5.0)
            return
        if len(self.pending) == self.pending.maxlen:
            self.get_logger().warn("sensor pose queue full: dropping oldest acquisition",
                                   throttle_duration_sec=5.0)
        self.pending.append((time.monotonic(), h, camera))
        self.flush()

    def flush(self):
        for _ in range(len(self.pending)):
            queued_at, header, camera = self.pending.popleft()
            try:
                tf = self.buffer.lookup_transform(self.get_parameter("level_frame").value,
                                                  header.frame_id, Time.from_msg(header.stamp))
            except TransformException:
                if time.monotonic() - queued_at < self.wait_timeout:
                    self.pending.append((queued_at, header, camera))
                else:
                    self.get_logger().warn(f"no acquisition-time TF for {header.frame_id}: dropping pose",
                                           throttle_duration_sec=5.0)
                continue
            pose = PoseStamped()
            pose.header.stamp = header.stamp
            pose.header.frame_id = self.get_parameter("level_frame").value
            p, q = tf.transform.translation, tf.transform.rotation
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p.x, p.y, p.z
            if camera:
                x, y, z, w = optical_to_mujoco_quaternion([q.x, q.y, q.z, q.w])
                pose.pose.orientation.x, pose.pose.orientation.y = float(x), float(y)
                pose.pose.orientation.z, pose.pose.orientation.w = float(z), float(w)
            else:
                pose.pose.orientation = q
            (self.camera_pub if camera else self.tof_pub).publish(pose)


def main():
    rclpy.init()
    node = None
    try:
        node = SensorPoseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
