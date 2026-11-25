"""Hardware ToF publisher with an explicit, installation-specific backend.

Backend parameter is 'module:factory'. factory(config_dict) returns an object
with nonblocking read() -> TofFrame | None and close(). No backend is selected
implicitly; see HARDWARE_SENSORS.md for the contract and remaining bench work.
Grid size and field of view are parameters: zones:=8 fov_deg:=45 (VL53L5CX) or zones:=25 fov_deg:=70
fov_v_deg:=60 (Sipeed A010 binned 4 x 4, backend duck_nav.a010_backend:create_backend). rays_file: optional
JSON list of calibrated per-zone unit rays (zones*zones x 3, sensor frame) replacing the pinhole model.
"""
import importlib
import json

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

from .sensor_contract import acquisition_is_fresh, positive_seconds, tof_records, zone_rays


class TofNode(Node):
    def __init__(self):
        super().__init__("duck_tof")
        for name, value in (("backend", ""), ("backend_config", "{}"), ("zone_order_verified", False),
                            ("range_convention", ""), ("frame_id", "tof"), ("fov_deg", 45.0),
                            ("min_range", 0.02), ("max_range", 2.0), ("max_age", 0.5), ("zones", 8),
                            ("fov_v_deg", 0.0), ("rays_file", "")):
            self.declare_parameter(name, value)
        self.backend = None
        self.last_stamp = 0
        if not self.get_parameter("zone_order_verified").value:
            raise ValueError("verify physical zone order before setting zone_order_verified:=true")
        self.convention = self.get_parameter("range_convention").value
        if self.convention not in ("radial", "axial"):
            raise ValueError("set measured range_convention:=radial or axial")
        self.max_age = positive_seconds(self.get_parameter("max_age").value, "max_age")
        self.zones = int(self.get_parameter("zones").value)
        self.fov = float(self.get_parameter("fov_deg").value)
        self.fov_v = float(self.get_parameter("fov_v_deg").value) or None
        path = self.get_parameter("rays_file").value
        rays = None
        if path:
            with open(path, encoding="utf-8") as f:
                rays = np.asarray(json.load(f), float)
        self.rays = zone_rays(self.zones, self.fov, self.fov_v, rays)     # validated once, reused per frame
        module, sep, factory = self.get_parameter("backend").value.partition(":")
        if not sep or not module or not factory:
            raise ValueError("backend must be an installed module:factory; no hardware driver is bundled")
        config = json.loads(self.get_parameter("backend_config").value)
        if not isinstance(config, dict):
            raise ValueError("backend_config must be a JSON object")
        self.pub = self.create_publisher(PointCloud2, "tof/zones", 5)
        self.pub_status = self.create_publisher(String, "tof/status", 1)
        self.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                       for i, n in enumerate(("x", "y", "z", "range"))] + [
            PointField(name="status", offset=16, datatype=PointField.UINT8, count=1),
            PointField(name="zone", offset=18, datatype=PointField.UINT16, count=1)]
        self.backend = getattr(importlib.import_module(module), factory)(config)
        self.create_timer(0.005, self.poll)
        self.create_timer(0.5, self.status)

    def poll(self):
        try:
            frame = self.backend.read()
            if frame is None:
                return
            now = self.get_clock().now().nanoseconds
            if not acquisition_is_fresh(frame.stamp_ns, now, self.max_age, self.last_stamp):
                raise ValueError("duplicate, out-of-order, future or stale acquisition stamp")
            records = tof_records(frame, self.convention, self.fov,
                                  float(self.get_parameter("min_range").value),
                                  float(self.get_parameter("max_range").value),
                                  zones=self.zones, fov_v_deg=self.fov_v, rays=self.rays)
            header = Header(frame_id=self.get_parameter("frame_id").value)
            header.stamp.sec, header.stamp.nanosec = divmod(frame.stamp_ns, 1_000_000_000)
            cloud = point_cloud2.create_cloud(header, self.fields, records)
            cloud.height = cloud.width = self.zones
            cloud.row_step = cloud.point_step * self.zones
            cloud.is_dense = False
            self.pub.publish(cloud)
            self.last_stamp = frame.stamp_ns
        except Exception as exc:
            self.get_logger().error(f"ToF frame rejected: {exc}", throttle_duration_sec=5.0)

    def status(self):
        now = self.get_clock().now().nanoseconds
        fresh = acquisition_is_fresh(self.last_stamp, now, self.max_age)
        self.pub_status.publish(String(data="ok" if fresh else "stale"))

    def destroy_node(self):
        try:
            if self.backend is not None:
                self.backend.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = TofNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
