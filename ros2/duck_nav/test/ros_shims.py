"""Minimal stand-ins for the ROS modules the duck_nav nodes import, so node logic can be tested without ROS.

These shims only make the modules importable and record what a node asks of them (subscriptions, QoS). They do not
model ROS transport: QoS compatibility between real publishers and subscribers needs a real ROS job (TODO.md).
"""
import enum
import sys
from types import ModuleType


class ReliabilityPolicy(enum.Enum):
    RELIABLE = 1
    BEST_EFFORT = 2


class HistoryPolicy(enum.Enum):
    KEEP_LAST = 1
    KEEP_ALL = 2


class DurabilityPolicy(enum.Enum):
    VOLATILE = 1
    TRANSIENT_LOCAL = 2


class QoSProfile:
    def __init__(self, depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                 durability=DurabilityPolicy.VOLATILE):
        self.depth, self.reliability, self.history, self.durability = depth, reliability, history, durability


QOS_SENSOR_DATA = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

MSG_TYPES = {
    "geometry_msgs.msg": ["PointStamped", "PoseStamped", "TransformStamped", "Twist", "Point", "Quaternion"],
    "nav_msgs.msg": ["OccupancyGrid", "Path"],
    "std_msgs.msg": ["String", "Bool", "Float64", "Empty", "Header"],
    "sensor_msgs.msg": ["CameraInfo", "Image", "Imu", "PointCloud2", "PointField", "JointState"],
    "vision_msgs.msg": ["Detection2DArray", "Detection2D", "ObjectHypothesisWithPose"],
}


def install(monkeypatch, extra=None):
    """Put the shim modules into sys.modules (undone by monkeypatch). `extra`: {module: {name: value}}."""
    mods = {"rclpy": {}, "rclpy.executors": {"ExternalShutdownException": Exception}, "rclpy.node": {"Node": object},
            "rclpy.qos": {"QoSProfile": QoSProfile, "ReliabilityPolicy": ReliabilityPolicy,
                          "HistoryPolicy": HistoryPolicy, "DurabilityPolicy": DurabilityPolicy,
                          "qos_profile_sensor_data": QOS_SENSOR_DATA},
            "tf2_ros": {"TransformBroadcaster": object, "Buffer": object, "TransformListener": object}}
    for m, names in MSG_TYPES.items():
        mods[m] = {n: type(n, (), {}) for n in names}
    for m, names in (extra or {}).items():
        mods.setdefault(m, {}).update(names)
    for name, values in mods.items():
        mod = ModuleType(name); mod.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, mod)
