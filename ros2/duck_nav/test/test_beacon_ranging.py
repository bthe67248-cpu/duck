"""Beacon ranging in slam_node (camera bearing + ToF depth, box-size depth), with ROS shims only."""
import importlib
import math
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import duck_nav.duckslam as ds  # noqa: E402

PARAMS = {"target_height": 0.28, "size_agreement": 0.3, "size_max_range": 2.5, "range_spread_max": 0.15}


@pytest.fixture
def node(monkeypatch):
    names = {"rclpy": {}, "rclpy.executors": {"ExternalShutdownException": Exception}, "rclpy.node": {"Node": object},
             "geometry_msgs.msg": dict.fromkeys(["PointStamped", "PoseStamped", "TransformStamped", "Twist"], object),
             "nav_msgs.msg": {"OccupancyGrid": object}, "std_msgs.msg": {"String": object},
             "sensor_msgs.msg": dict.fromkeys(["CameraInfo", "Image", "Imu", "PointCloud2"], object),
             "tf2_ros": {"TransformBroadcaster": object}, "vision_msgs.msg": {"Detection2DArray": object}}
    for name, values in names.items():
        m = ModuleType(name); m.__dict__.update(values); monkeypatch.setitem(sys.modules, name, m)
    mod = importlib.import_module("duck_nav.slam_node")
    n = object.__new__(mod.SlamNode)
    n.get_parameter = lambda k: SimpleNamespace(value=PARAMS[k])
    n.K = ds.Intrinsics.from_fovy(320, 240, 58.0)
    yield n
    monkeypatch.delitem(sys.modules, "duck_nav.slam_node", raising=False)


def camera():
    """Camera 0.15 m up, looking along +x of the level frame (MuJoCo camera: -z forward, y up)."""
    R = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    return R, np.array([0.0, 0.0, 0.15])


def beacon_box(n, p, R, t):
    """Pixel box of a 0.28 m tall target whose near surface centre is at level point p (on the floor plane)."""
    u, v, _ = ds.level_to_pixel(np.array([[p[0], p[1], 0.0], [p[0], p[1], 0.28]]), R, t, n.K)
    return float(u.mean()), float(v.mean()), 12.0, float(v[0] - v[1])


def tof_zones(p, spread=0.0):
    """Four ToF zone hits around the target, quantized ~3 deg off sideways as an 8x8 zone grid does."""
    r = math.hypot(*p[:2]); a = math.atan2(p[1], p[0]) + math.radians(3)
    pts = np.array([[r * math.cos(a), r * math.sin(a), z] for z in (0.06, 0.10, 0.14, 0.18)])
    pts[:, :2] *= (1 + np.array([0, spread, 0, 0]))[:, None]
    return pts, np.linalg.norm(pts[:, :2], axis=1)


def test_camera_bearing_with_tof_depth_beats_zone_centres(node):
    R, t = camera(); p = np.array([1.2, 0.3, 0.14])
    cx, cy, w, h = beacon_box(node, p, R, t)
    pts, rng = tof_zones(p)
    pb, why = node.range_tof((pts, rng), cx, cy, w, h, R, t)
    assert why == ""
    lateral_new = abs(math.atan2(pb[1], pb[0]) - math.atan2(p[1], p[0]))
    lateral_old = abs(math.atan2(*np.median(pts, axis=0)[1::-1]) - math.atan2(p[1], p[0]))
    assert lateral_new < 0.005 < lateral_old              # < 0.3 deg instead of the zone's 3 deg
    assert math.hypot(*(pb[:2] - p[:2])) < 0.03


def test_box_size_gives_range_without_tof_and_rejects_background(node):
    R, t = camera(); p = np.array([1.5, -0.2, 0.14])
    cx, cy, w, h = beacon_box(node, p, R, t)
    pb, why = node.range_tof((np.zeros((0, 3)), np.zeros(0)), cx, cy, w, h, R, t)
    assert why == "" and math.hypot(*(pb[:2] - p[:2])) < 0.08
    wall = np.array([2.6, -0.35, 0.14])                     # ToF zones see the wall behind the target
    pts, rng = tof_zones(wall)
    pts[:, :2] = wall[:2] * (1 + 0 * pts[:, :1]); rng = np.linalg.norm(pts[:, :2], axis=1)
    u, v, z = ds.level_to_pixel(pts, R, t, node.K)
    pb, why = node.range_tof((pts, rng), float(u.mean()), cy, w, h, R, t)
    assert pb is None and "background" in why


def test_clipped_box_has_no_size_range(node):
    assert node.size_depth(cy=10.0, h=30.0) is None        # top edge cut off by the image border
    assert node.size_depth(cy=120.0, h=5.0) is None        # too small to trust
    assert node.size_depth(cy=120.0, h=40.0) == pytest.approx(node.K.fy * 0.28 / 40.0)


def test_graph_revision_discards_map_frame_beacon_caches(node, monkeypatch):
    from collections import deque
    from duck_nav import slam_node
    monkeypatch.setattr(slam_node, 'String', lambda **kwargs: SimpleNamespace(**kwargs))
    node.frames = SimpleNamespace(d={1: 'old frame'})
    node.pending_dets = deque([1]); node.candidates = [1]
    node.goal = (1., 2.); node.goal_n = 2
    node.loc = SimpleNamespace(graph=SimpleNamespace(revision=2))
    node.now_ns = lambda: 100
    events = []; maps = []
    node.pub_revision = SimpleNamespace(publish=events.append)
    node.publish_map = lambda: maps.append(True)
    node.invalidate_map_caches()
    assert not node.frames.d and not node.pending_dets and not node.candidates
    assert node.goal is None and node.goal_n == 0
    assert events[0].data == '2:100'
    assert maps == [True]
