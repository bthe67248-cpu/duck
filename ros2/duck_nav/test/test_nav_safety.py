"""Node safety logic with message/ROS shims; no ROS executor or transport claims."""
import importlib
import sys
import threading
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def message(**kwargs):
    return SimpleNamespace(**kwargs)


def twist():
    return message(linear=message(x=0.0, y=0.0, z=0.0), angular=message(x=0.0, y=0.0, z=0.0))


@pytest.fixture
def nav(monkeypatch):
    # Import the actual node methods while keeping the test independent of a ROS
    # installation. Constructors/executors/transports are intentionally not run.
    definitions = {
        "rclpy": {},
        "rclpy.callback_groups": {"MutuallyExclusiveCallbackGroup": object},
        "rclpy.executors": {"ExternalShutdownException": Exception, "MultiThreadedExecutor": object},
        "rclpy.node": {"Node": object},
        "rclpy.qos": {"DurabilityPolicy": object, "QoSProfile": object},
        "geometry_msgs.msg": {"PointStamped": message, "PoseStamped": message, "Twist": twist},
        "nav_msgs.msg": {"OccupancyGrid": message, "Path": message},
        "std_msgs.msg": {"Bool": message, "Float64": message, "String": message},
    }
    for name, values in definitions.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    module = importlib.import_module("duck_nav.nav_node")
    node = object.__new__(module.NavNode)
    parameters = {"pose_timeout": 0.5, "map_timeout": 3.0, "slam_state_timeout": 0.75}
    node.get_parameter = lambda key: message(value=parameters[key])
    node.now = lambda: 5.0
    node.get_logger = lambda: message(info=lambda _: None)
    node.enabled = True; node.finished = False; node.mode = "explore"
    node.pose = (0.0, 0.0, 0.0); node.pose_t = 5.0
    node.grid = np.zeros((5, 5), np.int8); node.map_t = 5.0
    node.slam_state = "ok"; node.slam_state_t = 5.0
    node.path = [(0.0, 0.0), (1.0, 0.0)]
    node.status = ""; node.lock = threading.RLock()
    node.unstick_until = 0.0; node.progress_ref = None
    for name in ("pub_cmd", "pub_head", "pub_status"):
        records = []
        setattr(node, name, message(publish=records.append, records=records))
    yield node
    monkeypatch.delitem(sys.modules, "duck_nav.nav_node", raising=False)


def test_pose_arrival_does_not_freshen_old_acquisition_stamp(nav):
    msg = message(header=message(frame_id="map", stamp=message(sec=1, nanosec=0)),
                  pose=message(position=message(x=0.1, y=0.2), orientation=message(z=0.0, w=1.0)))
    nav.on_pose(msg)
    assert nav.pose_t == 1.0
    assert nav.fault() == "stale pose"


@pytest.mark.parametrize("field,value,reason", [
    ("slam_state_t", None, "stale slam state"),
    ("slam_state_t", 4.0, "stale slam state"),
    ("pose_t", 6.0, "stale pose"),
    ("map_t", 1.0, "stale map"),
    ("slam_state", "weak", "slam weak"),
])
def test_navigation_fails_closed_for_stale_or_weak_inputs(nav, field, value, reason):
    setattr(nav, field, value)
    assert nav.fault() == reason
    nav.drive_step()
    assert len(nav.pub_cmd.records) == 1
    cmd = nav.pub_cmd.records[0]
    assert cmd.linear.x == cmd.angular.z == 0.0
    assert nav.path is None


def test_weak_localization_stops_body_but_preserves_head_sweep(nav):
    nav.slam_state = "weak"
    nav.drive_step()
    assert nav.pub_head.records[0].data == pytest.approx(0.9)
    assert len(nav.pub_head.records) == 1  # stop must not immediately recenter the head
    assert nav.pub_status.records[-1].data == "slam weak"
