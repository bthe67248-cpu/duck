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
    parameters = {"pose_timeout": 0.5, "map_timeout": 3.0, "slam_state_timeout": 0.75,
                  "survey_turn": 0.0, "survey_rate": 0.5, "weak_look_after": 0.0}   # body-depth turns off by default
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
    node.survey_done = False; node.survey_last = None; node.survey_acc = 0.0; node.weak_since = None
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


def test_map_revision_stops_and_waits_for_corrected_map_and_pose(nav):
    nav.goal = (1., 0.); nav.visited_goal = True
    nav.on_map_revision(message(data='1:5100000000'))
    assert nav.path is None and nav.goal is None and not nav.visited_goal
    assert nav.pub_cmd.records[-1].linear.x == 0
    assert nav.fault() == 'waiting for corrected map and pose'
    nav.now = lambda: 5.2
    nav.map_t = 5.2
    assert nav.fault() == 'waiting for corrected map and pose'
    nav.pose_t = 5.2
    assert nav.fault() is None
    nav.on_goal(message(header=message(frame_id='map', stamp=message(sec=5, nanosec=0)), point=message(x=1., y=0.)))
    assert nav.goal is None
    nav.on_map_revision(message(data='1:5000000000'))  # stale duplicate cannot reopen the gate
    assert nav.revision_stamp == pytest.approx(5.1)


def test_revision_during_planning_discards_old_plan(nav, monkeypatch):
    import duck_nav.duckslam as ds
    nav.info = ds.MapInfo(nx=5, ny=5)
    nav.goal = None; nav.visited_goal = False
    original_get = nav.get_parameter
    nav.get_parameter = lambda key: message(value=.125) if key == 'clearance' else original_get(key)
    def plan(*args, **kwargs):
        nav.on_map_revision(message(data='1:5000000000'))
        return [(0., 0.), (1., 0.)], 'explore'
    monkeypatch.setattr(ds, 'plan', plan)
    nav.apply_plan = lambda *args: pytest.fail('stale plan applied after graph optimization')
    nav.replan()
    assert nav.path is None


def test_survey_turns_in_place_while_weak_but_never_on_stale_inputs(nav):
    params = {"pose_timeout": 0.5, "map_timeout": 3.0, "slam_state_timeout": 0.75,
              "survey_turn": 1.0, "survey_rate": 0.5, "weak_look_after": 0.0}
    nav.get_parameter = lambda key: message(value=params[key])
    nav.slam_state = "weak"
    nav.drive_step()
    assert nav.pub_cmd.records[-1].angular.z == pytest.approx(0.5) and nav.pub_cmd.records[-1].linear.x == 0
    nav.pose = (0.0, 0.0, 1.2); nav.drive_step()               # turned past survey_turn: survey done
    assert nav.survey_done and nav.pub_status.records[-1].data == "slam weak"
    nav.survey_done = False; nav.survey_last = None
    nav.pose_t = 3.0                                            # stale pose: no turning
    nav.drive_step()
    assert nav.pub_cmd.records[-1].angular.z == 0 and nav.pub_status.records[-1].data == "stale pose"


def test_look_around_only_after_a_long_weak_spell(nav):
    params = {"pose_timeout": 0.5, "map_timeout": 3.0, "slam_state_timeout": 0.75,
              "survey_turn": 0.0, "survey_rate": 0.5, "weak_look_after": 2.0}
    nav.get_parameter = lambda key: message(value=params[key])
    nav.slam_state = "weak"
    nav.drive_step(); assert nav.pub_cmd.records[-1].angular.z == 0
    nav.now = lambda: 7.5; nav.pose_t = nav.map_t = nav.slam_state_t = 7.5
    nav.drive_step(); assert nav.pub_cmd.records[-1].angular.z == pytest.approx(0.5)
    nav.slam_state = "ok"; nav.drive_step(); assert nav.weak_since is None


def test_blind_sensor_turns_to_look_but_never_drives(nav):
    params = {"pose_timeout": 0.5, "map_timeout": 3.0, "slam_state_timeout": 0.75,
              "survey_turn": 0.0, "survey_rate": 0.5, "weak_look_after": 2.0}
    nav.get_parameter = lambda key: message(value=params[key])
    nav.slam_state = "blind"
    nav.drive_step(); assert nav.pub_cmd.records[-1].linear.x == 0 and nav.pub_cmd.records[-1].angular.z == 0
    nav.now = lambda: 7.5; nav.pose_t = nav.map_t = nav.slam_state_t = 7.5
    nav.drive_step(); assert nav.pub_cmd.records[-1].angular.z == pytest.approx(0.5) and nav.pub_cmd.records[-1].linear.x == 0
