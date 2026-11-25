"""Node safety logic with message/ROS shims; no ROS executor or transport claims."""
import importlib
import math
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


PARAMS = {"pose_timeout": 0.5, "map_timeout": 3.0, "slam_state_timeout": 0.75,
          "survey_turn": 0.0, "survey_rate": 0.5, "weak_look_after": 0.0,   # body-depth turns off by default
          "clearance": 0.125, "speed_bound_mps": 0.35, "walk_cmd": 0.5, "yaw_sweep_rate": 0.0, "yaw_sweep_period": 3.0,
          "corner_handling": True, "plan_comfort": 0.10, "guard_recover_s": 2.0, "frontier_avoid_s": 60.0,
          "min_explored_m2": 2.0, "no_frontier_plans": 5, "goal_reached_m": 0.36}


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
    parameters = dict(PARAMS)
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
    node.guard_time = 0.0; node.episode_recoveries = 0; node.avoid = []; node.manoeuvre = None
    node.info = None; node.distances = None
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
    params = dict(PARAMS, survey_turn=1.0)
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
    params = dict(PARAMS, weak_look_after=2.0)
    nav.get_parameter = lambda key: message(value=params[key])
    nav.slam_state = "weak"
    nav.drive_step(); assert nav.pub_cmd.records[-1].angular.z == 0
    nav.now = lambda: 7.5; nav.pose_t = nav.map_t = nav.slam_state_t = 7.5
    nav.drive_step(); assert nav.pub_cmd.records[-1].angular.z == pytest.approx(0.5)
    nav.slam_state = "ok"; nav.drive_step(); assert nav.weak_since is None


def test_blind_sensor_turns_to_look_but_never_drives(nav):
    params = dict(PARAMS, weak_look_after=2.0)
    nav.get_parameter = lambda key: message(value=params[key])
    nav.slam_state = "blind"
    nav.drive_step(); assert nav.pub_cmd.records[-1].linear.x == 0 and nav.pub_cmd.records[-1].angular.z == 0
    nav.now = lambda: 7.5; nav.pose_t = nav.map_t = nav.slam_state_t = 7.5
    nav.drive_step(); assert nav.pub_cmd.records[-1].angular.z == pytest.approx(0.5) and nav.pub_cmd.records[-1].linear.x == 0


# ---- corners: planner / controller agreement near walls ------------------------------------------
def l_corridor():
    """0.5 m wide corridor along +x that turns left into +y (2 x 2 m map, 2.5 cm cells)."""
    import duck_nav.duckslam as ds
    info = ds.MapInfo(res=0.025, x0=0.0, y0=0.0, nx=80, ny=80)
    g = np.full((80, 80), 100, np.int8)
    g[4:64, 4:24] = 0; g[44:64, 4:76] = 0
    return g, info, ds._clearance_m(g, info)[1]


START, GOAL = (0.3, 0.35, 0.0), (1.35, 1.7)
CUT_POSE = (1.0, 0.45, 0.3)        # before the corner: aiming 0.22 m ahead cuts the inner corner
ARC_POSE = (0.95, 0.45, 0.3)       # even the shortened lookahead's arc clips the inner wall; others are clear
WALL_POSE = (1.05, 0.45, 0.9)      # at the margin, facing the outer wall: no forward arc is clear


def pursuit_rate(path, pose, lookahead=0.22):
    import duck_nav.duckslam as ds
    tgt = ds.path_target(path, pose, lookahead)
    e = (math.atan2(tgt[1] - pose[1], tgt[0] - pose[0]) - pose[2] + math.pi) % (2 * math.pi) - math.pi
    return float(np.clip(2.0 * e, -1.0, 1.0))


def hugging_path():
    import duck_nav.duckslam as ds
    g, info, d = l_corridor()
    return ds.plan(g, START, goal=GOAL, info=info, comfort=0.0)[0]   # plain shortest path: hugs the inner corner


def min_clearance(path, info, d):
    return min(d[info.cell(*p)] for p in path)


def test_weighted_plan_keeps_off_the_inner_corner_where_there_is_room():
    import duck_nav.duckslam as ds
    g, info, d = l_corridor()
    path, mode = ds.plan(g, START, goal=GOAL, info=info)
    assert mode == "goal"
    assert min_clearance(hugging_path(), info, d) == pytest.approx(0.125)
    assert min_clearance(path, info, d) >= 0.17                     # 0.5 m corridor: the centre line has 0.25 m


def test_lookahead_shrinks_at_corners_and_near_walls():
    import duck_nav.duckslam as ds
    g, info, d = l_corridor()
    path = hugging_path()
    la_corner, turn_corner = ds.adaptive_lookahead(path, (1.0, 0.4375, 0.0))
    assert la_corner < 0.13 and turn_corner > 1.2
    la_straight, turn_straight = ds.adaptive_lookahead([(0.05 * i, 0.0) for i in range(20)], (0.0, 0.0, 0.0))
    assert la_straight == pytest.approx(0.22) and turn_straight == pytest.approx(0.0)
    # straight stretch of the hugging path, right at the clearance margin: shortest lookahead
    la_wall, turn_wall = ds.adaptive_lookahead(path, (1.2125, 0.9, 1.57), info=info, distances=d)
    assert turn_wall < 0.1 and la_wall == pytest.approx(0.10)


def test_arc_search_finds_a_clear_arc_where_pure_pursuit_fails():
    import duck_nav.duckslam as ds
    from duck_nav.nav_node import ARC_RATES
    g, info, d = l_corridor()
    path, pose = hugging_path(), CUT_POSE
    w_pp = pursuit_rate(path, pose)
    assert not ds.motion_clear(g, pose, 0.35, w_pp, info=info, distances=d)        # the pure-pursuit arc
    w = ds.choose_arc(g, pose, path, 0.35, ARC_RATES, preferred=w_pp, info=info, distances=d)
    assert w is not None and ds.motion_clear(g, pose, 0.35, w, info=info, distances=d)
    end = ds.arc_end(pose, 0.35, w, 0.6)
    assert math.hypot(end[0] - GOAL[0], end[1] - GOAL[1]) < math.hypot(pose[0] - GOAL[0], pose[1] - GOAL[1])
    # at the margin facing the outer wall of the corner: nothing forward is clear, so no arc is offered
    assert ds.choose_arc(g, WALL_POSE, path, 0.35, ARC_RATES, info=info, distances=d) is None


def test_plan_skips_avoided_frontiers_but_not_the_goal():
    import duck_nav.duckslam as ds
    info = ds.MapInfo(res=0.025, x0=0.0, y0=0.0, nx=80, ny=40)
    g = np.full((80, 40), 100, np.int8); g[4:76, 4:36] = 0
    g[4:10, 4:36] = -1; g[70:76, 4:36] = -1                         # unknown at both ends; the west end is nearer
    pose = (0.7, 0.5, 0.0)
    path, mode = ds.plan(g, pose, info=info)
    assert mode == "explore" and path[-1][0] < 0.5
    path, mode = ds.plan(g, pose, info=info, avoid=[(path[-1][0], path[-1][1], 0.4)])
    assert mode == "explore" and path[-1][0] > 1.5
    assert ds.plan(g, pose, info=info, avoid=[(0.2, 0.5, 0.6), (1.8, 0.5, 0.6)])[1] == "no_frontier"
    path, mode = ds.plan(g, pose, goal=(1.2, 0.5), info=info, avoid=[(1.2, 0.5, 0.6)])
    assert mode == "goal"


def corner_node(nav, pose):
    g, info, d = l_corridor()
    nav.grid, nav.info, nav.distances = g, info, d
    nav.pose = pose; nav.path = hugging_path(); nav.mode = "explore"
    return g, info, d


def test_short_lookahead_at_the_corner_keeps_walking_without_the_guard(nav):
    import duck_nav.duckslam as ds
    g, info, d = corner_node(nav, CUT_POSE)                          # the 0.22 m lookahead arc fails here
    nav.drive_step()
    cmd = nav.pub_cmd.records[-1]
    assert 0.25 < cmd.linear.x < 0.5                                 # slowed for the corner, still walking
    assert ds.motion_clear(g, nav.pose, 0.35, cmd.angular.z, info=info, distances=d)
    assert nav.pub_status.records == []


def test_drive_takes_a_clear_arc_instead_of_stopping(nav):
    import duck_nav.duckslam as ds
    g, info, d = corner_node(nav, ARC_POSE)
    nav.drive_step()
    cmd = nav.pub_cmd.records[-1]
    assert cmd.linear.x > 0.25                                       # still walking (below 0.25 the gait stands)
    assert ds.motion_clear(g, nav.pose, 0.35, cmd.angular.z, info=info, distances=d)
    assert nav.pub_status.records[-1].data == "near an obstacle: steering on a clear arc"


def test_guard_still_stops_when_no_arc_is_clear_then_recovers(nav):
    corner_node(nav, WALL_POSE)
    nav.drive_step()
    cmd = nav.pub_cmd.records[-1]
    assert cmd.linear.x == 0.0 and abs(cmd.angular.z) >= 0.5         # turn in place, never sit at 0 rad/s
    assert nav.pub_status.records[-1].data.startswith("path clearance guard")
    goal_end = nav.path[-1]
    for _ in range(40):                                              # 2 s of guard without progress
        nav.drive_step()
        assert nav.pub_cmd.records[-1].linear.x == 0.0
    assert nav.path is None and nav.avoid and nav.avoid[0][:2] == goal_end   # exploring: give up that frontier
    assert nav.pub_status.records[-1].data == "path clearance guard: trying another frontier"
    nav.path = hugging_path()                                        # replanned into the same spot: manoeuvre
    for _ in range(40):
        nav.drive_step()
    assert nav.manoeuvre is not None
    assert nav.pub_status.records[-1].data == "path clearance guard: local manoeuvre"


def test_local_manoeuvre_turns_then_walks_only_while_clear(nav):
    import duck_nav.duckslam as ds
    g, info, d = corner_node(nav, WALL_POSE)
    nav.mode = "goal"
    nav.recover(5.0)
    heading = nav.manoeuvre[0]
    assert ds.motion_clear(g, (*nav.pose[:2], heading), 0.35, 0.0, info=info, distances=d)
    nav.drive_step()
    assert nav.pub_cmd.records[-1].linear.x == 0.0 and nav.pub_cmd.records[-1].angular.z != 0.0
    nav.pose = (*nav.pose[:2], heading)                             # turned
    nav.drive_step()
    assert nav.pub_cmd.records[-1].linear.x > 0.25
    nav.pose = WALL_POSE                                            # facing the wall again: the step ends
    nav.drive_step()
    assert nav.manoeuvre is None and nav.path is None


def test_boxed_in_falls_back_to_turning_in_place(nav):
    import duck_nav.duckslam as ds
    nav.grid = np.full((5, 5), 100, np.int8); nav.grid[2, 2] = 0
    nav.info = ds.MapInfo(res=0.025, x0=-0.0625, y0=-0.0625, nx=5, ny=5)
    nav.distances = ds._clearance_m(nav.grid, nav.info)[1]
    nav.mode = "goal"
    nav.recover(5.0)
    assert nav.manoeuvre is None and nav.unstick_until == pytest.approx(7.5)


def test_corner_handling_off_is_plain_pursuit_and_guard(nav):
    corner_node(nav, CUT_POSE)
    nav.get_parameter = lambda key: message(value=dict(PARAMS, corner_handling=False)[key])
    for _ in range(60):
        nav.drive_step()
    cmd = nav.pub_cmd.records[-1]
    assert cmd.linear.x == 0.0 and nav.avoid == [] and nav.manoeuvre is None
    assert nav.pub_status.records[-1].data == "path clearance guard: turning in place and replanning"


def test_avoided_frontiers_never_end_exploration(nav, monkeypatch):
    import duck_nav.duckslam as ds
    nav.info = ds.MapInfo(nx=5, ny=5); nav.goal = None; nav.visited_goal = False; nav.map_revision = 0
    nav.avoid = [(1.0, 0.0, 0.4, 100.0), (2.0, 0.0, 0.4, 1.0)]       # the second one has expired
    calls = []

    def plan(*args, avoid=(), **kwargs):
        calls.append(list(avoid))
        return (None, "no_frontier") if avoid else ([(0., 0.), (1., 0.)], "explore")
    monkeypatch.setattr(ds, "plan", plan)
    applied = []
    nav.apply_plan = lambda path, mode, state: applied.append(mode)
    nav.replan()
    assert calls == [[(1.0, 0.0, 0.4)], []] and applied == ["explore"]
    assert len(nav.avoid) == 1
