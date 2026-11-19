"""Scoring completion protocol; real write method with ROS imports stubbed."""
import importlib
import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_completion_marker_follows_artifacts_even_when_optional_map_fails(monkeypatch, tmp_path):
    definitions = {
        'rclpy': {}, 'rclpy.executors': {'ExternalShutdownException': Exception},
        'rclpy.node': {'Node': object},
        'rclpy.qos': {'DurabilityPolicy': object, 'QoSProfile': object},
        'geometry_msgs.msg': {'PointStamped': object, 'PoseStamped': object},
        'nav_msgs.msg': {'OccupancyGrid': object}, 'std_msgs.msg': {'String': object},
        'vision_msgs.msg': {'Detection2DArray': object},
    }
    for name, fields in definitions.items():
        module = ModuleType(name); module.__dict__.update(fields)
        monkeypatch.setitem(sys.modules, name, module)
    module = importlib.import_module('duck_nav.score_node')
    try:
        node = object.__new__(module.ScoreNode)
        values = dict(out_dir=str(tmp_path), beacon_x=4., beacon_y=1.3,
                      profile='realistic', beacon_reach_m=.45)
        node.get_parameter = lambda key: NS(value=values[key])
        node.now = lambda: 600.
        node.true_xy = [(0., 0.), (1., 0.)]; node.est_xy = [(0., 0.), (1.1, 0.)]
        node.result = None; node.goal = None; node.errs = [.0, .1]
        node.t_beacon = None; node.d_min = 2.; node.falls = 1
        node.det = {}; node.frames = 2; node.events = []
        node.grid = None  # No map received: optional rendering must fail visibly.
        node.coverage = lambda: 0.
        warnings = []
        def on_finished(_):
            assert (tmp_path / 'result.json').is_file()
            assert (tmp_path / 'summary.txt').is_file()
            assert not (tmp_path / 'run_complete.json').exists()
        node.get_logger = lambda: NS(warn=warnings.append, info=on_finished)
        node.write()
        marker = json.loads((tmp_path / 'run_complete.json').read_text())
        assert marker == dict(schema_version=1, sim_time_s=600., artifacts=['result.json', 'summary.txt'])
        assert warnings and not (tmp_path / 'run_complete.json.tmp').exists()
        result = json.loads((tmp_path / 'result.json').read_text())
        assert result['slam_error_m']['samples'] == 2
        assert result['walked_m'] == 1.
    finally:
        sys.modules.pop('duck_nav.score_node', None)
